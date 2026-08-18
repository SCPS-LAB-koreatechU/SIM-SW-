#!/usr/bin/env python3
"""
make_collision_matrix.py — SRDF 의 disable_collisions 목록을 자동 생성한다.

MoveIt Setup Assistant 가 하는 일과 같은 알고리즘이다.
  1) 인접(부모-자식) 링크쌍   -> 항상 접촉하므로 Adjacent 로 제외
  2) 기본자세에서 이미 충돌   -> Default 로 제외
  3) N회 랜덤 샘플에서 단 한 번도 충돌하지 않음 -> Never 로 제외
나머지 쌍만 실제 자기충돌 검사 대상으로 남는다.

Setup Assistant 를 GUI 로 띄우지 않고도 같은 결과를 얻기 위해 만들었다.
(자유도 8개짜리 손이라 샘플 수를 넉넉히 줘도 몇 십 초면 끝난다.)
"""

import argparse
import itertools
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "dexhand_bringup"))
from dexhand_bringup.kinematics import (  # noqa: E402
    axis_angle_to_matrix, rpy_to_matrix,
)


def parse_urdf(path, mesh_root):
    root = ET.parse(path).getroot()

    links = {}
    for l in root.findall("link"):
        col = l.find("collision")
        if col is None:
            continue
        m = col.find("geometry/mesh")
        if m is None:
            continue
        o = col.find("origin")
        xyz = np.array([float(v) for v in o.get("xyz").split()]) if o is not None and o.get("xyz") \
            else np.zeros(3)
        scale = np.array([float(v) for v in m.get("scale").split()]) if m.get("scale") \
            else np.ones(3)
        fn = m.get("filename").replace("package://dexhandv2_description/", "")
        links[l.get("name")] = dict(mesh=os.path.join(mesh_root, fn), xyz=xyz, scale=scale)

    joints = []
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = np.zeros(3)
        rpy = np.zeros(3)
        if o is not None:
            if o.get("xyz"):
                xyz = np.array([float(v) for v in o.get("xyz").split()])
            if o.get("rpy"):
                rpy = np.array([float(v) for v in o.get("rpy").split()])
        ax = j.find("axis")
        axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None \
            else np.array([0.0, 0.0, 1.0])
        lim = j.find("limit")
        lo = float(lim.get("lower")) if lim is not None and lim.get("lower") else 0.0
        hi = float(lim.get("upper")) if lim is not None and lim.get("upper") else 0.0
        joints.append(dict(name=j.get("name"), type=j.get("type"),
                           parent=j.find("parent").get("link"),
                           child=j.find("child").get("link"),
                           xyz=xyz, rot=rpy_to_matrix(rpy), axis=axis, lo=lo, hi=hi))
    return links, joints


def link_poses(joints, q, base="base_link"):
    """조인트 트리를 훑어 각 링크의 (R, p) 를 계산한다."""
    poses = {base: (np.eye(3), np.zeros(3))}
    changed = True
    qi = {j["name"]: i for i, j in enumerate(
        [j for j in joints if j["type"] == "revolute"])}
    while changed:
        changed = False
        for j in joints:
            if j["parent"] in poses and j["child"] not in poses:
                R, p = poses[j["parent"]]
                p2 = p + R @ j["xyz"]
                R2 = R @ j["rot"]
                if j["type"] == "revolute":
                    R2 = R2 @ axis_angle_to_matrix(j["axis"], float(q[qi[j["name"]]]))
                poses[j["child"]] = (R2, p2)
                changed = True
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--mesh-root", required=True,
                    help="dexhandv2_description 패키지 루트 (meshes/ 의 부모)")
    ap.add_argument("--samples", type=int, default=4000)
    ap.add_argument("--max-faces", type=int, default=3000,
                    help="링크당 충돌 메시 삼각형 상한 (데시메이션)")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    links, joints = parse_urdf(args.urdf, args.mesh_root)
    rev = [j for j in joints if j["type"] == "revolute"]
    lo = np.array([j["lo"] for j in rev])
    hi = np.array([j["hi"] for j in rev])
    print(f"구동 조인트 {len(rev)}개: {[j['name'] for j in rev]}")

    # 메시 로드.
    # 볼록껍질(convex hull)은 이 손에서 못 쓴다. 손바닥이 오목하고 손가락이 촘촘해서
    # 껍질을 씌우면 실제로는 절대 안 닿는 링크들이 전부 충돌로 잡힌다
    # (검증: 껍질 기준으로는 프리셋 12개가 전부 충돌 판정).
    # MoveIt 도 실제 삼각형 메시에 BVH 를 씌워 검사하므로 여기서도 원본 메시를 쓴다.
    # 다만 2만 삼각형은 과하므로 정점 병합 + 데시메이션으로 가볍게 만든다.
    geo = {}
    for name, d in links.items():
        m = trimesh.load(d["mesh"], force="mesh")
        m.apply_scale(d["scale"])
        m.apply_translation(d["xyz"])
        m.merge_vertices()
        if len(m.faces) > args.max_faces:
            try:
                m = m.simplify_quadric_decimation(face_count=args.max_faces)
            except Exception:
                pass
        geo[name] = m
        print(f"  {name:26s} faces={len(m.faces)}")

    names = sorted(geo.keys())
    pairs = list(itertools.combinations(names, 2))

    adjacent = set()
    for j in joints:
        if j["parent"] in geo and j["child"] in geo:
            adjacent.add(tuple(sorted((j["parent"], j["child"]))))

    cm = trimesh.collision.CollisionManager()
    for n in names:
        cm.add_object(n, geo[n])

    def colliding(q):
        poses = link_poses(joints, q)
        for n in names:
            R, p = poses[n]
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = p
            cm.set_transform(n, T)
        _, cpairs = cm.in_collision_internal(return_names=True)
        return {tuple(sorted(c)) for c in cpairs}

    default_col = colliding(np.zeros(len(rev)))
    print(f"\n기본자세 충돌쌍 {len(default_col)}개")

    rng = np.random.default_rng(0)
    ever = set(default_col)
    always = set(default_col)
    for i in range(args.samples):
        q = rng.uniform(lo, hi)
        c = colliding(q)
        ever |= c
        always &= c
        if (i + 1) % 500 == 0:
            print(f"  샘플 {i+1}/{args.samples}  누적 충돌쌍={len(ever)}")

    never = [p for p in pairs if p not in ever]

    entries = []
    for p in sorted(pairs):
        if p in adjacent:
            entries.append((p, "Adjacent"))
        elif p in always:
            entries.append((p, "Default"))
        elif p in never:
            entries.append((p, "Never"))

    with open(args.out, "w", encoding="utf-8") as fp:
        for (a, b), reason in entries:
            fp.write(f'    <disable_collisions link1="{a}" link2="{b}" reason="{reason}"/>\n')

    kept = len(pairs) - len(entries)
    print(f"\n전체 쌍 {len(pairs)} / 제외 {len(entries)} / 실제 검사대상 {kept}")
    print(f"  Adjacent={sum(1 for _,r in entries if r=='Adjacent')}"
          f" Default={sum(1 for _,r in entries if r=='Default')}"
          f" Never={sum(1 for _,r in entries if r=='Never')}")
    print(f"wrote {args.out}")
    for (a, b), r in entries:
        if r == "Default":
            print(f"  [Default] {a} <-> {b}  (기본자세에서 이미 접촉 — 메시 간섭 확인 요망)")


if __name__ == "__main__":
    main()
