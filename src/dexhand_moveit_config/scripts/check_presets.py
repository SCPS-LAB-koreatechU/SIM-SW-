#!/usr/bin/env python3
"""
check_presets.py — 프리셋 그립 포즈가 실제로 안전한지 검사한다.

검사 항목
  1) 관절 한계 초과 여부
  2) SRDF 의 disable_collisions 를 뺀 자기충돌 (실제 삼각형 메시 기준)
  3) 서보 명령 포화 여부: 각 포즈를 서보 us 로 환산해 캘리브레이션 min/max 를 넘지 않는지
     (넘으면 실물에서는 조용히 clamp 되어 GUI 와 실제 손 모양이 어긋난다)

프리셋을 고칠 때마다 이걸 돌려야 한다. GUI 버튼 하나가 손가락끼리 부딪히게 만들면
텐던이 늘어나거나 서보가 스톨로 타 버린다.
"""

import argparse
import importlib.util
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))

spec = importlib.util.spec_from_file_location(
    "mcm", os.path.join(HERE, "make_collision_matrix.py"))
mcm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mcm)

FINGERS = ["Index", "Middle", "Ring", "Pinky"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--srdf", required=True)
    ap.add_argument("--presets", required=True)
    ap.add_argument("--servo-map", default=None,
                    help="dexhand_bringup/config/servo_map.yaml (있으면 us 포화 검사도 수행)")
    ap.add_argument("--mesh-root", required=True)
    args = ap.parse_args()

    links, joints = mcm.parse_urdf(args.urdf, args.mesh_root)
    rev = [j for j in joints if j["type"] == "revolute"]
    order = [j["name"] for j in rev]
    lim = {j["name"]: (j["lo"], j["hi"]) for j in rev}

    geo = {}
    for n, d in links.items():
        m = trimesh.load(d["mesh"], force="mesh")
        m.apply_scale(d["scale"])
        m.apply_translation(d["xyz"])
        m.merge_vertices()
        geo[n] = m

    cm = trimesh.collision.CollisionManager()
    for n in geo:
        cm.add_object(n, geo[n])

    srdf = ET.parse(args.srdf).getroot()
    disabled = {tuple(sorted((d.get("link1"), d.get("link2"))))
                for d in srdf.findall("disable_collisions")}

    presets = yaml.safe_load(open(args.presets, encoding="utf-8"))["presets"]

    smap = None
    if args.servo_map and os.path.exists(args.servo_map):
        smap = yaml.safe_load(open(args.servo_map, encoding="utf-8"))
        smap = smap["dexhand_driver"]["ros__parameters"]

    fails = 0
    for name, p in presets.items():
        q = {}
        for i, f in enumerate(FINGERS):
            q[f"R_{f}_Yaw"] = float(p["yaw"][i])
            q[f"R_{f}_Pitch"] = float(p["pitch"][i])
        qv = np.array([q[n] for n in order])

        msgs = []

        for n, v in q.items():
            lo, hi = lim[n]
            if v < lo - 1e-9 or v > hi + 1e-9:
                msgs.append(f"관절한계초과 {n}={v:.3f} not in [{lo:.3f},{hi:.3f}]")

        poses = mcm.link_poses(joints, qv)
        for n in geo:
            R, pp = poses[n]
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = pp
            cm.set_transform(n, T)
        _, cp = cm.in_collision_internal(return_names=True)
        active = [tuple(sorted(c)) for c in cp if tuple(sorted(c)) not in disabled]
        if active:
            msgs.append(f"자기충돌 {active}")

        if smap:
            pmax = smap["pitch_max_rad"]
            ymax = smap["yaw_max_rad"]
            fspan = smap["flex_span_us"]
            yspan = smap["yaw_span_us"]
            for i, f in enumerate(FINGERS):
                flex = q[f"R_{f}_Pitch"] / pmax
                yaw = q[f"R_{f}_Yaw"] / ymax
                for k, tag in enumerate(("out", "in")):
                    ch = i * 2 + k
                    c = smap["servo_cal"][ch]
                    us = c["neutral"] + c["flex_sign"] * flex * fspan \
                        + c["yaw_sign"] * yaw * yspan
                    if us < c["min"] - 0.5 or us > c["max"] + 0.5:
                        msgs.append(
                            f"서보포화 CH{ch}({smap['servo_names'][ch]}) "
                            f"{us:.0f}us not in [{c['min']},{c['max']}]")

        if msgs:
            fails += 1
            print(f"  [FAIL] {name}")
            for m in msgs:
                print(f"         - {m}")
        else:
            print(f"  [ ok ] {name}")

    print(f"\n프리셋 {len(presets)}개 중 문제 {fails}개")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
