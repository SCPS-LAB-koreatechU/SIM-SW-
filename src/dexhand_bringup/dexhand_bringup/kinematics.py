#!/usr/bin/env python3
"""
kinematics.py — DexHand v2 8서보 손의 순기구학 / 손끝 역기구학

이 모듈은 ROS 에 의존하지 않는다 (numpy 만 필요).
그래서 ROS 없이도 단독으로 검증할 수 있고, 노드에서는 import 만 해서 쓴다.

기구 구조 (손가락 1개):
    base_link --[R_<F>_Yaw]--> <F>_Knuckle_Cross_1 --[R_<F>_Pitch]--> <F>_Knuckle_1
              --(fixed)--> <F>_Middle_1 --(fixed)--> <F>_Tip_1 --(fixed)--> R_<F>_tip_frame

구동 자유도가 2개(Yaw, Pitch)뿐이므로 손끝 위치(3차원)는 일반적으로 정확히
맞출 수 없다. 손끝이 도달 가능한 집합은 3차원 공간 안의 2차원 곡면이다.
따라서 IK 는 "목표점에 가장 가까운 도달 가능점"을 찾는 최소자승 문제로 푼다.
이것이 KDL 같은 표준 IK 플러그인이 이 손에서 실패하는 이유이기도 하다.
(KDL 은 6-DOF 완전 해를 전제하고, position_only_ik 를 켜도 3개 구속에 2개
 자유도라 과결정계가 되어 수렴하지 못한다.)

푸는 방법: 감쇠 최소자승(damped least squares) 가우스-뉴턴 +
관절 한계 클램핑 + 다중 초기값. 자유도가 2개라 반복 20회 안에 수렴한다.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np

FINGERS = ("Index", "Middle", "Ring", "Pinky")

#: 8서보 손의 구동 조인트. 드라이버/컨트롤러의 조인트 순서는 항상 이 순서를 따른다.
JOINT_NAMES = tuple(f"R_{f}_{k}" for f in FINGERS for k in ("Yaw", "Pitch"))


def rpy_to_matrix(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """로드리게스 공식. axis 는 단위벡터여야 한다."""
    k = np.asarray(axis, dtype=float)
    n = np.linalg.norm(k)
    if n < 1e-12:
        return np.eye(3)
    k = k / n
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    s, c = math.sin(angle), math.cos(angle)
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


@dataclass
class JointSpec:
    name: str
    jtype: str
    parent: str
    child: str
    xyz: np.ndarray
    rot: np.ndarray          # origin rpy 를 미리 행렬로
    axis: np.ndarray
    lower: float = 0.0
    upper: float = 0.0


@dataclass
class FingerChain:
    """base_link 에서 손끝 프레임까지의 직렬 체인 하나."""
    finger: str
    joints: list = field(default_factory=list)   # JointSpec 리스트 (root -> tip 순)
    tip_frame: str = ""

    @property
    def dof_names(self):
        return [j.name for j in self.joints if j.jtype == "revolute"]

    @property
    def limits(self):
        return np.array([[j.lower, j.upper] for j in self.joints if j.jtype == "revolute"])

    def fk(self, q) -> np.ndarray:
        """구동 관절값 q(rad, [yaw, pitch]) → 손끝 위치 (base_link 좌표계, m)."""
        p = np.zeros(3)
        R = np.eye(3)
        qi = 0
        for j in self.joints:
            p = p + R @ j.xyz
            R = R @ j.rot
            if j.jtype == "revolute":
                R = R @ axis_angle_to_matrix(j.axis, float(q[qi]))
                qi += 1
        return p

    def jacobian(self, q) -> np.ndarray:
        """3x2 위치 자코비안. 축과 관절 원점이 필요하므로 FK 를 한 번 더 훑는다."""
        p = np.zeros(3)
        R = np.eye(3)
        origins, axes = [], []
        qi = 0
        for j in self.joints:
            p = p + R @ j.xyz
            R = R @ j.rot
            if j.jtype == "revolute":
                origins.append(p.copy())
                axes.append(R @ j.axis)
                R = R @ axis_angle_to_matrix(j.axis, float(q[qi]))
                qi += 1
        tip = self.fk(q)
        J = np.zeros((3, len(axes)))
        for i, (o, a) in enumerate(zip(origins, axes)):
            J[:, i] = np.cross(a, tip - o)
        return J


class DexHandKinematics:
    """URDF 문자열 하나에서 손가락 4개 체인을 뽑아 둔다."""

    def __init__(self, urdf_xml: str, base_link: str = "base_link"):
        root = ET.fromstring(urdf_xml)
        self.base_link = base_link

        specs = {}
        children = {}
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
            spec = JointSpec(
                name=j.get("name"),
                jtype=j.get("type"),
                parent=j.find("parent").get("link"),
                child=j.find("child").get("link"),
                xyz=xyz,
                rot=rpy_to_matrix(rpy),
                axis=axis,
                lower=lo,
                upper=hi,
            )
            specs[spec.name] = spec
            children.setdefault(spec.parent, []).append(spec)

        self.chains = {}
        for f in FINGERS:
            tip = f"R_{f}_tip_frame"
            path = self._path_to(children, base_link, tip)
            if path is None:
                raise ValueError(f"URDF 에서 {base_link} -> {tip} 경로를 찾지 못했다")
            self.chains[f] = FingerChain(finger=f, joints=path, tip_frame=tip)

        # 조인트 이름 -> (손가락, 체인 내 인덱스)
        self.joint_index = {name: i for i, name in enumerate(JOINT_NAMES)}

    @staticmethod
    def _path_to(children, start, goal):
        stack = [(start, [])]
        while stack:
            link, acc = stack.pop()
            if link == goal:
                return acc
            for j in children.get(link, []):
                stack.append((j.child, acc + [j]))
        return None

    # ---------- 순기구학 ----------

    def fk_finger(self, finger: str, q2) -> np.ndarray:
        return self.chains[finger].fk(q2)

    def fk_all(self, q8) -> dict:
        q8 = np.asarray(q8, dtype=float)
        return {f: self.chains[f].fk(q8[2 * i:2 * i + 2]) for i, f in enumerate(FINGERS)}

    def home_positions(self) -> dict:
        return self.fk_all(np.zeros(8))

    # ---------- 역기구학 ----------

    def ik_finger(self, finger: str, target_xyz, q_init=None,
                  iters: int = 40, damping: float = 1e-3, tol: float = 1e-5):
        """손끝을 target_xyz 에 최대한 붙이는 (yaw, pitch) 를 찾는다.

        반환: (q2, residual_m, converged)
          residual_m 은 도달 가능한 최근접점과 목표점 사이 거리. 0 이 아니어도
          그것이 이 손의 물리적 한계다 (2-DOF 이므로 대개 0 이 아니다).
        """
        chain = self.chains[finger]
        lim = chain.limits
        target = np.asarray(target_xyz, dtype=float)

        # 다중 초기값: 국소최소해에 갇히는 걸 막는다. 2-DOF 라 4개면 충분하다.
        if q_init is not None:
            seeds = [np.asarray(q_init, dtype=float)]
        else:
            seeds = []
        seeds += [
            np.array([0.0, 0.0]),
            np.array([0.0, 0.5 * (lim[1, 0] + lim[1, 1])]),
            np.array([lim[0, 0] * 0.6, lim[1, 1] * 0.3]),
            np.array([lim[0, 1] * 0.6, lim[1, 1] * 0.7]),
        ]

        best_q, best_err = None, float("inf")
        for seed in seeds:
            q = np.clip(np.asarray(seed, dtype=float), lim[:, 0], lim[:, 1])
            for _ in range(iters):
                e = target - chain.fk(q)
                err = float(np.linalg.norm(e))
                if err < tol:
                    break
                J = chain.jacobian(q)
                # damped least squares: dq = J^T (J J^T + λ²I)^-1 e  대신
                # 2-DOF 라 정규방정식 쪽이 싸다: dq = (J^T J + λ²I)^-1 J^T e
                A = J.T @ J + (damping ** 2) * np.eye(J.shape[1])
                dq = np.linalg.solve(A, J.T @ e)
                # 한 스텝 각도 제한 (진동 방지)
                step = float(np.max(np.abs(dq)))
                if step > 0.3:
                    dq *= 0.3 / step
                q = np.clip(q + dq, lim[:, 0], lim[:, 1])
            err = float(np.linalg.norm(target - chain.fk(q)))
            if err < best_err:
                best_err, best_q = err, q.copy()

        return best_q, best_err, best_err < 1e-3

    def ik_all(self, targets: dict, q_init=None):
        """손가락별 목표점 dict → 8관절 벡터. 없는 손가락은 q_init(기본 0) 유지."""
        q = np.zeros(8) if q_init is None else np.asarray(q_init, dtype=float).copy()
        residuals = {}
        for i, f in enumerate(FINGERS):
            if f not in targets or targets[f] is None:
                continue
            q2, res, _ = self.ik_finger(f, targets[f], q_init=q[2 * i:2 * i + 2])
            q[2 * i:2 * i + 2] = q2
            residuals[f] = res
        return q, residuals


def load_from_file(path: str) -> DexHandKinematics:
    with open(path, "r", encoding="utf-8") as fp:
        return DexHandKinematics(fp.read())
