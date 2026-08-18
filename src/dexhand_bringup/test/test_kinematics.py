"""colcon test 로 도는 순수 파이썬 검사. ROS 도 아두이노도 필요 없다."""

import os

import numpy as np
import pytest

from dexhand_bringup.kinematics import FINGERS, JOINT_NAMES, DexHandKinematics

URDF = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "dexhand_moveit_config", "config", "dexhandv2_right_8servo.urdf")


@pytest.fixture(scope="module")
def kin():
    if not os.path.exists(URDF):
        pytest.skip(f"URDF 없음: {URDF}")
    with open(URDF, encoding="utf-8") as fp:
        return DexHandKinematics(fp.read())


def test_joint_names_order():
    assert JOINT_NAMES == (
        "R_Index_Yaw", "R_Index_Pitch",
        "R_Middle_Yaw", "R_Middle_Pitch",
        "R_Ring_Yaw", "R_Ring_Pitch",
        "R_Pinky_Yaw", "R_Pinky_Pitch",
    )


def test_each_finger_has_exactly_two_dof(kin):
    for f in FINGERS:
        assert kin.chains[f].dof_names == [f"R_{f}_Yaw", f"R_{f}_Pitch"]


def test_ik_round_trip(kin):
    """FK 로 만든 목표는 IK 가 0.01mm 안으로 되찾아야 한다."""
    rng = np.random.default_rng(0)
    for f in FINGERS:
        lim = kin.chains[f].limits
        for _ in range(50):
            q = rng.uniform(lim[:, 0], lim[:, 1])
            target = kin.fk_finger(f, q)
            qs, _res, _ok = kin.ik_finger(f, target)
            err = np.linalg.norm(kin.fk_finger(f, qs) - target)
            assert err < 1e-5, f"{f} 오차 {err*1000:.4f}mm"


def test_unreachable_target_reports_residual(kin):
    """자유도가 2개뿐이라 도달 못 하는 목표가 반드시 있다.
    그때 조용히 아무 자세나 주면 안 되고 잔차를 보고해야 한다."""
    base = kin.fk_finger("Index", [0.0, 0.0])
    q, res, ok = kin.ik_finger("Index", base + np.array([0.0, 0.0, 0.05]))
    assert not ok
    assert res > 0.04
    lim = kin.chains["Index"].limits
    assert np.all(q >= lim[:, 0] - 1e-9) and np.all(q <= lim[:, 1] + 1e-9)


def test_ik_respects_joint_limits(kin):
    """한계 밖 목표를 줘도 해는 항상 한계 안이어야 한다.
    이게 깨지면 서보가 기구 한계를 밀어 텐던이 늘어난다."""
    rng = np.random.default_rng(1)
    for f in FINGERS:
        lim = kin.chains[f].limits
        for _ in range(50):
            target = rng.uniform(-0.2, 0.2, 3)
            q, _res, _ok = kin.ik_finger(f, target)
            assert np.all(q >= lim[:, 0] - 1e-9)
            assert np.all(q <= lim[:, 1] + 1e-9)


def test_yaw_direction_is_uniform(kin):
    """+yaw 는 네 손끝을 모두 같은 방향(-y)으로 민다.
    프리셋의 '벌리기' 부호가 이 사실에 의존한다."""
    for f in FINGERS:
        p0 = kin.fk_finger(f, [0.0, 0.0])
        p1 = kin.fk_finger(f, [0.1, 0.0])
        assert (p1 - p0)[1] < -0.005


def test_pitch_direction_is_flexion(kin):
    """+pitch 는 손끝을 손바닥 앞(+x)으로 보낸다 = 굽힘."""
    for f in FINGERS:
        p0 = kin.fk_finger(f, [0.0, 0.0])
        p1 = kin.fk_finger(f, [0.0, 0.1])
        assert (p1 - p0)[0] > 0.005
