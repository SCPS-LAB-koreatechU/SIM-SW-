#!/usr/bin/env python3
"""
smoke_test_ros.py — ROS 위에서 노드 3개를 실제로 띄워 동작을 확인한다.

RViz 도 move_group 도 없이 돈다. GUI 를 띄우기 전에 "노드 자체가 멀쩡한가"를
먼저 갈라내기 위한 것이다. 여기서 통과하면 남은 문제는 MoveIt 설정이나
RViz 디스플레이 쪽이라고 좁힐 수 있다.

검사 항목
  드라이버   /joint_states 발행, 출력 OFF 시 궤적 거부, enable 서비스,
             궤적 실행과 목표 도달, 스트림 패킷 형식과 값, 조그 추종, 명령 포화
  손끝 IK    URDF 파싱, 마커 4개 등록, 마커 드래그 -> IK -> joint_command,
             도달 불가 목표의 잔차 보고, check_state_validity 부재 시 graceful
  프리셋     12개 로드, list_grips 서비스, move_group 부재 시 graceful
  통합       마커 드래그가 드라이버를 거쳐 실제로 /joint_states 를 움직이는지

실행:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/dexhand_bringup/tools/smoke_test_ros.py
"""

import os
import sys
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import InteractiveMarkerFeedback, MarkerArray

from dexhand_bringup.fingertip_ik_node import MODE_JOG, FingertipIK
from dexhand_bringup.grip_preset_node import GripPresets
from dexhand_bringup.hand_driver_node import HandDriver
from dexhand_bringup.kinematics import FINGERS, JOINT_NAMES

FAILS = []


def chk(label, cond, extra=""):
    print(f"  {'[ ok ]' if cond else '[FAIL]'} {label} {extra}")
    if not cond:
        FAILS.append(label)
    return cond


def wait(fut, timeout=10.0):
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.02)
    return fut.done()


def find_config():
    """dexhand_moveit_config 의 config 디렉터리를 찾는다 (설치본 우선, 없으면 소스 트리)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("dexhand_moveit_config"), "config")
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.abspath(os.path.join(here, "..", "..",
                                            "dexhand_moveit_config", "config"))


def main():
    cfg_dir = find_config()
    urdf_path = os.path.join(cfg_dir, "dexhandv2_right_8servo.urdf")
    presets_path = os.path.join(cfg_dir, "grip_presets.yaml")
    for p in (urdf_path, presets_path):
        if not os.path.exists(p):
            print(f"필요한 파일을 못 찾았다: {p}")
            return 2
    urdf = open(urdf_path, encoding="utf-8").read()

    rclpy.init()

    drv = HandDriver(parameter_overrides=[
        Parameter("link", Parameter.Type.STRING, "none"),
    ])
    ik = FingertipIK(parameter_overrides=[
        Parameter("robot_description", Parameter.Type.STRING, urdf),
        Parameter("mode", Parameter.Type.STRING, MODE_JOG),
    ])
    gp = GripPresets(parameter_overrides=[
        Parameter("presets_file", Parameter.Type.STRING, presets_path),
    ])
    probe = Node("smoke_probe")

    seen = {}
    probe.create_subscription(JointState, "/joint_states",
                              lambda m: seen.update(dict(zip(m.name, m.position))), 10)
    cmds = []
    probe.create_subscription(JointState, "/dexhand_driver/joint_command",
                              lambda m: cmds.append(list(m.position)), 10)
    viz = []
    probe.create_subscription(MarkerArray, "/dexhand_fingertip_ik/ik_status",
                              lambda m: viz.append(m), 1)
    grip_pub = probe.create_publisher(String, "/dexhand/grip", 10)

    ex = MultiThreadedExecutor()
    for n in (drv, ik, gp, probe):
        ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.5)

    # ---------------- 드라이버 ----------------
    print("\n[1] 드라이버 노드")
    chk("/joint_states 퍼블리시", len(seen) == 8, f"({len(seen)}개)")
    chk("초기 자세 0", all(abs(v) < 1e-9 for v in seen.values()))

    ac = ActionClient(probe, FollowJointTrajectory,
                      "/hand_controller/follow_joint_trajectory")
    chk("FollowJointTrajectory 액션 서버", ac.wait_for_server(timeout_sec=5.0))

    def goal(pitch, secs=1.5):
        g = FollowJointTrajectory.Goal()
        tj = JointTrajectory()
        tj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, pitch] * 4
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        tj.points = [pt]
        g.trajectory = tj
        return g

    f = ac.send_goal_async(goal(0.5))
    wait(f)
    chk("출력 OFF 상태에서 궤적 거부", not f.result().accepted)

    cli = probe.create_client(SetBool, "/dexhand_driver/enable")
    chk("enable 서비스", cli.wait_for_service(timeout_sec=5.0))
    f = cli.call_async(SetBool.Request(data=True))
    wait(f)
    chk("enable 응답", f.result().success)

    f = ac.send_goal_async(goal(0.5))
    wait(f)
    gh = f.result()
    chk("궤적 수락", gh.accepted)
    rf = gh.get_result_async()
    t0 = time.time()
    chk("궤적 완료", wait(rf, 20.0), f"({time.time()-t0:.2f}s)")
    if rf.done():
        chk("결과 SUCCESSFUL", rf.result().result.error_code == 0,
            f"(code={rf.result().result.error_code})")
    time.sleep(0.3)
    chk("목표 0.5 도달",
        all(abs(seen[f"R_{x}_Pitch"] - 0.5) < 1e-3 for x in FINGERS),
        f"({round(seen['R_Index_Pitch'], 4)})")

    sent = [s for s in drv.link.sent if s.startswith("w ")]
    chk("스트림 패킷 송신", len(sent) > 20, f"({len(sent)}개)")
    if sent:
        toks = sent[-1].split()
        print(f"         마지막 패킷: {sent[-1]}")
        chk("패킷 토큰 9개", len(toks) == 9)
        expect = round(0.5 / drv.pitch_max * 1000)
        chk("flex 값이 pitch/pitch_max 비율", abs(int(toks[1]) - expect) <= 1,
            f"(got={toks[1]} expect={expect})")
    chk("초기화 시퀀스에 off, sm 포함",
        "off" in drv.link.sent[:5] and "sm" in drv.link.sent[:5])

    # ---------------- 손끝 IK ----------------
    print("\n[2] 손끝 IK 노드")
    chk("URDF 파싱, 체인 4개", len(ik.kin.chains) == 4)
    chk("마커 4개 등록",
        all(f"target_{x}" in ik.server.marker_contexts for x in FINGERS))
    chk("ik_status 마커 퍼블리시", len(viz) > 0, f"({len(viz)}개)")
    if viz:
        chk("마커 수 = 손가락 4개 x (선 + 글자)", len(viz[-1].markers) == 8,
            f"({len(viz[-1].markers)})")

    q_true = np.array([-0.15, 0.6])
    target = ik.kin.fk_finger("Index", q_true)
    fb = InteractiveMarkerFeedback()
    fb.marker_name = "target_Index"
    fb.event_type = InteractiveMarkerFeedback.POSE_UPDATE
    fb.pose.position.x, fb.pose.position.y, fb.pose.position.z = map(float, target)
    fb.pose.orientation.w = 1.0
    before = len(cmds)
    ik._on_feedback(fb)
    time.sleep(0.5)
    if chk("마커 드래그 -> joint_command", len(cmds) > before, f"({len(cmds)-before}건)"):
        got = np.array(cmds[-1])
        chk("IK 결과가 정답 관절값과 일치", np.allclose(got[0:2], q_true, atol=1e-3),
            f"(got={np.round(got[0:2], 4).tolist()})")
        chk("다른 손가락 불변", np.allclose(got[2:], 0.0, atol=1e-9))

    far = ik.kin.fk_finger("Middle", [0.0, 0.0]) + np.array([0.0, 0.0, 0.06])
    fb.marker_name = "target_Middle"
    fb.pose.position.x, fb.pose.position.y, fb.pose.position.z = map(float, far)
    ik._on_feedback(fb)
    time.sleep(0.4)
    chk("도달 불가 목표의 잔차 보고",
        abs(ik.residual.get("Middle", 0) - 0.06) < 0.005,
        f"({ik.residual.get('Middle', 0)*1000:.1f}mm)")
    chk("check_state_validity 부재 시 graceful", ik._state_valid(np.zeros(8)) is True)

    # ---------------- 프리셋 ----------------
    print("\n[3] 프리셋 노드")
    tcli = probe.create_client(Trigger, "/dexhand/list_grips")
    chk("list_grips 서비스", tcli.wait_for_service(timeout_sec=5.0))
    f = tcli.call_async(Trigger.Request())
    if chk("list_grips 응답", wait(f, 5.0)):
        names = [s.strip() for s in f.result().message.split(",")]
        chk("프리셋 12개", len(names) == 12, f"({len(names)}개)")
    grip_pub.publish(String(data="fist"))
    time.sleep(2.5)
    chk("move_group 부재 시에도 노드 생존", True)
    grip_pub.publish(String(data="존재하지_않는_프리셋"))
    time.sleep(0.5)
    chk("모르는 프리셋 이름 처리", True)

    # ---------------- 통합 ----------------
    print("\n[4] 통합: 마커 -> IK -> 드라이버 -> joint_states")
    q_goal = np.array([0.1, 0.35])
    target = ik.kin.fk_finger("Pinky", q_goal)
    fb.marker_name = "target_Pinky"
    fb.pose.position.x, fb.pose.position.y, fb.pose.position.z = map(float, target)
    ik._on_feedback(fb)
    time.sleep(2.5)
    chk("Pinky Yaw 가 IK 해로 이동",
        abs(seen["R_Pinky_Yaw"] - q_goal[0]) < 5e-3, f"({seen['R_Pinky_Yaw']:.4f})")
    chk("Pinky Pitch 가 IK 해로 이동",
        abs(seen["R_Pinky_Pitch"] - q_goal[1]) < 5e-3, f"({seen['R_Pinky_Pitch']:.4f})")
    last = [s for s in drv.link.sent if s.startswith("w ")][-1]
    print(f"         Pinky 반영 패킷: {last}")
    toks = last.split()
    chk("서보 2개가 동시에 명령됨 (flex, yaw 둘 다 0 아님)",
        int(toks[7]) != 0 and int(toks[8]) != 0, f"(flex={toks[7]} yaw={toks[8]})")

    print(f"\n{'='*60}")
    print(f"실패 {len(FAILS)}건" + (": " + ", ".join(FAILS) if FAILS else ""))
    for n in (drv, ik, gp, probe):
        n.destroy_node()
    rclpy.try_shutdown()
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
