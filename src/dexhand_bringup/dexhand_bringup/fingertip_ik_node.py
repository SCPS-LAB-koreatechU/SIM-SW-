#!/usr/bin/env python3
"""
fingertip_ik_node.py — RViz 인터랙티브 마커로 손끝 목표점을 끌면 서보 2개가 같이 움직인다.

이 노드가 이번 작업의 핵심이다.
손가락 하나는 Yaw 서보와 Pitch 서보 2개로 구동되고, 손끝 위치는 그 둘의
"동시" 값으로만 결정된다. 마커를 끌면
    3D 목표점 -> 2-DOF 감쇠최소자승 IK -> (yaw, pitch) -> 서보 2개 동시 명령
이 한 번에 계산된다.

두 가지 모드
  jog  : 드래그하는 즉시 /dexhand_driver/joint_command 로 흘려보낸다. 반응이 즉각적이지만
         MoveIt 의 경로계획을 거치지 않는다. 대신 매 프레임 /check_state_validity 로
         자기충돌을 확인하고, 충돌하면 마지막 안전한 자세를 유지한다.
  plan : 마우스를 놓는 순간 MoveIt 에 관절공간 목표로 넘긴다. 충돌회피 경로계획을
         거쳐 FollowJointTrajectory 로 실행된다. 느리지만 안전하다.
마커 우클릭 메뉴에서 전환한다. 기본값은 plan.

도달 불가 목표 처리
  자유도가 2개라 손끝은 3차원 공간의 2차원 곡면 위에만 있을 수 있다. 목표점이 그
  곡면에서 벗어나면 IK 는 "가장 가까운 도달 가능점"을 준다. 이때 목표점과 실제
  손끝을 잇는 빨간 선과 잔차(mm)를 RViz 에 같이 그린다. 아무 표시 없이 대충 가까운
  자세로 가 버리면 사용자가 손이 고장난 걸로 오해한다.
"""

from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose
from interactive_markers import InteractiveMarkerServer, MenuHandler
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions, RobotState)
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker, MarkerArray)

from dexhand_bringup.kinematics import FINGERS, JOINT_NAMES, DexHandKinematics

MODE_PLAN = "plan"
MODE_JOG = "jog"

FINGER_COLOR = {
    "Index": (0.20, 0.65, 1.00),
    "Middle": (0.30, 0.85, 0.40),
    "Ring": (1.00, 0.75, 0.20),
    "Pinky": (0.95, 0.40, 0.70),
}


class FingertipIK(Node):

    def __init__(self, **kwargs):
        super().__init__("dexhand_fingertip_ik", **kwargs)

        self.declare_parameter("robot_description", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("mode", MODE_PLAN)
        self.declare_parameter("planning_group", "hand")
        self.declare_parameter("check_collisions", True)
        self.declare_parameter("jog_rate_hz", 20.0)
        self.declare_parameter("velocity_scaling", 0.3)

        self.base_frame = self.get_parameter("base_frame").value
        self.mode = self.get_parameter("mode").value
        self.group = self.get_parameter("planning_group").value
        self.check_collisions = bool(self.get_parameter("check_collisions").value)
        self.vel_scale = float(self.get_parameter("velocity_scaling").value)

        urdf = self.get_parameter("robot_description").value
        if not urdf:
            urdf = self._wait_for_robot_description()
        self.kin = DexHandKinematics(urdf)

        self._lock = threading.Lock()
        self.q = np.zeros(8)            # 현재(=드라이버가 보고한) 관절값
        self.q_cmd = np.zeros(8)        # 마지막으로 검증에 통과한 명령값
        self.targets = {f: self.kin.fk_finger(f, [0.0, 0.0]) for f in FINGERS}
        self.residual = {f: 0.0 for f in FINGERS}
        self._pending = False
        # 마커 자동 추종. 프리셋/슬라이더 등 다른 경로로 손이 움직이면 마커가 허공에 남는다.
        # 끌고 있지 않고 마지막 조작 뒤 hold 초가 지났는데 어떤 손가락의 관절값이 이 노드가
        # 마지막으로 명령한 값(q_cmd)에서 벗어나 있으면 그 손가락 마커만 실제 손끝으로 옮긴다.
        # 이 노드가 보낸 목표는 q_cmd 와 같으므로 도달 불가 잔차(빨간 선)는 그대로 남는다.
        # 문턱 0.02rad 는 MoveIt 관절 목표 허용오차(±0.01)보다 크게 잡은 값이다.
        self._dragging = set()
        self._last_interact = -1e9
        self._follow_hold_sec = 1.5
        self._follow_eps = 0.02

        cb = ReentrantCallbackGroup()
        self.create_subscription(JointState, "/joint_states", self._on_js, 10,
                                 callback_group=cb)
        self.pub_cmd = self.create_publisher(
            JointState, "/dexhand_driver/joint_command", 10)
        self.pub_viz = self.create_publisher(MarkerArray, "~/ik_status", 1)

        self.cli_valid = self.create_client(
            GetStateValidity, "/check_state_validity", callback_group=cb)
        self.move_group = ActionClient(self, MoveGroup, "/move_action",
                                       callback_group=cb)

        self.server = InteractiveMarkerServer(self, "dexhand_fingertip_targets")
        self.menu = MenuHandler()
        self._build_menu()
        for f in FINGERS:
            self._make_marker(f)
        self.server.applyChanges()

        self.create_timer(1.0 / float(self.get_parameter("jog_rate_hz").value),
                          self._publish_status, callback_group=cb)

        self.get_logger().info(
            f"손끝 IK 준비 완료. 모드={self.mode}. RViz 에 InteractiveMarkers 디스플레이를 "
            f"추가하고 토픽을 /dexhand_fingertip_targets/update 로 잡아라.")

    # ---------------- robot_description ----------------

    def _wait_for_robot_description(self) -> str:
        """파라미터로 안 받았으면 /robot_description 토픽(transient local)에서 받는다."""
        from std_msgs.msg import String
        got = {}
        ev = threading.Event()

        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)

        def cb(msg):
            got["urdf"] = msg.data
            ev.set()

        sub = self.create_subscription(String, "/robot_description", cb, qos)
        self.get_logger().info("/robot_description 대기 중...")
        t0 = self.get_clock().now()
        while not ev.is_set():
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.get_clock().now() - t0).nanoseconds > 30e9:
                raise RuntimeError("/robot_description 을 30초 안에 못 받았다. "
                                   "robot_state_publisher 가 떠 있는지 확인해라")
        self.destroy_subscription(sub)
        return got["urdf"]

    # ---------------- 마커 ----------------

    def _build_menu(self):
        self.mi_plan = self.menu.insert("모드: 계획 후 실행 (MoveIt)",
                                        callback=lambda fb: self._set_mode(MODE_PLAN))
        self.mi_jog = self.menu.insert("모드: 즉시 조그 (충돌검사만)",
                                       callback=lambda fb: self._set_mode(MODE_JOG))
        self.menu.insert("마커를 현재 손끝 위치로 되돌리기", callback=self._reset_markers)
        self.menu.insert("이 손가락 펴기 (0,0)", callback=self._zero_finger)
        self._refresh_menu_check()

    def _refresh_menu_check(self):
        self.menu.setCheckState(self.mi_plan, MenuHandler.CHECKED
                                if self.mode == MODE_PLAN else MenuHandler.UNCHECKED)
        self.menu.setCheckState(self.mi_jog, MenuHandler.CHECKED
                                if self.mode == MODE_JOG else MenuHandler.UNCHECKED)

    def _make_marker(self, finger: str):
        p = self.targets[finger]
        im = InteractiveMarker()
        im.header.frame_id = self.base_frame
        im.name = f"target_{finger}"
        im.description = f"{finger} 손끝 목표"
        im.scale = 0.03
        im.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        im.pose.orientation.w = 1.0

        sph = Marker()
        sph.type = Marker.SPHERE
        sph.scale.x = sph.scale.y = sph.scale.z = 0.010
        r, g, b = FINGER_COLOR[finger]
        sph.color = ColorRGBA(r=r, g=g, b=b, a=0.9)

        vis = InteractiveMarkerControl()
        vis.always_visible = True
        vis.interaction_mode = InteractiveMarkerControl.MOVE_3D
        vis.markers.append(sph)
        im.controls.append(vis)

        # 축별 이동 핸들. MOVE_3D 만 있으면 깊이 방향을 정확히 잡기 어렵다.
        for axis, quat in (("x", (1.0, 0.0, 0.0)), ("y", (0.0, 0.0, 1.0)),
                           ("z", (0.0, 1.0, 0.0))):
            c = InteractiveMarkerControl()
            c.orientation.w = 0.7071068
            c.orientation.x, c.orientation.y, c.orientation.z = \
                [v * 0.7071068 for v in quat]
            c.name = f"move_{axis}"
            c.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            im.controls.append(c)

        self.server.insert(im, feedback_callback=self._on_feedback)
        self.menu.apply(self.server, im.name)

    # ---------------- 콜백 ----------------

    def _on_js(self, msg: JointState):
        with self._lock:
            for n, v in zip(msg.name, msg.position):
                if n in JOINT_NAMES:
                    self.q[JOINT_NAMES.index(n)] = float(v)

    def _set_mode(self, mode):
        self.mode = mode
        self._refresh_menu_check()
        self.menu.reApply(self.server)
        self.server.applyChanges()
        self.get_logger().info(f"모드 -> {mode}")

    def _reset_markers(self, _fb=None):
        self._last_interact = self._now_sec()
        with self._lock:
            q = self.q.copy()
            self.q_cmd = q.copy()
        for f in FINGERS:
            p = self.kin.fk_finger(f, q[2 * FINGERS.index(f):2 * FINGERS.index(f) + 2])
            self.targets[f] = p
            pose = Pose()
            pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            pose.orientation.w = 1.0
            self.server.setPose(f"target_{f}", pose)
        self.server.applyChanges()

    def _zero_finger(self, fb):
        f = fb.marker_name.replace("target_", "")
        if f not in FINGERS:
            return
        self.targets[f] = self.kin.fk_finger(f, [0.0, 0.0])
        pose = Pose()
        p = self.targets[f]
        pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        pose.orientation.w = 1.0
        self.server.setPose(fb.marker_name, pose)
        self.server.applyChanges()
        self._solve_and_dispatch(force_plan=(self.mode == MODE_PLAN))

    def _on_feedback(self, fb: InteractiveMarkerFeedback):
        if not fb.marker_name.startswith("target_"):
            return
        f = fb.marker_name.replace("target_", "")
        if f not in FINGERS:
            return
        self._last_interact = self._now_sec()
        if fb.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self._dragging.add(f)
        elif fb.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._dragging.discard(f)
        self.targets[f] = np.array([fb.pose.position.x, fb.pose.position.y,
                                    fb.pose.position.z])

        if fb.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            if self.mode == MODE_JOG:
                self._solve_and_dispatch(force_plan=False)
            else:
                self._solve_only()      # 미리보기만 (빨간 선/잔차 갱신)
        elif fb.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._solve_and_dispatch(force_plan=(self.mode == MODE_PLAN))

    # ---------------- IK ----------------

    def _solve_only(self):
        with self._lock:
            q0 = self.q.copy()
        q, residuals = self.kin.ik_all(self.targets, q_init=q0)
        with self._lock:
            self.q_preview = q
            self.residual = residuals
        return q

    def _solve_and_dispatch(self, force_plan: bool):
        q = self._solve_only()

        if self.check_collisions and not self._state_valid(q):
            self.get_logger().warning(
                "이 목표는 자기충돌이 난다. 마지막 안전 자세를 유지한다.")
            return

        with self._lock:
            self.q_cmd = q.copy()

        if force_plan:
            self._send_to_moveit(q)
        else:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(JOINT_NAMES)
            msg.position = [float(v) for v in q]
            self.pub_cmd.publish(msg)

    def _state_valid(self, q) -> bool:
        if not self.cli_valid.service_is_ready():
            # move_group 없이 단독 실행하는 경우. 검사 없이 진행하되 한 번 경고한다.
            if not getattr(self, "_warned_valid", False):
                self._warned_valid = True
                self.get_logger().warning(
                    "/check_state_validity 가 없다. 자기충돌 검사 없이 동작한다. "
                    "move_group 을 같이 띄우면 검사가 켜진다.")
            return True
        req = GetStateValidity.Request()
        req.group_name = self.group
        rs = RobotState()
        js = JointState()
        js.name = list(JOINT_NAMES)
        js.position = [float(v) for v in q]
        rs.joint_state = js
        rs.is_diff = False
        req.robot_state = rs
        fut = self.cli_valid.call_async(req)
        # 주의: 여기서 spin_until_future_complete 를 쓰면 안 된다.
        # 이 함수는 이미 executor 콜백 안에서 돌고 있어서, 같은 노드를 다시 spin 하면
        # MultiThreadedExecutor 와 충돌해 교착한다.
        # ReentrantCallbackGroup 이라 응답은 다른 스레드가 처리해 주므로 그냥 기다린다.
        deadline = time.monotonic() + 0.2
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not fut.done():
            self.get_logger().warning("자기충돌 검사 응답이 200ms 안에 안 왔다. 통과 처리한다")
            return True
        try:
            return bool(fut.result().valid)
        except Exception:
            return True

    def _send_to_moveit(self, q):
        if not self.move_group.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("/move_action 이 없다. move_group 이 떠 있는지 확인해라")
            return
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.vel_scale

        c = Constraints()
        for name, val in zip(JOINT_NAMES, q):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        goal.request = req
        opt = PlanningOptions()
        opt.plan_only = False
        opt.planning_scene_diff.is_diff = True
        opt.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options = opt

        self.move_group.send_goal_async(goal)
        self.get_logger().info("MoveIt 으로 계획 + 실행 요청을 보냈다")

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _follow_fingertips(self, q):
        """다른 경로로 움직인 손가락의 마커를 실제 손끝으로 옮긴다 (jog_rate 로 호출)."""
        if self._dragging or self._now_sec() - self._last_interact < self._follow_hold_sec:
            return
        moved = False
        with self._lock:
            for i, f in enumerate(FINGERS):
                qi = q[2 * i:2 * i + 2]
                if float(np.max(np.abs(qi - self.q_cmd[2 * i:2 * i + 2]))) < self._follow_eps:
                    continue
                self.q_cmd[2 * i:2 * i + 2] = qi
                p = self.kin.fk_finger(f, qi)
                self.targets[f] = p
                self.residual[f] = 0.0
                pose = Pose()
                pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                pose.orientation.w = 1.0
                self.server.setPose(f"target_{f}", pose)
                moved = True
        if moved:
            self.server.applyChanges()

    # ---------------- 시각화 ----------------

    def _publish_status(self):
        with self._lock:
            q = self.q.copy()
        self._follow_fingertips(q)
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, f in enumerate(FINGERS):
            tip = self.kin.fk_finger(f, q[2 * i:2 * i + 2])
            tgt = self.targets[f]
            d = float(np.linalg.norm(np.asarray(tgt) - tip))

            line = Marker()
            line.header.frame_id = self.base_frame
            line.header.stamp = now
            line.ns = "ik_error"
            line.id = i
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.0015
            far = d > 0.002
            line.color = ColorRGBA(r=1.0 if far else 0.2, g=0.15 if far else 0.9,
                                   b=0.15, a=0.9)
            line.points = [Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2])),
                           Point(x=float(tgt[0]), y=float(tgt[1]), z=float(tgt[2]))]
            line.pose.orientation.w = 1.0
            arr.markers.append(line)

            txt = Marker()
            txt.header.frame_id = self.base_frame
            txt.header.stamp = now
            txt.ns = "ik_error_text"
            txt.id = i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.scale.z = 0.008
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            txt.pose.position = Point(x=float(tgt[0]), y=float(tgt[1]),
                                      z=float(tgt[2]) + 0.012)
            txt.pose.orientation.w = 1.0
            txt.text = f"{f} {d*1000:.1f}mm"
            arr.markers.append(txt)
        self.pub_viz.publish(arr)


def main(argv=None):
    rclpy.init(args=argv)
    node = FingertipIK()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
