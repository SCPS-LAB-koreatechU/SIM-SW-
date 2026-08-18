#!/usr/bin/env python3
"""
hand_driver_node.py — DexHand 8서보 ROS 2 드라이버

역할
  1) FollowJointTrajectory 액션 서버      -> MoveIt 이 계획한 궤적을 실행
  2) /dexhand/joint_command (JointState)  -> 인터랙티브 마커 드래그 같은 실시간 조그
  3) /joint_states 퍼블리시               -> RViz, MoveIt 이 현재 손 자세를 안다
  4) 아두이노 스트림 링크                 -> 정규화된 flex/yaw 패킷을 50Hz 로 송신
  5) /dexhand/enable (SetBool)            -> 서보 출력 ON/OFF (기동 시 기본 OFF)

설계 메모
  - 서보에 위치 피드백이 없다. 그래서 /joint_states 는 "드라이버가 명령한 값"이다.
    램프 속도 제한을 드라이버에서도 걸어 두었기 때문에 실제 손과 큰 차이는 없지만,
    이건 측정값이 아니라 추정값이다. 파지 중 서보가 밀리면 알 방법이 없다.
  - us 환산, 클램핑, EEPROM 캘리브레이션의 진실은 아두이노에 있다.
    PC 는 -1000..1000 정규화 값만 보낸다. 두 곳에서 같은 계산을 하면
    어느 쪽이 잘랐는지 알 수 없게 되기 때문이다.
  - 목표가 안 변해도 계속 보낸다. 펌웨어 워치독이 0.4초 무패킷이면 정지시킨다.
"""

from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from dexhand_bringup.kinematics import JOINT_NAMES
from dexhand_bringup.serial_link import make_link


def _dur_to_sec(d: DurationMsg) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


class HandDriver(Node):

    def __init__(self, **kwargs):
        # kwargs 는 rclpy Node 로 그대로 넘긴다.
        # 테스트에서 parameter_overrides 를 주입할 때 쓴다 (tools/smoke_test_ros.py).
        super().__init__("dexhand_driver", **kwargs)

        p = self.declare_parameters("", [
            ("link", "none"),
            ("serial_port", "/dev/ttyACM0"),
            ("baud", 115200),
            ("tcp_host", "127.0.0.1"),
            ("tcp_port", 5555),
            ("stream_rate_hz", 50.0),
            ("joint_state_rate_hz", 50.0),
            ("use_high_res_stream", True),
            ("pitch_max_rad", 0.95),
            ("yaw_max_rad", 0.30),
            ("flex_span_us", 450),
            ("yaw_span_us", 150),
            ("flex_dir", [1.0, 1.0, 1.0, 1.0]),
            ("yaw_dir", [1.0, 1.0, 1.0, 1.0]),
            ("max_joint_speed", 2.0),
            ("auto_enable_output", False),
        ])
        g = {x.name: x.value for x in p}
        self.cfg = g

        self.pitch_max = float(g["pitch_max_rad"])
        self.yaw_max = float(g["yaw_max_rad"])
        self.flex_dir = np.array(g["flex_dir"], dtype=float)
        self.yaw_dir = np.array(g["yaw_dir"], dtype=float)
        self.max_speed = float(g["max_joint_speed"])
        self.hi_res = bool(g["use_high_res_stream"])
        self.scale = 1000.0 if self.hi_res else 100.0
        self.verb = "w" if self.hi_res else "v"

        # 상태. q 는 JOINT_NAMES 순서 (Index_Yaw, Index_Pitch, Middle_Yaw, ...)
        self._lock = threading.Lock()
        self.q = np.zeros(8)          # 드라이버가 실제로 내보내는 값 (램프 적용 후)
        self.q_goal = np.zeros(8)     # 조그 목표
        self.output_on = False
        self.traj_active = False
        self.last_fw_line = ""

        # ---- 링크 ----
        self.link = make_link(g["link"], serial_port=g["serial_port"], baud=g["baud"],
                              tcp_host=g["tcp_host"], tcp_port=g["tcp_port"])
        self.get_logger().info(f"link = {g['link']}")

        # ---- ROS 인터페이스 ----
        cb = ReentrantCallbackGroup()
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_fw = self.create_publisher(String, "~/firmware", 10)
        self.create_subscription(JointState, "~/joint_command", self._on_jog, 10,
                                 callback_group=cb)
        self.create_service(SetBool, "~/enable", self._on_enable, callback_group=cb)

        self._action = ActionServer(
            self, FollowJointTrajectory,
            "hand_controller/follow_joint_trajectory",
            execute_callback=self._execute_traj,
            goal_callback=self._accept_goal,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        dt = 1.0 / float(g["stream_rate_hz"])
        self._dt = dt
        self.create_timer(dt, self._tick, callback_group=cb)
        self.create_timer(1.0 / float(g["joint_state_rate_hz"]), self._publish_js,
                          callback_group=cb)
        self.create_timer(0.1, self._drain_firmware, callback_group=cb)

        self._init_firmware()
        if bool(g["auto_enable_output"]):
            self.get_logger().warning("auto_enable_output=true — 기동과 동시에 서보 출력을 켠다")
            self._set_output(True)
        else:
            self.get_logger().info(
                "서보 출력은 꺼진 상태로 시작한다. 켜려면:\n"
                "  ros2 service call /dexhand_driver/enable std_srvs/srv/SetBool '{data: true}'")

    # ---------------- 펌웨어 ----------------

    def _init_firmware(self):
        """부팅 직후 안전 상태로 맞춘 뒤 스트림 모드로 들어간다."""
        for cmd in ("", "off", "sm", "dump"):
            self.link.write_line(cmd)
            time.sleep(0.05)

    def _set_output(self, on: bool):
        with self._lock:
            self.output_on = bool(on)
        if on:
            # 켜기 전에 현재 자세를 중립으로 되돌려 놓는다. 서보가 갑자기 튀는 걸 막는다.
            with self._lock:
                self.q[:] = 0.0
                self.q_goal[:] = 0.0
            self._send_stream(np.zeros(8))
            self.link.write_line("on")
            self.get_logger().info("서보 출력 ON")
        else:
            self.link.write_line("off")
            self.get_logger().info("서보 출력 OFF")

    def _on_enable(self, req, res):
        self._set_output(req.data)
        res.success = True
        res.message = "output on" if req.data else "output off"
        return res

    def _drain_firmware(self):
        for line in self.link.read_lines():
            if not line:
                continue
            self.last_fw_line = line
            self.pub_fw.publish(String(data=line))
            low = line.lower()
            if "error" in low or "!!" in low or "lost" in low or "timeout" in low:
                self.get_logger().warning(f"[firmware] {line}")
                if "output off" in low or "lost" in low:
                    with self._lock:
                        self.output_on = False

    # ---------------- 스트리밍 ----------------

    def _to_cmd(self, q: np.ndarray):
        """8관절 rad -> 펌웨어 손가락 순서의 (flex, yaw) 정수쌍 4개."""
        out = []
        for i in range(4):
            yaw = q[2 * i]
            pitch = q[2 * i + 1]
            flex = self.flex_dir[i] * pitch / self.pitch_max
            yawn = self.yaw_dir[i] * yaw / self.yaw_max
            f = int(round(max(-1.0, min(1.0, flex)) * self.scale))
            y = int(round(max(-1.0, min(1.0, yawn)) * self.scale))
            out.append((f, y))
        return out

    def _send_stream(self, q: np.ndarray):
        pairs = self._to_cmd(q)
        args = " ".join(f"{f} {y}" for f, y in pairs)
        self.link.write_line(f"{self.verb} {args}")

    def _tick(self):
        """램프 + 송신. 궤적 실행 중이면 궤적 스레드가 q 를 직접 쓴다."""
        with self._lock:
            if not self.traj_active:
                step = self.max_speed * self._dt
                d = self.q_goal - self.q
                np.clip(d, -step, step, out=d)
                self.q += d
            q = self.q.copy()
            on = self.output_on
        if on:
            self._send_stream(q)

    def _publish_js(self):
        with self._lock:
            q = self.q.copy()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in q]
        self.pub_js.publish(msg)

    # ---------------- 조그 ----------------

    def _on_jog(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            return
        with self._lock:
            for n, v in zip(msg.name, msg.position):
                if n in JOINT_NAMES:
                    self.q_goal[JOINT_NAMES.index(n)] = float(v)

    # ---------------- 궤적 실행 ----------------

    def _accept_goal(self, goal):
        names = list(goal.trajectory.joint_names)
        unknown = [n for n in names if n not in JOINT_NAMES]
        if unknown:
            self.get_logger().error(f"모르는 조인트: {unknown}")
            return GoalResponse.REJECT
        if not goal.trajectory.points:
            return GoalResponse.REJECT
        with self._lock:
            if not self.output_on:
                self.get_logger().error(
                    "서보 출력이 꺼져 있어 궤적을 거부한다. /dexhand_driver/enable 로 먼저 켜라")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_traj(self, gh):
        traj = gh.request.trajectory
        names = list(traj.joint_names)
        cols = [JOINT_NAMES.index(n) for n in names]
        pts = traj.points
        times = [_dur_to_sec(p.time_from_start) for p in pts]

        with self._lock:
            q_start = self.q.copy()
            self.traj_active = True

        t0 = time.monotonic()
        total = times[-1] if times else 0.0
        fb = FollowJointTrajectory.Feedback()
        fb.joint_names = names
        result = FollowJointTrajectory.Result()

        try:
            while True:
                if gh.is_cancel_requested:
                    gh.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "canceled"
                    return result

                t = time.monotonic() - t0
                q_des = q_start.copy()

                if t >= total:
                    last = pts[-1]
                    for k, c in enumerate(cols):
                        q_des[c] = last.positions[k]
                else:
                    j = 0
                    while j + 1 < len(times) and times[j + 1] < t:
                        j += 1
                    t_a, t_b = times[j], times[min(j + 1, len(times) - 1)]
                    p_a, p_b = pts[j], pts[min(j + 1, len(pts) - 1)]
                    a = 0.0 if t_b <= t_a else (t - t_a) / (t_b - t_a)
                    a = max(0.0, min(1.0, a))
                    for k, c in enumerate(cols):
                        q_des[c] = (1 - a) * p_a.positions[k] + a * p_b.positions[k]

                with self._lock:
                    # 궤적이 시간최적화를 거쳤어도 서보가 못 따라오면 소용없다.
                    # 여기서 한 번 더 속도를 자른다.
                    step = self.max_speed * self._dt
                    d = np.clip(q_des - self.q, -step, step)
                    self.q += d
                    self.q_goal[:] = self.q
                    q_now = self.q.copy()
                    on = self.output_on

                if not on:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = "궤적 도중 서보 출력이 꺼졌다"
                    gh.abort()
                    return result

                self._send_stream(q_now)

                fb.actual.positions = [float(q_now[c]) for c in cols]
                fb.desired.positions = [float(q_des[c]) for c in cols]
                fb.error.positions = [float(q_des[c] - q_now[c]) for c in cols]
                gh.publish_feedback(fb)

                if t >= total and float(np.max(np.abs(q_des - q_now))) < 1e-4:
                    break
                time.sleep(self._dt)

            gh.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = ""
            return result
        finally:
            with self._lock:
                self.traj_active = False

    def destroy_node(self):
        try:
            self.link.write_line("z")
            self.link.write_line("off")
            self.link.close()
        except Exception:
            pass
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = HandDriver()
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
