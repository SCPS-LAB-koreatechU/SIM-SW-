#!/usr/bin/env python3
"""
grip_preset_node.py — 프리셋 그립 포즈를 GUI 버튼과 토픽으로 실행한다.

MoveIt 의 MotionPlanning 패널에서도 SRDF named state 를 그대로 고를 수 있지만,
그건 "목표 상태 고르기 -> Plan -> Execute" 3단계다. 시연할 때는 한 번에 가는 편이 낫다.

제공하는 인터페이스
  1) RViz 인터랙티브 마커: 손바닥 위에 뜨는 회색 구를 우클릭하면 프리셋 목록이 나온다
  2) 토픽 /dexhand/grip (std_msgs/String): 이름을 쏘면 실행된다
       ros2 topic pub --once /dexhand/grip std_msgs/msg/String '{data: fist}'
  3) 서비스 /dexhand/list_grips (std_srvs/Trigger): 사용 가능한 이름 목록

실행 경로는 항상 MoveIt 이다 (계획 -> 충돌검사 -> FollowJointTrajectory).
프리셋 자체는 check_presets.py 로 자기충돌과 서보 포화를 미리 검증해 두었지만,
"현재 자세에서 그 자세로 가는 도중" 은 계획을 돌려 봐야 안다.
"""

from __future__ import annotations

import os

import rclpy
import yaml
from geometry_msgs.msg import Point
from interactive_markers import InteractiveMarkerServer, MenuHandler
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    Marker)

from dexhand_bringup.kinematics import FINGERS

JOINT_ORDER = [f"R_{f}_{k}" for f in FINGERS for k in ("Yaw", "Pitch")]


class GripPresets(Node):

    def __init__(self, **kwargs):
        super().__init__("dexhand_grip_presets", **kwargs)

        self.declare_parameter("presets_file", "")
        self.declare_parameter("planning_group", "hand")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("velocity_scaling", 0.3)
        self.declare_parameter("marker_position", [0.0, 0.0, 0.06])

        path = self.get_parameter("presets_file").value
        if not path or not os.path.exists(path):
            raise RuntimeError(
                f"presets_file 파라미터가 비었거나 파일이 없다: {path!r}\n"
                f"dexhand_moveit_config/config/grip_presets.yaml 경로를 넘겨라")
        self.presets = yaml.safe_load(open(path, encoding="utf-8"))["presets"]
        self.group = self.get_parameter("planning_group").value
        self.base_frame = self.get_parameter("base_frame").value
        self.vel = float(self.get_parameter("velocity_scaling").value)

        cb = ReentrantCallbackGroup()
        self.move_group = ActionClient(self, MoveGroup, "/move_action",
                                       callback_group=cb)
        self.create_subscription(String, "/dexhand/grip", self._on_topic, 10,
                                 callback_group=cb)
        self.create_service(Trigger, "/dexhand/list_grips", self._on_list,
                            callback_group=cb)

        self.server = InteractiveMarkerServer(self, "dexhand_grip_menu")
        self.menu = MenuHandler()
        for name, p in self.presets.items():
            desc = p.get("description", "")
            label = f"{name}  —  {desc}" if desc else name
            self.menu.insert(label, callback=self._make_cb(name))
        self._make_marker()
        self.server.applyChanges()

        self.get_logger().info(
            f"프리셋 {len(self.presets)}개 로드: {', '.join(self.presets)}")

    def _make_cb(self, name):
        def _cb(_fb):
            self.execute(name)
        return _cb

    def _make_marker(self):
        pos = self.get_parameter("marker_position").value
        im = InteractiveMarker()
        im.header.frame_id = self.base_frame
        im.name = "grip_menu"
        im.description = "그립 프리셋 (우클릭)"
        im.scale = 0.04
        im.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        im.pose.orientation.w = 1.0

        m = Marker()
        m.type = Marker.CUBE
        m.scale.x = m.scale.y = m.scale.z = 0.014
        m.color = ColorRGBA(r=0.75, g=0.75, b=0.78, a=0.85)

        c = InteractiveMarkerControl()
        c.always_visible = True
        c.interaction_mode = InteractiveMarkerControl.MENU
        c.markers.append(m)
        im.controls.append(c)

        self.server.insert(im)
        self.menu.apply(self.server, im.name)

    def _on_topic(self, msg: String):
        self.execute(msg.data.strip())

    def _on_list(self, _req, res):
        res.success = True
        res.message = ", ".join(self.presets)
        return res

    def execute(self, name: str):
        if name not in self.presets:
            self.get_logger().error(
                f"모르는 프리셋 {name!r}. 사용 가능: {', '.join(self.presets)}")
            return
        p = self.presets[name]
        values = {}
        for i, f in enumerate(FINGERS):
            values[f"R_{f}_Yaw"] = float(p["yaw"][i])
            values[f"R_{f}_Pitch"] = float(p["pitch"][i])

        if not self.move_group.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("/move_action 이 없다. move_group 을 먼저 띄워라")
            return

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.group
        req.num_planning_attempts = 5
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = self.vel
        req.max_acceleration_scaling_factor = self.vel

        c = Constraints()
        c.name = name
        for jn in JOINT_ORDER:
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = values[jn]
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
        self.get_logger().info(f"그립 실행: {name}  ({p.get('description','')})")


def main(argv=None):
    rclpy.init(args=argv)
    node = GripPresets()
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
