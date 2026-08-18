#!/usr/bin/env python3
"""
display.launch.py — URDF 만 확인하는 최소 실행 (MoveIt, 아두이노 없음)

문제가 생겼을 때 어디가 깨졌는지 좁히는 용도다. 이게 안 뜨면 URDF/메시 경로 문제고,
이건 뜨는데 dexhand_moveit.launch.py 가 안 뜨면 MoveIt 설정 문제다.

  ros2 launch dexhand_moveit_config display.launch.py

joint_state_publisher_gui 슬라이더로 8관절을 직접 움직여 볼 수 있다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("dexhand_moveit_config")
    urdf = os.path.join(pkg, "config", "dexhandv2_right_8servo.urdf")
    with open(urdf, "r", encoding="utf-8") as fp:
        robot_description = fp.read()

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
             output="log"),
        Node(package="rviz2", executable="rviz2", output="log",
             arguments=["-d", os.path.join(pkg, "rviz", "dexhand_display.rviz")]),
    ])
