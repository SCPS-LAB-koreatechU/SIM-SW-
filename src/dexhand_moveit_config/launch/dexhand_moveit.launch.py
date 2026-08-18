#!/usr/bin/env python3
"""
dexhand_moveit.launch.py — DexHand v2 8서보 MoveIt GUI 일괄 실행

띄우는 것
  robot_state_publisher   URDF -> TF
  static_transform_pub    world -> base_link (SRDF virtual joint)
  move_group              MoveIt 계획 + 자기충돌 검사 + 궤적 실행
  rviz2                   MotionPlanning 패널 + 인터랙티브 마커
  dexhand_driver          FollowJointTrajectory 실행 + 아두이노 스트리밍 + /joint_states
  fingertip_ik            손끝 목표 마커 -> 2-DOF IK -> 서보 2개 동시 제어
  grip_presets            프리셋 그립 메뉴

실행 예
  # 실물 없이 GUI 만 (드라이런)
  ros2 launch dexhand_moveit_config dexhand_moveit.launch.py

  # 리눅스에 아두이노가 직접 꽂힌 경우
  ros2 launch dexhand_moveit_config dexhand_moveit.launch.py \
       link:=serial serial_port:=/dev/ttyACM0

  # 아두이노가 Windows PC(COM15)에 있고 브리지로 중계하는 경우
  ros2 launch dexhand_moveit_config dexhand_moveit.launch.py \
       link:=tcp tcp_host:=192.168.0.20 tcp_port:=5555
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("dexhand_moveit_config")
    bringup = get_package_share_directory("dexhand_bringup")

    link = LaunchConfiguration("link").perform(context)
    serial_port = LaunchConfiguration("serial_port").perform(context)
    tcp_host = LaunchConfiguration("tcp_host").perform(context)
    tcp_port = LaunchConfiguration("tcp_port").perform(context)
    use_rviz = LaunchConfiguration("rviz")
    ik_mode = LaunchConfiguration("ik_mode").perform(context)
    sliders = LaunchConfiguration("sliders").perform(context).lower() in ("1", "true", "yes")
    auto_enable = LaunchConfiguration("auto_enable").perform(context).lower() in ("1", "true", "yes")

    moveit_config = (
        MoveItConfigsBuilder("dexhandv2_right_8servo", package_name="dexhand_moveit_config")
        .robot_description(file_path="config/dexhandv2_right_8servo.urdf")
        .robot_description_semantic(file_path="config/dexhandv2_right_8servo.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .to_moveit_configs()
    )

    nodes = []

    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    ))

    # SRDF 의 virtual_joint 가 world -> base_link 라서 이 TF 가 없으면
    # RViz 가 계획 프레임을 못 찾는다.
    nodes.append(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="log",
    ))

    nodes.append(Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            # 서보에 위치 피드백이 없다. 실행 후 관절값이 목표와 다르다고
            # 계속 실패 처리하는 걸 막는다.
            {"trajectory_execution.allowed_execution_duration_scaling": 3.0,
             "trajectory_execution.allowed_goal_duration_margin": 2.0,
             "trajectory_execution.execution_duration_monitoring": False,
             "publish_robot_description_semantic": True},
        ],
    ))

    nodes.append(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(pkg, "rviz", "dexhand_moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    ))

    # servo_map.yaml 의 servo_cal(딕셔너리 리스트)과 servo_names 는 검증 전용이라
    # ROS 파라미터 타입에 없다. --params-file 로 그대로 넘기면 rcl 파서가 거부하므로
    # 여기서 읽어 그 두 키만 빼고 dict 로 넘긴다. 슬라이더 패널과 오프라인 툴은
    # 파일을 직접 읽으니 yaml 형식은 그대로 둔다.
    with open(os.path.join(bringup, "config", "servo_map.yaml"), encoding="utf-8") as fp:
        servo_params = yaml.safe_load(fp)["dexhand_driver"]["ros__parameters"]
    for k in ("servo_cal", "servo_names"):
        servo_params.pop(k, None)

    nodes.append(Node(
        package="dexhand_bringup",
        executable="hand_driver",
        name="dexhand_driver",
        output="screen",
        parameters=[
            servo_params,
            {"link": link, "serial_port": serial_port,
             "tcp_host": tcp_host, "tcp_port": int(tcp_port),
             "auto_enable_output": auto_enable},
        ],
    ))

    nodes.append(Node(
        package="dexhand_bringup",
        executable="fingertip_ik",
        name="dexhand_fingertip_ik",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"mode": ik_mode, "planning_group": "hand", "base_frame": "base_link"},
        ],
    ))

    nodes.append(Node(
        package="dexhand_bringup",
        executable="grip_presets",
        name="dexhand_grip_presets",
        output="screen",
        parameters=[{
            "presets_file": os.path.join(pkg, "config", "grip_presets.yaml"),
            "planning_group": "hand",
            "base_frame": "base_link",
        }],
    ))

    if sliders:
        nodes.append(Node(
            package="dexhand_bringup",
            executable="joint_sliders",
            name="dexhand_joint_sliders",
            output="screen",
            parameters=[{
                "urdf_file": os.path.join(pkg, "config", "dexhandv2_right_8servo.urdf"),
                "servo_map_file": os.path.join(bringup, "config", "servo_map.yaml"),
                "presets_file": os.path.join(pkg, "config", "grip_presets.yaml"),
            }],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("link", default_value="none",
                              description="아두이노 링크: none(드라이런) | serial | tcp"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("tcp_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("tcp_port", default_value="5555"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("ik_mode", default_value="plan",
                              description="손끝 마커 모드: plan(계획 후 실행) | jog(즉시 스트리밍)"),
        DeclareLaunchArgument("sliders", default_value="false",
                              description="8관절 슬라이더 패널을 같이 띄울지"),
        DeclareLaunchArgument("auto_enable", default_value="false",
                              description="기동과 동시에 서보 출력을 켤지. "
                                          "실물이 붙어 있으면 반드시 false 로 둬라"),
        OpaqueFunction(function=_setup),
    ])
