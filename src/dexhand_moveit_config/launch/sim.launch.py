#!/usr/bin/env python3
"""
sim.launch.py — 시뮬레이션 전용. 시리얼 포트를 아예 열지 않는다.

실물을 붙이기 전에 GUI 조작 전체를 여기서 익히고 검증한다.
`dexhand_moveit.launch.py` 를 다음 값으로 고정해 부르는 얇은 껍데기다.

    link:=none          아두이노 링크를 만들지 않는다 (NullLink)
    auto_enable:=true   시뮬에서는 출력 ON/OFF 개념이 없으므로 바로 켠다.
                        이걸 안 켜면 궤적이 전부 거부되어 "왜 안 움직이지" 가 된다.
    sliders:=true       8관절 슬라이더 패널을 같이 띄운다

실물이 붙은 상태에서는 이 런치를 쓰지 마라. auto_enable 때문에 기동과 동시에
서보에 힘이 들어간다. 실물은 `dexhand_moveit.launch.py link:=serial` 로 띄우고
enable 서비스를 사람이 직접 호출하는 게 맞다.

  ros2 launch dexhand_moveit_config sim.launch.py
  ros2 launch dexhand_moveit_config sim.launch.py ik_mode:=jog   # 마커 즉시 반응
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("dexhand_moveit_config")
    inner = os.path.join(pkg, "launch", "dexhand_moveit.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("ik_mode", default_value="plan",
                              description="손끝 마커 모드: plan | jog"),
        DeclareLaunchArgument("sliders", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(inner),
            launch_arguments={
                "link": "none",
                "auto_enable": "true",
                "sliders": LaunchConfiguration("sliders"),
                "rviz": LaunchConfiguration("rviz"),
                "ik_mode": LaunchConfiguration("ik_mode"),
            }.items(),
        ),
    ])
