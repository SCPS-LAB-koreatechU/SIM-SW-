# dexhand_ros2_ws

DexHand v2 (4손가락 8서보) ROS 2 Humble + MoveIt 2 GUI 제어 패키지.

```
src/
├── dexhand_moveit_config/          MoveIt 설정 (ament_cmake)
│   ├── config/
│   │   ├── dexhandv2_right_8servo.urdf    8자유도로 정리한 URDF (생성물)
│   │   ├── dexhandv2_right_8servo.srdf    그룹, 프리셋, 충돌행렬 (생성물)
│   │   ├── grip_presets.yaml              프리셋 그립 단일 진실 소스
│   │   ├── kinematics.yaml                IK 플러그인을 왜 안 붙였는지 설명 포함
│   │   ├── joint_limits.yaml
│   │   ├── moveit_controllers.yaml
│   │   └── ompl_planning.yaml
│   ├── launch/
│   │   ├── dexhand_moveit.launch.py       전체 실행
│   │   └── display.launch.py              URDF 만 확인
│   ├── rviz/
│   └── scripts/                           재생성 + 검증 스크립트
│       ├── make_8servo_urdf.py
│       ├── make_collision_matrix.py
│       ├── make_srdf.py
│       └── check_presets.py
└── dexhand_bringup/                ROS 노드 (ament_python)
    ├── dexhand_bringup/
    │   ├── kinematics.py                  FK + 2-DOF 감쇠최소자승 IK (ROS 비의존)
    │   ├── hand_driver_node.py            궤적 실행 + 시리얼 스트리밍 + joint_states
    │   ├── fingertip_ik_node.py           손끝 마커 IK
    │   ├── grip_preset_node.py            프리셋 그립
    │   └── serial_link.py                 serial / tcp / none 링크
    ├── config/servo_map.yaml              rad <-> 서보 명령 매핑
    ├── test/test_kinematics.py
    └── tools/
        ├── verify_offline.py              ROS 없이 도는 전체 검증
        ├── serial_bridge_win.py           Windows COM <-> TCP 브리지
        └── run_bridge.bat

firmware/DexHand_Hand8/DexHand_Hand8.ino   rev.4 (span 700us, 고해상도 스트림 w)
DexHand_MoveIt_가이드.md                    설치, 실행, 캘리브레이션, 트러블슈팅
```

빠른 시작은 `DexHand_MoveIt_가이드.md` 4장을 보라.

```bash
# 실물 없이 GUI 만
ros2 launch dexhand_moveit_config dexhand_moveit.launch.py

# 실물 (리눅스 직결)
ros2 launch dexhand_moveit_config dexhand_moveit.launch.py link:=serial serial_port:=/dev/ttyACM0

# 실물 (Windows COM15 브리지 경유)
ros2 launch dexhand_moveit_config dexhand_moveit.launch.py link:=tcp tcp_host:=<IP> tcp_port:=5555

# 서보 출력 켜기 (기동 직후는 항상 꺼져 있다)
ros2 service call /dexhand_driver/enable std_srvs/srv/SetBool "{data: true}"
```

원본 URDF: https://github.com/iotdesignshop/dexhandv2_description (별도 clone 필요)
