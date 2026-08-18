# CLAUDE.md — DexHand v2 8서보 워크스페이스

이 파일은 이 워크스페이스에서 작업하는 Claude Code 를 위한 것이다.

## 이게 뭔가

DexHand v2 (TheRobotStudio 계열) 로봇손을 ROS 2 Humble + MoveIt 2 GUI 로 제어한다.
현재 하드웨어는 **손가락 4개, 너클 서보 8개**다. 엄지와 굽힘 텐던(PIP/DIP)은 아직 안 달렸다.

손가락 하나 = 서보 2개(Outward, Inward)가 길항으로 물린 2자유도.
- 두 서보를 같은 방향 → MCP 굽힘 (`pitch`)
- 반대 방향 → 좌우 벌림 (`yaw`)

그래서 "손끝을 목표 지점으로" 는 **3D 목표점 → (pitch, yaw) → 서보 2개 동시 명령** 이라는
2자유도 IK 문제다.

```
RViz ─ 손끝 마커 / 프리셋 / MotionPlanning
        ↓
   move_group (경로계획 + 자기충돌)
        ↓ FollowJointTrajectory
   hand_driver_node  rad → 정규화 flex/yaw, 50Hz
        ↓ 시리얼 or TCP
   Arduino Mega + PCA9685  → us 환산, 램프, 클램핑, 워치독 → 서보 8개
```

## 절대 어기면 안 되는 것

1. **서보 출력을 사람 확인 없이 켜지 마라.**
   `/dexhand_driver/enable` 을 `true` 로 부르거나 `auto_enable:=true` 로 런치하면
   실물에 힘이 들어간다. 실물이 붙어 있을 가능성이 있으면 먼저 사람에게 물어라.
   `sim.launch.py` 는 `auto_enable:=true` 지만 `link:=none` 이라 시리얼을 아예 안 연다.

2. **세 값은 항상 같이 고쳐야 한다.** 하나만 고치면 GUI 각도와 실제 손이 어긋난다.
   - URDF 조인트 한계 (`scripts/make_8servo_urdf.py --pitch-limit / --yaw-limit`)
   - `dexhand_bringup/config/servo_map.yaml` 의 `pitch_max_rad` / `yaw_max_rad`
   - 펌웨어와 servo_map 의 `flex_span_us` / `yaw_span_us`

   `tools/verify_offline.py` 가 이 일치를 자동으로 검사한다. 고친 뒤 반드시 돌려라.

3. **생성물을 손으로 고치지 마라.** 아래 둘은 스크립트 산출물이다.
   - `config/dexhandv2_right_8servo.urdf`
   - `config/dexhandv2_right_8servo.srdf`

   고칠 곳은 `config/grip_presets.yaml` 과 `scripts/*.py` 다. 재생성 절차는 README 참고.

4. **프리셋을 고쳤으면 `scripts/check_presets.py` 를 반드시 돌려라.**
   관절한계, 자기충돌, 서보 us 포화를 한 번에 본다. GUI 버튼 하나가 손가락끼리
   부딪히게 만들면 텐던이 늘어나거나 서보가 스톨로 탄다.

## 알아 둬야 할 하드웨어 사실

- **flexSign 이 원본 명명 규칙과 반대다.** 전 채널 Outward = −1, Inward = +1. 실물로 확정됨.
- **+yaw 는 네 손끝을 전부 같은 방향(−y, 소지쪽)으로 민다.** 벌리기는 검지 −, 소지 + 다.
  반대로 주면 손가락끼리 부딪힌다.
- **손가락 사이 여유가 밀리미터 단위로 좁다.** "모으기" 는 yaw 0.10 rad 이 한계다.
- 서보 펄스폭 한계 800~2200us. 다만 명령은 중립 기준 대칭이라 **실사용 여유는 편도 630~700us**.
  중지 MIN(중립 1570)의 630us 가 전체 병목이다.
- **위치 피드백이 없다.** `/joint_states` 는 측정값이 아니라 드라이버가 명령한 값이다.
- `pitch_max_rad = 0.95` 는 **아직 실측 안 된 잠정값**이다 (가이드 7.1).
- 중지, 약지, 소지의 flexSign 과 전 채널 yawSign 은 **아직 실물 미검증**이다 (가이드 7.2).

## 자주 쓰는 명령

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# 빌드
colcon build --symlink-install

# 검증 3종 (전부 시리얼 안 씀)
python3 -m pytest src/dexhand_bringup/test -q
python3 src/dexhand_bringup/tools/verify_offline.py \
  --urdf src/dexhand_moveit_config/config/dexhandv2_right_8servo.urdf \
  --servo-map src/dexhand_bringup/config/servo_map.yaml \
  --presets src/dexhand_moveit_config/config/grip_presets.yaml
python3 src/dexhand_bringup/tools/smoke_test_ros.py

# 실행 (단계별로 좁혀 가며)
ros2 launch dexhand_moveit_config display.launch.py       # URDF 만
ros2 launch dexhand_moveit_config sim.launch.py           # 시뮬 전체
ros2 launch dexhand_moveit_config sim.launch.py ik_mode:=jog

# 프리셋
ros2 topic pub --once /dexhand/grip std_msgs/msg/String "{data: fist}"
ros2 service call /dexhand/list_grips std_srvs/srv/Trigger
```

## 문제를 좁히는 순서

`display.launch.py` → `sim.launch.py` → 실물. 각 단계가 다른 원인을 가른다.

| 어디서 깨지나 | 원인 후보 |
|---|---|
| display 부터 안 뜸 | URDF 파싱, 메시 경로, dexhandv2_description 미설치 |
| display 는 되는데 sim 이 안 됨 | MoveIt 설정 (SRDF, kinematics, ompl 키 이름), 런치 |
| RViz 는 뜨는데 손이 안 움직임 | 드라이버 미기동, 출력 OFF, 토픽 이름 |
| 시뮬은 되는데 실물이 다름 | 부호(flexSign/yawSign), span, 중립, us 클램프 |

`smoke_test_ros.py` 가 통과하면 노드 자체는 멀쩡하다는 뜻이다.
그러면 남은 건 MoveIt 설정이나 RViz 디스플레이 쪽이다.

## 상세 문서

`DexHand_MoveIt_가이드.md` 에 설치, 실행, 캘리브레이션 절차, 안전 절차,
트러블슈팅 표가 전부 들어 있다. 작업 전에 읽어라.
