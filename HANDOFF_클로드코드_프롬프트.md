# 우분투 Claude Code CLI 에 붙여넣을 지시문

우분투 머신에서 아래 순서로 진행하면 됩니다.

---

## 0. 압축 풀기 (Claude Code 를 켜기 전에)

```bash
mkdir -p ~/dexhand_ws/src
cd ~/dexhand_ws
unzip ~/Downloads/dexhand_moveit_ws.zip        # 받은 zip 경로에 맞게
ls src            # dexhand_moveit_config, dexhand_bringup 두 개가 보여야 한다
```

zip 안에 `src/` 가 통째로 들어 있으므로 `~/dexhand_ws` 에서 풀면 경로가 맞습니다.
`CLAUDE.md` 도 같이 풀리는데, Claude Code 가 자동으로 읽어 이 프로젝트의 규칙을 파악합니다.

---

## 1. Claude Code 실행

```bash
cd ~/dexhand_ws
claude
```

---

## 2. 아래를 그대로 붙여넣으세요

```
이 워크스페이스는 DexHand v2 로봇손(손가락 4개, 서보 8개)을 ROS 2 Humble + MoveIt 2 로
제어하는 프로젝트야. 루트의 CLAUDE.md 와 DexHand_MoveIt_가이드.md 를 먼저 읽어줘.

지금 목표는 실물 없이 시뮬레이션만 띄워서 GUI 제어를 확인하는 거야.
서보는 절대 건드리지 마. 시리얼 포트를 여는 명령은 실행하지 마.

이 순서로 진행해줘:

1. bash src/dexhand_bringup/tools/setup_ubuntu2204.sh 를 실행해.
   MoveIt 의존성 설치, dexhandv2_description 클론, colcon 빌드,
   검증 3종(pytest / verify_offline / smoke_test_ros)까지 한 번에 한다.
   sudo 비밀번호가 필요하면 나한테 알려줘.

2. 실패한 게 있으면 로그를 그대로 보여주고 원인을 짚어줘.
   특히 이런 것들을 확인해:
   - moveit_configs_utils 의 MoveItConfigsBuilder 가 config 파일들을 제대로 읽는지
   - config/ompl_planning.yaml 이 Humble 형식(planning_plugin 단수형)이 맞는지
   - SRDF 의 group_state 12개가 로드되는지
   고쳐야 하면 고치고, 왜 고쳤는지 설명해줘.

3. 전부 통과하면 다음을 순서대로 띄워서 화면을 확인해줘.
   각 단계마다 내가 화면을 보고 답할 테니 기다려줘.

   (a) ros2 launch dexhand_moveit_config display.launch.py
       → 손 모델이 보이고 슬라이더로 8관절이 움직이는지

   (b) ros2 launch dexhand_moveit_config sim.launch.py
       → RViz + MoveIt + 손끝 마커 4개 + 프리셋 큐브 + 슬라이더 패널

4. sim 이 뜨면 이걸 확인해줘:
   - 손끝 마커(색깔 구 4개)를 끌면 손가락이 따라오는지
   - 프리셋 실행이 되는지:
     ros2 topic pub --once /dexhand/grip std_msgs/msg/String "{data: fist}"
   - 슬라이더 패널에서 값을 바꾸면 RViz 의 손이 같이 움직이는지
   - 슬라이더 패널에 뜨는 서보 us 값이 800~2200 안에 있고 CLAUDE 빨간 표시가 없는지

5. RViz 창이 안 뜨거나 디스플레이가 비어 있으면
   rviz/dexhand_moveit.rviz 설정을 고쳐줘. 특히 InteractiveMarkers 두 개의
   네임스페이스가 /dexhand_fingertip_targets 와 /dexhand_grip_menu 로 잡혀야 한다.

작업 중에 config/dexhandv2_right_8servo.urdf 와 .srdf 는 직접 고치지 마.
그건 scripts/ 의 생성 스크립트 산출물이야. 고쳐야 하면 스크립트를 고치고 다시 뽑아.
```

---

## 3. 잘 안 될 때 추가로 던질 지시

**RViz 가 아예 안 뜰 때 (원격 접속 등)**
```
RViz 가 디스플레이를 못 잡는 것 같아. echo $DISPLAY 와 ros2 launch 로그를 확인하고,
X 포워딩이나 로컬 세션 문제인지 판단해줘. 헤드리스면 rviz:=false 로 띄우고
ros2 topic echo /joint_states 로 노드 동작만 먼저 확인하자.
```

**빌드가 깨질 때**
```
colcon build 에러 전문을 보여주고, 어느 패키지 어느 파일인지 짚어줘.
dexhand_bringup 은 순수 파이썬이라 보통 setup.py 나 의존성 문제고,
dexhand_moveit_config 는 ament_cmake 라 CMakeLists 나 package.xml 문제다.
```

**MoveIt 이 계획을 못 세울 때**
```
move_group 로그에서 planning group 'hand' 를 찾았는지, SRDF 가 로드됐는지 확인해줘.
그리고 ros2 service call /check_state_validity 로 현재 자세가 유효한지 봐줘.
자기충돌이면 어느 링크쌍인지 알려줘.
```

**실물로 넘어갈 준비가 됐을 때** (시뮬이 전부 확인된 뒤에만)
```
이제 실물을 붙일 거야. 가이드 6장 안전 절차와 7장 실측 절차를 읽고,
아두이노가 /dev/ttyACM* 로 보이는지부터 확인해줘.
서보 출력을 켜는 건 내가 직접 할 테니 너는 켜지 마.
```

---

## 4. 참고: 이 세션에서 이미 검증한 것

우분투에서 처음 돌리는 게 아니라, 아래는 ROS 런타임 위에서 이미 통과시켜 놨습니다.
따라서 setup 스크립트가 깨진다면 대부분 **환경 쪽(패키지 설치, 경로, 디스플레이)** 입니다.

- 드라이버 노드: joint_states 발행, 출력 OFF 시 궤적 거부, enable 서비스,
  궤적 실행과 목표 도달, 스트림 패킷 형식과 값, 조그 추종, 명령 포화 — 14항목
- 손끝 IK 노드: URDF 파싱, 마커 4개, 드래그 → IK → joint_command,
  도달 불가 잔차 보고, check_state_validity 부재 시 graceful — 8항목
- 프리셋 노드: 12개 로드, list_grips, move_group 부재 시 graceful — 5항목
- 통합: 마커 → IK → 드라이버 → joint_states → 서보 2개 동시 명령 — 3항목
- 슬라이더 패널: 생성, 프리셋 적용, us 미리보기 일치, CLAUDE 감지, 발행 — 10항목

단, 검증은 ROS 2 **Jazzy** 런타임에서 했습니다 (작업 컨테이너가 Ubuntu 24.04라
Humble 을 못 깝니다). rclpy 와 메시지 API 는 Humble 과 사실상 같지만,
**MoveIt 설정 파일의 키 이름은 배포판마다 다릅니다.** `config/ompl_planning.yaml` 은
Humble 형식(`planning_plugin` 단수형)으로 써 뒀는데, 여기가 어긋나면 가장 먼저 의심할 곳입니다.
