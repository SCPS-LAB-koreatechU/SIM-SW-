# DexHand v2 8서보 MoveIt GUI 제어 가이드

작성 2026-08-18. 대상은 4손가락 8서보 상태의 DexHand (엄지, 굽힘 텐던 미장착).

---

## 1. 무엇을 만들었나

`iotdesignshop/dexhandv2_description` 의 URDF 를 받아 **실제로 서보가 달린 8자유도만
움직이는 모델**로 정리하고, 그 위에 ROS 2 Humble + MoveIt 2 GUI 를 얹었다.
RViz 에서 손끝 목표점을 마우스로 끌면 해당 손가락의 서보 2개(Outward, Inward)가
동시에 계산되어 실물로 나간다.

```
 [RViz2]
  ├ 손끝 목표 마커 4개 ──▶ fingertip_ik_node ──┐   2-DOF 감쇠최소자승 IK
  ├ 그립 프리셋 메뉴   ──▶ grip_preset_node ──┤
  └ MotionPlanning 패널 ─────────────────────┤
                                             ▼
                                        [move_group]  경로계획 + 자기충돌 검사
                                             │ FollowJointTrajectory
                                             ▼
                                     hand_driver_node   rad ─▶ 정규화 flex/yaw
                                             │ 시리얼 또는 TCP, 50Hz
                                             ▼
                            [Arduino Mega + PCA9685]  us 환산, 램프, 클램핑, 워치독
                                             ▼
                                        서보 8개
```

### 핵심 아이디어: 왜 "모터 2개 동시 제어"가 IK 문제인가

손가락 하나는 Outward/Inward 서보 2개가 길항으로 물려 있고,
- 두 서보를 **같은 방향**으로 밀면 → MCP 굽힘 (`pitch`)
- **반대 방향**으로 밀면 → 좌우 벌림 (`yaw`)

즉 서보 2개의 값은 독립적으로 의미가 없고, 항상 (pitch, yaw) 쌍으로만 뜻이 있다.
손끝 위치는 그 두 값의 함수다. 그래서 "목표 지점 도달"은
**3차원 목표점 → (pitch, yaw) 2개 → 서보 2개 동시 명령** 이라는 IK 문제가 된다.

```
us_outward = neutral + flexSign_out * flex * flexSpan + yawSign_out * yaw * yawSpan
us_inward  = neutral + flexSign_in  * flex * flexSpan + yawSign_in  * yaw * yawSpan
```
(우리 하드웨어는 flexSign 이 Outward = −1, Inward = +1. 원본 DexHand 명명 규칙과 반대다.
 3차 세션에서 실물로 확정한 값이다.)

---

## 2. 표준 IK 플러그인을 안 쓴 이유

MoveIt 의 기본 IK(KDL, LMA)는 이 손가락 체인을 **풀지 못한다**.
자유도가 2개인데 손끝 위치 구속은 3개라서 과결정계다. `position_only_ik: true` 를 켜도
그건 자세 3구속을 빼 줄 뿐, 위치 3구속은 그대로 남는다.

그래서 `dexhand_bringup/kinematics.py` 에 3x2 자코비안 감쇠최소자승 솔버를 직접 넣었다.
목표점이 도달 가능 곡면 위에 있으면 정확히 찾고, 벗어나 있으면 **가장 가까운 도달 가능점 +
잔차(mm)** 를 준다. RViz 에 목표점과 실제 손끝을 잇는 빨간 선과 숫자로 같이 표시한다.

> 나중에 굽힘 텐던 서보를 달아 손가락당 자유도가 3개 이상이 되면
> `config/kinematics.yaml` 의 주석을 풀고 표준 IK 플러그인으로 갈아탈 수 있다.

---

## 3. 설치

### 3.1 devbox (Ubuntu 22.04) — ROS 2 Humble + MoveIt 2

```bash
sudo apt update
sudo apt install -y ros-humble-desktop \
    ros-humble-moveit ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-visualization ros-humble-moveit-planners-ompl \
    ros-humble-moveit-simple-controller-manager ros-humble-moveit-configs-utils \
    ros-humble-joint-state-publisher-gui ros-humble-interactive-markers \
    python3-colcon-common-extensions python3-serial python3-rosdep
```

### 3.2 워크스페이스

**한 번에 하려면** — 압축을 `~/dexhand_ws/src` 에 푼 뒤:

```bash
cd ~/dexhand_ws
bash src/dexhand_bringup/tools/setup_ubuntu2204.sh
```

환경 확인, 의존성 설치, `dexhandv2_description` 클론, colcon 빌드,
검증 3종까지 하고 다음에 칠 명령을 안내한다. 서보는 건드리지 않는다.

**직접 하려면**:

```bash
mkdir -p ~/dexhand_ws/src && cd ~/dexhand_ws/src

# 원본 description (메시와 원본 URDF 가 여기 있다. 반드시 필요)
git clone https://github.com/iotdesignshop/dexhandv2_description.git

# 이번에 만든 두 패키지 압축을 여기에 푼다
#   dexhand_moveit_config/
#   dexhand_bringup/

cd ~/dexhand_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`dexhandv2_description` 은 ament_python 패키지라 `meshes/` 가 share 로 설치된다.
`package://dexhandv2_description/meshes/right/...` 경로가 그대로 먹는다.

---

## 4. 실행

### 4.1 1단계 — URDF 만 확인 (실물, MoveIt 없음)

```bash
ros2 launch dexhand_moveit_config display.launch.py
```
슬라이더 8개로 손이 움직이면 URDF 와 메시 경로가 정상이다.
여기서 안 되면 그 아래 단계는 볼 필요 없다.

### 4.2 2단계 — 시뮬레이션 전체 (실물 없음, 여기서 다 익힌다)

```bash
ros2 launch dexhand_moveit_config sim.launch.py
```

시리얼 포트를 **아예 열지 않는다** (`link:=none`). GUI 조작을 마음껏 해도 안전하다.
`dexhand_moveit.launch.py` 를 다음으로 고정해 부르는 껍데기다.

| 인자 | 값 | 이유 |
|---|---|---|
| `link` | `none` | 아두이노 링크를 만들지 않는다 |
| `auto_enable` | `true` | 시뮬엔 출력 ON/OFF 개념이 없다. 안 켜면 궤적이 전부 거부되어 "왜 안 움직이지"가 된다 |
| `sliders` | `true` | 8관절 슬라이더 패널을 같이 띄운다 |

```bash
ros2 launch dexhand_moveit_config sim.launch.py ik_mode:=jog   # 마커가 즉시 반응
ros2 launch dexhand_moveit_config sim.launch.py sliders:=false # 슬라이더 없이
```

> 실물이 붙은 상태에서는 `sim.launch.py` 를 쓰지 마라. `auto_enable` 때문에
> 기동과 동시에 서보에 힘이 들어간다. 실물은 4.3 을 따른다.

#### 관절 슬라이더 패널

시뮬 단계에서 가장 많이 쓰게 될 창이다. 손가락별 Yaw/Pitch 슬라이더 8개와,
각 값 옆에 **그게 실제 서보에 나가면 몇 us 인지**가 같이 뜬다.
캘리브레이션 min/max 를 넘으면 빨간 `CLAMP` 가 붙는다. 실물에 물리기 전에
"이 자세는 서보가 못 낸다"를 미리 잡아내는 게 이 표시의 목적이다.

- 상단 드롭다운에서 프리셋 12개를 바로 불러올 수 있다
- 하단에 네 손끝의 base_link 기준 좌표(mm)가 실시간으로 뜬다
- 발행 대상은 `/joint_states` 가 아니라 `/dexhand_driver/joint_command` 다.
  그래서 이 패널은 **시뮬이든 실물이든 똑같이** 동작한다. 나중에 부호 검증(7.2)에도 그대로 쓴다

### 4.3 3단계 — 실물 연결

**A. 아두이노를 devbox 에 직접 꽂은 경우 (권장)**

```bash
sudo usermod -aG dialout $USER   # 최초 1회, 재로그인 필요
ros2 launch dexhand_moveit_config dexhand_moveit.launch.py \
     link:=serial serial_port:=/dev/ttyACM0
```

**B. 아두이노가 Windows PC (COM15) 에 있는 경우**

Windows 쪽에서 `tools/run_bridge.bat` 더블클릭 (또는):
```
python serial_bridge_win.py --port COM15 --tcp-port 5555
```
devbox 쪽에서:
```bash
ros2 launch dexhand_moveit_config dexhand_moveit.launch.py \
     link:=tcp tcp_host:=<Windows PC IP> tcp_port:=5555
```

브리지는 클라이언트가 끊기면 자동으로 `z`(전 손가락 중립) + `off`(출력 차단)를 보낸다.
ROS 쪽이 죽었는데 손이 계속 힘을 주고 있는 상황을 막기 위한 것이다.

### 4.4 서보 출력 켜기

**기동 직후에는 출력이 항상 꺼져 있다.** 이건 의도된 동작이다.

```bash
ros2 service call /dexhand_driver/enable std_srvs/srv/SetBool "{data: true}"
```

끄기는 `{data: false}`. 궤적 실행은 출력이 꺼져 있으면 거부된다.

---

## 5. GUI 사용법

### 5.1 손끝 목표 마커 (요청하신 "목표 지점 도달")

RViz 에 손가락 색깔별 구 4개가 뜬다. 끌면 그 손가락의 서보 2개가 같이 움직인다.

- **드래그** — 구를 잡고 끌거나, 축 화살표로 한 방향씩
- **우클릭 메뉴**
  - `모드: 계획 후 실행 (MoveIt)` — 마우스를 놓는 순간 MoveIt 이 충돌회피 경로를
    계획해 실행한다. 기본값이고 안전하다.
  - `모드: 즉시 조그` — 끄는 동안 실시간으로 나간다. 반응이 즉각적이지만
    경로계획을 안 거친다. 대신 매 프레임 `/check_state_validity` 로 자기충돌만 본다.
  - `마커를 현재 손끝 위치로 되돌리기`
  - `이 손가락 펴기 (0,0)`
- **빨간 선과 숫자** — 목표점과 실제 손끝의 거리(mm).
  2mm 넘게 벌어지면 빨간색이 된다. 자유도가 2개라 **정상적으로도 자주 벌어진다.**
  손이 고장난 게 아니라 그 점이 도달 불가능한 것이다.

### 5.2 그립 프리셋

손바닥 위 회색 큐브를 우클릭하면 12개 프리셋이 나온다.
또는 토픽으로:
```bash
ros2 topic pub --once /dexhand/grip std_msgs/msg/String "{data: fist}"
ros2 service call /dexhand/list_grips std_srvs/srv/Trigger
```

| 이름 | 설명 |
|---|---|
| open | 완전 폄 (홈) |
| close / fist / hook | 굽힘 3단계 |
| spread / together | 벌림, 모음 |
| count_1 ~ count_4 | 손가락 세기 |
| cylinder_grip | 원통 파지 |
| wave_ready | 웨이브 시작 자세 |

12개 전부 자기충돌과 서보 포화를 사전 검증했다 (`scripts/check_presets.py`).
프리셋을 고칠 때는 `config/grip_presets.yaml` 만 고치고 그 스크립트를 반드시 다시 돌려라.

### 5.3 MotionPlanning 패널

MoveIt 표준 패널이다. Planning 탭의 Goal State 드롭다운에 SRDF named state 12개가
그대로 뜬다. Plan → Execute 로 궤적을 미리 보고 실행할 수 있다.

---

## 6. 안전 절차

순서를 지켜라. 이 손은 텐던 장력으로 서 있어서 잘못 명령하면 텐던이 늘어나거나
서보가 스톨로 탄다.

1. 서보 전원(V+, 5V 15A SMPS)을 **켜기 전에** 런치부터 띄운다
2. `link:=none` 으로 GUI 동작을 먼저 확인
3. 서보 전원 인가 → 아두이노 시리얼에 `BOOT: outputs disabled` 확인
4. `enable` 서비스로 출력 ON
5. `open` 프리셋으로 홈 복귀가 되는지부터 확인
6. 그 다음에 손끝 마커를 만진다

**비상 정지**
- 서비스: `ros2 service call /dexhand_driver/enable std_srvs/srv/SetBool "{data: false}"`
- 시리얼 모니터에 `off` 또는 빈 줄(엔터)
- 최종 수단은 항상 서보 전원(V+) 물리 차단

**워치독** — PC 가 0.4초 패킷을 못 보내면 손이 그 자리에 멈추고, 2초면 출력이 꺼진다.
드라이버는 목표가 안 바뀌어도 50Hz 로 계속 보내기 때문에 정상 상태에서는 걸리지 않는다.

---

## 7. 반드시 해야 할 실측 두 가지

지금 값은 **잠정값**이다. 이걸 안 맞추면 GUI 각도와 실제 손 모양이 계속 어긋난다.

### 7.1 가동범위 (pitch_max_rad)

URDF 의 pitch 상한을 0.95 rad(54도)로 잠가 두었다. 원본 CAD 한계는 1.309(75도)지만
현재 서보 스팬(450us)으로 실제 몇 도가 나오는지는 아직 측정하지 않았다.

절차:
1. 시리얼 모니터에서 `fm` → `on` → `fa 0` (전부 폄) 상태에서 검지 MCP 각도를 각도기로 측정
2. `fa 100` (최대 굽힘) 상태에서 다시 측정
3. 두 값의 차이가 실제 가동범위다. 이걸 rad 로 바꿔서 세 곳을 **같이** 고친다
   - `scripts/make_8servo_urdf.py --pitch-limit <값>` 으로 URDF 재생성
   - `dexhand_bringup/config/servo_map.yaml` 의 `pitch_max_rad`
   - (스팬을 바꿨다면) 펌웨어 `flexSpanUs` 와 servo_map 의 `flex_span_us`
4. `tools/verify_offline.py` 를 돌려 세 값이 일치하는지 확인 (자동으로 검사한다)

yaw 도 동일한 방식으로 `yaw_max_rad` 를 잡는다.

### 7.2 부호 (flexSign, yawSign)

3차 세션 기록 기준 미해결 사항 두 가지가 그대로 남아 있다.
- 중지, 약지, 소지의 `flexSign` 실물 미검증
- 전 채널 `yawSign` 미검증

이제는 GUI 로 훨씬 쉽게 판별할 수 있다.

```bash
# 굽힘 부호: RViz 화면에서 손가락이 굽는데 실물이 펴지면 그 손가락 부호가 반대다
ros2 topic pub --once /dexhand/grip std_msgs/msg/String "{data: close}"

# 벌림 부호: RViz 는 부채꼴로 벌어지는데 실물이 모이면 yawSign 이 반대다
ros2 topic pub --once /dexhand/grip std_msgs/msg/String "{data: spread}"
```

정지 자세를 맨눈으로 판별하기 어려웠던 게 3차 세션의 문제였는데,
**RViz 화면과 실물을 나란히 놓고 비교**하면 판정이 훨씬 쉽다.

고치는 곳은 둘 중 하나다.
- 펌웨어 쪽 (권장, EEPROM 에 남는다): 시리얼에서 `ff <손가락>` / `yf <손가락>` → `save`
- PC 쪽 임시 반전: `servo_map.yaml` 의 `flex_dir` / `yaw_dir` 를 `-1.0` 으로

**yaw 부호에 대한 중요한 사실**: URDF FK 로 확인한 결과 `+yaw` 는 네 손끝을 **전부 같은
방향(−y, 소지쪽)** 으로 민다. 그래서 "벌리기"는 검지 −, 소지 + 로 줘야 한다.
부호를 반대로 주면 손가락끼리 부딪힌다. 프리셋은 이미 이 사실에 맞춰져 있다.

---

## 8. 펌웨어 (rev.4) 변경점

`DexHand_Hand8.ino` 를 다시 올려야 한다. 안 올리면 `servo_map.yaml` 의
`use_high_res_stream: false` 로 바꿔서 구버전 `v` 명령으로 동작시킬 수는 있다.

1. **`fspan`/`yspan` 상한 200us → 700us**
   3차 세션 기록의 최우선 미해결 항목이었다. 실측 여유가 630us 인데 상한이 200us 로
   하드코딩되어 있어서 `fspan 450` 을 보내도 조용히 200 으로 잘렸다.
   파지에 필요한 350~500us 를 쓸 수 없던 원인이다.
2. **고해상도 스트림 명령 `w` 추가** (−1000..1000)
   기존 `v` 의 1% 해상도로는 700us 스팬에서 7us 계단이 생겨 손끝이 눈에 띄게 끊긴다.
   내부 명령값을 per-mille 로 바꿨고, `f`/`y`/`fa`/`ya`/`v` 는 −100..100 그대로 받는다.
   **하위호환된다.**
3. **`room` 명령** — 채널별 여유와 현재 span 의 clamp 여부를 한눈에
4. `HARD_MIN_US/MAX_US` 800/2200, 기본 캘리브레이션을 3차 세션 실측값으로,
   EEPROM 매직 `0xDE8A0802`
5. `ping` 명령 (링크 생존 확인)
6. `t5` 스프레드 포즈의 yaw 부호를 FK 결과에 맞게 수정

avr-g++ (atmega2560) 크로스 컴파일 `-Wall -Wextra` 경고 0으로 검증했다.

> 주의: 프로젝트 문서의 `DexHand_Hand8.ino` 는 1차 세션 버전이라
> 3차 세션의 변경(800/2200, 매직 0802, `room`)이 반영되어 있지 않다.
> rev.4 는 진행상황 문서에 적힌 3차 세션 변경 내용을 재구성해 얹은 것이므로,
> 업로드 전에 EEPROM 캘리브레이션이 그대로 읽히는지(`dump`) 먼저 확인해라.

---

## 9. 재생성 스크립트

모델을 손으로 고치지 말고 스크립트로 다시 뽑아라. 셋은 이 순서로 돈다.

```bash
cd src/dexhand_moveit_config

# 1) 8서보 URDF (가동범위를 실측한 뒤 --pitch-limit 를 올린다)
python3 scripts/make_8servo_urdf.py \
    --input ../dexhandv2_description/urdf/dexhandv2_right.urdf \
    --output config/dexhandv2_right_8servo.urdf \
    --pitch-limit 0.95 --yaw-limit 0.30

# 2) 자기충돌 행렬 (Setup Assistant 와 같은 알고리즘)
python3 scripts/make_collision_matrix.py \
    --urdf config/dexhandv2_right_8servo.urdf \
    --mesh-root ../dexhandv2_description \
    --samples 6000 --out /tmp/dc.xml

# 3) SRDF (프리셋 + 충돌행렬)
python3 scripts/make_srdf.py --collisions /tmp/dc.xml \
    --presets config/grip_presets.yaml \
    --out config/dexhandv2_right_8servo.srdf

# 4) 검증
python3 scripts/check_presets.py \
    --urdf config/dexhandv2_right_8servo.urdf \
    --srdf config/dexhandv2_right_8servo.srdf \
    --presets config/grip_presets.yaml \
    --servo-map ../dexhand_bringup/config/servo_map.yaml \
    --mesh-root ../dexhandv2_description
```

`trimesh`, `python-fcl`, `rtree` 가 필요하다 (`pip install trimesh python-fcl rtree`).
런타임에는 필요 없고 재생성할 때만 쓴다.

---

## 10. 검증 결과

### 10.1 ROS 없이 (모델과 산출물)

| 항목 | 결과 |
|---|---|
| URDF 파싱 (`check_urdf`) | 통과, 트리 정상 |
| 구동 조인트 | 8개 (Yaw/Pitch x 4), 나머지 13개 fixed |
| 손끝 IK 왕복 오차 | 평균 0.00006mm, **최대 0.002mm** (손가락당 300회 x 4) |
| 도달 불가 목표 | 잔차 정확히 보고, 관절 한계 안에서 최근접해 반환 |
| 자기충돌 행렬 | 231쌍 중 194쌍 제외 → **검사 대상 37쌍** (6000 샘플) |
| 프리셋 12개 | 관절한계, 자기충돌, 서보 us 포화 **전부 통과** |
| span 여유 | 450+150 = 600us ≤ 최소 room 630us (clamp 없음) |
| URDF 한계 ↔ servo_map | 일치 확인 (자동 검사) |
| 펌웨어 | avr-g++ atmega2560, `-Wall -Wextra` 경고 0 |
| 파이썬 | pyflakes 클린, pytest 7개 통과 |

### 10.2 ROS 런타임에서 (노드 실제 기동)

`tools/smoke_test_ros.py` 로 노드 3개를 실제로 띄워 확인했다. 40항목 전부 통과.

| 대상 | 확인한 것 |
|---|---|
| 드라이버 | `/joint_states` 발행, 출력 OFF 시 궤적 **거부**, enable 서비스, 궤적 실행과 목표 도달, 스트림 패킷 형식과 값(`w 526 0 ...`), 조그 추종, 한계 초과 시 명령 포화 |
| 손끝 IK | URDF 파싱, 마커 4개 등록, 드래그 → IK → `joint_command`, IK 해가 정답 관절값과 일치, 도달 불가 시 잔차 59.9mm 보고, `check_state_validity` 부재 시 graceful |
| 프리셋 | 12개 로드, `list_grips` 서비스, move_group 부재 시에도 노드 생존, 모르는 이름 처리 |
| 슬라이더 | 패널 생성, 프리셋 적용, us 미리보기가 오프라인 계산과 일치, span 초과 시 CLAMP 감지, 발행값 일치 |
| **통합** | 마커 드래그 → IK → 드라이버 → `/joint_states` 이동 → **서보 2개 동시 명령**(`flex=368 yaw=333`) |

검증을 직접 다시 돌리려면:
```bash
python3 src/dexhand_bringup/tools/smoke_test_ros.py     # ROS 필요, 시리얼 안 씀
python3 src/dexhand_bringup/tools/verify_offline.py \
    --urdf src/dexhand_moveit_config/config/dexhandv2_right_8servo.urdf \
    --servo-map src/dexhand_bringup/config/servo_map.yaml \
    --presets src/dexhand_moveit_config/config/grip_presets.yaml
python3 -m pytest src/dexhand_bringup/test -q
```

## 11. 정직하게 밝혀 두는 한계

- **노드 검증은 Humble 이 아니라 Jazzy 런타임에서 했다.** 작업 컨테이너가 Ubuntu 24.04라
  Humble 을 깔 수 없었다. rclpy 와 메시지 API 는 두 배포판이 사실상 같아서 노드 코드는
  그대로 돌 것으로 본다. 다만 **MoveIt 설정 파일의 키 이름은 배포판마다 다르다.**
  `config/ompl_planning.yaml` 은 Humble 형식(`planning_plugin` 단수형)으로 써 두었고,
  런치와 RViz 설정도 Humble 기준이다. 여기가 어긋나면 가장 먼저 의심할 곳이다.
- **`move_group` 과 RViz 는 아직 실제로 띄워 보지 못했다.** 스모크 테스트는 노드 3개만
  검증한다. 4.1 → 4.2 순서를 밟으면 어디가 깨졌는지 바로 좁힐 수 있다.
- **위치 피드백이 없다.** `/joint_states` 는 측정값이 아니라 드라이버가 명령한 값이다.
  파지 중 서보가 밀려도 ROS 는 모른다.
- **`pitch_max_rad`, `yaw_max_rad` 는 잠정값**이다 (7.1 참조).
- **중지, 약지, 소지 flexSign 과 전 채널 yawSign 은 여전히 미검증**이다 (7.2 참조).
- **엄지와 PIP/DIP 는 없다.** 그래서 핀치 계열 프리셋을 일부러 넣지 않았다.
  물리적으로 불가능한 포즈를 GUI 에 넣으면 GUI 가 거짓말을 하게 된다.
- `Thumb_Dist_KC_1` ↔ `base_link` 는 기본자세에서 이미 메시가 겹친다.
  엄지가 fixed 라 상수라서 충돌 제외 목록에 넣었다. 원본 CAD 의 간섭으로 보인다.

---

## 12. 트러블슈팅

| 증상 | 원인과 조치 |
|---|---|
| RViz 에 손이 안 보인다 | `display.launch.py` 부터 확인. 메시 경로 문제면 `dexhandv2_description` 이 빌드/설치 안 된 것 |
| `No planning group` | SRDF 가 안 읽혔다. `colcon build` 후 `source install/setup.bash` 다시 |
| 궤적이 거부된다 | 서보 출력이 꺼져 있다. `enable` 서비스 호출 |
| 손이 GUI 보다 덜 움직인다 | 시리얼에 `room` 을 쳐 봐라. `<< CLAMP` 가 뜨면 span 이 여유를 넘은 것 |
| 손이 반대로 움직인다 | 7.2 부호 검증 |
| 0.4초마다 멈춘다 | 브리지 지연이나 시리얼 병목. `stream_rate_hz` 를 30 으로 낮춰 본다 |
| `!! STREAM LOST -> OUTPUT OFF` | 링크가 2초 끊겼다. 브리지 로그(`bridge_log.txt`) 확인 |
| 마커를 끌어도 안 움직인다 | `plan` 모드에서는 마우스를 **놓는** 순간 실행된다. 즉시 반응은 `jog` 모드 |
| 자기충돌로 계속 거부된다 | 목표가 물리적으로 손가락끼리 부딪히는 위치다. 이웃 손가락 수렴 여유가 밀리미터 단위로 좁다 |

---

## 13. 다음 단계 제안

1. `setup_ubuntu2204.sh` 로 빌드와 검증 → 4.1 → 4.2 (시뮬) 순서로 확인
2. 7.1 가동범위 실측 → URDF 재생성 → span 을 450 이상으로 올려 파지력 확보
3. 7.2 부호 확정 → EEPROM `save`
4. 그다음에야 ZED 텔레옵과 붙일 수 있다.
   `zed_hand_teleop.py` 가 뽑는 flex/yaw 를 `/dexhand_driver/joint_command` 로
   퍼블리시하도록 바꾸면 이 드라이버와 그대로 물린다.
   그러면 텔레옵도 MoveIt 의 자기충돌 검사를 거치게 된다.
5. 굽힘 텐던 서보(CH8~CH11) 추가 시
   - 펌웨어 `NUM_FINGERS`/`SERVOS_PER_FINGER` 확장
   - `make_8servo_urdf.py --coupling` 으로 Flexor/DIP 를 mimic 이나 독립 관절로
   - 손가락당 자유도 3개 이상이 되면 `kinematics.yaml` 의 표준 IK 플러그인 활성화
