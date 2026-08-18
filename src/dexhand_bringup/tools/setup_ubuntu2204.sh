#!/usr/bin/env bash
# setup_ubuntu2204.sh — Ubuntu 22.04 + ROS 2 Humble 에서 워크스페이스를 세우고 검증한다.
#
# 하는 일
#   1) 환경 확인 (Ubuntu 버전, ROS distro)
#   2) MoveIt 과 필요한 패키지 설치 (sudo 필요)
#   3) dexhandv2_description 클론 (없을 때만)
#   4) colcon 빌드
#   5) 검증 3종: 파이썬 단위테스트, 오프라인 검증, ROS 스모크 테스트
#   6) 다음에 칠 명령 안내
#
# 실행:
#   cd ~/dexhand_ws && bash src/dexhand_bringup/tools/setup_ubuntu2204.sh
#
# 이 스크립트는 서보를 절대 건드리지 않는다. 시리얼 포트를 열지 않는다.

set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
RED=$'\e[31m'; GRN=$'\e[32m'; YLW=$'\e[33m'; RST=$'\e[0m'
step() { echo; echo "${GRN}==> $*${RST}"; }
warn() { echo "${YLW}[!] $*${RST}"; }
die()  { echo "${RED}[X] $*${RST}"; exit 1; }

step "0. 환경 확인"
echo "  워크스페이스: $WS"
[ -d "$WS/src" ] || die "$WS/src 가 없다. 압축을 <워크스페이스>/src 아래에 풀었는지 확인해라."

. /etc/os-release
echo "  OS: $PRETTY_NAME"
if [ "${VERSION_ID:-}" != "22.04" ]; then
  warn "Ubuntu 22.04 가 아니다 ($VERSION_ID). Humble 은 22.04 전용이다."
  warn "24.04 라면 Jazzy 를 써야 하고, config/ompl_planning.yaml 의 키 이름을"
  warn "planning_plugins / response_adapters (복수형) 으로 바꿔야 한다."
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  ROS_SETUP=/opt/ros/humble/setup.bash
  DISTRO=humble
elif [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
  DISTRO="$ROS_DISTRO"
  warn "humble 이 아니라 $DISTRO 를 쓴다."
else
  die "ROS 2 를 못 찾았다. /opt/ros/humble 이 있는지 확인해라."
fi
echo "  ROS: $DISTRO"

# ROS setup.bash 는 미정의 변수를 참조하므로 set -u 를 잠시 끈다.
# shellcheck disable=SC1090
set +u; source "$ROS_SETUP"; set -u

step "1. 의존 패키지 설치"
PKGS=(
  "ros-$DISTRO-moveit"
  "ros-$DISTRO-moveit-ros-move-group"
  "ros-$DISTRO-moveit-ros-visualization"
  "ros-$DISTRO-moveit-planners-ompl"
  "ros-$DISTRO-moveit-simple-controller-manager"
  "ros-$DISTRO-moveit-configs-utils"
  "ros-$DISTRO-interactive-markers"
  "ros-$DISTRO-joint-state-publisher-gui"
  "ros-$DISTRO-python-qt-binding"
  "ros-$DISTRO-tf2-ros"
  "python3-colcon-common-extensions"
  "python3-serial"
  "python3-numpy"
  "python3-yaml"
)
MISSING=()
for p in "${PKGS[@]}"; do
  dpkg -s "$p" >/dev/null 2>&1 || MISSING+=("$p")
done
if [ ${#MISSING[@]} -eq 0 ]; then
  echo "  전부 이미 설치되어 있다."
else
  echo "  설치할 것: ${MISSING[*]}"
  sudo apt-get update -qq
  sudo apt-get install -y "${MISSING[@]}" || die "apt 설치 실패"
fi

step "2. dexhandv2_description 확인"
if [ -d "$WS/src/dexhandv2_description" ]; then
  echo "  이미 있다."
else
  echo "  클론한다 (원본 URDF 와 STL 메시가 여기 있다. 필수)"
  git clone --depth 1 https://github.com/iotdesignshop/dexhandv2_description.git \
      "$WS/src/dexhandv2_description" || die "클론 실패 (네트워크 확인)"
fi

step "3. colcon 빌드"
cd "$WS"
colcon build --symlink-install || die "빌드 실패. 위 에러를 그대로 읽어라."
# shellcheck disable=SC1091
set +u; source "$WS/install/setup.bash"; set -u

step "4-1. 파이썬 단위테스트 (ROS 불필요)"
python3 -m pytest "$WS/src/dexhand_bringup/test" -q || warn "단위테스트 실패"

step "4-2. 오프라인 검증 (URDF, IK, 프리셋, 서보 환산)"
python3 "$WS/src/dexhand_bringup/tools/verify_offline.py" \
  --urdf    "$WS/src/dexhand_moveit_config/config/dexhandv2_right_8servo.urdf" \
  --servo-map "$WS/src/dexhand_bringup/config/servo_map.yaml" \
  --presets "$WS/src/dexhand_moveit_config/config/grip_presets.yaml" \
  || warn "오프라인 검증 실패"

step "4-3. ROS 스모크 테스트 (노드 3개 실제 기동, 시리얼 안 씀)"
python3 "$WS/src/dexhand_bringup/tools/smoke_test_ros.py" || warn "스모크 테스트 실패"

step "완료. 다음 순서로 띄워라"
cat <<EOF

  # 매 터미널에서 먼저
  source $ROS_SETUP
  source $WS/install/setup.bash

  # 1) URDF 만 확인 (여기서 안 되면 메시 경로 문제)
  ros2 launch dexhand_moveit_config display.launch.py

  # 2) 시뮬레이션 전체 (RViz + MoveIt + 손끝 마커 + 프리셋 + 슬라이더)
  ros2 launch dexhand_moveit_config sim.launch.py

  # 3) 손끝 마커를 끌면 바로 반응하게 (계획 없이 즉시)
  ros2 launch dexhand_moveit_config sim.launch.py ik_mode:=jog

  실물은 시뮬이 다 확인된 뒤에:
  ros2 launch dexhand_moveit_config dexhand_moveit.launch.py link:=serial serial_port:=/dev/ttyACM0
  ros2 service call /dexhand_driver/enable std_srvs/srv/SetBool "{data: true}"

EOF
