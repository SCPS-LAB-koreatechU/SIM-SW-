#!/usr/bin/env bash
# rev.4 펌웨어 업로드 + span 최대 설정. 사용: firmware/flash_rev4.sh [/dev/ttyACM0] [FSPAN] [YSPAN]
# 서보 출력은 켜지 않는다. 업로드 중 서보 전원(V+)은 꺼 두는 것을 권장.
set -e
PORT=${1:-/dev/ttyACM0}; FSPAN=${2:-480}; YSPAN=${3:-150}
CLI=$HOME/bin/arduino-cli
WS=$(cd "$(dirname "$0")/.." && pwd)
TERM_PY="$WS/src/dexhand_bringup/tools/fw_term.py"

echo "== [1/4] 업로드 전 EEPROM dump (구버전 펌웨어)"
python3 "$TERM_PY" --port "$PORT" dump || echo "(dump 실패 — 계속 진행)"

echo "== [2/4] rev.4 컴파일 + 업로드"
$CLI compile --fqbn arduino:avr:mega "$WS/firmware/DexHand_Hand8" | tail -2
$CLI upload  --fqbn arduino:avr:mega -p "$PORT" "$WS/firmware/DexHand_Hand8"

echo "== [3/4] 재부팅 후 dump/ping"
sleep 2
python3 "$TERM_PY" --port "$PORT" --wait 4 ping dump

echo "== [4/4] span 설정 fspan=$FSPAN yspan=$YSPAN → save → room"
python3 "$TERM_PY" --port "$PORT" "fspan $FSPAN" "yspan $YSPAN" save room
echo "완료. room 출력에 CLAMP/WARNING 이 없어야 한다."
