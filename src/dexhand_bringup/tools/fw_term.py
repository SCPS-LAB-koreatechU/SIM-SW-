#!/usr/bin/env python3
"""fw_term.py — DexHand 펌웨어에 시리얼 명령을 보내고 응답을 출력한다.

예)
  python3 fw_term.py dump                # 현재 캘리브레이션
  python3 fw_term.py room                # span 여유(clamp) 확인
  python3 fw_term.py "fspan 480" "yspan 150" save room
  python3 fw_term.py --port /dev/ttyACM0 --wait 6 dump   # 리셋 직후엔 부팅 대기 필요

주의: on / t1~t7 등 출력을 켜는 명령은 실물이 움직인다. 사람이 확인하고 보낼 것.
"""
import argparse, sys, time
import serial

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wait", type=float, default=2.5, help="포트 열고 부팅 대기(초). Mega 는 DTR 리셋됨")
    ap.add_argument("--gap", type=float, default=0.6, help="명령 간 응답 대기(초)")
    ap.add_argument("cmds", nargs="+")
    a = ap.parse_args()

    with serial.Serial(a.port, a.baud, timeout=0.2) as s:
        time.sleep(a.wait)
        boot = s.read(4096).decode(errors="replace")
        if boot.strip():
            print(boot.rstrip())
        for c in a.cmds:
            print(f"\n>>> {c}")
            s.write((c + "\n").encode())
            t0 = time.time()
            while time.time() - t0 < a.gap:
                out = s.read(4096).decode(errors="replace")
                if out:
                    sys.stdout.write(out); sys.stdout.flush()
                    t0 = time.time()
        print()

if __name__ == "__main__":
    main()
