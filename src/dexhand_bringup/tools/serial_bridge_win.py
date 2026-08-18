#!/usr/bin/env python3
"""
serial_bridge_win.py — Windows PC 의 COM 포트를 TCP 로 중계한다.

왜 필요한가
  아두이노는 Windows PC(COM15)에 꽂혀 있고, ROS 2 Humble + MoveIt 은 Ubuntu 22.04
  devbox 에서 돈다. 두 기계 사이를 잇는 가장 단순한 방법이 이 브리지다.
  (아두이노 USB 를 devbox 로 옮겨 꽂을 수 있으면 브리지 없이 link:=serial 로 끝난다.
   그쪽이 지연도 적고 고장날 여지도 적다. 브리지는 옮기기 곤란할 때의 대안이다.)

Windows 쪽에서
    pip install pyserial
    python serial_bridge_win.py --port COM15 --tcp-port 5555
  또는 같이 만들어 둔 run_bridge.bat 을 더블클릭.

devbox 쪽에서
    ros2 launch dexhand_moveit_config dexhand_moveit.launch.py \
         link:=tcp tcp_host:=<Windows PC IP> tcp_port:=5555

안전
  - 클라이언트가 끊기면 즉시 "z" (전 손가락 중립) 와 "off" (출력 차단) 를 보낸다.
    ROS 쪽이 죽었는데 손이 마지막 자세로 계속 힘을 주고 있으면 서보가 탄다.
  - 한 번에 클라이언트 하나만 받는다. 두 곳에서 동시에 명령을 쏘면 손이 어떻게
    움직일지 아무도 예측할 수 없다.
  - 아두이노가 뱉는 줄은 그대로 클라이언트로 넘기고 콘솔에도 찍는다.
"""

import argparse
import socket
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("pyserial 이 없다:  pip install pyserial")
    sys.exit(1)


SAFE_SHUTDOWN = ["", "z", "off"]


class Bridge:
    def __init__(self, port, baud, tcp_port, bind):
        self.ser = serial.Serial(port, baud, timeout=0)
        print(f"[bridge] {port} @ {baud} 열림")
        time.sleep(2.0)          # 아두이노 리셋 대기
        self.ser.reset_input_buffer()

        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((bind, tcp_port))
        self.srv.listen(1)
        print(f"[bridge] TCP {bind}:{tcp_port} 대기 중")

        self.client = None
        self.lock = threading.Lock()
        threading.Thread(target=self._serial_to_tcp, daemon=True).start()

    def _serial_to_tcp(self):
        buf = b""
        while True:
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n) if n else b""
            except Exception as e:
                print(f"[bridge] 시리얼 읽기 실패: {e}")
                time.sleep(0.5)
                continue
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", "replace").rstrip()
                    print(f"  < {text}")
                with self.lock:
                    c = self.client
                if c is not None:
                    try:
                        c.sendall(data)
                    except Exception:
                        pass
            time.sleep(0.005)

    def _safe_shutdown(self, why):
        print(f"[bridge] {why} -> 안전 정지 (z, off) 송신")
        for cmd in SAFE_SHUTDOWN:
            try:
                self.ser.write((cmd + "\n").encode())
                time.sleep(0.05)
            except Exception:
                break

    def serve_forever(self):
        while True:
            conn, addr = self.srv.accept()
            print(f"[bridge] 접속: {addr}")
            conn.settimeout(0.1)
            with self.lock:
                self.client = conn
            try:
                while True:
                    try:
                        data = conn.recv(4096)
                    except socket.timeout:
                        continue
                    if data == b"":
                        raise ConnectionError("클라이언트 종료")
                    self.ser.write(data)
            except Exception as e:
                print(f"[bridge] 연결 종료: {e}")
            finally:
                with self.lock:
                    self.client = None
                try:
                    conn.close()
                except Exception:
                    pass
                self._safe_shutdown("클라이언트 끊김")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM15", help="아두이노 COM 포트")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tcp-port", type=int, default=5555)
    ap.add_argument("--bind", default="0.0.0.0",
                    help="0.0.0.0 이면 LAN 전체에 열린다. 같은 PC 안에서만 쓸 거면 127.0.0.1")
    args = ap.parse_args()

    b = Bridge(args.port, args.baud, args.tcp_port, args.bind)
    try:
        b.serve_forever()
    except KeyboardInterrupt:
        b._safe_shutdown("Ctrl-C")
        print("\n[bridge] 종료")


if __name__ == "__main__":
    main()
