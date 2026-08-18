#!/usr/bin/env python3
"""
serial_link.py — 아두이노와의 링크 3종.

  SerialLink : 리눅스에 아두이노가 직접 꽂힌 경우 (/dev/ttyACM0)
  TcpLink    : 아두이노가 Windows PC 에 있고 tools/serial_bridge_win.py 가 중계하는 경우
  NullLink   : 실물 없이 RViz 만 볼 때 (드라이런)

셋 다 write_line / read_lines / close 만 갖는 같은 인터페이스다.
읽기는 절대 블로킹하지 않는다. 제어 루프가 시리얼 때문에 멈추면
펌웨어 워치독(0.4초)이 물어서 손이 멈춘다.
"""

from __future__ import annotations

import socket
import threading
import time


class LinkBase:
    def write_line(self, text: str) -> None:
        raise NotImplementedError

    def read_lines(self) -> list:
        """지금까지 도착한 완성된 줄들을 반환하고 버퍼를 비운다. 블로킹하지 않는다."""
        raise NotImplementedError

    def close(self) -> None:
        pass

    @property
    def connected(self) -> bool:
        return True


class NullLink(LinkBase):
    """드라이런. 보낸 줄을 기억만 해 둔다."""

    def __init__(self):
        self.sent = []

    def write_line(self, text: str) -> None:
        self.sent.append(text)
        if len(self.sent) > 500:
            del self.sent[:250]

    def read_lines(self) -> list:
        return []


class _BufferedReader:
    """바이트 스트림을 줄 단위로 쪼개 모아 두는 공통 부분."""

    def __init__(self):
        self._buf = b""
        self._lines = []
        self._lock = threading.Lock()

    def feed(self, data: bytes) -> None:
        if not data:
            return
        with self._lock:
            self._buf += data
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                self._lines.append(line.decode("utf-8", "replace").strip())
            # 개행 없이 무한히 쌓이는 걸 막는다
            if len(self._buf) > 4096:
                self._buf = self._buf[-1024:]

    def take(self) -> list:
        with self._lock:
            out, self._lines = self._lines, []
        return out


class SerialLink(LinkBase):
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.0):
        import serial  # pyserial. import 를 늦춰서 드라이런에서는 없어도 되게 한다.

        self._ser = serial.Serial(port, baud, timeout=timeout)
        self._reader = _BufferedReader()
        self._alive = True
        self._t = threading.Thread(target=self._pump, daemon=True)
        self._t.start()
        # 아두이노는 포트 열릴 때 리셋된다. 부팅 배너가 끝날 때까지 기다린다.
        time.sleep(2.0)
        self._reader.take()

    def _pump(self):
        while self._alive:
            try:
                n = self._ser.in_waiting
                self._reader.feed(self._ser.read(n) if n else b"")
            except Exception:
                pass
            time.sleep(0.005)

    def write_line(self, text: str) -> None:
        self._ser.write((text + "\n").encode("ascii", "ignore"))

    def read_lines(self) -> list:
        return self._reader.take()

    def close(self) -> None:
        self._alive = False
        try:
            self._ser.close()
        except Exception:
            pass

    @property
    def connected(self) -> bool:
        return self._ser.is_open


class TcpLink(LinkBase):
    """tools/serial_bridge_win.py 와 붙는다. 끊기면 알아서 재접속한다."""

    def __init__(self, host: str, port: int):
        self._host, self._port = host, port
        self._sock = None
        self._reader = _BufferedReader()
        self._alive = True
        self._lock = threading.Lock()
        self._t = threading.Thread(target=self._pump, daemon=True)
        self._t.start()

    def _connect(self):
        s = socket.create_connection((self._host, self._port), timeout=3.0)
        s.settimeout(0.05)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return s

    def _pump(self):
        while self._alive:
            if self._sock is None:
                try:
                    self._sock = self._connect()
                except Exception:
                    time.sleep(1.0)
                    continue
            try:
                data = self._sock.recv(4096)
                if data == b"":
                    raise ConnectionError("bridge closed")
                self._reader.feed(data)
            except socket.timeout:
                pass
            except Exception:
                with self._lock:
                    try:
                        self._sock.close()
                    except Exception:
                        pass
                    self._sock = None
                time.sleep(0.5)

    def write_line(self, text: str) -> None:
        with self._lock:
            if self._sock is None:
                return
            try:
                self._sock.sendall((text + "\n").encode("ascii", "ignore"))
            except Exception:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

    def read_lines(self) -> list:
        return self._reader.take()

    def close(self) -> None:
        self._alive = False
        with self._lock:
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None


def make_link(kind: str, **kw) -> LinkBase:
    kind = (kind or "none").lower()
    if kind == "serial":
        return SerialLink(kw["serial_port"], kw.get("baud", 115200))
    if kind == "tcp":
        return TcpLink(kw["tcp_host"], int(kw["tcp_port"]))
    return NullLink()
