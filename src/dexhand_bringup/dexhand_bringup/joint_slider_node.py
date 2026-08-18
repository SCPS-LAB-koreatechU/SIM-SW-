#!/usr/bin/env python3
"""
joint_slider_node.py — 8관절을 직접 미는 슬라이더 패널.

시뮬레이션 단계에서 가장 많이 쓰게 될 창이다.
  - 손가락 4개 x (Yaw, Pitch) 슬라이더 8개
  - 각 슬라이더 옆에 rad, deg, 그리고 **그 값이 실제 서보에 나가면 몇 us 인지**를 같이 띄운다.
    캘리브레이션 min/max 를 넘으면 빨간 CLAMP 표시가 뜬다. 시뮬에서 미리 보이는 게 핵심이다.
  - 프리셋 드롭다운, 전체 0 버튼, 스트리밍 on/off

publish 대상은 `/joint_states` 가 아니라 `/dexhand_driver/joint_command` 다.
드라이버가 속도 제한을 걸어 따라간 뒤 `/joint_states` 를 내보낸다.
그래서 이 패널은 시뮬이든 실물이든 **똑같이** 동작한다.

발행은 사용자가 슬라이더/프리셋/전부0 을 만졌을 때만 한다. 상시 발행하면 손끝 마커나
MoveIt 프리셋으로 움직인 손이 곧바로 슬라이더 값으로 끌려온다. 만지지 않는 동안은
반대로 `/joint_states` 를 따라가서 슬라이더가 항상 현재 자세를 보여 준다. 시뮬에서 익힌 조작이
실물에서 그대로 먹는다는 뜻이고, 나중에 부호(flexSign/yawSign) 검증에도 그대로 쓴다.

  ros2 run dexhand_bringup joint_sliders --ros-args \
      -p urdf_file:=<...>/dexhandv2_right_8servo.urdf \
      -p servo_map_file:=<...>/servo_map.yaml \
      -p presets_file:=<...>/grip_presets.yaml
"""

from __future__ import annotations

import math
import os
import sys
import threading

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

from dexhand_bringup.kinematics import FINGERS, JOINT_NAMES, DexHandKinematics

# Qt 는 import 시점에 죽이지 않는다. 이 모듈은 테스트에서 import 만 하기도 하고,
# 헤드리스 환경에서 다른 노드와 같이 빌드되기도 한다. 실패는 main() 에서 알린다.
QT_ERROR = None
try:
    from python_qt_binding.QtCore import Qt, QTimer
    from python_qt_binding.QtWidgets import (QApplication, QCheckBox, QComboBox,
                                             QGridLayout, QGroupBox, QHBoxLayout,
                                             QLabel, QPushButton, QSlider,
                                             QVBoxLayout, QWidget)
except ImportError:
    try:
        from PyQt5.QtCore import Qt, QTimer
        from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox,
                                     QGridLayout, QGroupBox, QHBoxLayout,
                                     QLabel, QPushButton, QSlider,
                                     QVBoxLayout, QWidget)
    except ImportError as e:
        QT_ERROR = e
        QWidget = object     # 아래 클래스 정의가 깨지지 않게만 해 둔다

# 슬라이더 정수 눈금 수. 1000 이면 pitch 전 구간(0.95rad)이 0.00095rad 단위가 되고
# 이는 서보 0.45us 에 해당해 펌웨어 분해능(1us) 아래다. 프리셋 값을 정확히 재현하려고
# 10000 으로 잡아 둔다. 눈금이 많다고 느려지지 않는다.
TICKS = 10000


class SliderPanel(QWidget):

    def __init__(self, node: Node, kin: DexHandKinematics, servo_cfg, presets):
        super().__init__()
        self.node = node
        self.kin = kin
        self.cfg = servo_cfg
        self.presets = presets

        self.setWindowTitle("DexHand 8서보 관절 제어")
        root = QVBoxLayout(self)

        # ---- 상단 컨트롤 ----
        top = QHBoxLayout()
        self.chk_stream = QCheckBox("명령 송신")
        self.chk_stream.setChecked(True)
        top.addWidget(self.chk_stream)

        self.combo = QComboBox()
        self.combo.addItem("— 프리셋 선택 —")
        for name, p in presets.items():
            self.combo.addItem(f"{name}  ({p.get('description','')})", name)
        self.combo.currentIndexChanged.connect(self._on_preset)
        top.addWidget(self.combo, 1)

        btn_zero = QPushButton("전부 0 (펴기)")
        btn_zero.clicked.connect(self._zero)
        top.addWidget(btn_zero)
        root.addLayout(top)

        # ---- 슬라이더 ----
        self.sliders = {}
        self.labels = {}
        for f in FINGERS:
            box = QGroupBox(f)
            grid = QGridLayout(box)
            lim = kin.chains[f].limits          # [[yaw_lo, yaw_hi], [pitch_lo, pitch_hi]]
            for row, (kind, lo, hi) in enumerate(
                    (("Yaw", lim[0, 0], lim[0, 1]), ("Pitch", lim[1, 0], lim[1, 1]))):
                name = f"R_{f}_{kind}"
                grid.addWidget(QLabel(kind), row, 0)

                s = QSlider(Qt.Horizontal)
                s.setMinimum(0)
                s.setMaximum(TICKS)
                s.setValue(int(round((0.0 - lo) / (hi - lo) * TICKS)))
                s.valueChanged.connect(self._on_user_slider)
                grid.addWidget(s, row, 1)
                self.sliders[name] = (s, float(lo), float(hi))

                lab = QLabel("")
                lab.setMinimumWidth(330)
                lab.setTextFormat(Qt.RichText)
                grid.addWidget(lab, row, 2)
                self.labels[name] = lab
            root.addWidget(box)

        self.status = QLabel("")
        self.status.setTextFormat(Qt.RichText)
        root.addWidget(self.status)

        self.pub = node.create_publisher(JointState, "/dexhand_driver/joint_command", 10)

        # /joint_states 는 rclpy 스핀 스레드에서 오므로 값만 받아 두고 GUI 스레드의
        # 타이머에서 슬라이더에 반영한다 (Qt 위젯은 GUI 스레드에서만 만져야 한다).
        self._js_lock = threading.Lock()
        self._js_latest = None
        node.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        # 사용자가 마지막으로 만진 뒤 이 시간(초) 동안은 joint_states 로 덮어쓰지 않는다.
        # 드라이버 램프(max_joint_speed)가 따라오는 동안 슬라이더가 되돌아가는 걸 막는다.
        self._hold_sec = 0.8
        self._last_user_cmd = -1e9

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._follow_joint_states)
        self.timer.start(50)          # 20Hz
        self._refresh()

    # ---------------- 값 ----------------

    def _values(self):
        q = {}
        for name, (s, lo, hi) in self.sliders.items():
            q[name] = lo + (hi - lo) * s.value() / TICKS
        return q

    def _set_values(self, q):
        for name, v in q.items():
            if name not in self.sliders:
                continue
            s, lo, hi = self.sliders[name]
            s.blockSignals(True)
            s.setValue(int(round((min(max(v, lo), hi) - lo) / (hi - lo) * TICKS)))
            s.blockSignals(False)
        self._refresh()

    def _zero(self):
        self.combo.setCurrentIndex(0)
        self._set_values({n: 0.0 for n in self.sliders})
        self._publish()

    def _on_preset(self, idx):
        name = self.combo.itemData(idx)
        if not name:
            return
        p = self.presets[name]
        q = {}
        for i, f in enumerate(FINGERS):
            q[f"R_{f}_Yaw"] = float(p["yaw"][i])
            q[f"R_{f}_Pitch"] = float(p["pitch"][i])
        self._set_values(q)
        self._publish()

    def _on_user_slider(self, _value):
        # blockSignals 로 막힌 프로그램적 setValue 는 여기 안 온다. 사용자 조작만이다.
        self._refresh()
        self._publish()

    # ---------------- joint_states 추종 ----------------

    def _on_joint_states(self, msg: JointState):
        with self._js_lock:
            self._js_latest = dict(zip(msg.name, msg.position))

    def _follow_joint_states(self):
        if any(s.isSliderDown() for s, _, _ in self.sliders.values()):
            return
        if self._now() - self._last_user_cmd < self._hold_sec:
            return
        with self._js_lock:
            js = self._js_latest
            self._js_latest = None
        if not js:
            return
        q = {n: float(js[n]) for n in self.sliders if n in js}
        if q:
            self._set_values(q)

    def _now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    # ---------------- 서보 us 미리보기 ----------------

    def _servo_us(self, finger_idx, yaw, pitch):
        """펌웨어 calcServoUs 재현. (us_outward, us_inward, clamped) 반환."""
        c = self.cfg
        flex = c["flex_dir"][finger_idx] * pitch / c["pitch_max_rad"]
        yawn = c["yaw_dir"][finger_idx] * yaw / c["yaw_max_rad"]
        flex = max(-1.0, min(1.0, flex)) * 1000.0
        yawn = max(-1.0, min(1.0, yawn)) * 1000.0
        out = []
        clamped = False
        for k in range(2):
            cal = c["servo_cal"][finger_idx * 2 + k]
            raw = (cal["neutral"]
                   + cal["flex_sign"] * flex * c["flex_span_us"] / 1000.0
                   + cal["yaw_sign"] * yawn * c["yaw_span_us"] / 1000.0)
            us = min(max(raw, cal["min"]), cal["max"])
            if abs(raw - us) > 0.5:
                clamped = True
            out.append(us)
        return out[0], out[1], clamped

    def _refresh(self):
        q = self._values()
        any_clamp = False
        for i, f in enumerate(FINGERS):
            yaw = q[f"R_{f}_Yaw"]
            pitch = q[f"R_{f}_Pitch"]
            uo, ui, cl = self._servo_us(i, yaw, pitch)
            any_clamp = any_clamp or cl
            names = self.cfg["servo_names"]
            tag = ' <b><span style="color:#d33">CLAMP</span></b>' if cl else ""
            for kind, v in (("Yaw", yaw), ("Pitch", pitch)):
                self.labels[f"R_{f}_{kind}"].setText(
                    f"{v:+.4f} rad ({math.degrees(v):+6.1f}°) &nbsp;|&nbsp; "
                    f"{names[i*2]}={uo:.0f}us {names[i*2+1]}={ui:.0f}us{tag}")

        tip = self.kin.fk_all([q[n] for n in JOINT_NAMES])
        txt = " &nbsp; ".join(
            f"{f}: ({tip[f][0]*1000:.0f}, {tip[f][1]*1000:.0f}, {tip[f][2]*1000:.0f})"
            for f in FINGERS)
        warn = ('<br><b><span style="color:#d33">서보 명령이 잘리고 있다. '
                'span 을 줄이거나 중립을 옮겨라 (시리얼에서 room 확인)</span></b>'
                if any_clamp else "")
        self.status.setText(f"손끝 위치 mm (base_link): {txt}{warn}")

    # ---------------- 송신 ----------------

    def _publish(self):
        self._last_user_cmd = self._now()
        if not self.chk_stream.isChecked():
            return
        q = self._values()
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(q[n]) for n in JOINT_NAMES]
        self.pub.publish(msg)


def main(argv=None):
    if QT_ERROR is not None:
        print(f"Qt 바인딩이 없다 ({QT_ERROR}). 다음 중 하나를 설치해라:\n"
              "  sudo apt install ros-humble-python-qt-binding\n"
              "  sudo apt install python3-pyqt5")
        return 1

    rclpy.init(args=argv)
    node = Node("dexhand_joint_sliders")

    node.declare_parameter("urdf_file", "")
    node.declare_parameter("servo_map_file", "")
    node.declare_parameter("presets_file", "")

    urdf = node.get_parameter("urdf_file").value
    smap = node.get_parameter("servo_map_file").value
    pres = node.get_parameter("presets_file").value

    for label, path in (("urdf_file", urdf), ("servo_map_file", smap),
                        ("presets_file", pres)):
        if not path or not os.path.exists(path):
            node.get_logger().error(f"{label} 파라미터가 비었거나 파일이 없다: {path!r}")
            return 1

    kin = DexHandKinematics(open(urdf, encoding="utf-8").read())
    servo_cfg = yaml.safe_load(open(smap, encoding="utf-8"))
    servo_cfg = servo_cfg["dexhand_driver"]["ros__parameters"]
    presets = yaml.safe_load(open(pres, encoding="utf-8"))["presets"]

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    app = QApplication(sys.argv)
    panel = SliderPanel(node, kin, servo_cfg, presets)
    panel.resize(760, 620)
    panel.show()
    code = app.exec_()

    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
