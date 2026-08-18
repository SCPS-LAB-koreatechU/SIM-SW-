#!/usr/bin/env python3
"""
make_8servo_urdf.py

iotdesignshop/dexhandv2_description 의 dexhandv2_right.urdf 를 읽어서
"실제로 서보가 달린 관절만 움직이는" URDF 를 만든다.

우리 하드웨어(2026-08 기준):
  - 손가락 4개 x 너클 서보 2개 = 8서보
  - 손가락 1개당 자유도 2개: Yaw(벌림) + Pitch(MCP 굽힘)
  - 긴 굽힘 텐던(PIP/DIP 구동)과 엄지는 아직 미장착

따라서 원본 URDF 의
  R_*_Flexor, R_*_DIP   -> 고정(fixed) 또는 Pitch 에 종속(mimic)
  R_Thumb_*             -> 고정(fixed)
로 바꾼다. 시뮬레이션이 실물보다 자유도가 많으면 MoveIt 이 실물로 재현 불가능한
해를 내놓기 때문에, 이 정리가 없으면 GUI 제어 자체가 성립하지 않는다.

추가로 손가락마다 tip_frame 링크를 붙인다.
Tip 메시에서 실측한 손끝점(링크 좌표계 기준 [-0.001, -0.003, 0.0179] m)에
고정 조인트로 매달아 두면, IK 목표점을 이 프레임 원점으로 잡을 수 있다.

사용법:
  python3 make_8servo_urdf.py \
      --input  <path>/dexhandv2_description/urdf/dexhandv2_right.urdf \
      --output ../config/dexhandv2_right_8servo.urdf \
      --coupling 0.0

  --coupling 0.0  : Flexor/DIP 를 fixed 로 (실물과 동일, 기본값)
  --coupling 0.6  : Flexor/DIP 를 Pitch 의 mimic 으로 (텐던 서보 장착 후 시각화용)
"""

import argparse
import xml.etree.ElementTree as ET
from xml.dom import minidom

FINGERS = ["Index", "Middle", "Ring", "Pinky"]

# 각 손가락 Tip 링크 이름 (원본 URDF 오타 "Midle_Tip_1" 포함)
TIP_LINK = {
    "Index": "Index_Tip_1",
    "Middle": "Midle_Tip_1",
    "Ring": "Ring_Tip_1",
    "Pinky": "Pinky_Tip_1",
}

# Tip STL 메시에서 산출한 손끝 접촉점 (Tip 링크 좌표계, meter)
# 메시 정점 중 +z 상위 4mm 구간의 무게중심. 4손가락 모두 편차 1mm 이내라 공통값을 쓴다.
TIP_OFFSET = (-0.00110, -0.00309, 0.01790)

ACTUATED = [f"R_{f}_{k}" for f in FINGERS for k in ("Yaw", "Pitch")]


def indent(elem, level=0):
    pad = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        for child in elem:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = pad


def lock_joint(joint):
    """revolute 조인트를 fixed 로 바꾼다. axis/limit/dynamics 는 제거."""
    joint.set("type", "fixed")
    for tag in ("axis", "limit", "dynamics", "safety_controller", "mimic"):
        for e in joint.findall(tag):
            joint.remove(e)


def mimic_joint(joint, source, multiplier):
    """revolute 를 유지하되 source 조인트에 종속시킨다."""
    for e in joint.findall("mimic"):
        joint.remove(e)
    m = ET.SubElement(joint, "mimic")
    m.set("joint", source)
    m.set("multiplier", f"{multiplier:.4f}")
    m.set("offset", "0.0")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--coupling", type=float, default=0.0,
                    help="0.0 이면 Flexor/DIP 를 fixed, >0 이면 Pitch 의 mimic 배율")
    ap.add_argument("--pitch-limit", type=float, default=0.95,
                    help="MCP 굽힘 상한(rad). 원본 CAD 한계는 1.308997(75deg)이지만 "
                         "현재 서보 스팬(450us)으로 실제 도달하는 각도는 그보다 작다. "
                         "실측 후 이 값을 올려라.")
    ap.add_argument("--yaw-limit", type=float, default=0.30,
                    help="벌림 한계(rad, +-). 원본 CAD 한계는 0.349066(20deg).")
    args = ap.parse_args()

    tree = ET.parse(args.input)
    root = tree.getroot()
    root.set("name", "dexhandv2_right_8servo")

    # 1) 원본의 transmission / gazebo 태그 제거
    #    (원본 transmission 은 조인트 이름을 R_ 접두사 없이 참조하는 버그가 있고,
    #     우리는 ros2_control 이 아니라 자체 드라이버 노드로 서보를 몰기 때문에 불필요)
    for tag in ("transmission", "gazebo"):
        for e in root.findall(tag):
            root.remove(e)

    joints = {j.get("name"): j for j in root.findall("joint")}
    missing = [n for n in ACTUATED if n not in joints]
    if missing:
        raise SystemExit(f"원본 URDF 에 없는 조인트: {missing}")

    # 2) 미장착 자유도 잠그기
    locked = []
    for f in FINGERS:
        for k in ("Flexor", "DIP"):
            name = f"R_{f}_{k}"
            if args.coupling > 0.0:
                mimic_joint(joints[name], f"R_{f}_Pitch", args.coupling)
            else:
                lock_joint(joints[name])
            locked.append(name)

    for name, j in joints.items():
        if name.startswith("R_Thumb_"):
            lock_joint(j)
            locked.append(name)

    # 3) 손끝 프레임 추가
    x, y, z = TIP_OFFSET
    for f in FINGERS:
        link = ET.SubElement(root, "link")
        link.set("name", f"R_{f}_tip_frame")

        j = ET.SubElement(root, "joint")
        j.set("name", f"R_{f}_tip_fixed")
        j.set("type", "fixed")
        o = ET.SubElement(j, "origin")
        o.set("xyz", f"{x} {y} {z}")
        o.set("rpy", "0 0 0")
        ET.SubElement(j, "parent").set("link", TIP_LINK[f])
        ET.SubElement(j, "child").set("link", f"R_{f}_tip_frame")

    # 4) 구동 조인트에 현실적인 한계 / effort / velocity 부여
    #    원본은 effort=100, velocity=100 (사실상 무제한). 서보 기준으로 낮춘다.
    #
    #    위치 한계도 좁힌다. 원본 CAD 한계(pitch 75deg, yaw 20deg)를 그대로 두면
    #    MoveIt 이 실물에서 재현 불가능한 자세를 계획해 놓고 "성공" 이라고 보고한다.
    #    GUI 와 실물이 어긋나는 가장 흔한 원인이라 여기서 막는다.
    for name in ACTUATED:
        lim = joints[name].find("limit")
        lim.set("effort", "1.5")      # N·m, 서보 스톨토크의 보수적 추정
        lim.set("velocity", "3.0")    # rad/s
        if name.endswith("_Pitch"):
            lim.set("lower", "0.0")
            lim.set("upper", f"{args.pitch_limit:.6f}")
        else:
            lim.set("lower", f"{-args.yaw_limit:.6f}")
            lim.set("upper", f"{args.yaw_limit:.6f}")

    indent(root)
    xml = minidom.parseString(ET.tostring(root, "utf-8")).toprettyxml(indent="  ")
    xml = "\n".join(line for line in xml.splitlines() if line.strip())

    with open(args.output, "w", encoding="utf-8") as fp:
        fp.write(xml + "\n")

    print(f"wrote {args.output}")
    print(f"  구동 조인트 {len(ACTUATED)}개: {', '.join(ACTUATED)}")
    print(f"  잠근 조인트 {len(locked)}개 (coupling={args.coupling})")
    print("  손끝 프레임 4개 추가: R_*_tip_frame")


if __name__ == "__main__":
    main()
