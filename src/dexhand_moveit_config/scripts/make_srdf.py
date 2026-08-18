#!/usr/bin/env python3
"""
make_srdf.py — grip_presets.yaml + 자동 생성된 충돌행렬로 SRDF 를 만든다.

MoveIt Setup Assistant 없이 SRDF 를 재생성할 수 있게 하는 목적이다.
프리셋을 고쳤거나 URDF 를 다시 뽑았으면 이 스크립트를 다시 돌리면 된다.

  python3 scripts/make_collision_matrix.py --urdf config/dexhandv2_right_8servo.urdf \
      --mesh-root <dexhandv2_description 경로> --samples 6000 --out /tmp/dc.xml
  python3 scripts/make_srdf.py --collisions /tmp/dc.xml \
      --presets config/grip_presets.yaml --out config/dexhandv2_right_8servo.srdf
"""

import argparse

import yaml

FINGERS = ["Index", "Middle", "Ring", "Pinky"]

HEADER = '''<?xml version="1.0" encoding="UTF-8"?>
<!--
  dexhandv2_right_8servo.srdf   (scripts/make_srdf.py 가 생성함 — 직접 고치지 말 것)

  실제로 서보가 달린 8개 관절(손가락 4개 x Yaw/Pitch)만 계획 대상으로 잡는다.
  Flexor/DIP/엄지는 URDF 단계에서 이미 fixed 이므로 여기서는 다루지 않는다.

  disable_collisions 는 scripts/make_collision_matrix.py 가 원본 삼각형 메시로
  랜덤 샘플링해 만들었다 (Setup Assistant 와 같은 알고리즘).
-->
<robot name="dexhandv2_right_8servo">

  <!-- ===== 계획 그룹 ===== -->

  <!-- 손 전체: 8관절 동시 제어. 프리셋 그립과 관절공간 계획은 이 그룹을 쓴다. -->
  <group name="hand">
'''

CHAIN_NOTE = '''
  <!--
    손가락별 그룹: base_link -> 손끝 프레임 체인.
    구동 자유도가 2개뿐이라 KDL/LMA 같은 표준 IK 플러그인은 이 체인을 풀지 못한다
    (3개 위치 구속 vs 2개 자유도 = 과결정계). 그래서 kinematics.yaml 에서 이 그룹들에는
    IK 플러그인을 붙이지 않았고, 손끝 목표 IK 는 dexhand_bringup 의 fingertip_ik_node 가
    3x2 자코비안 감쇠최소자승으로 직접 푼다.
    여기 그룹은 FK, 상태 표시, 손가락 단위 부분 계획용이다.
  -->
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--collisions", required=True)
    ap.add_argument("--presets", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    presets = yaml.safe_load(open(args.presets, encoding="utf-8"))["presets"]
    dis = open(args.collisions, encoding="utf-8").read()

    out = [HEADER]
    for f in FINGERS:
        out.append(f'    <joint name="R_{f}_Yaw"/>\n')
        out.append(f'    <joint name="R_{f}_Pitch"/>\n')
    out.append("  </group>\n")

    out.append(CHAIN_NOTE)
    for f in FINGERS:
        out.append(f'  <group name="{f.lower()}">\n')
        out.append(f'    <chain base_link="base_link" tip_link="R_{f}_tip_frame"/>\n')
        out.append("  </group>\n")

    out.append("\n  <!-- ===== 프리셋 그립 포즈 (config/grip_presets.yaml 에서 생성) ===== -->\n")
    for name, p in presets.items():
        out.append(f'  <!-- {p.get("description", "")} -->\n')
        out.append(f'  <group_state name="{name}" group="hand">\n')
        for i, f in enumerate(FINGERS):
            out.append(f'    <joint name="R_{f}_Yaw" value="{float(p["yaw"][i]):.4f}"/>\n')
            out.append(f'    <joint name="R_{f}_Pitch" value="{float(p["pitch"][i]):.4f}"/>\n')
        out.append("  </group_state>\n")

    out.append("\n  <!-- ===== 손끝 end effector ===== -->\n")
    for f in FINGERS:
        out.append(f'  <end_effector name="{f.lower()}_tip" parent_link="R_{f}_tip_frame" '
                   f'group="{f.lower()}"/>\n')

    out.append("\n  <!-- ===== 월드 고정 ===== -->\n")
    out.append('  <virtual_joint name="world_to_hand" type="fixed" '
               'parent_frame="world" child_link="base_link"/>\n')

    out.append("\n  <!-- ===== 자기충돌 제외 목록 (자동 생성) ===== -->\n")
    out.append(dis)
    out.append("\n</robot>\n")

    with open(args.out, "w", encoding="utf-8") as fp:
        fp.write("".join(out))
    print(f"wrote {args.out}  (그룹 5개, 프리셋 {len(presets)}개)")


if __name__ == "__main__":
    main()
