#!/usr/bin/env python3
"""
verify_offline.py — ROS 도, 아두이노도 없이 돌려 보는 검증 스크립트.

무엇을 확인하나
  1) URDF 파싱과 손가락 4개 체인 구성
  2) 손끝 IK 왕복 정확도 (FK -> IK -> FK 오차)
  3) 도달 불가 목표에서 잔차를 제대로 보고하는지
  4) 프리셋 12개를 rad -> 정규화 명령 -> 서보 us 까지 끝까지 환산해
     실제 펄스가 캘리브레이션 min/max 안에 들어오는지
  5) 스트림 패킷 문자열이 펌웨어 파서가 받는 형식인지

빌드 없이 그냥 돌아간다:
    python3 tools/verify_offline.py \
        --urdf ../dexhand_moveit_config/config/dexhandv2_right_8servo.urdf \
        --servo-map config/servo_map.yaml \
        --presets ../dexhand_moveit_config/config/grip_presets.yaml
"""

import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from dexhand_bringup.kinematics import FINGERS, DexHandKinematics  # noqa: E402


def to_stream(q, cfg, hi_res=True):
    """hand_driver_node._to_cmd 와 같은 계산. 여기서 독립적으로 다시 구현해
    두 곳이 어긋나면 눈에 띄게 한다."""
    scale = 1000.0 if hi_res else 100.0
    out = []
    for i in range(4):
        yaw, pitch = q[2 * i], q[2 * i + 1]
        flex = cfg["flex_dir"][i] * pitch / cfg["pitch_max_rad"]
        yawn = cfg["yaw_dir"][i] * yaw / cfg["yaw_max_rad"]
        out.append((int(round(np.clip(flex, -1, 1) * scale)),
                    int(round(np.clip(yawn, -1, 1) * scale))))
    return out


def firmware_us(pairs, cfg):
    """펌웨어 calcServoUs 를 그대로 재현. 반환: (us, clamped?) 8개"""
    res = []
    for f, (flex, yaw) in enumerate(pairs):
        for k in range(2):
            ch = f * 2 + k
            c = cfg["servo_cal"][ch]
            raw = (c["neutral"]
                   + c["flex_sign"] * flex * cfg["flex_span_us"] / 1000.0
                   + c["yaw_sign"] * yaw * cfg["yaw_span_us"] / 1000.0)
            us = min(max(raw, c["min"]), c["max"])
            res.append((ch, raw, us, abs(raw - us) > 0.5))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--servo-map", required=True)
    ap.add_argument("--presets", required=True)
    args = ap.parse_args()

    fails = 0

    print("=" * 72)
    print("1) URDF 파싱")
    kin = DexHandKinematics(open(args.urdf, encoding="utf-8").read())
    for f in FINGERS:
        ch = kin.chains[f]
        print(f"   {f:7s} dof={ch.dof_names} limits={np.round(ch.limits, 4).tolist()}")

    print()
    print("2) 손끝 IK 왕복 정확도")
    rng = np.random.default_rng(0)
    worst = 0.0
    for f in FINGERS:
        lim = kin.chains[f].limits
        errs = []
        for _ in range(300):
            q = rng.uniform(lim[:, 0], lim[:, 1])
            tgt = kin.fk_finger(f, q)
            qs, _res, _ok = kin.ik_finger(f, tgt)
            errs.append(np.linalg.norm(kin.fk_finger(f, qs) - tgt))
        e = np.array(errs)
        worst = max(worst, e.max())
        print(f"   {f:7s} mean={e.mean()*1e3:.5f}mm  max={e.max()*1e3:.5f}mm")
    if worst > 1e-4:
        print(f"   [FAIL] 왕복 오차 {worst*1e3:.3f}mm 는 너무 크다")
        fails += 1
    else:
        print(f"   [ ok ] 최대 {worst*1e3:.5f}mm")

    print()
    print("3) 도달 불가 목표 처리")
    base = kin.fk_finger("Index", [0.0, 0.0])
    for d in (0.01, 0.03, 0.05):
        q, res, ok = kin.ik_finger("Index", base + np.array([0.0, 0.0, d]))
        print(f"   +{d*100:.0f}cm 위 -> 잔차 {res*1000:6.1f}mm, converged={ok}, "
              f"q={np.round(q,4).tolist()}")
        if ok:
            print("   [FAIL] 도달 불가인데 수렴했다고 보고했다")
            fails += 1

    print()
    print("4) 프리셋 -> 스트림 -> 서보 us 전 구간 환산")
    cfg = yaml.safe_load(open(args.servo_map, encoding="utf-8"))
    cfg = cfg["dexhand_driver"]["ros__parameters"]
    presets = yaml.safe_load(open(args.presets, encoding="utf-8"))["presets"]

    # URDF 한계와 servo_map 의 pitch_max/yaw_max 가 어긋나면 GUI 와 실물이 어긋난다
    urdf_pitch = kin.chains["Index"].limits[1, 1]
    urdf_yaw = kin.chains["Index"].limits[0, 1]
    print(f"   URDF pitch 상한={urdf_pitch:.4f}  servo_map pitch_max_rad={cfg['pitch_max_rad']}")
    print(f"   URDF yaw  상한={urdf_yaw:.4f}  servo_map yaw_max_rad={cfg['yaw_max_rad']}")
    if abs(urdf_pitch - cfg["pitch_max_rad"]) > 1e-6 or abs(urdf_yaw - cfg["yaw_max_rad"]) > 1e-6:
        print("   [FAIL] URDF 한계와 servo_map 이 다르다. 둘은 반드시 같아야 한다")
        fails += 1
    else:
        print("   [ ok ] URDF 한계와 servo_map 일치")

    rooms = [min(c["max"] - c["neutral"], c["neutral"] - c["min"])
             for c in cfg["servo_cal"]]
    need = cfg["flex_span_us"] + cfg["yaw_span_us"]
    print(f"   span 합계 {need}us vs 채널별 여유 최솟값 {min(rooms)}us "
          f"({'ok' if need <= min(rooms) else 'CLAMP 발생'})")
    if need > min(rooms):
        fails += 1

    for name, p in presets.items():
        q = np.zeros(8)
        for i, f in enumerate(FINGERS):
            q[2 * i] = p["yaw"][i]
            q[2 * i + 1] = p["pitch"][i]
        pairs = to_stream(q, cfg)
        us = firmware_us(pairs, cfg)
        clamped = [f"CH{ch}" for ch, _raw, _u, cl in us if cl]
        pkt = "w " + " ".join(f"{a} {b}" for a, b in pairs)
        mark = "[ ok ]" if not clamped else "[FAIL]"
        if clamped:
            fails += 1
        print(f"   {mark} {name:14s} {pkt}")
        if clamped:
            print(f"          clamp 발생: {', '.join(clamped)}")

    print()
    print("5) 스트림 패킷 형식")
    pkt = "w " + " ".join(f"{a} {b}" for a, b in to_stream(np.zeros(8), cfg))
    ok = pkt.startswith("w ") and len(pkt.split()) == 9
    print(f"   {'[ ok ]' if ok else '[FAIL]'} '{pkt}'  (토큰 {len(pkt.split())}개, 9여야 함)")
    if not ok:
        fails += 1

    print()
    print("=" * 72)
    print(f"실패 {fails}건")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
