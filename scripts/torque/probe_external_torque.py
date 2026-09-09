#!/usr/bin/env python3
"""独占 SDK 读取大臂关节力矩（轴外力 ExternalTorEst + 对比 SensorTor）。

用途：接好大臂后，验证控制器外力矩估计是否可用，再决定写入
`/gento/{left,right}_joint_states.effort` 的字段。

须先停止占用控制器的进程（同一控制器只能一个 SDK 客户端）:
  pkill -f skye_robot_driver || true
  pkill -f gento_robot_driver || true

用法（仓库根目录）:
  /usr/bin/python3 scripts/torque/probe_external_torque.py
  /usr/bin/python3 scripts/torque/probe_external_torque.py --duration 20 --hz 10
  /usr/bin/python3 scripts/torque/probe_external_torque.py --arm left --mode imp_joint

看什么:
  - ExternalTorEst: 控制器轴外力矩估计（上位机工具参数已设好时，空载应接近 0，残差约 ±5）
  - SensorTor:      RT 力矩传感器/总力矩反馈（对照）
  - SG Joint_Tor:   慢组关节力矩（若有）
  - 轻推末端时 ExternalTorEst 对应轴应明显增大

字段偏移来自本机 g++ offsetof(ROBOT_RT / ROBOT_SG)，与
third_party/gento_sdk/include/Common/FXCommon.h 布局一致（SDK 4.4.x）。
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
import statistics
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK_PATH = str(REPO_ROOT / "third_party/gento_sdk/lib/x86_64/libGentoSDK.so")

DEFAULT_IP = "6.6.7.190"

# ROBOT_RT layout (bytes) — verified by offsetof against FXCommon.h
RT_SIZE = 1240
RT_ARM0_POS = 160
RT_ARM0_SENSOR = 244
RT_ARM0_EXT = 272
RT_ARM1_POS = 492  # 160 + sizeof(ARM_RT)=332
RT_ARM1_SENSOR = 576
RT_ARM1_EXT = 604

# ROBOT_SG
SG_SIZE = 1068
SG_ARM0_JOINT_TOR = 264
SG_ARM1_JOINT_TOR = 544

FX_OBJ_ARM0 = 0
FX_OBJ_ARM1 = 1
STATE_IDLE = 0
STATE_IMP_JOINT = 2


def parse_ip(text: str) -> tuple[int, int, int, int]:
    parts = text.strip().split(".")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(f"bad ip: {text}")
    return tuple(int(p) for p in parts)  # type: ignore[return-value]


def load_sdk() -> ctypes.CDLL:
    if not os.path.isfile(SDK_PATH):
        raise FileNotFoundError(SDK_PATH)
    sdk = ctypes.CDLL(SDK_PATH)
    sdk.FX_L1_System_Link.argtypes = [ctypes.c_ubyte] * 4 + [ctypes.c_uint]
    sdk.FX_L1_System_Link.restype = ctypes.c_int
    sdk.FX_L1_System_Unlink.argtypes = []
    sdk.FX_L1_System_Unlink.restype = None
    sdk.FX_L1_Fbk_GetRT.argtypes = []
    sdk.FX_L1_Fbk_GetRT.restype = ctypes.c_void_p
    sdk.FX_L1_Fbk_GetSG.argtypes = []
    sdk.FX_L1_Fbk_GetSG.restype = ctypes.c_void_p
    sdk.FX_L1_State_SwitchToIdle.argtypes = [ctypes.c_int, ctypes.c_uint]
    sdk.FX_L1_State_SwitchToIdle.restype = ctypes.c_int
    sdk.FX_L1_State_SwitchToImpJointMode.argtypes = [ctypes.c_int, ctypes.c_uint]
    sdk.FX_L1_State_SwitchToImpJointMode.restype = ctypes.c_int
    sdk.FX_L1_Fbk_CurrentState.argtypes = [ctypes.c_int]
    sdk.FX_L1_Fbk_CurrentState.restype = ctypes.c_int
    return sdk


def f7(base: int, offset: int) -> list[float]:
    arr = (ctypes.c_float * 7).from_address(base + offset)
    return [float(v) for v in arr]


def fmt7(vals: list[float], width: int = 7) -> str:
    return " ".join(f"{v:+{width}.2f}" for v in vals)


def deg7(rad_or_deg: list[float], already_deg: bool = True) -> list[float]:
    if already_deg:
        return rad_or_deg
    return [math.degrees(v) for v in rad_or_deg]


class RunningStats:
    def __init__(self) -> None:
        self._samples: list[list[float]] = []

    def add(self, vals: list[float]) -> None:
        self._samples.append(list(vals))

    def report(self, label: str) -> None:
        if not self._samples:
            print(f"  {label}: (no samples)")
            return
        cols = list(zip(*self._samples))
        print(f"  {label}  n={len(self._samples)}")
        print("       " + " ".join(f"{'J'+str(i+1):>7}" for i in range(7)))
        means = [statistics.fmean(c) for c in cols]
        mins = [min(c) for c in cols]
        maxs = [max(c) for c in cols]
        peak = [max(abs(a), abs(b)) for a, b in zip(mins, maxs)]
        print("    mean " + fmt7(means))
        print("    min  " + fmt7(mins))
        print("    max  " + fmt7(maxs))
        print("    |p|  " + fmt7(peak) + f"   max|τ|={max(peak):.2f}")


def maybe_switch_mode(sdk: ctypes.CDLL, mode: str, timeout_ms: int = 3000) -> None:
    if mode == "none":
        return
    arms = (FX_OBJ_ARM0, FX_OBJ_ARM1)
    if mode == "idle":
        for arm in arms:
            ret = sdk.FX_L1_State_SwitchToIdle(arm, timeout_ms)
            print(f"  SwitchToIdle arm{arm} ret={ret} state={sdk.FX_L1_Fbk_CurrentState(arm)}")
    elif mode == "imp_joint":
        for arm in arms:
            sdk.FX_L1_State_SwitchToIdle(arm, timeout_ms)
            ret = sdk.FX_L1_State_SwitchToImpJointMode(arm, timeout_ms)
            print(
                f"  SwitchToImpJoint arm{arm} ret={ret} "
                f"state={sdk.FX_L1_Fbk_CurrentState(arm)}"
            )
    else:
        raise ValueError(mode)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", type=parse_ip, default=parse_ip(DEFAULT_IP))
    p.add_argument("--hz", type=float, default=5.0, help="print rate (default 5)")
    p.add_argument("--duration", type=float, default=15.0, help="seconds (0=until Ctrl-C)")
    p.add_argument(
        "--arm",
        choices=("both", "left", "right"),
        default="both",
        help="which arm to print",
    )
    p.add_argument(
        "--mode",
        choices=("none", "idle", "imp_joint"),
        default="none",
        help="optional state switch after Link (default: leave controller as-is)",
    )
    p.add_argument(
        "--no-sg",
        action="store_true",
        help="skip ROBOT_SG Joint_Tor",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    print("=== probe ExternalTorEst (SDK GetRT) ===")
    print(f"SDK: {SDK_PATH}")
    print(f"IP:  {'.'.join(str(x) for x in args.ip)}")
    print("NOTE: stop skye_robot_driver first if Link fails.\n")

    sdk = load_sdk()
    ret = sdk.FX_L1_System_Link(
        args.ip[0], args.ip[1], args.ip[2], args.ip[3], 2
    )
    if ret < 0:
        print(f"Link 失败 ret={ret}。请先: pkill -f skye_robot_driver")
        return 1
    print(f"Link ok ret={ret}")

    try:
        maybe_switch_mode(sdk, args.mode)
        period = 1.0 / max(args.hz, 0.1)
        deadline = (
            None if args.duration <= 0 else time.monotonic() + args.duration
        )

        stats_ext = {"left": RunningStats(), "right": RunningStats()}
        stats_sensor = {"left": RunningStats(), "right": RunningStats()}

        print(
            "\n推一下大臂末端（轻推），看 ExternalTorEst 是否跟着变；"
            "空载时 |τ| 应大致落在你说的 ±5 内。\n"
        )

        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break

            rt = sdk.FX_L1_Fbk_GetRT()
            if not rt:
                print("GetRT returned NULL")
                time.sleep(period)
                continue

            left_pos = f7(rt, RT_ARM0_POS)
            left_sensor = f7(rt, RT_ARM0_SENSOR)
            left_ext = f7(rt, RT_ARM0_EXT)
            right_pos = f7(rt, RT_ARM1_POS)
            right_sensor = f7(rt, RT_ARM1_SENSOR)
            right_ext = f7(rt, RT_ARM1_EXT)

            sg_left = sg_right = None
            if not args.no_sg:
                sg = sdk.FX_L1_Fbk_GetSG()
                if sg:
                    sg_left = f7(sg, SG_ARM0_JOINT_TOR)
                    sg_right = f7(sg, SG_ARM1_JOINT_TOR)

            def show(side: str, pos, sensor, ext, sg_tor) -> None:
                print(f"[{side}] pos_deg  {fmt7(pos)}")
                print(f"[{side}] SensorTor{fmt7(sensor)}")
                print(f"[{side}] ExtTorEst{fmt7(ext)}   max|ext|={max(abs(x) for x in ext):.2f}")
                if sg_tor is not None:
                    print(f"[{side}] SG_JointT{fmt7(sg_tor)}")

            print("-" * 72)
            print(time.strftime("%H:%M:%S"))
            if args.arm in ("both", "left"):
                show("L", left_pos, left_sensor, left_ext, sg_left)
                stats_ext["left"].add(left_ext)
                stats_sensor["left"].add(left_sensor)
            if args.arm in ("both", "right"):
                show("R", right_pos, right_sensor, right_ext, sg_right)
                stats_ext["right"].add(right_ext)
                stats_sensor["right"].add(right_sensor)

            time.sleep(period)

        print("\n=== session stats ===")
        sides = ("left", "right") if args.arm == "both" else (args.arm,)
        for side in sides:
            print(f"-- {side} --")
            stats_ext[side].report("ExternalTorEst")
            stats_sensor[side].report("SensorTor")

    except KeyboardInterrupt:
        print("\nCtrl-C")
    finally:
        if args.mode != "none":
            for arm in (FX_OBJ_ARM0, FX_OBJ_ARM1):
                sdk.FX_L1_State_SwitchToIdle(arm, 3000)
        sdk.FX_L1_System_Unlink()
        print("unlink")

    return 0


if __name__ == "__main__":
    sys.exit(main())
