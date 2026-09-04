#!/usr/bin/env python3
"""双臂 Robotiq Hand-E 夹爪开闭控制（Gento SDK RS485 直连）.

使用前须停止 skye_robot_driver（同一控制器只能一个 SDK 客户端）:
  pkill -f skye_robot_driver

用法:
  /usr/bin/python3 scripts/robotiq/control_dual_gripper.py open
  /usr/bin/python3 scripts/robotiq/control_dual_gripper.py close
  /usr/bin/python3 scripts/robotiq/control_dual_gripper.py left open
  /usr/bin/python3 scripts/robotiq/control_dual_gripper.py right close
  /usr/bin/python3 scripts/robotiq/control_dual_gripper.py demo
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

# 允许从同目录导入 test_robotiq_right_485
sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_robotiq_right_485 import (  # noqa: E402
    CH485A,
    ARM0,
    ARM1,
    GentoTerminal485,
    REG_POS_CUR,
    RobotiqHandEGento,
    SLAVE_ID_DEFAULT,
    load_sdk,
    mm_from_pos_reg,
)

# =============================================================================
# 用户配置 — 按需修改开/闭目标位置
# =============================================================================
#
# 【物理开度 opening_mm】两指间距，单位 mm
#   - 闭合端: 0.0 mm（完全夹紧，实测反馈约 1~2 mm）
#   - 张开端: 50.0 mm（完全张开，实测反馈约 49 mm）
#   - 建议工作区间: 0 ~ 50 mm；中间值可任意插值，如 25.0 半开
#
# 【Modbus 寄存器 POS (0x03E9)】内部 0~255 字节，与开度关系:
#   reg_pos = round((50.0 - opening_mm) / 0.1953125)
#   opening_mm = 50.0 - reg_pos * 0.1953125
#
# 【速度 / 力度】speed、force 均为 0~255
#   - speed 默认 136 (0x88)
#   - force 默认 16 (0x10)，约 20N~185N
#
# 【若走 ROS 遥操（skye_robot_driver）】topic: /left|right_teleop_gripper/ctrl
#   position[0] 为归一化 0~1（FACTR 扳机语义，gripper_invert=true）:
#     1.0 = 张开，0.0 = 闭合
#   与 opening_mm 换算（driver 内）:
#     motor_norm = 1 - trigger_norm  →  opening_mm = 50 * (1 - motor_norm)
#
# 【硬件接线（本机实测）】
#   左臂: ARM0 / 485A / slave 9
#   右臂: ARM1 / 485A / slave 9
# =============================================================================

ROBOT_IP = "6.6.7.190"

# --- 左臂目标开度 (mm) ---
# 实机遥操：左闭 2 mm / 右闭 13 mm
LEFT_OPEN_MM = 50.0   # 张开：改这里
LEFT_CLOSE_MM = 2.0   # 闭合：改这里

# --- 右臂目标开度 (mm) ---
RIGHT_OPEN_MM = 50.0  # 张开：改这里
RIGHT_CLOSE_MM = 13.0  # 闭合：改这里

# 运动参数
SPEED = 0x88   # 0~255
FORCE = 0x10   # 0~255
MOVE_SETTLE_S = 2.0  # 动作后等待反馈稳定时间 (s)

# 硬件映射
GRIPPERS = {
    "left": {"arm": ARM0, "label": "左臂", "chn": CH485A, "slave": SLAVE_ID_DEFAULT},
    "right": {"arm": ARM1, "label": "右臂", "chn": CH485A, "slave": SLAVE_ID_DEFAULT},
}


def make_gripper(sdk, side: str) -> tuple[RobotiqHandEGento, str]:
    cfg = GRIPPERS[side]
    term = GentoTerminal485(sdk, cfg["arm"], cfg["chn"])
    return RobotiqHandEGento(term, slave=cfg["slave"]), cfg["label"]


def ensure_activated(gripper: RobotiqHandEGento, label: str) -> None:
    print(f"[{label}] 复位 + 激活 ...")
    gripper.reset()
    gripper.activate()
    pos = gripper.read_reg(REG_POS_CUR)
    print(f"[{label}] 激活完成，当前开度 ≈ {mm_from_pos_reg(pos):.1f} mm")


def move_and_report(
    gripper: RobotiqHandEGento, label: str, opening_mm: float, action: str
) -> None:
    print(f"[{label}] → {action}: 目标 {opening_mm:.1f} mm")
    gripper.move_mm(opening_mm, speed=SPEED, force=FORCE)
    time.sleep(MOVE_SETTLE_S)
    pos = gripper.read_reg(REG_POS_CUR)
    print(f"[{label}]   反馈 ≈ {mm_from_pos_reg(pos):.1f} mm")


def run_side(sdk, side: str, opening_mm: float, action: str) -> None:
    gripper, label = make_gripper(sdk, side)
    ensure_activated(gripper, label)
    move_and_report(gripper, label, opening_mm, action)


def run_both(sdk, left_mm: float, right_mm: float, action: str) -> None:
    left_g, left_label = make_gripper(sdk, "left")
    right_g, right_label = make_gripper(sdk, "right")
    for g, label in ((left_g, left_label), (right_g, right_label)):
        ensure_activated(g, label)
    move_and_report(left_g, left_label, left_mm, f"{action}(左)")
    move_and_report(right_g, right_label, right_mm, f"{action}(右)")


def run_demo(sdk) -> None:
    """演示：双臂依次 开 → 闭 → 开。"""
    sequence = [
        ("张开", LEFT_OPEN_MM, RIGHT_OPEN_MM),
        ("闭合", LEFT_CLOSE_MM, RIGHT_CLOSE_MM),
        ("张开", LEFT_OPEN_MM, RIGHT_OPEN_MM),
    ]
    for action, l_mm, r_mm in sequence:
        print(f"\n--- {action} ---")
        run_both(sdk, l_mm, r_mm, action)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="双臂 Robotiq 夹爪开闭控制")
    ap.add_argument("--ip", default=ROBOT_IP)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_open = sub.add_parser("open", help="双臂张开到 LEFT/RIGHT_OPEN_MM")
    p_open.add_argument("--ip", default=ROBOT_IP)

    p_close = sub.add_parser("close", help="双臂闭合到 LEFT/RIGHT_CLOSE_MM")
    p_close.add_argument("--ip", default=ROBOT_IP)

    p_demo = sub.add_parser("demo", help="演示：开-闭-开")
    p_demo.add_argument("--ip", default=ROBOT_IP)

    p_side = sub.add_parser("left", help="仅左臂")
    p_side.add_argument("action", choices=["open", "close"])
    p_side.add_argument("--ip", default=ROBOT_IP)

    p_side = sub.add_parser("right", help="仅右臂")
    p_side.add_argument("action", choices=["open", "close"])
    p_side.add_argument("--ip", default=ROBOT_IP)

    return ap.parse_args()


def main() -> int:
    args = parse_args()
    ip = args.ip if hasattr(args, "ip") else ROBOT_IP
    ip_tuple = tuple(int(x) for x in ip.split("."))

    sdk = load_sdk()
    ret = sdk.FX_L1_System_Link(*ip_tuple, 2)
    if ret < 0:
        print(f"link 失败 ret={ret}，请先停止 skye_robot_driver")
        return 1
    print(f"link OK {ret}ms ip={ip}")

    try:
        if args.cmd == "open":
            run_both(sdk, LEFT_OPEN_MM, RIGHT_OPEN_MM, "张开")
        elif args.cmd == "close":
            run_both(sdk, LEFT_CLOSE_MM, RIGHT_CLOSE_MM, "闭合")
        elif args.cmd == "demo":
            run_demo(sdk)
        elif args.cmd == "left":
            mm = LEFT_OPEN_MM if args.action == "open" else LEFT_CLOSE_MM
            run_side(sdk, "left", mm, "张开" if args.action == "open" else "闭合")
        elif args.cmd == "right":
            mm = RIGHT_OPEN_MM if args.action == "open" else RIGHT_CLOSE_MM
            run_side(sdk, "right", mm, "张开" if args.action == "open" else "闭合")
        return 0
    finally:
        sdk.FX_L1_System_Unlink()
        print("unlink")


if __name__ == "__main__":
    sys.exit(main())
