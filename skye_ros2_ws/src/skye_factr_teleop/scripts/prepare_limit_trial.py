#!/usr/bin/env python3
# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Prepare one limit-test run; this script never opens a servo or ROS node."""

from datetime import datetime, timezone
from pathlib import Path
import shlex

from skye_factr_teleop.config import load_config
from skye_factr_teleop.control_core import LimitTestSettings


PACKAGE = Path(__file__).resolve().parents[1]
SESSION = Path("/tmp/skye_factr_limit_trial.env")
EXPECTED_USB_BY_ID = {
    "left": "usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0",
    "right": "usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0",
}


def choose(prompt, allowed):
    while True:
        value = input(prompt).strip().lower()
        if value in allowed:
            return value
        print(f"请输入 {' / '.join(allowed)}")


def number(prompt, default, cast):
    while True:
        raw = input(f"{prompt} [{default}]: ").strip()
        try:
            return cast(raw) if raw else default
        except ValueError:
            print("请输入数字")


def main():
    side = choose("侧别 [left/right]: ", ("left", "right"))
    joint_id = number("物理关节 ID 1--8", 1, int)
    boundary = choose("测试边界 [lower/upper]: ", ("lower", "upper"))
    operator = input("操作者姓名: ").strip()
    if not operator:
        raise ValueError("操作者姓名不能为空")
    gain_scale = number("gain_scale", 0.1, float)
    current_limit_ma = number("本轮电流上限 mA", 20.0, float)
    ramp_seconds = number("渐增时间 s", 2.0, float)
    max_duration_seconds = number("最长通电时间 s", 10.0, float)
    settings = LimitTestSettings(joint_id, gain_scale, current_limit_ma,
                                 ramp_seconds, max_duration_seconds)
    config_path = PACKAGE / "config" / f"grav_comp_m6_{side}.yaml"
    config = load_config(config_path)
    if config.dynamixel.dynamixel_port != EXPECTED_USB_BY_ID[side]:
        raise ValueError(f"设备 A {side} 侧 USB by-id 与 2.5.1 验收记录不一致")
    if (config.dynamixel.joint_offsets is None
            or config.dynamixel.current_limit_ma is None
            or settings.current_limit_ma > min(config.dynamixel.current_limit_ma)):
        raise ValueError("该侧配置缺少标定，或本轮电流上限超过 YAML 上限")
    if not config.controller.limit_barrier_control.enable:
        raise ValueError("该侧限位屏障未启用")
    if side not in Path(config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
        raise ValueError("侧别与 leader URDF 不一致")
    barrier = config.controller.limit_barrier_control
    if joint_id == 8:
        lower, upper, kp = barrier.gripper.limit_min, barrier.gripper.limit_max, barrier.gripper.kp
    else:
        index = joint_id - 1
        lower = barrier.joint.limit_min[index] + barrier.joint.safety_margin
        upper = barrier.joint.limit_max[index] - barrier.joint.safety_margin
        kp = barrier.joint.kp
    if kp <= 0:
        raise ValueError("所选限位位置增益必须为正")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    base = f"{side}_id{joint_id}_{boundary}_{stamp}"
    record_dir = PACKAGE / "validation_records" / "phase7_2_2_A"
    output = record_dir / f"{base}.json"
    readback = record_dir / f"{base}_torque_disable.json"
    if output.exists() or output.with_suffix(".trace.jsonl").exists() or readback.exists():
        raise FileExistsError("本轮记录文件已存在，请重新准备")
    values = {
        "SIDE": side, "JOINT_ID": joint_id, "BOUNDARY": boundary,
        "OPERATOR": operator, "CONFIG": config_path, "OUTPUT": output,
        "READBACK": readback, "GAIN_SCALE": gain_scale,
        "CURRENT_LIMIT_MA": current_limit_ma, "RAMP_SECONDS": ramp_seconds,
        "MAX_DURATION_SECONDS": max_duration_seconds,
        "LAUNCH_FILE": f"limit_test_m6_{side}.launch.py",
        "USB_BY_ID": config.dynamixel.dynamixel_port,
    }
    SESSION.write_text("".join(f"{key}={shlex.quote(str(value))}\n"
                               for key, value in values.items()), encoding="utf-8")
    print(f"\n本轮：{side} ID{joint_id} {boundary}，有效边界 [{lower:.6f}, {upper:.6f}] rad")
    print(f"USB: /dev/serial/by-id/{config.dynamixel.dynamixel_port}")
    print(f"记录: {output}")
    print(f"会话文件: {SESSION}")
    print("请核对实体、有效边界与机械余量，再按清单启动记录器。")


if __name__ == "__main__":
    main()
