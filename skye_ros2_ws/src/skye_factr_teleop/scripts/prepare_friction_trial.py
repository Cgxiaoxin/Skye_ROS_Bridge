#!/usr/bin/env python3
# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Prepare one read-only session file for a static-friction test run."""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import shlex

from skye_factr_teleop.config import load_config
from skye_factr_teleop.control_core import FrictionTestSettings


PACKAGE = Path(__file__).resolve().parents[1]
SESSION = Path("/tmp/skye_factr_friction_trial.env")
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--single", action="store_true",
                        help="prepare the earlier separate off/on workflow")
    args = parser.parse_args()
    paired = not args.single
    side = choose("侧别 [left/right]: ", ("left", "right"))
    joint_id = number("物理关节 ID 1--7", 1, int)
    enabled = ("false" if paired else
               choose("静摩擦输出 [false=对照轮/true=测试轮]: ", ("false", "true")))
    operator = input("操作者姓名: ").strip()
    if not operator:
        raise ValueError("操作者姓名不能为空")
    settings = FrictionTestSettings(
        joint_id=joint_id,
        gain_scale=number("gain_scale", 0.1, float),
        current_limit_ma=number("本轮电流上限 mA", 20.0, float),
        ramp_seconds=number("渐增时间 s", 2.0, float),
        max_duration_seconds=number("最长通电时间 s", 25.0 if paired else 10.0, float),
        enabled=enabled == "true",
    )
    config_path = PACKAGE / "config" / f"grav_comp_m6_{side}.yaml"
    config = load_config(config_path)
    if config.dynamixel.dynamixel_port != EXPECTED_USB_BY_ID[side]:
        raise ValueError("设备 A USB by-id 与 2.5.1 验收记录不一致")
    if (config.dynamixel.joint_offsets is None
            or config.dynamixel.current_limit_ma is None
            or settings.current_limit_ma > min(config.dynamixel.current_limit_ma)):
        raise ValueError("配置缺少标定，或本轮电流上限超过 YAML 上限")
    if side not in Path(config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
        raise ValueError("侧别与 leader URDF 不一致")
    index = joint_id - 1
    gravity_gain = config.controller.joint_gravity_comp.gain[index]
    friction_gain = config.controller.static_friction_comp.gain[index]
    threshold = config.controller.static_friction_comp.enable_speed[index]
    if min(gravity_gain, friction_gain, threshold) <= 0:
        raise ValueError("所选关节增益和速度阈值必须为正")
    if (paired or settings.enabled) and not config.controller.static_friction_comp.enable:
        raise ValueError("配置中的静摩擦项未启用")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run_kind = "paired" if paired else "on" if settings.enabled else "off"
    base = f"{side}_id{joint_id}_{run_kind}_{stamp}"
    record_dir = PACKAGE / "validation_records" / "phase7_2_3_A"
    output = record_dir / f"{base}.json"
    readback = record_dir / f"{base}_torque_disable.json"
    if output.exists() or output.with_suffix(".trace.jsonl").exists() or readback.exists():
        raise FileExistsError("本轮记录文件已存在，请重新准备")
    values = {
        "SIDE": side, "JOINT_ID": joint_id, "ENABLED": enabled,
        "PAIRED": str(paired).lower(),
        "OPERATOR": operator, "CONFIG": config_path, "OUTPUT": output,
        "READBACK": readback, "GAIN_SCALE": settings.gain_scale,
        "CURRENT_LIMIT_MA": settings.current_limit_ma,
        "RAMP_SECONDS": settings.ramp_seconds,
        "MAX_DURATION_SECONDS": settings.max_duration_seconds,
        "LAUNCH_FILE": f"friction_test_m6_{side}.launch.py",
        "USB_BY_ID": config.dynamixel.dynamixel_port,
    }
    SESSION.write_text("".join(f"{key}={shlex.quote(str(value))}\n"
                               for key, value in values.items()), encoding="utf-8")
    print(f"\n本轮：{side} ID{joint_id} {'连续 off→on' if paired else 'enabled=' + enabled}；"
          f"速度阈值={threshold} rad/s")
    print(f"重力增益={gravity_gain}，静摩擦增益={friction_gain}")
    print(f"USB: /dev/serial/by-id/{config.dynamixel.dynamixel_port}")
    print(f"记录: {output}\n会话文件: {SESSION}")


if __name__ == "__main__":
    main()
