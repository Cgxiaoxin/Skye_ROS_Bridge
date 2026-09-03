#!/usr/bin/env python3
"""Robotiq Hand-E 夹爪通讯测试 — Gento SDK 末端 RS485 Modbus RTU 透传.

用法:
  /usr/bin/python3 scripts/robotiq/test_robotiq_right_485.py --arm L --scan
  /usr/bin/python3 scripts/robotiq/test_robotiq_right_485.py --arm L --open-close
  /usr/bin/python3 scripts/robotiq/test_robotiq_right_485.py --arm R --open-close
  /usr/bin/python3 scripts/robotiq/test_robotiq_right_485.py --arm both --open-close
"""

from __future__ import annotations

import argparse
import ctypes
import os
import struct
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK = str(REPO_ROOT / "third_party/gento_sdk/lib/x86_64/libGentoSDK.so")
ARM0, ARM1 = 0, 1
CH485A, CH485B = 2, 3

SLAVE_ID_DEFAULT = 9
REG_ACTION = 0x03E8
REG_POS = 0x03E9
REG_SPEED = 0x03EA
REG_STATUS = 0x07D0
REG_POS_CUR = 0x07D2
GSTA_ACTIVATED = 0x03
POS_RATIO = 0.1953125
FULL_POS_MM = 50.0
SCAN_SLAVE_IDS = (9, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12)


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def modbus_read_holding(slave: int, addr: int, count: int = 1) -> bytes:
    payload = struct.pack(">BBHH", slave, 0x03, addr, count)
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)


def modbus_write_single(slave: int, addr: int, value: int) -> bytes:
    payload = struct.pack(">BBHH", slave, 0x06, addr, value & 0xFFFF)
    crc = crc16_modbus(payload)
    return payload + struct.pack("<H", crc)


def parse_read_response(frame: bytes, slave: int) -> int | None:
    if len(frame) < 7 or frame[0] != slave or frame[1] != 0x03:
        return None
    n = frame[2]
    if len(frame) < 3 + n + 2:
        return None
    body = frame[: 3 + n]
    if crc16_modbus(body) != struct.unpack("<H", frame[3 + n : 3 + n + 2])[0]:
        return None
    if n == 2:
        return (frame[3] << 8) | frame[4]
    return None


def action_reg(r_act: int, r_gto: int = 0, r_atr: int = 0, r_ard: int = 0) -> int:
    return (r_act << 8) | (r_gto << 11) | (r_atr << 12) | (r_ard << 13)


def decode_status(reg: int) -> dict:
    st = (reg >> 8) & 0xFF
    return {
        "gAct": st & 0x01,
        "gGto": (st >> 3) & 0x01,
        "gSta": ((st >> 4) & 0x03),
        "gObj": ((st >> 6) & 0x03),
        "raw": reg,
    }


def mm_from_pos_reg(reg: int) -> float:
    return FULL_POS_MM - ((reg >> 8) & 0xFF) * POS_RATIO


class GentoTerminal485:
    def __init__(self, sdk: ctypes.CDLL, arm: int, chn: int):
        self.sdk = sdk
        self.arm = arm
        self.chn = chn

    def clear(self) -> None:
        self.sdk.FX_L1_Terminal_ClearData(self.arm)

    def tx(self, payload: bytes, timeout_ms: int = 200) -> int:
        buf = (ctypes.c_ubyte * 64)(*payload)
        t = ctypes.c_uint(0)
        return self.sdk.FX_L1_Terminal_SetData(
            self.arm, self.chn, timeout_ms, buf, len(payload), ctypes.byref(t)
        )

    def rx(self, timeout_ms: int = 300) -> bytes | None:
        chn = ctypes.c_int(0)
        buf = (ctypes.c_ubyte * 64)()
        t = ctypes.c_uint(0)
        n = self.sdk.FX_L1_Terminal_GetData(
            self.arm, timeout_ms, ctypes.byref(chn), buf, ctypes.byref(t)
        )
        return bytes(buf[:n]) if n >= 1 else None


class RobotiqHandEGento:
    def __init__(self, term: GentoTerminal485, slave: int = SLAVE_ID_DEFAULT):
        self.term = term
        self.slave = slave

    def read_reg(self, addr: int) -> int:
        self.term.clear()
        if self.term.tx(modbus_read_holding(self.slave, addr, 1)) != 0:
            raise RuntimeError("Terminal_SetData 失败")
        deadline = time.time() + 1.0
        while time.time() < deadline:
            frame = self.term.rx(100)
            if frame:
                val = parse_read_response(frame, self.slave)
                if val is not None:
                    return val
            time.sleep(0.01)
        raise TimeoutError(f"读 0x{addr:04X} 超时")

    def write_reg(self, addr: int, value: int) -> None:
        self.term.clear()
        if self.term.tx(modbus_write_single(self.slave, addr, value)) != 0:
            raise RuntimeError("Terminal_SetData 失败")
        deadline = time.time() + 1.0
        while time.time() < deadline:
            frame = self.term.rx(100)
            if frame and len(frame) >= 8 and frame[0] == self.slave and frame[1] == 0x06:
                return
            time.sleep(0.01)

    def reset(self) -> None:
        self.write_reg(REG_ACTION, action_reg(0))
        time.sleep(0.05)

    def activate(self, timeout_s: float = 5.0) -> None:
        self.write_reg(REG_ACTION, action_reg(1))
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if decode_status(self.read_reg(REG_STATUS))["gSta"] == GSTA_ACTIVATED:
                return
            time.sleep(0.05)
        raise TimeoutError("夹爪激活超时")

    def move_mm(self, opening_mm: float, speed: int = 0x88, force: int = 0x10) -> None:
        reg_pos = max(0, min(255, round((FULL_POS_MM - opening_mm) / POS_RATIO)))
        self.write_reg(REG_POS, reg_pos)
        self.write_reg(REG_SPEED, (speed << 8) | force)
        self.write_reg(REG_ACTION, action_reg(1, 1))


def load_sdk() -> ctypes.CDLL:
    if not os.path.isfile(SDK):
        raise FileNotFoundError(SDK)
    sdk = ctypes.CDLL(SDK)
    sdk.FX_L1_System_Link.argtypes = [ctypes.c_ubyte] * 4 + [ctypes.c_uint]
    sdk.FX_L1_System_Link.restype = ctypes.c_int
    sdk.FX_L1_System_Unlink.argtypes = []
    sdk.FX_L1_Terminal_ClearData.argtypes = [ctypes.c_int]
    sdk.FX_L1_Terminal_ClearData.restype = ctypes.c_int
    sdk.FX_L1_Terminal_SetData.argtypes = [
        ctypes.c_int, ctypes.c_int, ctypes.c_uint,
        ctypes.POINTER(ctypes.c_ubyte), ctypes.c_uint, ctypes.POINTER(ctypes.c_uint),
    ]
    sdk.FX_L1_Terminal_SetData.restype = ctypes.c_int
    sdk.FX_L1_Terminal_GetData.argtypes = [
        ctypes.c_int, ctypes.c_uint, ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_uint),
    ]
    sdk.FX_L1_Terminal_GetData.restype = ctypes.c_int
    return sdk


def try_read_status(term: GentoTerminal485, slave: int) -> int | None:
    term.clear()
    if term.tx(modbus_read_holding(slave, REG_STATUS, 1)) != 0:
        return None
    for _ in range(12):
        frame = term.rx(120)
        if frame:
            val = parse_read_response(frame, slave)
            if val is not None:
                return val
        time.sleep(0.03)
    return None


def scan_arm(sdk: ctypes.CDLL, arm: int, arm_label: str):
    print(f"\n========== 扫描 {arm_label} (ARM{arm}) ==========")
    hits = []
    for chn, chn_name in ((CH485A, "485A"), (CH485B, "485B")):
        print(f"\n--- 通道 {chn_name} ---")
        term = GentoTerminal485(sdk, arm, chn)
        for slave in SCAN_SLAVE_IDS:
            val = try_read_status(term, slave)
            if val is None:
                print(f"  slave={slave:2d}  无响应")
            else:
                st = decode_status(val)
                print(f"  slave={slave:2d}  ✓ STATUS=0x{val:04X}  {st}")
                hits.append((chn, chn_name, slave))
    if not hits:
        return None
    for chn, chn_name, slave in hits:
        if slave == SLAVE_ID_DEFAULT and chn_name == "485A":
            print(f"\n>>> 选用: ARM{arm}/{chn_name} slave={slave}")
            return chn, chn_name, slave
    chn, chn_name, slave = hits[0]
    print(f"\n>>> 选用: ARM{arm}/{chn_name} slave={slave}")
    return chn, chn_name, slave


def probe_channel(sdk, arm, arm_label, chn, chn_name, slave) -> bool:
    print(f"\n--- 探测 {arm_label} ARM{arm}/{chn_name} slave={slave} ---")
    term = GentoTerminal485(sdk, arm, chn)
    term.clear()
    print(f"  TX: {modbus_read_holding(slave, REG_STATUS, 1).hex(' ')}")
    term.tx(modbus_read_holding(slave, REG_STATUS, 1))
    for i in range(20):
        frame = term.rx(150)
        if frame:
            print(f"  RX [{i}]: {frame.hex(' ')}")
            val = parse_read_response(frame, slave)
            if val is not None:
                print(f"  ✓ STATUS=0x{val:04X} {decode_status(val)}")
                return True
        time.sleep(0.05)
    print("  ✗ 无响应")
    return False


def parse_arm(s: str):
    key = s.strip().upper()
    if key in ("BOTH", "B", "LR", "ALL", "2"):
        return None  # dual-arm mode
    if key in ("L", "LEFT", "0", "ARM0"):
        return [(ARM0, "左臂")]
    if key in ("R", "RIGHT", "1", "ARM1"):
        return [(ARM1, "右臂")]
    raise SystemExit(f"未知 --arm: {s} (用 L / R / both)")


def test_one_arm(
    sdk: ctypes.CDLL, arm: int, arm_label: str, slave_override: int | None, open_close: bool
) -> bool:
    found = scan_arm(sdk, arm, arm_label)
    if not found:
        return False
    chn, chn_name, slave = found
    if slave_override is not None:
        slave = slave_override
    if not probe_channel(sdk, arm, arm_label, chn, chn_name, slave):
        return False
    gripper = RobotiqHandEGento(GentoTerminal485(sdk, arm, chn), slave=slave)
    run_control_test(gripper, open_close)
    return True


def run_control_test(gripper: RobotiqHandEGento, open_close: bool) -> int:
    print("复位 + 激活 ...")
    gripper.reset()
    gripper.activate()
    print("激活完成 ✓")
    pos_reg = gripper.read_reg(REG_POS_CUR)
    print(f"当前开度 ≈ {mm_from_pos_reg(pos_reg):.1f} mm (POS_CUR=0x{pos_reg:04X})")
    if open_close:
        print("\n--- 开合测试 ---")
        for label, mm in [("开", 50.0), ("闭", 0.0), ("开", 50.0)]:
            print(f"  → {label} ({mm:.0f} mm)")
            gripper.move_mm(mm)
            time.sleep(2.0)
            print(f"    反馈 ≈ {mm_from_pos_reg(gripper.read_reg(REG_POS_CUR)):.1f} mm")
        print("开合测试完成 ✓")
    else:
        print("通讯正常。开合测试请加 --open-close")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="6.6.7.190")
    ap.add_argument("--arm", default="R", help="L / R / both（双臂依次测）")
    ap.add_argument("--slave", type=int, default=None)
    ap.add_argument("--scan", action="store_true", help="已默认扫描；保留兼容")
    ap.add_argument("--open-close", action="store_true")
    args = ap.parse_args()

    arms = parse_arm(args.arm)
    if arms is None:
        arms = [(ARM0, "左臂"), (ARM1, "右臂")]

    sdk = load_sdk()
    ip = tuple(int(x) for x in args.ip.split("."))
    ret = sdk.FX_L1_System_Link(*ip, 2)
    if ret < 0:
        print(f"link 失败 ret={ret}，先停 skye_robot_driver")
        return 1
    if len(arms) == 1:
        print(f"link OK {ret}ms ip={args.ip} {arms[0][1]}(ARM{arms[0][0]})")
    else:
        print(f"link OK {ret}ms ip={args.ip} 双臂依次测试")

    ok_all = True
    try:
        for arm, arm_label in arms:
            if len(arms) > 1:
                print(f"\n{'=' * 50}\n>>> 开始 {arm_label} (ARM{arm})\n{'=' * 50}")
            if not test_one_arm(sdk, arm, arm_label, args.slave, args.open_close):
                ok_all = False
                if len(arms) > 1:
                    print(f"!!! {arm_label} 测试失败，继续下一臂 ...")
        return 0 if ok_all else 2
    finally:
        sdk.FX_L1_System_Unlink()
        print("unlink")


if __name__ == "__main__":
    sys.exit(main())
