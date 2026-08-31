#!/usr/bin/env python3
"""右臂 Robotiq Hand-E 夹爪通讯测试 — Gento SDK 末端 RS485 Modbus RTU 透传.

寄存器/协议与 wbc/robot/xcore/robotiq_gripper.py 一致 (slave=9).
须先停 skye_robot_driver（同一控制器只能一个 SDK link）.

用法:
  /usr/bin/python3 scripts/test_robotiq_right_485.py              # 仅测通讯
  /usr/bin/python3 scripts/test_robotiq_right_485.py --open-close # 通讯 OK 后开合
  /usr/bin/python3 scripts/test_robotiq_right_485.py --chn 485B   # 指定 485 通道
"""

from __future__ import annotations

import argparse
import ctypes
import os
import struct
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SDK = str(REPO_ROOT / "third_party/gento_sdk/lib/x86_64/libGentoSDK.so")
IP = (6, 6, 7, 190)
ARM1 = 1
CANFD, CH485A, CH485B = 1, 2, 3

# Robotiq Hand-E (同 xcore/robotiq_gripper.py)
SLAVE_ID = 9
REG_ACTION = 0x03E8
REG_POS = 0x03E9
REG_SPEED = 0x03EA
REG_STATUS = 0x07D0
REG_POS_CUR = 0x07D2
GSTA_ACTIVATED = 0x03
POS_RATIO = 0.1953125
FULL_POS_MM = 50.0


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
    opening = reg * POS_RATIO
    return FULL_POS_MM - opening


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
        if n < 1:
            return None
        return bytes(buf[:n])


class RobotiqHandEGento:
    def __init__(self, term: GentoTerminal485, slave: int = SLAVE_ID):
        self.term = term
        self.slave = slave

    def read_reg(self, addr: int) -> int:
        self.term.clear()
        ret = self.term.tx(modbus_read_holding(self.slave, addr, 1))
        if ret != 0:
            raise RuntimeError(f"Terminal_SetData 失败 ret={ret}")
        deadline = time.time() + 1.0
        while time.time() < deadline:
            frame = self.term.rx(100)
            if frame:
                val = parse_read_response(frame, self.slave)
                if val is not None:
                    return val
            time.sleep(0.01)
        raise TimeoutError(f"读寄存器 0x{addr:04X} 超时 (无 Modbus 响应)")

    def write_reg(self, addr: int, value: int) -> None:
        self.term.clear()
        ret = self.term.tx(modbus_write_single(self.slave, addr, value))
        if ret != 0:
            raise RuntimeError(f"Terminal_SetData 失败 ret={ret}")
        deadline = time.time() + 1.0
        while time.time() < deadline:
            frame = self.term.rx(100)
            if frame and len(frame) >= 8 and frame[0] == self.slave and frame[1] == 0x06:
                return
            time.sleep(0.01)
        # 写响应偶发被缓冲丢弃; 不立即失败, 由后续读验证

    def reset(self) -> None:
        self.write_reg(REG_ACTION, action_reg(0))
        time.sleep(0.05)

    def activate(self, timeout_s: float = 5.0) -> None:
        self.write_reg(REG_ACTION, action_reg(1))
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            st = decode_status(self.read_reg(REG_STATUS))
            if st["gSta"] == GSTA_ACTIVATED:
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
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_uint,
        ctypes.POINTER(ctypes.c_ubyte),
        ctypes.c_uint,
        ctypes.POINTER(ctypes.c_uint),
    ]
    sdk.FX_L1_Terminal_SetData.restype = ctypes.c_int
    sdk.FX_L1_Terminal_GetData.argtypes = [
        ctypes.c_int,
        ctypes.c_uint,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_ubyte),
        ctypes.POINTER(ctypes.c_uint),
    ]
    sdk.FX_L1_Terminal_GetData.restype = ctypes.c_int
    return sdk


def probe_channel(sdk: ctypes.CDLL, chn: int, chn_name: str) -> bool:
    print(f"\n--- 探测 ARM1 / {chn_name} (chn={chn}) ---")
    term = GentoTerminal485(sdk, ARM1, chn)
    term.clear()
    req = modbus_read_holding(SLAVE_ID, REG_STATUS, 1)
    print(f"  TX Modbus: {req.hex(' ')}")
    ret = term.tx(req)
    print(f"  Terminal_SetData ret={ret} (0=SDK已发出)")
    for i in range(20):
        frame = term.rx(150)
        if frame:
            print(f"  RX [{i}]: chn=? len={len(frame)} hex={frame.hex(' ')}")
            val = parse_read_response(frame, SLAVE_ID)
            if val is not None:
                st = decode_status(val)
                print(f"  ✓ Modbus 通讯 OK — STATUS=0x{val:04X} {st}")
                pos = term.rx(50)
                if pos:
                    print(f"  (额外帧: {pos.hex(' ')})")
                return True
        time.sleep(0.05)
    print("  ✗ 无有效 Modbus 响应")
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="右臂 Robotiq Hand-E RS485 通讯测试")
    ap.add_argument("--ip", default="6.6.7.190")
    ap.add_argument("--slave", type=int, default=SLAVE_ID)
    ap.add_argument("--chn", choices=["485A", "485B", "both"], default="both")
    ap.add_argument("--open-close", action="store_true", help="通讯通过后开合测试")
    args = ap.parse_args()

    ip = tuple(int(x) for x in args.ip.split("."))
    sdk = load_sdk()
    ret = sdk.FX_L1_System_Link(*ip, 2)
    if ret < 0:
        print(f"link 失败 ret={ret}。先停 skye_robot_driver: pkill -f skye_robot_driver")
        return 1
    print(f"link OK delay={ret}ms ip={args.ip} arm=ARM1(右) slave={args.slave}")

    channels = []
    if args.chn in ("485A", "both"):
        channels.append((CH485A, "485A"))
    if args.chn in ("485B", "both"):
        channels.append((CH485B, "485B"))

    ok_chn = None
    try:
        for chn, name in channels:
            if probe_channel(sdk, chn, name):
                ok_chn = (chn, name)
                break

        if ok_chn is None:
            print("\n=== 通讯失败 ===")
            print("排查: 1) 右臂 Robotiq 24V 是否上电  2) RS485 A/B 接线")
            print("      3) slave_id 是否=9  4) 485 走 A 还是 B 通道")
            return 2

        chn, name = ok_chn
        print(f"\n=== 使用 ARM1/{name} 做控制测试 ===")
        gripper = RobotiqHandEGento(GentoTerminal485(sdk, ARM1, chn), slave=args.slave)
        print("复位 + 激活 ...")
        gripper.reset()
        gripper.activate()
        print("激活完成 ✓")
        pos_reg = gripper.read_reg(REG_POS_CUR)
        opening = mm_from_pos_reg((pos_reg >> 8) & 0xFF)
        print(f"当前开度约 {opening:.1f} mm (POS_CUR=0x{pos_reg:04X})")

        if args.open_close:
            print("\n--- 开合测试 (开 50mm → 闭 0mm → 开 50mm) ---")
            for label, mm in [("开", 50.0), ("闭", 0.0), ("开", 50.0)]:
                print(f"  → {label} ({mm:.0f} mm) ...")
                gripper.move_mm(mm)
                time.sleep(2.0)
                pos_reg = gripper.read_reg(REG_POS_CUR)
                opening = mm_from_pos_reg((pos_reg >> 8) & 0xFF)
                print(f"    反馈开度 ≈ {opening:.1f} mm")
            print("开合测试完成 ✓")
        else:
            print("\n通讯正常。要测开合请加 --open-close")

        return 0
    finally:
        sdk.FX_L1_System_Unlink()
        print("unlink")


if __name__ == "__main__":
    sys.exit(main())
