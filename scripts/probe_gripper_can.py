#!/usr/bin/env python3
"""右爪 CAN 诊断：区分「代码/SDK」vs「硬件/挂接/总线」。

须先停 skye_robot_driver。用法：
  python3 scripts/probe_gripper_can.py

判定逻辑：
  A  ARM0+id1 有反馈     → 左通路/SDK Terminal 正常（对照基线）
  B  ARM1 任意 id 有反馈 → 右 Terminal CAN 物理通，只是 ID/配置问题
  C  ARM0+id2 有反馈     → 右爪其实挂在左总线（改 gripper_right_terminal:=0）
  D  ARM1 全 silent      → 代码已发出(SetData=0)但无回包 = 右末端 CAN/线/电机
  E  交叉：左爪线插到右腕口测 ARM1 → 有反馈=右腕口 OK、原右爪坏；仍无=右腕口 CAN
"""
from __future__ import annotations

import ctypes
import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SDK = str(REPO_ROOT / "third_party/gento_sdk/lib/x86_64/libGentoSDK.so")
IP = (6, 6, 7, 190)
ARM0, ARM1, CANFD = 0, 1, 1
PROBE_IDS = (1, 2, 3, 0, 4, 5, 6, 7, 8, 16, 17)
HOLD_S = 0.8
POS_MAX, KP, KD = 1.6, 3.0, 0.12
QMAX, DQMAX, TAUMAX = 12.5, 30.0, 10.0


def clamp(x, lo, hi):
    return lo if x <= lo else hi if x > hi else x


def f2u(x, xmin, xmax, bits):
    x = clamp(x, xmin, xmax)
    return int((x - xmin) / (xmax - xmin) * ((1 << bits) - 1))


def u2f(x, vmin, vmax, bits):
    return x / ((1 << bits) - 1) * (vmax - vmin) + vmin


def encode_mit(q):
    kp_u, kd_u = f2u(KP, 0, 500, 12), f2u(KD, 0, 5, 12)
    q_u = f2u(q, -QMAX, QMAX, 16)
    dq_u, tau_u = f2u(0.0, -DQMAX, DQMAX, 12), f2u(0.0, -TAUMAX, TAUMAX, 12)
    return bytes(
        [
            (q_u >> 8) & 0xFF,
            q_u & 0xFF,
            dq_u >> 4,
            ((dq_u & 0xF) << 4) | ((kp_u >> 8) & 0xF),
            kp_u & 0xFF,
            kd_u >> 4,
            ((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF),
            tau_u & 0xFF,
        ]
    )


def pack(can_id, data8):
    return can_id.to_bytes(4, "little") + data8


def decode_fb(payload):
    if payload is None or len(payload) < 12:
        return None
    cid = int.from_bytes(payload[:4], "little")
    d = payload[4:12]
    err = (d[0] & 0xF0) >> 4
    pos = u2f((d[1] << 8) | d[2], -QMAX, QMAX, 16)
    return cid, pos, err, d[6]


def main() -> int:
    if not os.path.isfile(SDK):
        print("找不到", SDK)
        return 1
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

    def tx(term, raw12):
        buf = (ctypes.c_ubyte * 64)(*raw12)
        t = ctypes.c_uint(0)
        return sdk.FX_L1_Terminal_SetData(term, CANFD, 100, buf, 12, ctypes.byref(t))

    def rx(term, timeout_ms=30):
        chn = ctypes.c_int(0)
        buf = (ctypes.c_ubyte * 64)()
        t = ctypes.c_uint(0)
        n = sdk.FX_L1_Terminal_GetData(
            term, timeout_ms, ctypes.byref(chn), buf, ctypes.byref(t)
        )
        return bytes(buf[:n]) if n >= 5 else None

    def enable(term, cid):
        return tx(term, pack(cid, bytes([0xFF] * 7 + [0xFC])))

    def fb_match(cid, fb):
        """DM 应答 can_id 常为 指令id 或 id|0x10；过滤总线上别的电机。"""
        if fb is None:
            return False
        rid = fb[0] & 0xFF
        return rid in (cid, cid | 0x10, (cid + 0x10) & 0xFF)

    def probe(term, cid, hold=HOLD_S):
        """enable + MIT，返回 (tx_ok_count, fb_count, last_fb)。只计匹配该 id 的回包。"""
        sdk.FX_L1_Terminal_ClearData(term)
        tx_ok = fb_n = 0
        last = None
        for _ in range(3):
            if enable(term, cid) == 0:
                tx_ok += 1
            time.sleep(0.03)
        t0 = time.time()
        while time.time() - t0 < hold:
            if tx(term, pack(cid, encode_mit(0.5 * POS_MAX))) == 0:
                tx_ok += 1
            fb = decode_fb(rx(term, 20))
            if fb_match(cid, fb):
                fb_n += 1
                last = fb
            time.sleep(0.01)
        return tx_ok, fb_n, last

    def listen(term, hold=1.0):
        """只收不发，看总线上有无自发帧。"""
        sdk.FX_L1_Terminal_ClearData(term)
        n, last = 0, None
        t0 = time.time()
        while time.time() - t0 < hold:
            fb = decode_fb(rx(term, 20))
            if fb:
                n += 1
                last = fb
            time.sleep(0.01)
        return n, last

    ret = sdk.FX_L1_System_Link(*IP, 2)
    if ret < 0:
        print(f"link 失败 {ret}。先: pkill -f skye_robot_driver")
        return 1
    print(f"link ok delay={ret}ms\n")

    results = {}
    try:
        # --- A 基线：左 ---
        print("=== A 基线 ARM0 + id1（左爪）===")
        tx_ok, fb_n, last = probe(ARM0, 1)
        results["A_L"] = fb_n
        print(f"  SetData成功约{tx_ok}次  反馈{fb_n}帧  last={last}")
        if fb_n == 0:
            print("  !! 左也无反馈 → 先修左/供电/停驱动占用，后续结论不可信")
            return 1

        # --- B ARM1 扫 ID ---
        print("\n=== B 扫 ARM1（右 Terminal）全 ID ===")
        arm1_hits = []
        for cid in PROBE_IDS:
            tx_ok, fb_n, last = probe(ARM1, cid, hold=0.5)
            mark = "OK" if fb_n else "--"
            print(f"  [{mark}] ARM1 id={cid:2d}  tx≈{tx_ok}  fb={fb_n}  {last}")
            if fb_n:
                arm1_hits.append((cid, fb_n, last))
        results["B_ARM1"] = arm1_hits

        # --- C 左总线找 id2（右爪是否共总线）---
        print("\n=== C ARM0 + id2（右爪是否挂在左总线）===")
        tx_ok, fb_n, last = probe(ARM0, 2)
        results["C_ARM0_id2"] = fb_n
        print(f"  SetData成功约{tx_ok}次  反馈{fb_n}帧  last={last}")

        # --- D ARM1 只听 ---
        print("\n=== D ARM1 只听 1s（不发，看有无挂起乱报）===")
        n, last = listen(ARM1, 1.0)
        results["D_listen"] = n
        print(f"  自发帧 {n}  last={last}")

        # --- E 厂商默认：ARM1+id2 再确认一次 ---
        print("\n=== E 确认 ARM1 + id2（厂商）===")
        tx_ok, fb_n, last = probe(ARM1, 2, hold=1.2)
        results["E_R"] = (tx_ok, fb_n, last)
        print(f"  SetData成功约{tx_ok}次  反馈{fb_n}帧  last={last}")

        # --- 汇总 ---
        print("\n======== 判定 ========")
        print(f"A 左 ARM0+id1: {'通' if results['A_L'] else '断'}")
        if results["B_ARM1"]:
            ids = ", ".join(str(c) for c, _, _ in results["B_ARM1"])
            print(f"B 右 ARM1: 有应答 ID=[{ids}] → 总线通，改 gripper_right_motor_id")
        else:
            print("B 右 ARM1: 全 ID 无应答")
        if results["C_ARM0_id2"]:
            print("C ARM0+id2 有【匹配 id2】应答 → 右爪可能挂在左总线，试 gripper_right_terminal:=0")
        else:
            print(
                "C ARM0+id2 无匹配应答 "
                "（若上次看到 can=17 那是左爪 id1 残留，不是右爪）→ 不是共左总线"
            )
        print(
            f"D ARM1 静默听: {results['D_listen']} 帧 "
            f"({'有异常自发' if results['D_listen'] else '无挂起乱报'})"
        )
        tx_ok, fb_n, _ = results["E_R"]
        if tx_ok > 0 and fb_n == 0:
            print(
                "E SDK 对 ARM1+id2 SetData=成功、GetData=0 → "
                "代码路径已发出，问题在右末端 CAN/线/电机，不是 GripperBridge 左右写反"
            )
        elif fb_n > 0:
            print("E ARM1+id2 正常 → 应用层应能控右爪")

        print(
            "\n交叉验证（人工）:\n"
            "  1) 把【左爪】插头插到【右腕】末端口，再跑本脚本看 B/E：\n"
            "     - 有反馈 → 右腕口 CAN OK，原右爪或线坏\n"
            "     - 仍 0 帧 → 右腕 Terminal CAN / 控制器右通道问题\n"
            "  2) 右爪有 24V 但本脚本 E 仍 0 帧 → 量 CAN H/L，别只量电源"
        )
    finally:
        # 尽量失能扫过的口
        for term, cid in ((ARM0, 1), (ARM1, 2), (ARM0, 2)):
            tx(term, pack(cid, bytes([0xFF] * 7 + [0xFD])))
        sdk.FX_L1_System_Unlink()
        print("\nunlink")
    return 0


if __name__ == "__main__":
    sys.exit(main())
