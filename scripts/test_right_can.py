#!/usr/bin/env python3
"""左右夹爪 Terminal CAN 对照：ARM0=左 / ARM1=右，目标 0.5 → 1.0 → 0.5。

须先停 skye_robot_driver（同一控制器只能一个 SDK link）。
"""
import ctypes
import os
import sys
import time

SDK = "/data/coding/tianji/Skye_ROS_Bridge/third_party/gento_sdk/lib/x86_64/libGentoSDK.so"
IP = (6, 6, 7, 190)
ARM0, ARM1, CANFD = 0, 1, 1
MOTOR_ID = 1
TARGETS = (0.5, 1.0, 0.5)  # 0=开 1=闭
HOLD_S = 2.0
POS_MAX = 1.6
KP, KD = 3.0, 0.12
QMAX, DQMAX, TAUMAX = 12.5, 30.0, 10.0

ARMS = (
    ("L", ARM0),
    ("R", ARM1),
)


def clamp(x, lo, hi):
    return lo if x <= lo else hi if x > hi else x


def f2u(x, xmin, xmax, bits):
    x = clamp(x, xmin, xmax)
    return int((x - xmin) / (xmax - xmin) * ((1 << bits) - 1))


def u2f(x, vmin, vmax, bits):
    return x / ((1 << bits) - 1) * (vmax - vmin) + vmin


def encode_mit(kp, kd, q, dq=0.0, tau=0.0):
    kp_u, kd_u = f2u(kp, 0, 500, 12), f2u(kd, 0, 5, 12)
    q_u = f2u(q, -QMAX, QMAX, 16)
    dq_u = f2u(dq, -DQMAX, DQMAX, 12)
    tau_u = f2u(tau, -TAUMAX, TAUMAX, 12)
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
    if len(d) < 8:
        return None
    err = (d[0] & 0xF0) >> 4
    pos = u2f((d[1] << 8) | d[2], -QMAX, QMAX, 16)
    vel = u2f((d[3] << 4) | (d[4] >> 4), -DQMAX, DQMAX, 12)
    tau = u2f(((d[4] & 0xF) << 8) | d[5], -TAUMAX, TAUMAX, 12)
    return cid, pos, vel, tau, err, d[6], d[7]


def main():
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

    def tx(arm, raw12):
        buf = (ctypes.c_ubyte * 64)(*raw12)
        t = ctypes.c_uint(0)
        return sdk.FX_L1_Terminal_SetData(
            arm, CANFD, 100, buf, 12, ctypes.byref(t)
        )

    def rx(arm, timeout_ms=20):
        chn = ctypes.c_int(0)
        buf = (ctypes.c_ubyte * 64)()
        t = ctypes.c_uint(0)
        n = sdk.FX_L1_Terminal_GetData(
            arm, timeout_ms, ctypes.byref(chn), buf, ctypes.byref(t)
        )
        if n < 5:
            return None
        return bytes(buf[:n])

    ret = sdk.FX_L1_System_Link(*IP, 2)
    if ret < 0:
        print(f"link 失败 {ret}。先停 skye_robot_driver: pkill -f skye_robot_driver")
        return 1
    print(f"link ok delay={ret}ms  id={MOTOR_ID}  targets={list(TARGETS)}")

    enable = pack(MOTOR_ID, bytes([0xFF] * 7 + [0xFC]))
    disable = pack(MOTOR_ID, bytes([0xFF] * 7 + [0xFD]))

    try:
        for name, arm in ARMS:
            sdk.FX_L1_Terminal_ClearData(arm)
        for i in range(3):
            codes = [f"{name}={tx(arm, enable)}" for name, arm in ARMS]
            print(f"enable #{i + 1} → " + " ".join(codes) + "  (0=SDK已发出)")
            time.sleep(0.05)

        totals = {"L": 0, "R": 0}
        for norm in TARGETS:
            mit = pack(MOTOR_ID, encode_mit(KP, KD, norm * POS_MAX))
            hits = {"L": 0, "R": 0}
            last = {"L": None, "R": None}
            print(f"\n=== target {norm} ({HOLD_S:.0f}s) 看左右爪是否跟上 ===")
            t0 = time.time()
            while time.time() - t0 < HOLD_S:
                for name, arm in ARMS:
                    tx(arm, mit)
                    fb = decode_fb(rx(arm, 10))
                    if fb:
                        hits[name] += 1
                        last[name] = fb
                time.sleep(0.01)
            for name, _arm in ARMS:
                totals[name] += hits[name]
                fb = last[name]
                if fb:
                    cid, pos, vel, tau, err, tmos, tmot = fb
                    print(
                        f"  {name}: {hits[name]}帧  can=0x{cid:02X} "
                        f"pos={pos:.3f}rad norm={pos / POS_MAX:.2f} "
                        f"vel={vel:.3f} err={err} mos={tmos}C"
                    )
                else:
                    print(f"  {name}: 0帧  无CAN反馈（未上电/CAN不通/未使能）")

        print("\n--- 汇总 ---")
        for name, _arm in ARMS:
            n = totals[name]
            if n:
                print(f"{name} 硬件 OK：共 {n} 帧反馈")
            else:
                print(f"{name} 硬件异常：全程 0 帧。查该臂 24V / CAN H/L / 末端板")
        print("对照：左能动、右 0 帧 = 右爪供电或右腕 CAN，不是双爪互斥。")

        for name, arm in ARMS:
            tx(arm, disable)
    finally:
        sdk.FX_L1_System_Unlink()
        print("unlink")
    return 0


if __name__ == "__main__":
    sys.exit(main())
