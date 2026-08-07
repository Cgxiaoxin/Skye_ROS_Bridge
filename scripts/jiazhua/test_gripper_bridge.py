#!/usr/bin/env python3
"""夹爪桥接单元测试 — 编码/解码、数据结构, 不连接硬件."""

import sys, os
_current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_current_dir, "..", ".."))

from robot.gripper_bridge import (
    encode_mit, decode_feedback, pack_terminal, unpack_terminal,
    GripperBridge, float_to_uint, uint_to_float,
)
from robot.robot_adapter import GripperConfig, GripperControlConfig

print("=" * 60)
print("GripperBridge 单元测试")
print("=" * 60)

# ── 1. 编解码 ──
mit = encode_mit(3.0, 0.12, 1.5, 0.0, 0.0)
assert len(mit) == 8, f"MIT 帧应为 8 字节, 实际 {len(mit)}"
print(f"✅ encode_mit → {mit.hex()} (8 bytes)")

# 边界值
mit_min = encode_mit(0.0, 0.0, -12.5, -30.0, -10.0)
mit_max = encode_mit(500.0, 5.0, 12.5, 30.0, 10.0)
assert len(mit_min) == len(mit_max) == 8
print("✅ encode_mit 边界值 OK")

# ── 2. pack / unpack ──
can_id, data = 0x01, mit
payload = pack_terminal(can_id, data)
assert len(payload) == 12, f"payload 应为 12 字节, 实际 {len(payload)}"
cid, d = unpack_terminal(payload)
assert cid == can_id and d == data, f"roundtrip 失败: {cid} {d.hex()}"
print(f"✅ pack/unpack roundtrip → id=0x{cid:X} data={d.hex()}")

# ── 3. 反馈解码 ──
fb_data = b'\x10\x12\x34\x56\x78\x9A\x20\x30'
fb = decode_feedback(fb_data)
assert "pos" in fb and "vel" in fb and "torque" in fb
assert fb["err_code"] == 1 and fb["err_msg"] == "On"
assert fb["temp_mos"] == 0x20 and fb["temp_motor"] == 0x30
print(f"✅ decode_feedback → pos={fb['pos']:.4f} vel={fb['vel']:.4f} "
      f"tau={fb['torque']:.4f} err={fb['err_msg']}")

# ── 4. 浮点编解码精度 ──
for val in (0.0, 1.5, -3.2, 12.0, -11.8):
    u = float_to_uint(val, -12.5, 12.5, 16)
    v = uint_to_float(u, -12.5, 12.5, 16)
    assert abs(val - v) < 0.01, f"精度不足: {val} → {v}"
print("✅ float_to_uint/uint_to_float 精度 < 0.01")

# ── 5. Dataclass 默认值 ──
gc = GripperConfig()
assert gc.enabled and gc.arm == "AB"
gcc = GripperControlConfig()
assert gcc.kp == 3.0 and gcc.kd == 0.12 and gcc.rate_hz == 100.0
assert gcc.pos_min == 0.0 and gcc.pos_max == 1.6
print(f"✅ GripperConfig: {gc}")
print(f"✅ GripperControlConfig: {gcc}")

# ── 6. GripperBridge 接口 (不启动, 不连硬件) ──
api = [m for m in dir(GripperBridge) if not m.startswith("_")]
assert "set_left" in api and "set_right" in api
assert "start" in api and "stop" in api
assert "feedback_left" in api and "feedback_right" in api
print(f"✅ GripperBridge API: {[m for m in api if not m.isupper()]}")

print()
print("=" * 60)
print("全部通过")
print("=" * 60)
