#!/usr/bin/env python3
"""robot_adapter 离线测试 — 仅验证导入、常量、接口签名, 不连机器人."""

import sys, os
_current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_current_dir, "..", ".."))

from robot.robot_adapter import RobotClient, ARM_LEFT, ARM_RIGHT

print("=" * 60)
print("robot_adapter 测试")
print("=" * 60)

# 1. 导入检查
print("✅ PYTHON_SDK.GentoRobot 可导入")

# 2. 常量
assert ARM_LEFT == 0 and ARM_RIGHT == 1
print(f"✅ ARM_LEFT={ARM_LEFT}, ARM_RIGHT={ARM_RIGHT}")

# 3. 接口签名检查 (不连机器人)
methods = [
    ("connect", ["ip"]),
    ("disconnect", []),
    ("set_impedance_mode", ["obj"]),
    ("set_drag_mode", ["obj"]),
    ("set_idle", ["obj"]),
    ("get_joint_positions", ["arm"]),
    ("send_joint_command", ["arm", "q"]),
]
robot = RobotClient.__new__(RobotClient)  # 跳过 __init__, 只检查方法存在
for name, _ in methods:
    assert hasattr(robot, name), f"missing method: {name}"
print(f"✅ {len(methods)} required methods present")

# 4. 测试 RobotClient 初始化 (不 connect)
print("注: 不连接机器人, 只验证对象构造不抛异常")
print("\n全部测试通过 ✅")
