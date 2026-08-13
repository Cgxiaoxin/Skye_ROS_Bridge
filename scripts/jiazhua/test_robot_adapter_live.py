#!/usr/bin/env python3
"""robot_adapter 在线测试 — 连接机器人, 验证读写和模式切换.

使用:
  python test_robot_adapter_live.py                  # 使用默认 IP
  python test_robot_adapter_live.py --ip 6.6.7.190   # 指定 IP
"""

import sys, os, time, argparse
_current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_current_dir, "..", ".."))

from robot.robot_adapter import RobotClient, ARM_LEFT

p = argparse.ArgumentParser()
p.add_argument("--ip", default="6.6.7.190", help="机器人 IP (默认 6.6.7.190)")
args = p.parse_args()

print("=" * 60)
print("robot_adapter 在线测试")
print(f"  目标 IP: {args.ip}")
print("=" * 60)

robot = RobotClient()
# 连接
print("\n1. 连接...")
robot.connect(args.ip)
print(f"   SDK={robot.get_sdk_version()}  Controller={robot.get_controller_version()}")

# 读关节
print("\n2. 读取关节角...")
q = robot.get_joint_positions("A")
print(f"   关节角: {[round(x,2) for x in q]}")
assert len(q) == 7

# 状态
print(f"   当前状态: {robot.check_arm_state('A')}")

# 阻抗模式
print("\n3. 进入阻抗模式...")
robot.set_impedance_mode(ARM_LEFT)
time.sleep(0.3)

# 发送零关节指令 (仅验证调用不报错)
print("\n4. 发送关节指令 (保持当前位置)...")
robot.send_joint_command("A", q)
time.sleep(0.3)
q2 = robot.get_joint_positions("A")
print(f"   error: {[round(abs(q[i]-q2[i]),2) for i in range(7)]}")

# 空闲
print("\n5. 回到空闲...")
robot.set_idle(ARM_LEFT)

print("\n全部测试通过 ✅")
robot.disconnect()
