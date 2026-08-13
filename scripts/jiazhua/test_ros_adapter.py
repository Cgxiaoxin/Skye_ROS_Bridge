#!/usr/bin/env python3
"""ros_adapter 单元测试 — GripperROSBridge 订阅/回调/shutdown, 不连接硬件."""

import sys, os, threading
_current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_current_dir, "..", ".."))

import rclpy
from std_msgs.msg import Float32

from robot.ros_adapter import GripperROSBridge, _get_node, _shared_node, \
    GRIPPER_CMD_L, GRIPPER_CMD_R

# 重置共享节点状态
import robot.ros_adapter as ra
ra._shared_node = None

print("=" * 60)
print("ros_adapter 测试")
print("=" * 60)


class MockBridge:
    """模拟 GripperBridge — 记录 set_left/set_right 调用."""
    def __init__(self):
        self.left_vals = []
        self.right_vals = []
        self.feedback_left = {}
        self.feedback_right = {}
        self._started = True

    def set_left(self, value):
        self.left_vals.append(value)

    def set_right(self, value):
        self.right_vals.append(value)


# ── 1. arm="A" → 订阅 left, 调用 set_left ──
rclpy.init()
mb_a = MockBridge()
ros_a = GripperROSBridge(mb_a, arm="A")
assert len(ros_a._subs) == 1
print(f"✅ arm=A → {len(ros_a._subs)} sub(s)")

# 发布测试消息
node = _get_node()
pub_l = node.create_publisher(Float32, GRIPPER_CMD_L, 10)

# spin 等回调
deadline = rclpy.clock.Clock().now() + rclpy.time.Duration(seconds=1)
while rclpy.clock.Clock().now() < deadline:
    pub_l.publish(Float32(data=0.42))
    rclpy.spin_once(node, timeout_sec=0.01)

ros_a.shutdown()
node.destroy_publisher(pub_l)
assert len(mb_a.left_vals) > 0, "未收到 left 回调"
assert abs(mb_a.left_vals[-1] - 0.42) < 0.01, \
    f"期望 0.42, 实际 {mb_a.left_vals[-1]}"
assert len(mb_a.right_vals) == 0, "right 不应收到回调"
print(f"✅ arm=A 回调: left={mb_a.left_vals[-1]:.2f}, right 无")

# ── 2. arm="B" → 订阅 right ──
mb_b = MockBridge()
ros_b = GripperROSBridge(mb_b, arm="B")
assert len(ros_b._subs) == 1

pub_r = node.create_publisher(Float32, GRIPPER_CMD_R, 10)
deadline = rclpy.clock.Clock().now() + rclpy.time.Duration(seconds=1)
while rclpy.clock.Clock().now() < deadline:
    pub_r.publish(Float32(data=0.73))
    rclpy.spin_once(node, timeout_sec=0.01)

ros_b.shutdown()
node.destroy_publisher(pub_r)
assert len(mb_b.right_vals) > 0
assert abs(mb_b.right_vals[-1] - 0.73) < 0.01
assert len(mb_b.left_vals) == 0
print(f"✅ arm=B 回调: right={mb_b.right_vals[-1]}, left 无")

# ── 3. arm="AB" → 两个都订阅 ──
mb_ab = MockBridge()
ros_ab = GripperROSBridge(mb_ab, arm="AB")
assert len(ros_ab._subs) == 2

pl = node.create_publisher(Float32, GRIPPER_CMD_L, 10)
pr = node.create_publisher(Float32, GRIPPER_CMD_R, 10)
deadline = rclpy.clock.Clock().now() + rclpy.time.Duration(seconds=1)
while rclpy.clock.Clock().now() < deadline:
    pl.publish(Float32(data=0.1))
    pr.publish(Float32(data=0.9))
    rclpy.spin_once(node, timeout_sec=0.01)

ros_ab.shutdown()
node.destroy_publisher(pl)
node.destroy_publisher(pr)
assert len(mb_ab.left_vals) > 0 and len(mb_ab.right_vals) > 0
assert abs(mb_ab.left_vals[-1] - 0.1) < 0.01
assert abs(mb_ab.right_vals[-1] - 0.9) < 0.01
print(f"✅ arm=AB 回调: left={mb_ab.left_vals[-1]}, right={mb_ab.right_vals[-1]}")

# ── 4. 无效 arm → 不订阅, 不崩溃 ──
mb_inv = MockBridge()
ros_inv = GripperROSBridge(mb_inv, arm="X")
assert len(ros_inv._subs) == 0
ros_inv.shutdown()
print("✅ arm=X → 0 订阅, 无崩溃")

# ── 5. shutdown 后订阅被清理 ──
mb_cl = MockBridge()
ros_cl = GripperROSBridge(mb_cl, arm="A")
assert len(ros_cl._subs) == 1
ros_cl.shutdown()
assert len(ros_cl._subs) == 0
print("✅ shutdown → 订阅已清理")

# ── 6. bridge 属性 ──
mb_p = MockBridge()
ros_p = GripperROSBridge(mb_p, arm="A")
assert ros_p.bridge is mb_p
ros_p.shutdown()
print("✅ bridge 属性返回正确实例")

# ── 清理 ──
node.destroy_node()
rclpy.shutdown()

print()
print("=" * 60)
print("全部通过")
print("=" * 60)
