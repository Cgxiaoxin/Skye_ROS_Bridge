#!/usr/bin/env python3
"""Skeleton: turn a follower-space absolute trajectory into /skye/policy_action.

Wire this from remote_inference_async AFTER you disable CONTROLLER_IP streaming.
Requires: source skye_ros2_ws/install/setup.bash (PolicyActionChunk).

Action layout expected by control_arbiter:
  left_joints  = [step0_j1..j7, step1_j1..j7, ...]   len = chunk_size * 7
  right_joints = same
  left/right_gripper len = chunk_size
  joints are ABSOLUTE rad in /gento/joint_states space (not leader, not relative).
"""

from __future__ import annotations

from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from skye_hitl_dagger.msg import PolicyActionChunk

JOINTS_PER_ARM = 7


def flatten_arm_traj(steps: Sequence[Sequence[float]]) -> list[float]:
    out: list[float] = []
    for step in steps:
        if len(step) < JOINTS_PER_ARM:
            raise ValueError(f"each arm step needs {JOINTS_PER_ARM} joints, got {len(step)}")
        out.extend(float(v) for v in step[:JOINTS_PER_ARM])
    return out


class PolicyChunkBridge(Node):
    def __init__(self) -> None:
        super().__init__("policy_chunk_bridge")
        self.declare_parameter("policy_version", "unijepa")
        self.declare_parameter("dt", 1.0 / 30.0)
        self._policy_version = str(self.get_parameter("policy_version").value)
        self._dt = float(self.get_parameter("dt").value)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(PolicyActionChunk, "/skye/policy_action", qos)

    def publish_chunk(
        self,
        left_steps: Sequence[Sequence[float]],
        right_steps: Sequence[Sequence[float]],
        left_gripper: Optional[Sequence[float]] = None,
        right_gripper: Optional[Sequence[float]] = None,
        *,
        policy_version: Optional[str] = None,
        dt: Optional[float] = None,
    ) -> None:
        n = len(left_steps)
        if n == 0 or n != len(right_steps):
            raise ValueError("left/right step counts must match and be > 0")
        lg = list(left_gripper) if left_gripper is not None else [0.0] * n
        rg = list(right_gripper) if right_gripper is not None else [0.0] * n
        if len(lg) != n or len(rg) != n:
            raise ValueError("gripper length must equal chunk_size")

        msg = PolicyActionChunk()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.policy_version = policy_version or self._policy_version
        msg.chunk_size = n
        msg.dt = float(self._dt if dt is None else dt)
        msg.left_joints = flatten_arm_traj(left_steps)
        msg.right_joints = flatten_arm_traj(right_steps)
        msg.left_gripper = [float(v) for v in lg]
        msg.right_gripper = [float(v) for v in rg]
        self._pub.publish(msg)


def main() -> None:
    """Smoke: publishes a zero hold chunk once (needs joint_states elsewhere for real motion)."""
    rclpy.init()
    node = PolicyChunkBridge()
    zeros = [[0.0] * JOINTS_PER_ARM for _ in range(16)]
    node.publish_chunk(zeros, zeros)
    node.get_logger().info("published one zero PolicyActionChunk to /skye/policy_action")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
