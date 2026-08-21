#!/usr/bin/env python3
"""Publish hold-pose PolicyActionChunk for bench tests without a real VLA."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from skye_hitl_dagger.msg import PolicyActionChunk

CHUNK_SIZE = 16
JOINTS_PER_ARM = 7


class DummyPolicyChunkPublisher(Node):
    def __init__(self) -> None:
        super().__init__("pub_dummy_policy_chunk")
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("policy_version", "dummy_hold")
        self.declare_parameter("joint_states_topic", "/gento/joint_states")
        self.declare_parameter("allow_zero_fallback", False)

        self._dt = float(self.get_parameter("dt").value)
        self._policy_version = str(self.get_parameter("policy_version").value)
        self._joint_topic = str(self.get_parameter("joint_states_topic").value)
        self._allow_zero_fallback = bool(
            self.get_parameter("allow_zero_fallback").value)
        self._left = [0.0] * JOINTS_PER_ARM
        self._right = [0.0] * JOINTS_PER_ARM
        self._has_valid_joint_states = False
        self._latest_js: Optional[JointState] = None

        best_effort = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(
            PolicyActionChunk, "/skye/policy_action", best_effort)
        self.create_subscription(
            JointState, self._joint_topic, self._joint_callback, reliable)
        self.create_timer(self._dt, self._publish_chunk)
        if self._allow_zero_fallback:
            self.get_logger().warn(
                "allow_zero_fallback=true: publishing zero hold pose until "
                f"valid joint_states arrive on {self._joint_topic}"
            )
        else:
            self.get_logger().info(
                f"Waiting for >=14 joint positions on {self._joint_topic} "
                f"before publishing hold chunks every {self._dt:.3f}s"
            )

    def _joint_callback(self, msg: JointState) -> None:
        self._latest_js = msg
        positions = list(msg.position)
        if len(positions) >= 14:
            self._left = [float(v) for v in positions[:JOINTS_PER_ARM]]
            self._right = [float(v) for v in positions[JOINTS_PER_ARM:14]]
            if not self._has_valid_joint_states:
                self._has_valid_joint_states = True
                self.get_logger().info(
                    "Received valid joint_states; starting policy chunk publish"
                )

    def _publish_chunk(self) -> None:
        if not self._has_valid_joint_states and not self._allow_zero_fallback:
            return

        msg = PolicyActionChunk()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.policy_version = self._policy_version
        msg.chunk_size = CHUNK_SIZE
        msg.dt = self._dt
        msg.left_joints = self._left * CHUNK_SIZE
        msg.right_joints = self._right * CHUNK_SIZE
        msg.left_gripper = [0.0] * CHUNK_SIZE
        msg.right_gripper = [0.0] * CHUNK_SIZE
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DummyPolicyChunkPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
