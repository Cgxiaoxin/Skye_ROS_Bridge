"""ROS2 publisher for /skye/policy_action (HITL DAgger口 A)."""

from __future__ import annotations

from typing import Any, Optional

import numpy as np

from remote_inference_async.dagger_chunk import policy_chunk_to_arbiter_fields


class DaggerPolicyPublisher:
    """Lazy-init ROS node so unit tests can import without rclpy spun up."""

    def __init__(
        self,
        *,
        topic: str = "/skye/policy_action",
        policy_version: str = "unijepa",
        node_name: str = "unijepa_dagger_policy",
    ) -> None:
        self.topic = topic
        self.policy_version = policy_version
        self.node_name = node_name
        self._node = None
        self._pub = None

    def start(self) -> None:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from skye_hitl_dagger.msg import PolicyActionChunk

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(self.node_name)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self._node.create_publisher(PolicyActionChunk, self.topic, qos)
        self._msg_cls = PolicyActionChunk

    def publish_policy_actions(
        self,
        policy_actions: np.ndarray,
        *,
        dt: float,
        policy_version: Optional[str] = None,
    ) -> dict[str, Any]:
        if self._pub is None or self._node is None:
            raise RuntimeError("DaggerPolicyPublisher.start() was not called")
        fields = policy_chunk_to_arbiter_fields(policy_actions, dt=dt)
        msg = self._msg_cls()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.policy_version = policy_version or self.policy_version
        msg.chunk_size = int(fields["chunk_size"])
        msg.dt = float(fields["dt"])
        msg.left_joints = fields["left_joints"]
        msg.right_joints = fields["right_joints"]
        msg.left_gripper = fields["left_gripper"]
        msg.right_gripper = fields["right_gripper"]
        self._pub.publish(msg)
        return fields

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        if self._node is None:
            return
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
            self._pub = None
