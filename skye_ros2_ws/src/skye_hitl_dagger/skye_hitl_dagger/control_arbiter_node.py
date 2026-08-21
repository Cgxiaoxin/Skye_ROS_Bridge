#!/usr/bin/env python3
"""ROS 2 source arbiter for policy chunks and FACTR teleoperation."""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from skye_hitl_dagger.chunk_player import ChunkPlayer
from skye_hitl_dagger.control_mode import ControlArbiterLogic, ControlModeState
from skye_hitl_dagger.msg import ControlMode, PolicyActionChunk


def policy_gripper_value(value: float, invert: bool) -> float:
    """Convert policy motor-space gripper values for the driver input."""
    return 1.0 - value if invert else value


class ControlArbiterNode(Node):
    def __init__(self) -> None:
        super().__init__("control_arbiter")
        self.declare_parameter("gripper_invert_on_driver", True)
        self.declare_parameter("sync_timeout_s", 5.0)
        self.declare_parameter("chunk_stale_warn_s", 1.5)
        self.declare_parameter("gripper_rate_hz", 100.0)

        self._invert_gripper = bool(
            self.get_parameter("gripper_invert_on_driver").value)
        self._sync_timeout = float(self.get_parameter("sync_timeout_s").value)
        self._stale_warn = float(
            self.get_parameter("chunk_stale_warn_s").value)
        gripper_rate = max(1.0, float(
            self.get_parameter("gripper_rate_hz").value))

        self._logic = ControlArbiterLogic()
        self._player = ChunkPlayer()
        self._policy_version = ""
        self._last_target: Optional[dict] = None
        self._sync_started: Optional[float] = None
        self._last_stale_warning = 0.0
        self._teleop_state = ""

        abs_qos = QoSProfile(depth=10)
        best_effort = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self._left_abs_pub = self.create_publisher(
            JointState, "/gento/left_joint_control_abs", abs_qos)
        self._right_abs_pub = self.create_publisher(
            JointState, "/gento/right_joint_control_abs", abs_qos)
        self._left_pub = self.create_publisher(
            JointState, "/gento/left_joint_control", abs_qos)
        self._right_pub = self.create_publisher(
            JointState, "/gento/right_joint_control", abs_qos)
        self._left_gripper_pub = self.create_publisher(
            JointState, "/left_teleop_gripper/ctrl", abs_qos)
        self._right_gripper_pub = self.create_publisher(
            JointState, "/right_teleop_gripper/ctrl", abs_qos)
        self._sync_pub = self.create_publisher(String, "/mode/switch_sync", 10)
        self._teleop_pub = self.create_publisher(
            String, "/mode/switch_teleop", 10)
        self._mode_pub = self.create_publisher(ControlMode, "/skye/control_mode", 10)

        self.create_subscription(
            PolicyActionChunk, "/skye/policy_action", self._policy_callback,
            best_effort)
        self.create_subscription(
            JointState, "/skye/teleop_action_left",
            lambda msg: self._teleop_joint_callback(msg, self._left_pub), 10)
        self.create_subscription(
            JointState, "/skye/teleop_action_right",
            lambda msg: self._teleop_joint_callback(msg, self._right_pub), 10)
        self.create_subscription(
            JointState, "/skye/teleop_gripper_left",
            lambda msg: self._teleop_gripper_callback(
                msg, self._left_gripper_pub), 10)
        self.create_subscription(
            JointState, "/skye/teleop_gripper_right",
            lambda msg: self._teleop_gripper_callback(
                msg, self._right_gripper_pub), 10)
        self.create_subscription(
            String, "/skye/intervention_cmd", self._intervention_callback, 10)
        self.create_subscription(String, "/teleop/state", self._state_callback, 10)

        self.create_timer(1.0 / 250.0, self._joint_timer_callback)
        self.create_timer(1.0 / gripper_rate, self._gripper_timer_callback)
        self.create_timer(0.05, self._sync_timer_callback)
        self._publish_mode()

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _policy_callback(self, msg: PolicyActionChunk) -> None:
        t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t0 == 0.0:
            t0 = self._now_seconds()
        if not self._player.load(
                msg.chunk_size, msg.dt, t0, msg.left_joints, msg.right_joints,
                msg.left_gripper, msg.right_gripper):
            self.get_logger().error("rejecting invalid policy action chunk")
            return
        self._policy_version = msg.policy_version

    def _intervention_callback(self, msg: String) -> None:
        if msg.data == "takeover" and self._logic.request_takeover():
            self._sync_started = time.monotonic()
            sampled = self._player.sample(self._now_seconds())
            if sampled is not None:
                self._last_target = sampled
            self._publish_sync("sync")
            self._publish_mode()
        elif msg.data == "return" and self._logic.request_return():
            self._sync_started = None
            self._publish_teleop("teleop")
            self._publish_mode()

    def _state_callback(self, msg: String) -> None:
        self._teleop_state = msg.data
        if (self._logic.mode() == ControlModeState.HANDOVER_SYNC
                and msg.data == "TELEOP"):
            self._complete_sync()

    def _sync_timer_callback(self) -> None:
        if self._logic.mode() != ControlModeState.HANDOVER_SYNC:
            return
        if self._teleop_state == "TELEOP":
            self._complete_sync()
        elif (self._sync_started is not None
              and time.monotonic() - self._sync_started >= self._sync_timeout):
            self.get_logger().warning(
                "teleop sync timeout; remaining in HANDOVER_SYNC")
            self._sync_started = time.monotonic()

    def _complete_sync(self) -> None:
        if self._logic.sync_completed():
            self._publish_teleop("teleop")
            self._publish_mode()
            self._sync_started = None

    def _joint_timer_callback(self) -> None:
        if self._logic.mode() == ControlModeState.HUMAN:
            return
        sampled = self._player.sample(self._now_seconds())
        if sampled is not None:
            self._last_target = sampled
            self._publish_abs(sampled["left"], sampled["right"])
            if sampled["holding_tail"]:
                now = time.monotonic()
                if now - self._last_stale_warning >= self._stale_warn:
                    self.get_logger().warning(
                        "policy chunk ended; holding final joint target")
                    self._last_stale_warning = now
        elif self._last_target is not None:
            self._publish_abs(
                self._last_target["left"], self._last_target["right"])

    def _gripper_timer_callback(self) -> None:
        if self._logic.mode() == ControlModeState.HUMAN:
            return
        sampled = self._player.sample(self._now_seconds())
        if sampled is None:
            sampled = self._last_target
        if sampled is not None:
            self._publish_gripper(
                policy_gripper_value(sampled["left_gripper"], self._invert_gripper),
                policy_gripper_value(
                    sampled["right_gripper"], self._invert_gripper))

    def _teleop_joint_callback(self, msg: JointState, publisher) -> None:
        if self._logic.mode() == ControlModeState.HUMAN:
            publisher.publish(msg)

    def _teleop_gripper_callback(self, msg: JointState, publisher) -> None:
        if self._logic.mode() == ControlModeState.HUMAN:
            publisher.publish(msg)

    def _publish_abs(self, left, right) -> None:
        stamp = self.get_clock().now().to_msg()
        self._left_abs_pub.publish(self._joint_message(left, stamp))
        self._right_abs_pub.publish(self._joint_message(right, stamp))

    def _publish_gripper(self, left: float, right: float) -> None:
        stamp = self.get_clock().now().to_msg()
        self._left_gripper_pub.publish(self._joint_message([left], stamp))
        self._right_gripper_pub.publish(self._joint_message([right], stamp))

    @staticmethod
    def _joint_message(position, stamp) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.position = list(position)
        return msg

    def _publish_sync(self, value: str) -> None:
        self._sync_pub.publish(String(data=value))

    def _publish_teleop(self, value: str) -> None:
        self._teleop_pub.publish(String(data=value))

    def _publish_mode(self) -> None:
        mode = ControlMode()
        mode.header.stamp = self.get_clock().now().to_msg()
        mode.mode = self._logic.mode().name
        mode.source = self._logic.active_source()
        mode.policy_version = self._policy_version
        self._mode_pub.publish(mode)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
