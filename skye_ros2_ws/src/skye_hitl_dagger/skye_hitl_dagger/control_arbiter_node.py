#!/usr/bin/env python3
"""ROS 2 source arbiter for policy chunks and FACTR teleoperation."""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from skye_hitl_dagger.chunk_player import ChunkPlayer
from skye_hitl_dagger.control_mode import ControlArbiterLogic, ControlModeState
from skye_hitl_dagger.msg import ControlMode, PolicyActionChunk
from skye_hitl_dagger.policy_relative import PolicyRelativeSession
from skye_hitl_dagger.teleop_sync import TeleopHandshake

JOINTS_PER_ARM = 7
DEFAULT_JOINT_ORDER = list(range(JOINTS_PER_ARM))
DEFAULT_JOINT_SIGNS = [1.0] * JOINTS_PER_ARM
MODE_COMMANDS = ("switch_sync", "switch_teleop", "switch_stop")


def policy_gripper_value(value: float, invert: bool) -> float:
    """Convert policy motor-space gripper values for the driver input."""
    return 1.0 - value if invert else value


def chunk_is_fresh(
        stamp_s: float, return_time: Optional[float],
        receive_time: Optional[float] = None,
        return_wall_time: Optional[float] = None,
        now_wall_time: Optional[float] = None,
        fallback_after_s: float = 2.0) -> bool:
    """Accept post-return chunks by stamp, then fall back to receive time."""
    if return_time is None:
        return True
    if stamp_s > 0.0 and stamp_s >= return_time:
        return True
    if stamp_s == 0.0 and receive_time is not None:
        return (return_wall_time is None
                or receive_time >= return_wall_time)
    return (return_wall_time is not None and now_wall_time is not None
            and now_wall_time - return_wall_time >= fallback_after_s)


class ControlArbiterNode(Node):
    def __init__(self) -> None:
        super().__init__("control_arbiter")
        self.declare_parameter("gripper_invert_on_driver", True)
        self.declare_parameter("sync_timeout_s", 5.0)
        self.declare_parameter("chunk_stale_warn_s", 1.5)
        self.declare_parameter("chunk_freshness_fallback_s", 2.0)
        self.declare_parameter("feedback_stale_s", 0.2)
        self.declare_parameter("gripper_rate_hz", 100.0)
        self.declare_parameter("mode_publish_hz", 5.0)
        self.declare_parameter("return_mode_command", "switch_sync")
        self.declare_parameter("joint_states_topic", "/gento/joint_states")
        self.declare_parameter("left_joint_order", DEFAULT_JOINT_ORDER)
        self.declare_parameter("right_joint_order", DEFAULT_JOINT_ORDER)
        self.declare_parameter("left_joint_signs", DEFAULT_JOINT_SIGNS)
        self.declare_parameter("right_joint_signs", DEFAULT_JOINT_SIGNS)

        self._invert_gripper = bool(
            self.get_parameter("gripper_invert_on_driver").value)
        self._sync_timeout = float(self.get_parameter("sync_timeout_s").value)
        self._stale_warn = float(
            self.get_parameter("chunk_stale_warn_s").value)
        self._freshness_fallback = float(
            self.get_parameter("chunk_freshness_fallback_s").value)
        self._feedback_stale = float(
            self.get_parameter("feedback_stale_s").value)
        gripper_rate = max(1.0, float(
            self.get_parameter("gripper_rate_hz").value))
        mode_rate = max(0.1, float(self.get_parameter("mode_publish_hz").value))
        self._return_command = str(
            self.get_parameter("return_mode_command").value)
        if self._return_command not in ("switch_sync", "switch_stop"):
            self.get_logger().warning(
                f"return_mode_command={self._return_command} unsupported; "
                "falling back to switch_sync")
            self._return_command = "switch_sync"
        joint_states_topic = str(
            self.get_parameter("joint_states_topic").value)
        left_order = list(self.get_parameter("left_joint_order").value)
        right_order = list(self.get_parameter("right_joint_order").value)
        left_signs = [float(v) for v in self.get_parameter("left_joint_signs").value]
        right_signs = [float(v) for v in self.get_parameter("right_joint_signs").value]

        self._logic = ControlArbiterLogic()
        self._left_policy = PolicyRelativeSession(
            signs=left_signs, joint_order=left_order)
        self._right_policy = PolicyRelativeSession(
            signs=right_signs, joint_order=right_order)
        self._policy_session_dirty = True
        self._player = ChunkPlayer()
        self._handshake = TeleopHandshake()
        self._policy_version = ""
        self._last_target: Optional[dict] = None
        self._hold_target: Optional[dict] = None
        self._sync_started: Optional[float] = None
        self._return_time: Optional[float] = None
        self._return_wall_time: Optional[float] = None
        self._chunk_fallback_warned = False
        self._last_stale_warning = 0.0
        self._joint_feedback: Optional[dict] = None
        self._feedback_received_at: Optional[float] = None

        # Match the driver command QoS: latest sample only, no retransmission.
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self._left_pub = self.create_publisher(
            JointState, "/gento/left_joint_control", cmd_qos)
        self._right_pub = self.create_publisher(
            JointState, "/gento/right_joint_control", cmd_qos)
        self._left_gripper_pub = self.create_publisher(
            JointState, "/left_teleop_gripper/ctrl", cmd_qos)
        self._right_gripper_pub = self.create_publisher(
            JointState, "/right_teleop_gripper/ctrl", cmd_qos)
        self._mode_command_pubs = {
            name: self.create_publisher(String, f"/mode/{name}", 10)
            for name in MODE_COMMANDS
        }
        self._mode_pub = self.create_publisher(
            ControlMode, "/skye/control_mode", mode_qos)

        self.create_subscription(
            PolicyActionChunk, "/skye/policy_action", self._policy_callback,
            cmd_qos)
        self.create_subscription(
            JointState, "/skye/teleop_action_left",
            lambda msg: self._teleop_joint_callback(msg, self._left_pub),
            cmd_qos)
        self.create_subscription(
            JointState, "/skye/teleop_action_right",
            lambda msg: self._teleop_joint_callback(msg, self._right_pub),
            cmd_qos)
        self.create_subscription(
            JointState, "/skye/teleop_gripper_left",
            lambda msg: self._teleop_gripper_callback(
                msg, self._left_gripper_pub), cmd_qos)
        self.create_subscription(
            JointState, "/skye/teleop_gripper_right",
            lambda msg: self._teleop_gripper_callback(
                msg, self._right_gripper_pub), cmd_qos)
        self.create_subscription(
            String, "/skye/intervention_cmd", self._intervention_callback, 10)
        self.create_subscription(
            JointState, joint_states_topic, self._joint_states_callback,
            state_qos)
        teleop_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/teleop/state", self._state_callback, teleop_state_qos)

        self.create_timer(1.0 / 250.0, self._joint_timer_callback)
        self.create_timer(1.0 / gripper_rate, self._gripper_timer_callback)
        self.create_timer(0.05, self._sync_timer_callback)
        self.create_timer(1.0 / mode_rate, self._publish_mode)
        self._publish_mode()

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _policy_callback(self, msg: PolicyActionChunk) -> None:
        if self._logic.mode() != ControlModeState.AUTONOMOUS:
            self._policy_version = msg.policy_version
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        received_at = time.monotonic()
        fresh_by_stamp = self._return_time is None or (
            stamp > 0.0 and stamp >= self._return_time)
        if not chunk_is_fresh(
                stamp, self._return_time, received_at, self._return_wall_time,
                received_at, self._freshness_fallback):
            self.get_logger().warning(
                "discarding policy chunk stamped before the last return to "
                "AUTONOMOUS; holding pose until a fresh chunk arrives",
                throttle_duration_sec=1.0)
            return
        if not fresh_by_stamp and not self._chunk_fallback_warned:
            self.get_logger().warning(
                "no fresh stamped policy chunk after return; "
                "using receive time for freshness")
            self._chunk_fallback_warned = True
        t0 = stamp if fresh_by_stamp else self._now_seconds()
        if not self._player.load(
                msg.chunk_size, msg.dt, t0, msg.left_joints, msg.right_joints,
                msg.left_gripper, msg.right_gripper):
            self.get_logger().error("rejecting invalid policy action chunk")
            return
        self._return_time = None
        self._return_wall_time = None
        self._chunk_fallback_warned = False
        self._policy_version = msg.policy_version

    def _intervention_callback(self, msg: String) -> None:
        if msg.data == "takeover" and self._logic.request_takeover():
            self._invalidate_policy_sessions()
            self._sync_started = time.monotonic()
            sampled = self._player.sample(self._now_seconds())
            if sampled is not None:
                self._last_target = sampled
            self._hold_target = self._last_target or self._feedback_target()
            self._player.clear()
            self._publish_mode_command("switch_sync")
            self._handshake.start_sync()
            self._publish_mode()
        elif msg.data == "return" and self._logic.request_return():
            self._invalidate_policy_sessions()
            self._sync_started = None
            self._handshake.reset()
            self._player.clear()
            self._return_time = self._now_seconds()
            self._return_wall_time = time.monotonic()
            self._chunk_fallback_warned = False
            feedback = self._feedback_target() if self._feedback_is_recent() else None
            self._hold_target = feedback
            self._last_target = feedback
            self._publish_mode_command(self._return_command)
            self._publish_mode()

    def _joint_states_callback(self, msg: JointState) -> None:
        positions = list(msg.position)
        if len(positions) < 2 * JOINTS_PER_ARM:
            return
        self._joint_feedback = {
            "left": [float(v) for v in positions[:JOINTS_PER_ARM]],
            "right": [float(v) for v in
                      positions[JOINTS_PER_ARM:2 * JOINTS_PER_ARM]],
        }
        self._feedback_received_at = time.monotonic()
        if (self._logic.mode() == ControlModeState.AUTONOMOUS
                and self._return_time is not None):
            self._hold_target = self._feedback_target()
            self._last_target = self._hold_target

    def _feedback_is_recent(self) -> bool:
        return (self._feedback_received_at is not None
                and time.monotonic() - self._feedback_received_at
                <= self._feedback_stale)

    def _feedback_target(self) -> Optional[dict]:
        """Build a hold target from the follower feedback pose."""
        if self._joint_feedback is None:
            return None
        previous = self._last_target or {}
        return {
            "left": list(self._joint_feedback["left"]),
            "right": list(self._joint_feedback["right"]),
            "left_gripper": previous.get("left_gripper", 0.0),
            "right_gripper": previous.get("right_gripper", 0.0),
            "holding_tail": True,
        }

    def _state_callback(self, msg: String) -> None:
        self._handshake.on_state(msg.data)

    def _sync_timer_callback(self) -> None:
        if self._logic.mode() != ControlModeState.HANDOVER_SYNC:
            return
        if self._handshake.aligned_ready():
            self._sync_started = time.monotonic()
            self._publish_mode_command("switch_teleop")
            self._handshake.start_teleop()
            return
        if self._handshake.teleop_ready():
            self._complete_sync()
            return
        if (self._sync_started is not None
              and time.monotonic() - self._sync_started >= self._sync_timeout):
            pending = self._handshake.pending_command()
            self.get_logger().warning(
                f"teleop sync timeout in state {self._handshake.state()}; "
                f"re-publishing {pending}")
            if pending is not None:
                self._publish_mode_command(pending)
            self._sync_started = time.monotonic()

    def _complete_sync(self) -> None:
        if self._logic.sync_completed():
            self._invalidate_policy_sessions()
            self._handshake.reset()
            self._sync_started = None
            self._publish_mode()

    def _invalidate_policy_sessions(self) -> None:
        self._left_policy.invalidate()
        self._right_policy.invalidate()
        self._policy_session_dirty = True

    def _ensure_policy_sessions(self) -> bool:
        if not self._policy_session_dirty:
            return self._left_policy.active and self._right_policy.active
        if self._joint_feedback is None:
            return False
        self._left_policy.begin(self._joint_feedback["left"])
        self._right_policy.begin(self._joint_feedback["right"])
        self._policy_session_dirty = False
        return True

    def _joint_timer_callback(self) -> None:
        mode = self._logic.mode()
        if mode == ControlModeState.HUMAN:
            return
        if mode == ControlModeState.HANDOVER_SYNC:
            if self._hold_target is not None:
                self._publish_policy_relative(
                    self._hold_target["left"], self._hold_target["right"])
            return
        sampled = self._player.sample(self._now_seconds())
        if self._return_time is not None and self._hold_target is None:
            return
        if sampled is not None:
            self._last_target = sampled
            self._publish_policy_relative(sampled["left"], sampled["right"])
            if sampled["holding_tail"]:
                now = time.monotonic()
                if now - self._last_stale_warning >= self._stale_warn:
                    self.get_logger().warning(
                        "policy chunk ended; holding final joint target")
                    self._last_stale_warning = now
        elif self._last_target is not None:
            self._publish_policy_relative(
                self._last_target["left"], self._last_target["right"])

    def _gripper_timer_callback(self) -> None:
        mode = self._logic.mode()
        if mode == ControlModeState.HUMAN:
            return
        if mode == ControlModeState.AUTONOMOUS and self._return_time is not None:
            # Human owned the gripper until the return; do not fight it before
            # the policy sends its first post-return chunk.
            return
        sampled = (self._hold_target if mode == ControlModeState.HANDOVER_SYNC
                   else self._player.sample(self._now_seconds()))
        if sampled is None:
            sampled = self._last_target
        if sampled is None or "left_gripper" not in sampled:
            return
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

    def _publish_policy_relative(self, left, right) -> None:
        if not self._ensure_policy_sessions():
            return
        left_leader = self._left_policy.follower_target_to_leader(left)
        right_leader = self._right_policy.follower_target_to_leader(right)
        self._left_policy.commit_published(left_leader)
        self._right_policy.commit_published(right_leader)
        stamp = self.get_clock().now().to_msg()
        self._left_pub.publish(self._joint_message(left_leader, stamp))
        self._right_pub.publish(self._joint_message(right_leader, stamp))

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

    def _publish_mode_command(self, value: str) -> None:
        publisher = self._mode_command_pubs.get(value)
        if publisher is None:
            self.get_logger().error(f"unknown mode command {value}")
            return
        publisher.publish(String(data=value))

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
