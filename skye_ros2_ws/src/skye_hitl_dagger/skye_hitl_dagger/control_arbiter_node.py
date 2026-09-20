#!/usr/bin/env python3
"""ROS 2 source arbiter for policy chunks and FACTR teleoperation."""

from __future__ import annotations

import time
from typing import Optional, Sequence

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
from std_srvs.srv import Trigger

from skye_hitl_dagger.chunk_player import ChunkPlayer
from skye_hitl_dagger.chunk_continuity import (
    chunk_is_fresh,
    max_step0_jump_rad,
    rebase_arm_joints_to_pose,
)
from skye_hitl_dagger.control_mode import ControlArbiterLogic, ControlModeState
from skye_hitl_dagger.msg import ControlMode, PolicyActionChunk
from skye_hitl_dagger.policy_abs import follower_pose_to_leader_abs
from skye_hitl_dagger.teleop_sync import TeleopHandshake

JOINTS_PER_ARM = 7
DEFAULT_JOINT_ORDER = list(range(JOINTS_PER_ARM))
DEFAULT_JOINT_SIGNS = [1.0] * JOINTS_PER_ARM
MODE_COMMANDS = ("switch_sync", "switch_teleop", "switch_stop")


def policy_gripper_value(value: float, invert: bool) -> float:
    """Convert policy motor-space gripper values for the driver input."""
    return 1.0 - value if invert else value


class ControlArbiterNode(Node):
    def __init__(self) -> None:
        super().__init__("control_arbiter")
        self.declare_parameter("gripper_invert_on_driver", True)
        self.declare_parameter("sync_timeout_s", 5.0)
        self.declare_parameter("min_sync_hold_s", 2.0)
        self.declare_parameter("chunk_start_max_jump_rad", 0.1)
        self.declare_parameter("chunk_freshness_fallback_s", 2.0)
        self.declare_parameter("feedback_stale_s", 0.2)
        self.declare_parameter("gripper_rate_hz", 100.0)
        self.declare_parameter("mode_publish_hz", 5.0)
        self.declare_parameter("return_mode_command", "switch_sync")
        self.declare_parameter("joint_states_topic", "/gento/joint_states")
        self.declare_parameter("hold_current_service", "/gento/hold_current")
        self.declare_parameter("left_joint_order", DEFAULT_JOINT_ORDER)
        self.declare_parameter("right_joint_order", DEFAULT_JOINT_ORDER)
        self.declare_parameter("left_joint_signs", DEFAULT_JOINT_SIGNS)
        self.declare_parameter("right_joint_signs", DEFAULT_JOINT_SIGNS)
        self.declare_parameter(
            "left_joint_offsets", [0.0] * JOINTS_PER_ARM)
        self.declare_parameter(
            "right_joint_offsets", [0.0] * JOINTS_PER_ARM)

        self._invert_gripper = bool(
            self.get_parameter("gripper_invert_on_driver").value)
        self._sync_timeout = float(self.get_parameter("sync_timeout_s").value)
        self._min_sync_hold = max(
            0.0, float(self.get_parameter("min_sync_hold_s").value))
        self._chunk_start_max_jump = max(
            0.0, float(self.get_parameter("chunk_start_max_jump_rad").value))
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
        hold_service = str(self.get_parameter("hold_current_service").value)
        self._left_order = list(self.get_parameter("left_joint_order").value)
        self._right_order = list(self.get_parameter("right_joint_order").value)
        self._left_signs = [
            float(v) for v in self.get_parameter("left_joint_signs").value]
        self._right_signs = [
            float(v) for v in self.get_parameter("right_joint_signs").value]
        self._left_offsets = [
            float(v) for v in self.get_parameter("left_joint_offsets").value]
        self._right_offsets = [
            float(v) for v in self.get_parameter("right_joint_offsets").value]

        self._logic = ControlArbiterLogic()
        self._player = ChunkPlayer()
        self._handshake = TeleopHandshake(min_sync_hold_s=self._min_sync_hold)
        self._policy_version = ""
        self._last_target: Optional[dict] = None
        self._hold_target: Optional[dict] = None
        self._sync_started: Optional[float] = None
        self._return_time: Optional[float] = None
        self._return_wall_time: Optional[float] = None
        self._chunk_fallback_warned = False
        self._hold_tail_warned = False
        self._joint_feedback: Optional[dict] = None
        self._feedback_received_at: Optional[float] = None
        self._abs_hold_after_teleop_request = False

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

        # HUMAN teleop → relative path (FACTR leader angles).
        self._left_rel_pub = self.create_publisher(
            JointState, "/gento/left_joint_control", cmd_qos)
        self._right_rel_pub = self.create_publisher(
            JointState, "/gento/right_joint_control", cmd_qos)
        # AUTONOMOUS / HANDOVER hold → absolute path (follower-space policy).
        self._left_abs_pub = self.create_publisher(
            JointState, "/gento/left_joint_control_abs", cmd_qos)
        self._right_abs_pub = self.create_publisher(
            JointState, "/gento/right_joint_control_abs", cmd_qos)
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
        self._hold_client = self.create_client(Trigger, hold_service)

        self.create_subscription(
            PolicyActionChunk, "/skye/policy_action", self._policy_callback,
            cmd_qos)
        self.create_subscription(
            JointState, "/skye/teleop_action_left",
            lambda msg: self._teleop_joint_callback(msg, self._left_rel_pub),
            cmd_qos)
        self.create_subscription(
            JointState, "/skye/teleop_action_right",
            lambda msg: self._teleop_joint_callback(msg, self._right_rel_pub),
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
        left_joints = list(msg.left_joints)
        right_joints = list(msg.right_joints)
        if self._feedback_is_recent() and self._joint_feedback is not None:
            jump = max_step0_jump_rad(
                left_joints, right_joints,
                self._joint_feedback["left"], self._joint_feedback["right"])
            if jump > self._chunk_start_max_jump:
                left_joints = rebase_arm_joints_to_pose(
                    left_joints, self._joint_feedback["left"], msg.chunk_size)
                right_joints = rebase_arm_joints_to_pose(
                    right_joints, self._joint_feedback["right"], msg.chunk_size)
                self.get_logger().warning(
                    f"policy chunk step0 jump {jump:.3f} rad > "
                    f"{self._chunk_start_max_jump:.3f}; rebased to current "
                    "follower pose to avoid discontinuity")
        if not self._player.load(
                msg.chunk_size, msg.dt, t0, left_joints, right_joints,
                msg.left_gripper, msg.right_gripper):
            self.get_logger().error("rejecting invalid policy action chunk")
            return
        self._return_time = None
        self._return_wall_time = None
        self._chunk_fallback_warned = False
        self._hold_tail_warned = False
        self._hold_target = None
        self._policy_version = msg.policy_version

    def _intervention_callback(self, msg: String) -> None:
        if msg.data == "takeover" and self._logic.request_takeover():
            self._abs_hold_after_teleop_request = False
            self._sync_started = time.monotonic()
            sampled = self._player.sample(self._now_seconds())
            if sampled is not None:
                self._last_target = sampled
            self._hold_target = self._last_target or self._feedback_target()
            self._player.clear()
            self._publish_mode_command("switch_sync")
            self._handshake.start_sync()
            self._publish_mode()
            self.get_logger().info(
                "takeover: holding via abs + switch_sync; waiting for "
                "enter_teleop after SYNCED")
        elif msg.data == "enter_teleop" and self._logic.request_enter_teleop():
            if not self._handshake.aligned_ready():
                # Allow request but keep waiting; sync timer will fire teleop
                # once min_sync_hold is satisfied. If never aligned, timeout
                # republishes switch_sync until operator returns.
                self.get_logger().warning(
                    "enter_teleop before SYNCED hold complete; will switch_"
                    "teleop once alignment is ready")
            self._begin_enter_teleop()
            self._publish_mode()
        elif msg.data == "return" and self._logic.request_return():
            self._abs_hold_after_teleop_request = False
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

    def _begin_enter_teleop(self) -> None:
        """Stop abs stream, hold pose, request FACTR teleop."""
        self._call_hold_current()
        self._abs_hold_after_teleop_request = True
        self._sync_started = time.monotonic()
        if self._handshake.aligned_ready():
            self._publish_mode_command("switch_teleop")
            self._handshake.start_teleop()
            self.get_logger().info(
                "enter_teleop: hold_current done; switch_teleop published")
        else:
            self.get_logger().info(
                "enter_teleop: hold_current done; waiting for SYNCED before "
                "switch_teleop")

    def _call_hold_current(self) -> None:
        """Clear abs/relative streaming and freeze arms at current pose."""
        if not self._hold_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning(
                "hold_current service unavailable; continuing without reset")
            return
        future = self._hold_client.call_async(Trigger.Request())
        # Best-effort: do not block the executor; fire-and-forget with log.
        def _done(fut) -> None:
            try:
                result = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"hold_current failed: {exc}")
                return
            if result is None or not result.success:
                msg = getattr(result, "message", "")
                self.get_logger().warning(
                    f"hold_current rejected: {msg}")
            else:
                self.get_logger().info("hold_current ok; relative session reset")

        future.add_done_callback(_done)

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

        # Phase A: sync only — never auto-enter teleop.
        if not self._logic.teleop_requested():
            if (self._sync_started is not None
                    and time.monotonic() - self._sync_started
                    >= self._sync_timeout):
                pending = self._handshake.pending_command()
                if pending == "switch_sync":
                    self.get_logger().warning(
                        f"sync timeout in state {self._handshake.state()}; "
                        "re-publishing switch_sync")
                    self._publish_mode_command("switch_sync")
                    self._sync_started = time.monotonic()
            return

        # Phase B: operator requested enter_teleop.
        if self._handshake.teleop_ready():
            self._complete_sync()
            return
        if (self._handshake.aligned_ready()
                and self._handshake.pending_command() != "switch_teleop"):
            # Aligned after enter_teleop was pressed early — fire teleop now.
            self._publish_mode_command("switch_teleop")
            self._handshake.start_teleop()
            self._sync_started = time.monotonic()
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
            self._abs_hold_after_teleop_request = False
            self._handshake.reset()
            self._sync_started = None
            self._publish_mode()
            self.get_logger().info(
                "HUMAN: relative teleop forwarding enabled "
                "(driver will reseed on first frame)")

    def _joint_timer_callback(self) -> None:
        mode = self._logic.mode()
        if mode == ControlModeState.HUMAN:
            return
        # After enter_teleop + hold_current: do not fight relative reseed with abs.
        if (mode == ControlModeState.HANDOVER_SYNC
                and self._abs_hold_after_teleop_request):
            return
        if mode == ControlModeState.HANDOVER_SYNC:
            if self._hold_target is not None:
                self._publish_policy_abs(
                    self._hold_target["left"], self._hold_target["right"])
            return
        sampled = self._player.sample(self._now_seconds())
        if self._return_time is not None and self._hold_target is None:
            return
        if sampled is not None:
            self._last_target = sampled
            self._publish_policy_abs(sampled["left"], sampled["right"])
            if sampled["holding_tail"]:
                if not self._hold_tail_warned:
                    self.get_logger().warning(
                        "policy chunk ended; holding final joint target")
                    self._hold_tail_warned = True
            else:
                self._hold_tail_warned = False
        elif self._last_target is not None:
            self._publish_policy_abs(
                self._last_target["left"], self._last_target["right"])

    def _gripper_timer_callback(self) -> None:
        mode = self._logic.mode()
        if mode == ControlModeState.HUMAN:
            return
        if mode == ControlModeState.AUTONOMOUS and self._return_time is not None:
            return
        if (mode == ControlModeState.HANDOVER_SYNC
                and self._abs_hold_after_teleop_request):
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

    def _publish_policy_abs(
            self, left: Sequence[float], right: Sequence[float]) -> None:
        """Publish follower-space targets on abs topics (inverse-signed)."""
        try:
            left_cmd = follower_pose_to_leader_abs(
                left, self._left_signs, self._left_order, self._left_offsets)
            right_cmd = follower_pose_to_leader_abs(
                right, self._right_signs, self._right_order, self._right_offsets)
        except ValueError as exc:
            self.get_logger().error(f"policy abs mapping failed: {exc}")
            return
        stamp = self.get_clock().now().to_msg()
        self._left_abs_pub.publish(self._joint_message(left_cmd, stamp))
        self._right_abs_pub.publish(self._joint_message(right_cmd, stamp))

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
