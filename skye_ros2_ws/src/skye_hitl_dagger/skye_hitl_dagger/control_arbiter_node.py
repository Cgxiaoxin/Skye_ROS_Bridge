#!/usr/bin/env python3
"""ROS 2 source arbiter for policy chunks and FACTR teleoperation."""

from __future__ import annotations

import json
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
from skye_hitl_dagger.sync_retry import DualArmSyncTracker, SyncRetryPolicy
from skye_hitl_dagger.teleop_sync import SyncPhase, TeleopHandshake

JOINTS_PER_ARM = 7
DEFAULT_JOINT_ORDER = list(range(JOINTS_PER_ARM))
DEFAULT_JOINT_SIGNS = [1.0] * JOINTS_PER_ARM
MODE_COMMANDS = ("switch_sync", "switch_teleop", "switch_stop")
# 默认同步重试间隔时间，单位：秒
DEFAULT_SYNC_BACKOFF_S = [5.0, 10.0, 20.0]


def policy_gripper_value(value: float, invert: bool) -> float:
    """Convert policy motor-space gripper values for the driver input."""
    return 1.0 - value if invert else value


class ControlArbiterNode(Node):
    def __init__(self) -> None:
        super().__init__("control_arbiter")
        self.declare_parameter("gripper_invert_on_driver", True)
        self.declare_parameter("sync_timeout_s", 5.0)
        self.declare_parameter("min_sync_hold_s", 2.0)
        # One takeover → one switch_sync by default; operator re-issues takeover
        # to retry. Set true only if you want timed software republish.
        self.declare_parameter("sync_auto_republish", False)
        self.declare_parameter("sync_backoff_s", DEFAULT_SYNC_BACKOFF_S)
        self.declare_parameter("sync_max_attempts", 3)
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
        self._sync_auto_republish = bool(
            self.get_parameter("sync_auto_republish").value)
        backoff_raw = self.get_parameter("sync_backoff_s").value
        self._sync_backoff_s = [
            float(v) for v in (backoff_raw or DEFAULT_SYNC_BACKOFF_S)]
        self._sync_max_attempts = max(
            1, int(self.get_parameter("sync_max_attempts").value))
        if not self._sync_auto_republish:
            # Initial takeover publish still counts as attempt 1; keep policy
            # constructed but never drive timed republish / exhaust-fault.
            self._sync_max_attempts = 1
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
        self._sync_retry = SyncRetryPolicy(
            backoff_s=self._sync_backoff_s,
            max_attempts=self._sync_max_attempts)
        self._arm_sync = DualArmSyncTracker()
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
        self._hold_ack_pending = False
        self._enter_teleop_after_hold = False
        self._sync_hints_logged = False

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
        # AUTONOMOUS only → absolute path (follower-space policy).
        # HANDOVER_SYNC must not stream abs (leaders sync read-only to follower).
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
            String, "/mode/switch_stop", self._switch_stop_callback, 10)
        self.create_subscription(
            JointState, joint_states_topic, self._joint_states_callback,
            state_qos)
        teleop_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/teleop/state", self._state_callback, teleop_state_qos)
        progress_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for side in DualArmSyncTracker.SIDES:
            self.create_subscription(
                String, f"/leader_arm/{side}/sync_progress",
                lambda msg, s=side: self._sync_progress_callback(s, msg),
                progress_qos)

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
            self._hold_ack_pending = False
            self._enter_teleop_after_hold = False
            self._sync_hints_logged = False
            self._sync_started = time.monotonic()
            sampled = self._player.sample(self._now_seconds())
            if sampled is not None:
                self._last_target = sampled
            # Freeze follower once; do not stream abs hold during sync (that
            # nudges /gento and can push targets past leader safe bounds).
            self._hold_target = None
            self._player.clear()
            self._arm_sync.reset()
            self._sync_retry.reset()
            self._call_hold_current(require_ack_before_teleop=False)
            self._publish_mode_command("switch_sync")
            self._sync_retry.note_sync_published(time.monotonic())
            self._handshake.start_sync()
            self._publish_mode()
            republish = (
                "auto_republish=on" if self._sync_auto_republish
                else "auto_republish=off (re-takeover to retry)")
            self.get_logger().info(
                "event=takeover state=HANDOVER_SYNC cmd=switch_sync; "
                f"{republish}; follower quiet (hold_current); "
                "waiting for SYNCED then enter_teleop")
        elif msg.data == "enter_teleop" and self._logic.request_enter_teleop():
            if not self._handshake.aligned_ready():
                self.get_logger().warning(
                    "enter_teleop before SYNCED hold complete; will switch_"
                    "teleop once alignment is ready and hold_current acks")
            self._begin_enter_teleop()
            self._publish_mode()
        elif msg.data == "return" and self._logic.request_return():
            self._cancel_handover(publish_command=self._return_command)
        elif msg.data == "stop":
            self._handle_stop(source="intervention_cmd")

    def _switch_stop_callback(self, msg: String) -> None:
        if msg.data != "switch_stop":
            return
        self._handle_stop(source="mode/switch_stop")

    def _handle_stop(self, *, source: str) -> None:
        """Full-semantic stop: arbiter → AUTONOMOUS + leaders switch_stop."""
        already_idle = (
            self._logic.mode() == ControlModeState.AUTONOMOUS
            and self._handshake.phase() is SyncPhase.IDLE
            and self._sync_started is None
            and not self._enter_teleop_after_hold
            and not self._hold_ack_pending)
        self._logic.request_stop()
        if already_idle:
            # Avoid publish↔subscribe echo loops on /mode/switch_stop.
            self.get_logger().info(
                f"event=stop source={source} state=AUTONOMOUS (noop)")
            return
        self._cancel_handover(publish_command="switch_stop")
        self.get_logger().info(
            f"event=stop source={source} state=AUTONOMOUS "
            "cmd=switch_stop (idempotent)")

    def _cancel_handover(self, *, publish_command: str) -> None:
        self._abs_hold_after_teleop_request = False
        self._hold_ack_pending = False
        self._enter_teleop_after_hold = False
        self._sync_hints_logged = False
        self._sync_started = None
        self._handshake.reset()
        self._arm_sync.reset()
        self._sync_retry.reset()
        self._player.clear()
        self._return_time = self._now_seconds()
        self._return_wall_time = time.monotonic()
        self._chunk_fallback_warned = False
        feedback = self._feedback_target() if self._feedback_is_recent() else None
        self._hold_target = feedback
        self._last_target = feedback
        self._publish_mode_command(publish_command)
        self._publish_mode()

    def _begin_enter_teleop(self) -> None:
        """Hold pose with ack, then request FACTR teleop (explicit handshake)."""
        self._abs_hold_after_teleop_request = True
        self._enter_teleop_after_hold = True
        self._sync_started = time.monotonic()
        self._call_hold_current(require_ack_before_teleop=True)

    def _publish_switch_teleop_if_ready(self) -> None:
        if self._hold_ack_pending or not self._enter_teleop_after_hold:
            return
        if not self._logic.teleop_requested():
            return
        if not self._handshake.aligned_ready():
            self.get_logger().info(
                "enter_teleop: hold_current acked; waiting for SYNCED before "
                "switch_teleop")
            return
        if self._handshake.pending_command() == "switch_teleop":
            return
        self._publish_mode_command("switch_teleop")
        self._handshake.start_teleop()
        self._sync_started = time.monotonic()
        self.get_logger().info(
            "event=enter_teleop state=HANDOVER_SYNC cmd=switch_teleop "
            "after hold_current ack")

    def _call_hold_current(self, *, require_ack_before_teleop: bool) -> None:
        """Clear abs/relative streaming and freeze arms at current pose."""
        if not self._hold_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning(
                "hold_current service unavailable; continuing without reset")
            self._hold_ack_pending = False
            if require_ack_before_teleop:
                self._publish_switch_teleop_if_ready()
            return
        self._hold_ack_pending = True
        future = self._hold_client.call_async(Trigger.Request())

        def _done(fut) -> None:
            self._hold_ack_pending = False
            try:
                result = fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"hold_current failed: {exc}")
                if require_ack_before_teleop:
                    self._publish_switch_teleop_if_ready()
                return
            if result is None or not result.success:
                msg = getattr(result, "message", "")
                self.get_logger().warning(f"hold_current rejected: {msg}")
            else:
                self.get_logger().info(
                    "hold_current ok; relative session reset (ack)")
            if require_ack_before_teleop:
                self._publish_switch_teleop_if_ready()

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
        previous_ready = self._handshake.aligned_ready()
        self._handshake.on_state(msg.data)
        if (not previous_ready and self._handshake.aligned_ready()
                and self._logic.mode() == ControlModeState.HANDOVER_SYNC):
            self.get_logger().info(
                f"event=sync_ok state={self._handshake.state()} "
                "aligned_ready=1; stop switch_sync republish")
            self._log_arm_sync_hints()
            if self._logic.teleop_requested():
                self._publish_switch_teleop_if_ready()

    def _sync_progress_callback(self, side: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        outcome = payload.get("outcome")
        text = payload.get("text") or ""
        classified = self._arm_sync.on_progress(side, outcome, text)
        self.get_logger().info(
            f"event=sync_progress side={side} outcome={classified.name} "
            f"text={text[:160]}",
            throttle_duration_sec=1.0)
        if (self._logic.mode() == ControlModeState.HANDOVER_SYNC
                and self._arm_sync.all_deterministic_failed()):
            self._enter_sync_fault(
                "both arms rejected sync (deterministic); "
                + "; ".join(self._arm_sync.operator_hints()))

    def _log_arm_sync_hints(self) -> None:
        for hint in self._arm_sync.operator_hints():
            self.get_logger().error(
                f"sync arm fault (non-blocking for sibling): {hint}")

    def _enter_sync_fault(self, reason: str) -> None:
        if not self._logic.report_sync_fault(reason):
            return
        self._abs_hold_after_teleop_request = False
        self._hold_ack_pending = False
        self._enter_teleop_after_hold = False
        self._sync_started = None
        self._handshake.reset()
        self._sync_retry.reset()
        self._hold_target = None
        self._publish_mode()
        self.get_logger().error(f"event=sync_fail state=FAULT reason={reason}")

    def _sync_timer_callback(self) -> None:
        if self._logic.mode() != ControlModeState.HANDOVER_SYNC:
            return

        now = time.monotonic()
        aligned = self._handshake.aligned_ready()
        dual_terminal = self._arm_sync.dual_terminal()

        # Phase A: sync only — never auto-enter teleop.
        if not self._logic.teleop_requested():
            if aligned or dual_terminal:
                if dual_terminal and not aligned and not self._sync_hints_logged:
                    self._log_arm_sync_hints()
                    self._sync_hints_logged = True
                return
            if not self._sync_auto_republish:
                # Wait for SYNCED / deterministic dual failure / operator
                # re-takeover; do not re-fire switch_sync or exhaust→FAULT.
                return
            if self._sync_retry.exhausted():
                self._enter_sync_fault(
                    "sync republish exhausted without SYNCED; "
                    + "; ".join(self._arm_sync.operator_hints()
                                or ["check leader bounds / domain"]))
                return
            if self._sync_retry.should_republish_sync(
                    now, aligned_ready=False, dual_terminal=False):
                self.get_logger().warning(
                    f"event=sync_retry attempt={self._sync_retry.attempts + 1} "
                    f"state={self._handshake.state()} cmd=switch_sync "
                    f"backoff_s={self._sync_retry.next_backoff_s()}")
                self._publish_mode_command("switch_sync")
                self._sync_retry.note_sync_published(now)
                self._sync_started = now
            return

        # Phase B: operator requested enter_teleop.
        if self._handshake.teleop_ready():
            self._complete_sync()
            return
        self._publish_switch_teleop_if_ready()
        if (self._sync_started is not None
                and now - self._sync_started >= self._sync_timeout):
            pending = self._handshake.pending_command()
            if pending == "switch_teleop" and not self._hold_ack_pending:
                self.get_logger().warning(
                    f"event=teleop_retry state={self._handshake.state()} "
                    f"cmd={pending}")
                self._publish_mode_command(pending)
                self._sync_started = now

    def _complete_sync(self) -> None:
        if self._logic.sync_completed():
            self._abs_hold_after_teleop_request = False
            self._enter_teleop_after_hold = False
            self._hold_ack_pending = False
            self._handshake.reset()
            self._sync_retry.reset()
            self._sync_started = None
            self._publish_mode()
            self.get_logger().info(
                "event=enter_teleop_ok state=HUMAN source=teleop; "
                "relative forwarding on (driver seeds on first/cached frame)")

    def _joint_timer_callback(self) -> None:
        # Sync/teleop/fault: no abs writes to /gento (leaders read follower only).
        if not self._logic.should_publish_policy_abs():
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
        if not self._logic.should_publish_policy_abs():
            return
        if self._return_time is not None:
            return
        sampled = self._player.sample(self._now_seconds())
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
