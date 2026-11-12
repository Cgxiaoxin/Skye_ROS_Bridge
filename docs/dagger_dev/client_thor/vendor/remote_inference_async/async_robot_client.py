#!/usr/bin/env python3
"""Asynchronous Tianji robot client with LeRobot-style action aggregation."""

from __future__ import annotations

import argparse
import logging
import math
import os
import queue
import signal
import sys
import threading
import time
import uuid
from pathlib import Path

import numpy as np
import zmq

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from remote_inference_async.protocol import (  # noqa: E402
    ACTION_DIM,
    PROTOCOL_VERSION,
    STATE_DIM,
    float32_bytes,
    json_bytes,
    parse_float32,
    parse_json,
)
from remote_inference_async.action_history import ExecutedActionHistory  # noqa: E402
from remote_inference_async.runtime_control import (  # noqa: E402
    DEFAULT_RUNTIME_CONTROL_SOCKET,
    RuntimeControlServer,
)
from remote_inference_async.trajectory_recorder import (  # noqa: E402
    DEFAULT_CONTROL_SOCKET,
    DEFAULT_OUTPUT_DIR,
    DEFAULT_STATUS_PATH,
    RealtimeTrajectoryRecorder,
)
from remote_inference_async.motion_filter import (  # noqa: E402
    JointCommandLimiter,
    TrackingErrorGuard,
)
from remote_inference_async.units import (  # noqa: E402
    ACTION_GRIPPER_UNIT,
    STATE_GRIPPER_UNIT,
    POLICY_ARM_UNIT,
    policy_actions_to_robot,
    robot_state_to_policy,
)

LOGGER = logging.getLogger("tianji_async_client")
GRIPPER_COMMAND_MAX = 0.9


DEFAULT_ROS_CAMERA_TOPICS = {
    "head_color": "/head_d435/camera/color/image_raw",
    "wrist_left": "/left_wrist_d405/camera/color/image_raw",
    "wrist_right": "/right_wrist_d405/camera/color/image_raw",
}


def _right_vector_payload(values: np.ndarray) -> dict[str, object]:
    """Record the complete bimanual state/action vector."""

    vector = np.asarray(values, dtype=np.float64)
    if vector.shape != (ACTION_DIM,):
        raise ValueError(f"trajectory vector must have shape ({ACTION_DIM},), got {vector.shape}")
    return {
        "left_arm_joint_deg": vector[0:7].tolist(),
        "right_arm_joint_deg": vector[7:14].tolist(),
        "left_gripper_normalized": float(vector[14]),
        "right_gripper_normalized": float(vector[15]),
    }


def _right_chunk_payload(actions: np.ndarray) -> list[list[float]]:
    chunk = np.asarray(actions, dtype=np.float64)
    if chunk.ndim != 2 or chunk.shape[1] != ACTION_DIM:
        raise ValueError(f"trajectory action chunk must be [T,{ACTION_DIM}], got {chunk.shape}")
    return chunk.tolist()


def _arm_joint_labels(indices: tuple[int, ...]) -> list[str]:
    """Translate zero-based positions in the 14-DoF arm vector for logs."""

    labels: list[str] = []
    for index in indices:
        if 0 <= index < 7:
            labels.append(f"left:J{index + 1}")
        elif 7 <= index < 14:
            labels.append(f"right:J{index - 6}")
        else:
            labels.append(f"unknown:{index}")
    return labels


def _optional_float_list(value: object) -> list[float] | None:
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if array.ndim != 1:
        return None
    return array.tolist()


def _right_arm_rt_payload(rt: dict) -> dict[str, object]:
    """Copy already-fetched SDK feedback for both arms."""

    result: dict[str, object] = {"sdk_frame_serial": rt.get("frame_serial")}
    for arm_idx, label in ((0, "left"), (1, "right")):
        arm = rt["arms"][arm_idx]
        feedback = arm.get("fb", {})
        command = arm.get("cmd", {})
        result[label] = {
            "controller_state": dict(arm.get("state", {})),
            "sdk_input_joint_deg": _optional_float_list(command.get("joint_pos")),
            "sdk_input_joint_torque_nm": _optional_float_list(command.get("joint_trq")),
            "feedback_joint_deg": _optional_float_list(feedback.get("fb_pos")),
            "feedback_joint_velocity_deg_s": _optional_float_list(feedback.get("fb_vel")),
            "controller_command_joint_deg": _optional_float_list(feedback.get("cmd_pos")),
            "joint_sensor_torque_nm": _optional_float_list(feedback.get("fb_sensor")),
            "joint_external_torque_nm": _optional_float_list(feedback.get("fb_ext_torque")),
            "base_force_estimate": _optional_float_list(feedback.get("base_force")),
            "base_gyro": _optional_float_list(feedback.get("base_gyro")),
            "flange_force_sensor": _optional_float_list(feedback.get("flange_force")),
        }
    return result


class ActionBuffer:
    """Timestep-indexed action queue with native-LeRobot-style overlap aggregation."""

    def __init__(self, aggregate: str, ema_weight: float = 0.75) -> None:
        self.aggregate = aggregate
        self.ema_weight = ema_weight
        self._actions: dict[int, np.ndarray] = {}
        self.latest_executed = -1
        self._last_popped_timestep = -1
        self._last_popped_action: np.ndarray | None = None
        self.chunk_size = 1

    def __len__(self) -> int:
        return len(self._actions)

    def clear_pending(self) -> int:
        count = len(self._actions)
        self._actions.clear()
        return count

    def reset_phase(self, *, last_completed_timestep: int) -> int:
        """Drop queued/RTC state while preserving a monotonic sequence clock."""
        if last_completed_timestep < -1:
            raise ValueError("last completed timestep must be at least -1")
        count = self.clear_pending()
        self.latest_executed = int(last_completed_timestep)
        self._last_popped_timestep = -1
        self._last_popped_action = None
        self.chunk_size = 1
        return count

    def _combine(self, old: np.ndarray, new: np.ndarray) -> np.ndarray:
        if self.aggregate == "latest":
            return new
        if self.aggregate == "average":
            return (old + new) * 0.5
        if self.aggregate == "weighted_average":
            return old * (1.0 - self.ema_weight) + new * self.ema_weight
        if self.aggregate == "conservative":
            return old
        raise ValueError(f"unknown aggregation method {self.aggregate!r}")

    def merge(
        self,
        actions: np.ndarray,
        start_timestep: int,
        action_stride: int = 1,
    ) -> tuple[int, int]:
        actions = np.asarray(actions, dtype=np.float32)
        if actions.ndim != 2 or actions.shape[1] != ACTION_DIM:
            raise ValueError(f"action chunk must be [T,{ACTION_DIM}], got {actions.shape}")
        if action_stride <= 0:
            raise ValueError("action_stride must be positive")
        self.chunk_size = max(self.chunk_size, len(actions))
        accepted = stale = 0
        # Preserve the source offsets instead of compacting the sampled actions
        # onto adjacent logical timesteps.  A stride of 3 therefore consumes
        # source actions 0, 3, 6, ... and stale-reply rejection still compares
        # like-for-like model timesteps.  If network latency exhausts the sparse
        # queue, the main loop sends no new target and impedance control holds
        # the last published setpoint; it never rebases an old prediction.
        for offset in range(0, len(actions), action_stride):
            action = actions[offset]
            timestep = start_timestep + offset
            if timestep <= self.latest_executed:
                stale += 1
                continue
            old = self._actions.get(timestep)
            self._actions[timestep] = action.copy() if old is None else self._combine(old, action)
            accepted += 1
        return accepted, stale

    def pop_next(self) -> tuple[int, np.ndarray] | None:
        stale = [key for key in self._actions if key <= self.latest_executed]
        for key in stale:
            del self._actions[key]
        if not self._actions:
            return None
        timestep = min(self._actions)
        action = self._actions.pop(timestep)
        self.latest_executed = timestep
        self._last_popped_timestep = timestep
        self._last_popped_action = action.copy()
        return timestep, action

    def snapshot_rtc_prefix(
        self,
        *,
        action_horizon: int,
        max_prefix: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return contiguous actions already committed at the current anchor.

        Position zero is the action most recently removed for execution. Later
        positions are actions already queued for the immediately following
        timesteps. A timestep gap ends the valid prefix.
        """
        if action_horizon <= 0:
            raise ValueError("RTC action horizon must be positive")
        if max_prefix < 0 or max_prefix > action_horizon:
            raise ValueError("RTC max prefix must be in [0, action_horizon]")
        prefix = np.zeros((action_horizon, ACTION_DIM), dtype=np.float32)
        is_pad = np.ones(action_horizon, dtype=np.bool_)
        if (
            max_prefix == 0
            or self._last_popped_action is None
            or self._last_popped_timestep != self.latest_executed
        ):
            return prefix, is_pad

        prefix[0] = self._last_popped_action
        is_pad[0] = False
        for offset in range(1, max_prefix):
            action = self._actions.get(self.latest_executed + offset)
            if action is None:
                break
            prefix[offset] = action
            is_pad[offset] = False
        return prefix, is_pad


class AsyncDealer:
    """Own the DEALER socket in one thread; caller only exchanges Python frame lists."""

    def __init__(self, server: str, timeout_ms: int) -> None:
        self.server = server
        self.timeout_ms = timeout_ms
        self.outgoing: queue.Queue[list[bytes]] = queue.Queue(maxsize=1)
        self.incoming: queue.Queue[list[bytes] | BaseException] = queue.Queue()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="zmq-dealer", daemon=True)
        self.started = False

    def start(self) -> None:
        self.thread.start()
        self.started = True

    def send_latest(self, frames: list[bytes]) -> list[bytes] | None:
        try:
            replaced = self.outgoing.get_nowait()
        except queue.Empty:
            replaced = None
        self.outgoing.put_nowait(frames)
        return replaced

    def _run(self) -> None:
        context = zmq.Context()
        socket = context.socket(zmq.DEALER)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.IDENTITY, f"tianji-{uuid.uuid4()}".encode())
        socket.connect(self.server)
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        try:
            while not self.stop_event.is_set():
                newest: list[bytes] | None = None
                while True:
                    try:
                        newest = self.outgoing.get_nowait()
                    except queue.Empty:
                        break
                if newest is not None:
                    socket.send_multipart(newest)
                if socket in dict(poller.poll(timeout=5)):
                    self.incoming.put(socket.recv_multipart())
        except BaseException as exc:
            self.incoming.put(exc)
        finally:
            socket.close()
            context.term()

    def close(self) -> None:
        self.stop_event.set()
        if self.started:
            self.thread.join(timeout=max(1.0, self.timeout_ms / 1000))


def acknowledge_response(
    pending_requests: dict[str, float],
    request_id: str,
) -> tuple[float | None, int]:
    """Retire a response and any older requests it has superseded.

    The transport and policy worker are both latest-value queues.  Once a
    response for a newer request arrives, an older unacknowledged request can
    no longer be useful to the control loop; retaining it would let one lost
    ``observation_dropped`` reply trigger the watchdog even while newer action
    chunks continue to arrive.
    """
    responded_at = pending_requests.pop(request_id, None)
    if responded_at is None:
        return None, 0
    superseded = [
        pending_id
        for pending_id, sent_at in pending_requests.items()
        if sent_at <= responded_at
    ]
    for pending_id in superseded:
        pending_requests.pop(pending_id, None)
    return responded_at, len(superseded)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Tianji asynchronous robot client (shared by ACT and GR00T)"
    )
    parser.add_argument("--server", required=True, help="e.g. tcp://192.168.201.253:5556")
    parser.add_argument("--controller-ip", default="6.6.7.190")
    parser.add_argument(
        "--camera-source",
        choices=("opencv", "ros2"),
        default="ros2",
        help="Read /dev/video devices or ROS 2 camera topics",
    )
    parser.add_argument("--camera", default="0", help="OpenCV device number or URL")
    parser.add_argument("--camera-name", default="front")
    parser.add_argument(
        "--extra-camera",
        action="append",
        default=[],
        metavar="NAME=DEVICE",
        help="Repeatable additional camera, e.g. left_wrist=2",
    )
    parser.add_argument(
        "--ros-camera",
        action="append",
        default=[],
        metavar="NAME=TOPIC",
        help="Repeatable ROS 2 camera topic, e.g. head_color=/.../image_raw",
    )
    parser.add_argument(
        "--ros-camera-reliability",
        choices=("best_effort", "reliable"),
        default="reliable",
        help="Requested ROS camera subscription reliability",
    )
    parser.add_argument(
        "--ros-camera-qos-depth",
        type=int,
        default=10,
        help="KEEP_LAST depth for ROS camera subscriptions",
    )
    parser.add_argument("--camera-wait-timeout", type=float, default=10.0)
    parser.add_argument("--camera-read-timeout", type=float, default=0.25)
    parser.add_argument("--camera-max-age", type=float, default=0.5)
    parser.add_argument("--camera-sync-tolerance", type=float, default=0.15)
    parser.add_argument("--task", default="")
    parser.add_argument(
        "--start-paused",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Initialize cameras, robot, grippers, and joint impedance, then hold "
            "without sending policy observations until a runtime start command"
        ),
    )
    parser.add_argument(
        "--runtime-control-socket",
        default=str(DEFAULT_RUNTIME_CONTROL_SOCKET),
        help="Unix socket for start/pause/switch commands",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=640,
        help=(
            "OpenCV capture width; with --resize-ros-images, also the target "
            "width for every ROS camera frame"
        ),
    )
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help=(
            "OpenCV capture height; with --resize-ros-images, also the target "
            "height for every ROS camera frame"
        ),
    )
    parser.add_argument(
        "--resize-ros-images",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Resize every ROS camera frame to --width/--height before transport. Disabled "
            "by default so head and wrist cameras retain their native aspect ratios."
        ),
    )
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument("--action-hz", type=float, default=20.0)
    parser.add_argument(
        "--model-action-hz",
        type=float,
        default=None,
        help=(
            "Temporal frequency represented by adjacent policy actions; defaults "
            "to --action-hz"
        ),
    )
    parser.add_argument(
        "--observation-hz",
        type=float,
        default=10.0,
        help=(
            "Maximum latest-observation refresh rate while inference is active; "
            "the server retains only one pending observation"
        ),
    )
    parser.add_argument(
        "--max-pending-policy-requests",
        type=int,
        default=2,
        help=(
            "Client-side bound for active, server-pending, and not-yet-acknowledged "
            "latest-value policy requests"
        ),
    )
    parser.add_argument(
        "--action-stride",
        type=int,
        default=1,
        help=(
            "Execute every Nth action from each policy chunk while preserving "
            "the original policy timestep (1 executes every action)"
        ),
    )
    parser.add_argument(
        "--action-history-horizon",
        type=int,
        default=30,
        help="Number of executed policy actions carried in each observation",
    )
    parser.add_argument(
        "--rtc-action-horizon",
        type=int,
        default=50,
        help="Full RTC checkpoint action horizon transported to the server",
    )
    parser.add_argument(
        "--rtc-max-prefix",
        type=int,
        default=0,
        help=(
            "Maximum contiguous committed-action prefix sent for RTC; zero "
            "disables RTC fields for non-RTC policies"
        ),
    )
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument(
        "--startup-timeout-ms",
        type=int,
        default=60000,
        help=(
            "Maximum wait for the first action chunk. This is separate from "
            "--timeout-ms because GR00T may lazily initialize its visual-language "
            "processor on the first observation."
        ),
    )
    parser.add_argument("--chunk-size-threshold", type=float, default=0.5)
    parser.add_argument(
        "--aggregate",
        choices=("weighted_average", "latest", "average", "conservative"),
        default="weighted_average",
    )
    parser.add_argument(
        "--action-ema-weight",
        type=float,
        default=0.75,
        help="Weight assigned to the newer prediction during overlap aggregation",
    )
    parser.add_argument(
        "--max-arm-step-deg",
        type=float,
        default=8.0,
        help=(
            "Legacy recovery threshold and, only when explicitly enabled, the "
            "sustained tracking-error replan threshold"
        ),
    )
    parser.add_argument(
        "--hard-arm-jump-deg",
        type=float,
        default=20.0,
        help=(
            "Maximum component of the proportionally projected policy delta "
            "from the preceding accepted target"
        ),
    )
    parser.add_argument(
        "--arm-max-velocity-deg-s",
        type=float,
        default=40.0,
        help="Command-space velocity limit for right joints J1-J4",
    )
    parser.add_argument(
        "--wrist-max-velocity-deg-s",
        type=float,
        default=30.0,
        help="Command-space velocity limit for right joints J5-J7",
    )
    parser.add_argument(
        "--arm-max-acceleration-deg-s2",
        type=float,
        default=600.0,
        help="Command-space acceleration limit for right joints J1-J4",
    )
    parser.add_argument(
        "--wrist-max-acceleration-deg-s2",
        type=float,
        default=450.0,
        help="Command-space acceleration limit for right joints J5-J7",
    )
    parser.add_argument(
        "--tracking-error-timeout",
        "--recovery-band-timeout",
        dest="tracking_error_timeout",
        type=float,
        default=2.0,
        help="Consecutive policy time above the tracking threshold before replanning",
    )
    parser.add_argument(
        "--tracking-error-reset-timeout",
        "--recovery-band-reset-timeout",
        dest="tracking_error_reset_timeout",
        type=float,
        default=1.0,
        help="Normal policy time required to reset the tracking-error replan counter",
    )
    parser.add_argument(
        "--tracking-error-replan",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Enable the legacy sustained target-versus-feedback hold-and-replan "
            "mechanism (disabled by default)"
        ),
    )
    parser.add_argument("--max-gripper-step", type=float, default=0.9)
    parser.add_argument(
        "--arm-command-style",
        choices=("ptp", "stream"),
        default="stream",
        help="JointImp target strategy: direct policy waypoint or bounded slew",
    )
    parser.add_argument("--interpolation-step-deg", type=float, default=3.0)
    parser.add_argument("--interpolation-gripper-step", type=float, default=0.05)
    parser.add_argument("--gripper-left-min", type=float, default=0.025)
    parser.add_argument("--gripper-left-max", type=float, default=1.188)
    parser.add_argument("--gripper-right-min", type=float, default=0.097)
    parser.add_argument("--gripper-right-max", type=float, default=1.212)
    parser.add_argument("--gripper-feedback-timeout", type=float, default=3.0)
    parser.add_argument("--gripper-feedback-max-age", type=float, default=0.5)
    parser.add_argument("--gripper-command-hz", type=float, default=100.0)
    parser.add_argument("--gripper-kp", type=float, default=3.0)
    parser.add_argument("--gripper-kd", type=float, default=0.12)
    parser.add_argument("--gripper-init-timeout", type=float, default=3.0)
    parser.add_argument("--gripper-trace-interval", type=float, default=2.0)
    parser.add_argument(
        "--gripper-close-rate-rad-s",
        type=float,
        default=0.40,
        help=(
            "Close target rate used by calibrated initialization and postprocess; "
            "direct teleoperation targets are applied immediately"
        ),
    )
    parser.add_argument(
        "--gripper-control-mode",
        choices=("direct", "postprocess"),
        default="direct",
        help=(
            "direct uses the teleoperation motor mapping (command times range); "
            "postprocess enables close gain/overtravel and grasp hysteresis"
        ),
    )
    parser.add_argument(
        "--gripper-teleop-range-rad",
        type=float,
        default=1.6,
        help="Motor range used by direct teleoperation mapping (command * range)",
    )
    parser.add_argument(
        "--gripper-close-scale",
        type=float,
        default=1.25,
        help=(
            "Scale normalized close commands before mapping them to the calibrated "
            "physical endpoints; the effective command remains clipped to [0,0.9]"
        ),
    )
    parser.add_argument(
        "--gripper-close-overtravel-rad",
        type=float,
        default=0.18,
        help=(
            "Force-limited close target margin beyond the calibrated feedback "
            "endpoint; constrained to [0,0.20] rad"
        ),
    )
    parser.add_argument(
        "--gripper-hold-enter-command",
        type=float,
        default=0.60,
        help="Latch the largest close request once policy command reaches this value",
    )
    parser.add_argument(
        "--gripper-hold-release-command",
        type=float,
        default=0.35,
        help="Release a latched close request only at or below this policy command",
    )
    parser.add_argument(
        "--gripper-open-hold-seconds",
        type=float,
        default=1.5,
        help="Force a fully open command for this duration after releasing a grasp",
    )
    parser.add_argument(
        "--gripper-contact-release-command",
        type=float,
        default=0.55,
        help=(
            "Use this easier release threshold after hardware contact has been "
            "confirmed"
        ),
    )
    parser.add_argument("--gripper-contact-torque-nm", type=float, default=0.35)
    parser.add_argument("--gripper-contact-velocity-rad-s", type=float, default=0.12)
    parser.add_argument("--gripper-contact-cycles", type=int, default=5)
    parser.add_argument("--gripper-regrip-torque-nm", type=float, default=0.20)
    parser.add_argument("--gripper-regrip-cycles", type=int, default=10)
    parser.add_argument(
        "--dry-run-gripper-target",
        type=float,
        default=None,
        help="In --dry-run only, slowly test the direct gripper at this [0,0.9] target",
    )
    parser.add_argument("--velocity", type=float, default=10.0)
    parser.add_argument("--acceleration", type=float, default=10.0)
    parser.add_argument("--initial-position-velocity", type=float, default=40.0)
    parser.add_argument("--initial-position-acceleration", type=float, default=40.0)
    parser.add_argument("--arm-settle-timeout", type=float, default=10.0)
    parser.add_argument("--initial-move-hz", type=float, default=20.0)
    parser.add_argument(
        "--initial-move-step-deg",
        type=float,
        default=0.25,
        help="Maximum per-cycle joint setpoint change during the initial JointImp move",
    )
    parser.add_argument(
        "--initial-move-max-lead-deg",
        type=float,
        default=5.0,
        help=(
            "Pause initial trajectory progress when either arm's setpoint is this "
            "far ahead of feedback"
        ),
    )
    parser.add_argument("--initial-move-tolerance-deg", type=float, default=3.0)
    parser.add_argument("--initial-move-timeout", type=float, default=60.0)
    parser.add_argument("--initial-gripper-position", type=float, default=0.0)
    parser.add_argument("--initial-gripper-step", type=float, default=0.05)
    parser.add_argument("--initial-gripper-tolerance", type=float, default=0.05)
    parser.add_argument("--initial-gripper-timeout", type=float, default=20.0)
    parser.add_argument(
        "--skip-initial-pose",
        action=argparse.BooleanOptionalAction,
        default=os.environ.get("SKIP_INITIAL_POSE", "0") in ("1", "true", "TRUE", "yes"),
        help="Skip INITIAL_*_ARM_POSE joint move; keep gripper init and impedance switch",
    )
    parser.add_argument(
        "--trajectory-output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Directory containing one JSONL and summary file per inference run or operator segment",
    )
    parser.add_argument(
        "--trajectory-auto-start",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Automatically start a trajectory segment immediately before the first "
            "policy observation is sent (default: enabled)"
        ),
    )
    parser.add_argument(
        "--trajectory-control-socket",
        default=str(DEFAULT_CONTROL_SOCKET),
        help="Unix socket used by trajectory_keys.py for real-time S/E commands",
    )
    parser.add_argument(
        "--trajectory-status-path",
        default=str(DEFAULT_STATUS_PATH),
        help="Machine-readable recorder status JSON",
    )
    parser.add_argument(
        "--trajectory-queue-size",
        type=int,
        default=50_000,
        help="Maximum queued records before new samples are counted as dropped",
    )
    parser.add_argument(
        "--record-camera-frames",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Save the exact JPEG frames sent to the policy in the active trajectory "
            "segment's .cameras directory (default: enabled)"
        ),
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--dagger-mode",
        action="store_true",
        help=(
            "HITL DAgger: read /gento/joint_states, do not link Gento SDK, "
            "publish PolicyActionChunk to /skye/policy_action"
        ),
    )
    parser.add_argument(
        "--joint-states-topic",
        default="/gento/joint_states",
        help="Follower joint states used in --dagger-mode (radians)",
    )
    parser.add_argument(
        "--dagger-policy-version",
        default="unijepa",
        help="policy_version string stamped on PolicyActionChunk",
    )
    parser.add_argument(
        "--dagger-publish-mode",
        choices=("raw", "setpoint"),
        default="setpoint",
        help=(
            "HITL DAgger口 A: raw=publish model chunk on receive; "
            "setpoint=publish 16-step micro-ramp from last_cmd→slewed command "
            "each action tick (default)"
        ),
    )
    parser.add_argument(
        "--dagger-err-max-rad",
        type=float,
        default=0.15,
        help=(
            "Setpoint mode: max |last_cmd-fb| on arm joints (rad) before "
            "freezing advance this tick"
        ),
    )
    parser.add_argument(
        "--dagger-err-max-sustained-s",
        type=float,
        default=0.5,
        help="Setpoint mode: escalate ERR_MAX hold after this many seconds",
    )
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING"), default="INFO")
    return parser


def _observation_frames(
    *,
    state: np.ndarray,
    history_actions: np.ndarray,
    history_actions_is_pad: np.ndarray,
    episode_id: str,
    images: list[tuple[str, bytes, int, int]],
    timestep: int,
    must_go: bool,
    task: str,
    rtc_action_prefix: np.ndarray | None = None,
    rtc_action_prefix_is_pad: np.ndarray | None = None,
) -> tuple[str, list[bytes]]:
    request_id = str(uuid.uuid4())
    policy_state = robot_state_to_policy(state)
    history_actions = np.asarray(history_actions, dtype=np.float32)
    history_actions_is_pad = np.asarray(history_actions_is_pad, dtype=np.bool_)
    if history_actions.ndim != 2 or history_actions.shape[1] != ACTION_DIM:
        raise ValueError(
            f"history actions must be [T,{ACTION_DIM}], got {history_actions.shape}"
        )
    if history_actions_is_pad.shape != history_actions.shape[:1]:
        raise ValueError(
            "history action pad mask must match the history time dimension; "
            f"got {history_actions_is_pad.shape} for {history_actions.shape}"
        )
    if (rtc_action_prefix is None) != (rtc_action_prefix_is_pad is None):
        raise ValueError("RTC action prefix and padding mask must be provided together")
    rtc_prefix = None
    rtc_is_pad = None
    if rtc_action_prefix is not None:
        rtc_prefix = np.asarray(rtc_action_prefix, dtype=np.float32)
        rtc_is_pad = np.asarray(rtc_action_prefix_is_pad, dtype=np.bool_)
        if rtc_prefix.ndim != 2 or rtc_prefix.shape[1] != ACTION_DIM:
            raise ValueError(
                f"RTC action prefix must be [T,{ACTION_DIM}], got {rtc_prefix.shape}"
            )
        if rtc_is_pad.shape != rtc_prefix.shape[:1]:
            raise ValueError(
                "RTC action prefix pad mask must match the action time dimension; "
                f"got {rtc_is_pad.shape} for {rtc_prefix.shape}"
            )
    meta = {
        "type": "observation",
        "protocol_version": PROTOCOL_VERSION,
        "request_id": request_id,
        "sequence": timestep,
        "timestamp_ns": time.time_ns(),
        "must_go": must_go,
        "task": task,
        "episode_id": episode_id,
        "state": {
            "dtype": "float32",
            "shape": [STATE_DIM],
            "arm_unit": POLICY_ARM_UNIT,
            "gripper_unit": STATE_GRIPPER_UNIT,
        },
        "history_actions": {
            "dtype": "float32",
            "shape": list(history_actions.shape),
            "arm_unit": POLICY_ARM_UNIT,
            "gripper_unit": ACTION_GRIPPER_UNIT,
            "is_pad": history_actions_is_pad.tolist(),
        },
        "images": [
            {
                "name": name,
                "encoding": "jpeg",
                "width": width,
                "height": height,
                "channels": 3,
            }
            for name, _jpeg, width, height in images
        ],
    }
    if rtc_prefix is not None:
        meta["rtc_action_prefix"] = {
            "dtype": "float32",
            "shape": list(rtc_prefix.shape),
            "arm_unit": POLICY_ARM_UNIT,
            "gripper_unit": ACTION_GRIPPER_UNIT,
            "is_pad": rtc_is_pad.tolist(),
        }
    return request_id, [
        json_bytes(meta),
        float32_bytes(policy_state, expected_last_dim=STATE_DIM),
        float32_bytes(history_actions, expected_last_dim=ACTION_DIM),
        *(
            [float32_bytes(rtc_prefix, expected_last_dim=ACTION_DIM)]
            if rtc_prefix is not None
            else []
        ),
        *(jpeg for _name, jpeg, _width, _height in images),
    ]


def main() -> int:
    args = build_parser().parse_args()
    if args.model_action_hz is None:
        args.model_action_hz = args.action_hz
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    positive = (
        args.action_hz,
        args.model_action_hz,
        args.observation_hz,
        args.timeout_ms,
        args.startup_timeout_ms,
        args.max_arm_step_deg,
        args.hard_arm_jump_deg,
        args.arm_max_velocity_deg_s,
        args.wrist_max_velocity_deg_s,
        args.arm_max_acceleration_deg_s2,
        args.wrist_max_acceleration_deg_s2,
        args.tracking_error_timeout,
        args.tracking_error_reset_timeout,
        args.max_gripper_step,
        args.interpolation_step_deg,
        args.interpolation_gripper_step,
        args.gripper_feedback_timeout,
        args.gripper_feedback_max_age,
        args.gripper_command_hz,
        args.gripper_init_timeout,
        args.gripper_close_rate_rad_s,
        args.gripper_close_scale,
        args.gripper_open_hold_seconds,
        args.gripper_contact_torque_nm,
        args.gripper_contact_velocity_rad_s,
        args.gripper_contact_cycles,
        args.gripper_regrip_torque_nm,
        args.gripper_regrip_cycles,
        args.camera_wait_timeout,
        args.camera_read_timeout,
        args.camera_max_age,
        args.camera_sync_tolerance,
        args.arm_settle_timeout,
        args.initial_move_hz,
        args.initial_move_step_deg,
        args.initial_move_max_lead_deg,
        args.initial_move_tolerance_deg,
        args.initial_move_timeout,
        args.initial_gripper_step,
        args.initial_gripper_tolerance,
        args.initial_gripper_timeout,
    )
    if min(positive) <= 0:
        raise ValueError("frequency and safety/interpolation steps must be positive")
    execution_rate_ratio = args.action_hz / args.model_action_hz
    if args.action_stride <= 0:
        raise ValueError("--action-stride must be positive")
    if args.action_history_horizon <= 0:
        raise ValueError("--action-history-horizon must be positive")
    if args.dagger_mode:
        if args.dagger_publish_mode not in ("raw", "setpoint"):
            raise ValueError("--dagger-publish-mode must be raw or setpoint")
        if not math.isfinite(args.dagger_err_max_rad) or args.dagger_err_max_rad <= 0:
            raise ValueError("--dagger-err-max-rad must be finite and positive")
        if (
            not math.isfinite(args.dagger_err_max_sustained_s)
            or args.dagger_err_max_sustained_s <= 0
        ):
            raise ValueError("--dagger-err-max-sustained-s must be finite and positive")
    if args.rtc_action_horizon <= 0:
        raise ValueError("--rtc-action-horizon must be positive")
    if not 0 <= args.rtc_max_prefix <= args.rtc_action_horizon:
        raise ValueError("--rtc-max-prefix must be in [0, --rtc-action-horizon]")
    if args.rtc_max_prefix > 0 and args.action_stride != 1:
        raise ValueError("RTC committed prefixes require --action-stride 1")
    if args.max_pending_policy_requests <= 0:
        raise ValueError("--max-pending-policy-requests must be positive")
    if (
        not math.isclose(execution_rate_ratio, 1.0, rel_tol=0.01)
        and args.action_stride != 1
    ):
        raise ValueError(
            "execution frequency differing from --model-action-hz requires "
            "--action-stride 1"
        )
    if not 0 < args.velocity <= 100 or not 0 < args.acceleration <= 100:
        raise ValueError("--velocity and --acceleration must be percentages in (0,100]")
    if (
        not 0 < args.initial_position_velocity <= 100
        or not 0 < args.initial_position_acceleration <= 100
    ):
        raise ValueError(
            "--initial-position-velocity and --initial-position-acceleration "
            "must be percentages in (0,100]"
        )
    initial_move_frequency = int(round(args.initial_move_hz))
    if (
        initial_move_frequency <= 0
        or not math.isclose(args.initial_move_hz, initial_move_frequency)
        or 1000 % initial_move_frequency != 0
    ):
        raise ValueError("--initial-move-hz must be an integer divisor of 1000")
    if args.initial_move_max_lead_deg < args.initial_move_step_deg:
        raise ValueError(
            "--initial-move-max-lead-deg must be at least --initial-move-step-deg"
        )
    if args.gripper_regrip_torque_nm >= args.gripper_contact_torque_nm:
        raise ValueError("regrip torque must be below contact torque")
    if not 0 <= args.gripper_close_overtravel_rad <= 0.20:
        raise ValueError("--gripper-close-overtravel-rad must be in [0,0.20]")
    if not 0 < args.gripper_teleop_range_rad <= 1.6:
        raise ValueError("--gripper-teleop-range-rad must be in (0,1.6]")
    if not (
        0.0
        <= args.gripper_hold_release_command
        <= args.gripper_contact_release_command
        < args.gripper_hold_enter_command
        <= GRIPPER_COMMAND_MAX
    ):
        raise ValueError(
            "gripper hold thresholds must satisfy "
            "0 <= release <= contact release < enter <= 0.9"
        )
    if args.width <= 0 or args.height <= 0:
        raise ValueError("--width and --height must be positive")
    if args.ros_camera_qos_depth <= 0:
        raise ValueError("--ros-camera-qos-depth must be positive")
    if args.camera_source != "ros2" and args.resize_ros_images:
        raise ValueError("--resize-ros-images requires --camera-source ros2")
    if args.hard_arm_jump_deg <= args.max_arm_step_deg:
        raise ValueError("--hard-arm-jump-deg must exceed --max-arm-step-deg")
    if not 0 <= args.chunk_size_threshold <= 1:
        raise ValueError("--chunk-size-threshold must be in [0,1]")
    if not 0 <= args.action_ema_weight <= 1:
        raise ValueError("--action-ema-weight must be in [0,1]")
    if not 0 <= args.initial_gripper_position <= GRIPPER_COMMAND_MAX:
        raise ValueError("--initial-gripper-position must be in [0,0.9]")
    if args.trajectory_queue_size <= 0:
        raise ValueError("--trajectory-queue-size must be positive")
    if args.dry_run_gripper_target is not None:
        if not args.dry_run:
            raise ValueError("--dry-run-gripper-target requires --dry-run")
        if not 0 <= args.dry_run_gripper_target <= GRIPPER_COMMAND_MAX:
            raise ValueError("--dry-run-gripper-target must be in [0,0.9]")
    if Path(args.runtime_control_socket) == Path(args.trajectory_control_socket):
        raise ValueError(
            "--runtime-control-socket and --trajectory-control-socket must differ"
        )
    camera_specs: dict[str, str] = {}
    ros_topics: dict[str, str] = {}
    if args.camera_source == "opencv":
        if args.ros_camera:
            raise ValueError("--ros-camera options require --camera-source ros2")
        camera_specs[args.camera_name] = args.camera
        for spec in args.extra_camera:
            name, separator, device = spec.partition("=")
            if not separator or not name or not device:
                raise ValueError(f"invalid --extra-camera {spec!r}; expected NAME=DEVICE")
            if name in camera_specs:
                raise ValueError(f"duplicate camera name {name!r}")
            camera_specs[name] = device
    else:
        if args.extra_camera:
            raise ValueError("--extra-camera is only valid with --camera-source opencv")
        if args.ros_camera:
            for spec in args.ros_camera:
                name, separator, topic = spec.partition("=")
                if not separator or not name or not topic.startswith("/"):
                    raise ValueError(f"invalid --ros-camera {spec!r}; expected NAME=/absolute/topic")
                if name in ros_topics:
                    raise ValueError(f"duplicate ROS camera name {name!r}")
                ros_topics[name] = topic
        else:
            ros_topics = dict(DEFAULT_ROS_CAMERA_TOPICS)
        if len(ros_topics) != len(set(ros_topics.values())):
            raise ValueError("ROS camera topics must be unique")

    if args.dagger_mode:
        # Unused for link; keep parse soft so scripts can omit CONTROLLER_IP.
        ip = [0, 0, 0, 0]
        try:
            parts = [int(part) for part in args.controller_ip.split(".")]
            if len(parts) == 4 and all(0 <= part <= 255 for part in parts):
                ip = parts
        except ValueError:
            pass
    else:
        ip = [int(part) for part in args.controller_ip.split(".")]
        if len(ip) != 4 or any(part < 0 or part > 255 for part in ip):
            raise ValueError("--controller-ip must be an IPv4 address")

    right_velocity_limits = np.asarray(
        [args.arm_max_velocity_deg_s] * 4
        + [args.wrist_max_velocity_deg_s] * 3,
        dtype=np.float64,
    )
    right_acceleration_limits = np.asarray(
        [args.arm_max_acceleration_deg_s2] * 4
        + [args.wrist_max_acceleration_deg_s2] * 3,
        dtype=np.float64,
    )

    # Keep server-side imports and --help usable without the robot SDK / ROS 2
    # environment. These imports intentionally happen only on the robot.
    from remote_inference_async.robot_control import (
        Camera,
        GripperCommandProcessor,
        INITIAL_LEFT_ARM_POSE,
        InitialPosePlanRejected,
        INITIAL_RIGHT_ARM_POSE,
        JOINT_IMPEDANCE_D,
        JOINT_IMPEDANCE_K,
        execute_action,
        move_arms_to_joint_poses,
        move_gripper_to_position,
        move_grippers_to_position,
        prepare_arm,
        set_arm_impedance_speed,
        state_from_rt,
        validate_action,
    )

    if args.dagger_mode:
        # Sole writer for arms is control_arbiter → skye_robot_driver.
        args.dry_run = True
        args.skip_initial_pose = True
        from remote_inference_async.dagger_publisher import DaggerPolicyPublisher
        from remote_inference_async.dagger_ros_facade import (
            DaggerRosRobotFacade,
            RosGripperFeedback,
        )
        from remote_inference_async.dagger_setpoint import (
            ErrMaxGuard,
            micro_ramp_dt,
            robot_commands_to_setpoint_policy,
        )
        FXLogMask = FXObjMask = FXObjType = FXStateType = None  # noqa: N816
        DualGentoDmGripper = None  # type: ignore
        GentoRobot = None  # type: ignore
    else:
        from PYTHON_SDK.GentoRobot import (
            FXLogMask,
            FXObjMask,
            FXObjType,
            FXStateType,
            GentoRobot,
        )
        from remote_inference_async.gento_dm_gripper import DualGentoDmGripper

    enabled_arms = {0, 1}
    gripper_postprocess = args.gripper_control_mode == "postprocess"
    stop_event = threading.Event()
    requested_signal: int | None = None
    fatal_shutdown = False
    plan_rejected = False
    dagger_pub = None
    dagger_err_guard = None
    if args.dagger_mode:
        dagger_err_guard = ErrMaxGuard(
            err_max_rad=args.dagger_err_max_rad,
            sustained_s=args.dagger_err_max_sustained_s,
        )

    def request_stop(signum: int, _frame: object) -> None:
        nonlocal requested_signal
        requested_signal = signum
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    if args.dagger_mode:
        robot = DaggerRosRobotFacade(joint_states_topic=args.joint_states_topic)
        dagger_pub = DaggerPolicyPublisher(policy_version=args.dagger_policy_version)
        dagger_pub.start()
        LOGGER.info(
            "DAGGER mode: joint_states=%s → /skye/policy_action (no Gento SDK)",
            args.joint_states_topic,
        )
    else:
        robot = GentoRobot()
    trajectory_recorder = RealtimeTrajectoryRecorder(
        output_dir=args.trajectory_output_dir,
        control_socket=args.trajectory_control_socket,
        status_path=args.trajectory_status_path,
        queue_size=args.trajectory_queue_size,
        runtime_metadata={
            "process_id": os.getpid(),
            "protocol_version": PROTOCOL_VERSION,
            "client": "remote_inference_async.async_robot_client",
            "server": args.server,
            "controller_ip": args.controller_ip,
            "action_hz": args.action_hz,
            "model_action_hz": args.model_action_hz,
            "history_timestep_hz": args.action_hz,
            # Retain the historical metadata key for trajectory-analysis
            # compatibility; it is derived rather than user-configured.
            "time_scale": execution_rate_ratio,
            "observation_hz": args.observation_hz,
            "max_pending_policy_requests": args.max_pending_policy_requests,
            "action_stride": args.action_stride,
            "action_history_horizon": args.action_history_horizon,
            "rtc_action_horizon": args.rtc_action_horizon,
            "rtc_max_prefix": args.rtc_max_prefix,
            "aggregation": args.aggregate,
            "action_ema_weight": args.action_ema_weight,
            "execution_mode": "direct_guard",
            "impedance_velocity_percent": args.velocity,
            "impedance_acceleration_percent": args.acceleration,
            "joint_impedance_k": list(JOINT_IMPEDANCE_K),
            "joint_impedance_d": list(JOINT_IMPEDANCE_D),
            "initial_position_velocity_percent": args.initial_position_velocity,
            "initial_position_acceleration_percent": args.initial_position_acceleration,
            "initial_pose_planner": "synchronized_quintic_joint_trajectory",
            "initial_pose_control_mode": "joint_impedance",
            "initial_move_step_deg": args.initial_move_step_deg,
            "initial_move_max_lead_deg": args.initial_move_max_lead_deg,
            "robot_side_image_resize": args.resize_ros_images,
            "robot_side_image_target": (
                [args.width, args.height] if args.resize_ros_images else None
            ),
            "task": args.task,
            "start_paused": args.start_paused,
            "runtime_control_socket": args.runtime_control_socket,
            "dry_run": args.dry_run,
            "trajectory_auto_start": args.trajectory_auto_start,
            "record_camera_frames": args.record_camera_frames,
            "bimanual_action_layout": [
                "left_arm_joint_0_deg",
                "left_arm_joint_1_deg",
                "left_arm_joint_2_deg",
                "left_arm_joint_3_deg",
                "left_arm_joint_4_deg",
                "left_arm_joint_5_deg",
                "left_arm_joint_6_deg",
                "right_arm_joint_0_deg",
                "right_arm_joint_1_deg",
                "right_arm_joint_2_deg",
                "right_arm_joint_3_deg",
                "right_arm_joint_4_deg",
                "right_arm_joint_5_deg",
                "right_arm_joint_6_deg",
                "left_gripper_normalized",
                "right_gripper_normalized",
            ],
            "safety": {
                "max_arm_step_deg": args.max_arm_step_deg,
                "hard_arm_jump_deg": args.hard_arm_jump_deg,
                "hard_jump_reference": "preceding_accepted_policy_target",
                "hard_jump_response": "proportional_bimanual_projection",
                "max_gripper_step": args.max_gripper_step,
                "arm_command_style": args.arm_command_style,
                "interpolation_step_deg": args.interpolation_step_deg,
                "interpolation_gripper_step": args.interpolation_gripper_step,
                "right_velocity_limits_deg_s": right_velocity_limits.tolist(),
                "right_acceleration_limits_deg_s2": right_acceleration_limits.tolist(),
                "tracking_error_threshold_deg": args.max_arm_step_deg,
                "tracking_error_reference": "measured_joint_feedback",
                "tracking_error_timeout_s": args.tracking_error_timeout,
                "tracking_error_replan_enabled": args.tracking_error_replan,
                "tracking_error_response": (
                    "hold_and_replan_without_limit"
                    if args.tracking_error_replan
                    else "disabled"
                ),
                "tracking_error_reset_timeout_s": args.tracking_error_reset_timeout,
            },
            "gripper": {
                "control_mode": args.gripper_control_mode,
                "postprocess_enabled": gripper_postprocess,
                "left_min_rad": args.gripper_left_min,
                "left_max_rad": args.gripper_left_max,
                "right_min_rad": args.gripper_right_min,
                "right_max_rad": args.gripper_right_max,
                "command_hz": args.gripper_command_hz,
                "kp": args.gripper_kp,
                "kd": args.gripper_kd,
                "action_command_range": [0.0, GRIPPER_COMMAND_MAX],
                "direct_teleop_range_rad": args.gripper_teleop_range_rad,
                "direct_max_target_rad": (
                    GRIPPER_COMMAND_MAX * args.gripper_teleop_range_rad
                ),
                "target_update_mode": (
                    "rate_limited" if gripper_postprocess else "immediate"
                ),
                "close_scale": args.gripper_close_scale,
                "effective_close_scale": (
                    args.gripper_close_scale if gripper_postprocess else 1.0
                ),
                "close_overtravel_rad": args.gripper_close_overtravel_rad,
                "effective_close_overtravel_rad": (
                    args.gripper_close_overtravel_rad if gripper_postprocess else 0.0
                ),
                "hold_enter_command": args.gripper_hold_enter_command,
                "hold_release_command": args.gripper_hold_release_command,
                "open_hold_seconds": args.gripper_open_hold_seconds,
                "contact_release_command": args.gripper_contact_release_command,
                "close_rate_rad_s": args.gripper_close_rate_rad_s,
                "effective_policy_close_rate_rad_s": (
                    args.gripper_close_rate_rad_s if gripper_postprocess else None
                ),
                "contact_torque_nm": args.gripper_contact_torque_nm,
                "contact_velocity_rad_s": args.gripper_contact_velocity_rad_s,
                "contact_cycles": args.gripper_contact_cycles,
                "regrip_torque_nm": args.gripper_regrip_torque_nm,
                "regrip_cycles": args.gripper_regrip_cycles,
            },
        },
    )
    gripper = None
    gripper_sampler_stop = threading.Event()
    gripper_sampler_thread: threading.Thread | None = None
    opencv_cameras: dict[str, Camera] = {}
    ros_camera_bridge = None
    runtime_control = RuntimeControlServer(args.runtime_control_socket)
    network = AsyncDealer(args.server, args.timeout_ms)
    actions = ActionBuffer(args.aggregate, args.action_ema_weight)
    gripper_command_processor = GripperCommandProcessor(
        mode=args.gripper_control_mode,
        enter_command=args.gripper_hold_enter_command,
        release_command=args.gripper_hold_release_command,
        contact_release_command=args.gripper_contact_release_command,
        open_hold_cycles=max(1, math.ceil(args.gripper_open_hold_seconds * args.action_hz)),
    )
    must_go = True
    last_sent_timestep = -1
    pending_requests: dict[str, float] = {}
    last_policy_request_at = float("-inf")
    min_action_observation_sequence = 0
    received_first_action_chunk = False
    auto_trajectory_start_attempted = False
    policy_active = not args.start_paused
    current_task = args.task
    runtime_generation = 0
    command_limiter = JointCommandLimiter(
        command_hz=args.action_hz,
        velocity_limits_deg_s=right_velocity_limits,
        acceleration_limits_deg_s2=right_acceleration_limits,
    )
    tracking_error_guard = TrackingErrorGuard(
        command_hz=args.action_hz,
        timeout_s=args.tracking_error_timeout,
        reset_s=args.tracking_error_reset_timeout,
        dof=14,
    )

    try:
        trajectory_recorder.start()
        runtime_control.start()
        runtime_control.update_status(
            state="initializing",
            ready=False,
            task=current_task,
            generation=runtime_generation,
            process_id=os.getpid(),
        )
        LOGGER.info(
            "Bimanual direct-guard execution: policy-target jumps are proportionally "
            "projected to a %.2f-degree envelope without stopping; sustained "
            "target-versus-feedback hold-and-replan is %s",
            args.hard_arm_jump_deg,
            "enabled" if args.tracking_error_replan else "disabled",
        )
        if args.action_stride > 1:
            LOGGER.warning(
                "Action stride %d enabled: executing source offsets 0,%d,%d,...; "
                "an exhausted sparse queue holds the last impedance setpoint "
                "instead of rebasing a stale prediction",
                args.action_stride,
                args.action_stride,
                args.action_stride * 2,
            )
        if execution_rate_ratio > 1.0:
            LOGGER.warning(
                "Full-chunk time scaling enabled: model=%.1f Hz, execution=%.1f Hz, "
                "scale=%.2fx, observation refresh<=%.1f Hz, pending requests<=%d",
                args.model_action_hz,
                args.action_hz,
                execution_rate_ratio,
                args.observation_hz,
                args.max_pending_policy_requests,
            )
        LOGGER.info(
            "Real-time right-arm trajectory recorder is ready. Recording will start "
            "automatically with the first policy request; trajectory_keys.sh S/E "
            "remain available for manual cuts. Segments are saved under %s",
            trajectory_recorder.output_dir,
        )
        if args.camera_source == "opencv":
            for name, device in camera_specs.items():
                opencv_cameras[name] = Camera(
                    device,
                    args.width,
                    args.height,
                    args.jpeg_quality,
                )
        else:
            from remote_inference_async.ros_camera import RosCameraBridge

            ros_camera_bridge = RosCameraBridge(
                ros_topics,
                reliability=args.ros_camera_reliability,
                qos_depth=args.ros_camera_qos_depth,
                output_width=args.width if args.resize_ros_images else None,
                output_height=args.height if args.resize_ros_images else None,
                jpeg_quality=args.jpeg_quality,
            )
        if args.dagger_mode:
            ret = robot.link()
            LOGGER.info(
                "Connected via ROS joint_states (%s), type=%s",
                args.joint_states_topic,
                robot.get_robot_type(),
            )
            # Policy loop treats >200ms gripper age as stale (skip infer / hold).
            args.gripper_feedback_max_age = min(
                float(args.gripper_feedback_max_age), 0.2
            )
            gripper = RosGripperFeedback(warn_age_s=0.05)
            ready_gripper = gripper.start(
                timeout_s=max(5.0, float(args.gripper_feedback_timeout))
            )
        else:
            ret = robot.link(*ip, log_level=FXLogMask.FX_LOG_INFO_FLAG)
            if not robot._connected:
                detail = robot._get_operate_error_msg(ret)
                hint = (
                    "; stop the previous Gento SDK client and wait for the "
                    "controller to release its link"
                    if ret == -22
                    else ""
                )
                raise RuntimeError(f"robot link failed: {ret} ({detail}){hint}")
            LOGGER.info(
                "Connected to %s, controller=%s, controller_version=%s, sdk_version=%s",
                robot.get_robot_type(),
                args.controller_ip,
                robot.get_controller_version(),
                robot.get_sdk_version(),
            )
            gripper = DualGentoDmGripper(
                robot,
                left_limits=(args.gripper_left_min, args.gripper_left_max),
                right_limits=(args.gripper_right_min, args.gripper_right_max),
                command_hz=args.gripper_command_hz,
                kp=args.gripper_kp,
                kd=args.gripper_kd,
                initialization_timeout_s=args.gripper_init_timeout,
                trace_interval_s=args.gripper_trace_interval,
                close_rate_rad_s=args.gripper_close_rate_rad_s,
                close_scale=args.gripper_close_scale,
                close_overtravel_rad=args.gripper_close_overtravel_rad,
                policy_postprocess=gripper_postprocess,
                teleop_range_rad=args.gripper_teleop_range_rad,
                contact_torque_nm=args.gripper_contact_torque_nm,
                contact_velocity_rad_s=args.gripper_contact_velocity_rad_s,
                contact_cycles=args.gripper_contact_cycles,
                regrip_torque_nm=args.gripper_regrip_torque_nm,
                regrip_cycles=args.gripper_regrip_cycles,
            )
            ready_gripper = gripper.wait_for_feedback(
                args.gripper_feedback_timeout,
                args.gripper_feedback_max_age,
                enabled_arms,
            )
        LOGGER.info(
            "Gripper control ready: normalized feedback=%s",
            {arm: round(float(ready_gripper[arm]), 3) for arm in enabled_arms},
        )

        sampled_gripper = gripper

        def sample_gripper_for_trajectory() -> None:
            period_s = 1.0 / args.gripper_command_hz
            next_sample_at = time.monotonic()
            while not gripper_sampler_stop.is_set():
                if trajectory_recorder.is_recording:
                    try:
                        trajectory_recorder.record(
                            "gripper_sample",
                            gripper=sampled_gripper.diagnostics(),
                        )
                    except BaseException as exc:
                        LOGGER.error("Trajectory gripper sampler stopped: %s", exc)
                        return
                next_sample_at += period_s
                remaining = next_sample_at - time.monotonic()
                if remaining <= 0:
                    next_sample_at = time.monotonic()
                    continue
                gripper_sampler_stop.wait(remaining)

        gripper_sampler_thread = threading.Thread(
            target=sample_gripper_for_trajectory,
            name="trajectory-gripper-sampler",
            daemon=True,
        )
        gripper_sampler_thread.start()
        if ros_camera_bridge is not None:
            startup_images = ros_camera_bridge.read_jpegs(
                timeout_s=args.camera_wait_timeout,
                max_age_s=args.camera_max_age,
                sync_tolerance_s=args.camera_sync_tolerance,
            )
            LOGGER.info(
                "ROS cameras ready: %s",
                {name: f"{width}x{height}" for name, _data, width, height in startup_images},
            )
        if args.dry_run and args.dry_run_gripper_target is not None:
            LOGGER.info(
                "Dry-run: slowly testing direct right gripper at %.3f; arm will not move",
                args.dry_run_gripper_target,
            )
            tested_gripper = move_gripper_to_position(
                gripper,
                1,
                args.dry_run_gripper_target,
                step=args.initial_gripper_step,
                command_hz=args.initial_move_hz,
                tolerance=args.initial_gripper_tolerance,
                timeout_s=args.initial_gripper_timeout,
                feedback_max_age_s=args.gripper_feedback_max_age,
                stop_event=stop_event,
            )
            LOGGER.info("Dry-run direct gripper test ready: %.3f", tested_gripper)
        if not args.dry_run:
            if args.skip_initial_pose:
                LOGGER.info(
                    "Skipping initial joint pose (--skip-initial-pose); "
                    "arms stay where the motion script left them"
                )
            else:
                for arm_idx, obj in ((0, FXObjType.OBJ_ARM0), (1, FXObjType.OBJ_ARM1)):
                    if arm_idx in enabled_arms:
                        prepare_arm(
                            robot,
                            obj,
                            args.initial_position_velocity,
                            args.initial_position_acceleration,
                            args.arm_settle_timeout,
                        )
                LOGGER.info(
                    "Moving both arms to initial joint poses with a synchronized "
                    "quintic trajectory in JointImp: left=%s, right=%s",
                    INITIAL_LEFT_ARM_POSE.tolist(),
                    INITIAL_RIGHT_ARM_POSE.tolist(),
                )
                final_arms = move_arms_to_joint_poses(
                    robot,
                    {
                        FXObjType.OBJ_ARM0: INITIAL_LEFT_ARM_POSE,
                        FXObjType.OBJ_ARM1: INITIAL_RIGHT_ARM_POSE,
                    },
                    step_deg=args.initial_move_step_deg,
                    max_lead_deg=args.initial_move_max_lead_deg,
                    command_hz=args.initial_move_hz,
                    tolerance_deg=args.initial_move_tolerance_deg,
                    timeout_s=args.initial_move_timeout,
                    stop_event=stop_event,
                    required_state=FXStateType.FX_STATE_IMP_JOINT,
                )
                LOGGER.info("Left arm initial pose ready: %s", final_arms[0].round(3).tolist())
                LOGGER.info("Right arm initial pose ready: %s", final_arms[1].round(3).tolist())
            LOGGER.info(
                "Moving both grippers simultaneously to initial position %.3f",
                args.initial_gripper_position,
            )
            final_grippers = move_grippers_to_position(
                gripper,
                enabled_arms,
                args.initial_gripper_position,
                step=args.initial_gripper_step,
                command_hz=args.initial_move_hz,
                tolerance=args.initial_gripper_tolerance,
                timeout_s=args.initial_gripper_timeout,
                feedback_max_age_s=args.gripper_feedback_max_age,
                stop_event=stop_event,
            )
            LOGGER.info(
                "Both grippers reached initial position together: %s",
                {
                    arm_idx: round(value, 3)
                    for arm_idx, value in final_grippers.items()
                },
            )
            initial_rt = robot.get_rt_dict()
            if "error" in initial_rt:
                raise RuntimeError(initial_rt["error"])
            for arm_idx, arm_name in ((0, "Left"), (1, "Right")):
                if arm_idx in enabled_arms:
                    LOGGER.info(
                        "%s arm external torque at initial pose: %s",
                        arm_name,
                        initial_rt["arms"][arm_idx]["fb"]["fb_ext_torque"],
                    )
            for arm_idx, obj in ((0, FXObjType.OBJ_ARM0), (1, FXObjType.OBJ_ARM1)):
                if arm_idx in enabled_arms:
                    if args.skip_initial_pose:
                        prepare_arm(
                            robot,
                            obj,
                            args.velocity,
                            args.acceleration,
                            args.arm_settle_timeout,
                        )
                    else:
                        set_arm_impedance_speed(
                            robot,
                            obj,
                            args.velocity,
                            args.acceleration,
                        )
            LOGGER.info("Enabled arms ready in joint-impedance mode")
        limiter_rt = robot.get_rt_dict()
        limiter_state = state_from_rt(
            limiter_rt,
            gripper.state(args.gripper_feedback_max_age, enabled_arms),
            enabled_arms,
        )
        command_limiter.reset(limiter_state[7:14])
        last_accepted_action = limiter_state.copy()
        # Normal motion is generated from the last successfully published
        # setpoint, never from measured feedback. Feedback remains the source
        # for tracking/fault holds, but using it as every interpolation start
        # creates a sawtooth whenever the impedance controller has not caught
        # up with the preceding burst of commands.
        last_sent_action = limiter_state.copy()
        if args.start_paused and not args.dry_run:
            execute_action(robot, gripper, limiter_state, enabled_arms)
            LOGGER.info("Published measured-position HOLD setpoint after initialization")
        action_history = ExecutedActionHistory(args.action_history_horizon)
        latest_observation_state = limiter_state.copy()
        latest_observation_rt = limiter_rt
        latest_observation_timestep = 0
        pending_history_command: np.ndarray | None = None
        episode_id = str(uuid.uuid4())
        consecutive_invalid_action_replans = 0
        network.start()
        runtime_control.update_status(
            state="policy" if policy_active else "hold",
            ready=True,
            task=current_task,
            generation=runtime_generation,
            action_queue_depth=0,
            pending_policy_requests=0,
        )
        if policy_active:
            LOGGER.info("Robot runtime ready; policy inference is active")
        else:
            LOGGER.info(
                "Robot runtime READY in joint-impedance HOLD; use control_robot.sh "
                "start [--task ...] to begin inference"
            )
        LOGGER.info(
            "Policy watchdog ready: first action timeout=%d ms, runtime timeout=%d ms",
            args.startup_timeout_ms,
            args.timeout_ms,
        )

        period = 1.0 / args.action_hz
        feedback_sample_index = 0
        while not stop_event.is_set():
            started = time.perf_counter()
            cycle_rt = None
            cycle_state = None
            history_advanced_this_cycle = False
            for runtime_command in runtime_control.drain_commands():
                if runtime_command.name == "shutdown":
                    requested_signal = signal.SIGTERM
                    stop_event.set()
                    runtime_control.update_status(state="stopping", ready=False)
                    LOGGER.info("Runtime shutdown requested through control socket")
                    break

                transition_rt = robot.get_rt_dict()
                transition_state = state_from_rt(
                    transition_rt,
                    gripper.state(args.gripper_feedback_max_age, enabled_arms),
                    enabled_arms,
                )
                hold_command = transition_state.copy()
                # Arm HOLD follows measured joints. Preserve the last gripper
                # request so pausing during a grasp does not remove clamping
                # force by replacing its target with instantaneous feedback.
                hold_command[14:16] = last_sent_action[14:16]
                if not args.dry_run:
                    # Cancel the preceding moving setpoint before invalidating
                    # its software queues. Joint impedance then holds measured
                    # position throughout the state transition.
                    execute_action(robot, gripper, hold_command, enabled_arms)

                dropped_actions = len(actions)
                invalidated_requests = len(pending_requests)
                pending_requests.clear()
                next_sequence = max(
                    actions.latest_executed,
                    latest_observation_timestep,
                    last_sent_timestep,
                ) + 1
                actions.reset_phase(last_completed_timestep=next_sequence - 1)
                min_action_observation_sequence = next_sequence
                latest_observation_timestep = next_sequence
                latest_observation_rt = transition_rt
                latest_observation_state = transition_state.copy()
                action_history.reset()
                command_limiter.reset(transition_state[7:14])
                tracking_error_guard.reset_active(reset_attempts=True)
                gripper_command_processor.reset()
                last_accepted_action = transition_state.copy()
                last_sent_action = hold_command.copy()
                pending_history_command = None
                last_policy_request_at = float("-inf")
                received_first_action_chunk = False
                must_go = True
                runtime_generation += 1
                episode_id = str(uuid.uuid4())

                if runtime_command.name == "pause":
                    policy_active = False
                    runtime_state = "hold"
                else:
                    if runtime_command.task is not None:
                        current_task = runtime_command.task
                    policy_active = True
                    runtime_state = "policy"

                runtime_control.update_status(
                    state=runtime_state,
                    ready=True,
                    task=current_task,
                    generation=runtime_generation,
                    action_queue_depth=0,
                    pending_policy_requests=0,
                )
                trajectory_recorder.record(
                    "runtime_state_changed",
                    command=runtime_command.name,
                    runtime_state=runtime_state,
                    task=current_task,
                    generation=runtime_generation,
                    episode_id=episode_id,
                    queued_actions_dropped=dropped_actions,
                    pending_policy_requests_invalidated=invalidated_requests,
                    hold_state=_right_vector_payload(transition_state),
                    hold_command=_right_vector_payload(hold_command),
                )
                LOGGER.info(
                    "Runtime state -> %s (task=%r, generation=%d); held current "
                    "position, dropped %d actions and invalidated %d requests",
                    runtime_state,
                    current_task,
                    runtime_generation,
                    dropped_actions,
                    invalidated_requests,
                )

            if stop_event.is_set():
                continue
            # A command becomes part of the past-action prefix only after one
            # control period has elapsed and fresh feedback has been sampled.
            # The resulting snapshot is therefore s(t) with actions ending at
            # a(t-1), matching the training dataset's [idx-K, idx) history.
            if pending_history_command is not None:
                try:
                    cycle_rt = robot.get_rt_dict()
                    cycle_state = state_from_rt(
                        cycle_rt,
                        gripper.state(args.gripper_feedback_max_age, enabled_arms),
                        enabled_arms,
                    )
                except RuntimeError as exc:
                    if args.dagger_mode and "gripper" in str(exc).lower():
                        # >200ms gripper age: hold last command, skip this tick's
                        # history advance / new inference.
                        LOGGER.warning(
                            "gripper feedback stale; holding last_sent_action "
                            "and skipping policy request: %s",
                            exc,
                        )
                        pending_history_command = None
                        must_go = False
                        continue
                    raise
                action_history.append_robot_command(pending_history_command)
                pending_history_command = None
                latest_observation_rt = cycle_rt
                latest_observation_state = cycle_state.copy()
                latest_observation_timestep = max(actions.latest_executed + 1, 0)
                history_advanced_this_cycle = True
            # Receive all currently available chunks without blocking the control loop.
            while True:
                try:
                    reply = network.incoming.get_nowait()
                except queue.Empty:
                    break
                if isinstance(reply, BaseException):
                    raise RuntimeError("network thread failed") from reply
                if not reply:
                    continue
                meta = parse_json(reply[0])
                response_request_id = str(meta.get("request_id", ""))
                responded_at, superseded_count = acknowledge_response(
                    pending_requests,
                    response_request_id,
                )
                if responded_at is None:
                    trajectory_recorder.record(
                        "invalidated_policy_response_ignored",
                        request_id=response_request_id,
                        response_type=meta.get("type"),
                        observation_sequence=meta.get("observation_sequence"),
                        runtime_generation=runtime_generation,
                    )
                    LOGGER.debug(
                        "Ignored response for invalidated request %s",
                        response_request_id,
                    )
                    continue
                if superseded_count:
                    LOGGER.debug(
                        "Response %s superseded %d older pending request(s)",
                        response_request_id,
                        superseded_count,
                    )
                response_type = meta.get("type")
                if response_type == "observation_dropped":
                    trajectory_recorder.record(
                        "observation_dropped",
                        request_id=response_request_id,
                        observation_sequence=meta.get("observation_sequence"),
                    )
                    LOGGER.debug(
                        "Server dropped stale observation %s",
                        meta.get("observation_sequence"),
                    )
                    continue
                if response_type == "error":
                    trajectory_recorder.record(
                        "policy_server_error",
                        request_id=response_request_id,
                        message=str(meta.get("message", "")),
                    )
                    raise RuntimeError(f"policy server error: {meta.get('message')}")
                if response_type != "action_chunk" or len(reply) != 2:
                    raise RuntimeError(
                        f"invalid server response: type={response_type}, "
                        f"frames={len(reply)}"
                    )
                observation_sequence = int(meta["observation_sequence"])
                if observation_sequence < min_action_observation_sequence:
                    trajectory_recorder.record(
                        "stale_action_chunk_ignored",
                        request_id=response_request_id,
                        observation_sequence=observation_sequence,
                        minimum_observation_sequence=min_action_observation_sequence,
                    )
                    LOGGER.warning(
                        "Ignored action chunk from stale observation %d; "
                        "minimum accepted observation is %d",
                        observation_sequence,
                        min_action_observation_sequence,
                    )
                    continue
                action_meta = meta["actions"]
                action_dt_ns = int(meta.get("action_dt_ns", 0))
                if action_dt_ns <= 0:
                    raise RuntimeError("policy server response is missing a valid action_dt_ns")
                server_action_hz = 1e9 / action_dt_ns
                if not math.isclose(server_action_hz, args.model_action_hz, rel_tol=0.01):
                    raise RuntimeError(
                        "policy/server action frequency does not match --model-action-hz: "
                        f"server={server_action_hz:.3f} Hz, configured={args.model_action_hz:.3f} Hz"
                    )
                if action_meta.get("arm_unit") != POLICY_ARM_UNIT:
                    raise RuntimeError(
                        "policy server action arm_unit must be "
                        f"{POLICY_ARM_UNIT!r}, got {action_meta.get('arm_unit')!r}"
                    )
                if action_meta.get("gripper_unit") != ACTION_GRIPPER_UNIT:
                    raise RuntimeError(
                        "policy server action gripper_unit must be "
                        f"{ACTION_GRIPPER_UNIT!r}, got {action_meta.get('gripper_unit')!r}"
                    )
                policy_chunk = parse_float32(
                    reply[1],
                    action_meta["shape"],
                    expected_last_dim=ACTION_DIM,
                )
                if (
                    args.dagger_mode
                    and dagger_pub is not None
                    and args.dagger_publish_mode == "raw"
                ):
                    # Baseline / rollback: publish model chunk on receive.
                    dt = 1.0 / float(args.model_action_hz)
                    fields = dagger_pub.publish_policy_actions(
                        policy_chunk,
                        dt=dt,
                        policy_version=args.dagger_policy_version,
                        mode="raw",
                        observation_sequence=observation_sequence,
                    )
                    LOGGER.info(
                        "DAGGER raw PolicyActionChunk steps=%d dt=%.4f "
                        "stamp=%.3f interval_ms=%s obs_seq=%s",
                        fields["chunk_size"],
                        fields["dt"],
                        fields.get("header_stamp_sec", float("nan")),
                        fields.get("publish_interval_ms"),
                        observation_sequence,
                    )
                chunk = policy_actions_to_robot(policy_chunk)
                accepted, stale = actions.merge(
                    chunk,
                    int(meta.get("action_start_sequence", observation_sequence)),
                    args.action_stride,
                )
                selected_action_indices = list(range(0, len(chunk), args.action_stride))
                trajectory_recorder.record(
                    "policy_chunk",
                    request_id=response_request_id,
                    observation_sequence=observation_sequence,
                    action_start_sequence=int(
                        meta.get("action_start_sequence", observation_sequence)
                    ),
                    inference_time_ms=float(meta.get("inference_time_ms", 0.0)),
                    accepted_actions=accepted,
                    stale_actions=stale,
                    action_queue_depth=len(actions),
                    action_stride=args.action_stride,
                    model_action_hz=args.model_action_hz,
                    execution_action_hz=args.action_hz,
                    time_scale=execution_rate_ratio,
                    selected_action_indices=selected_action_indices,
                    policy_actions_radian_16d=policy_chunk.tolist(),
                    actions_bimanual_16d=_right_chunk_payload(chunk),
                )
                received_first_action_chunk = True
                must_go = True
                LOGGER.info(
                    "Received chunk obs=%s, stride=%d, accepted=%d, stale=%d, "
                    "queue=%d, inference=%.1f ms",
                    meta.get("observation_sequence"),
                    args.action_stride,
                    accepted,
                    stale,
                    len(actions),
                    float(meta.get("inference_time_ms", 0.0)),
                )

            next_action = actions.pop_next()
            if next_action is not None:
                timestep, raw_action = next_action
                if cycle_rt is None or cycle_state is None:
                    current_rt = robot.get_rt_dict()
                    current = state_from_rt(
                        current_rt,
                        gripper.state(args.gripper_feedback_max_age, enabled_arms),
                        enabled_arms,
                    )
                else:
                    current_rt = cycle_rt
                    current = cycle_state
                try:
                    action = validate_action(
                        raw_action,
                        current,
                        args.hard_arm_jump_deg,
                        args.max_gripper_step,
                        enabled_arms,
                        arm_reference=last_accepted_action,
                    )
                except ValueError as exc:
                    consecutive_invalid_action_replans += 1
                    # Stop pursuing the preceding controller target while a
                    # fresh observation is requested.  Merely clearing the
                    # Python queue would leave the impedance controller moving
                    # toward the last command.
                    command_limiter.reset(current[7:14])
                    tracking_error_guard.reset_active()
                    if not args.dry_run:
                        execute_action(robot, gripper, current, enabled_arms)
                    last_sent_action = current.copy()
                    action_history.reset()
                    pending_history_command = None
                    latest_observation_rt = current_rt
                    latest_observation_state = current.copy()
                    dropped = actions.clear_pending()
                    invalidated_requests = len(pending_requests)
                    pending_requests.clear()
                    # Give the recovery observation a sequence that is newer
                    # than both the rejected action and every pre-rejection
                    # reply. Otherwise an old in-flight request can poison the
                    # runtime watchdog while its stale chunk is being ignored.
                    actions.latest_executed = max(
                        actions.latest_executed,
                        timestep,
                    ) + 1
                    min_action_observation_sequence = actions.latest_executed
                    latest_observation_timestep = actions.latest_executed
                    must_go = True
                    trajectory_recorder.record(
                        "action_rejected",
                        policy_timestep=timestep,
                        reason=str(exc),
                        queued_actions_dropped=dropped,
                        pending_policy_requests_invalidated=invalidated_requests,
                        next_observation_timestep=actions.latest_executed,
                        raw_action=_right_vector_payload(raw_action),
                        feedback_state=_right_vector_payload(current),
                        right_arm_sdk=_right_arm_rt_payload(current_rt),
                        gripper=gripper.diagnostics(),
                        hold_command=_right_vector_payload(current),
                        replan_attempt=consecutive_invalid_action_replans,
                    )
                    LOGGER.warning(
                        "Rejected unsafe action[%d]: %s; dropped %d queued "
                        "actions, invalidated %d policy request(s), and requesting "
                        "fresh observation %d (continuous replan attempt %d)",
                        timestep,
                        exc,
                        dropped,
                        invalidated_requests,
                        actions.latest_executed,
                        consecutive_invalid_action_replans,
                    )
                    last_accepted_action = current.copy()
                else:
                    consecutive_invalid_action_replans = 0
                    enabled_joint_indices = np.asarray(
                        [
                            index
                            for arm_idx, offset in ((0, 0), (1, 7))
                            if arm_idx in enabled_arms
                            for index in range(offset, offset + 7)
                        ],
                        dtype=np.int64,
                    )
                    raw_policy_delta = (
                        np.asarray(raw_action[enabled_joint_indices], dtype=np.float64)
                        - np.asarray(
                            last_accepted_action[enabled_joint_indices],
                            dtype=np.float64,
                        )
                    )
                    max_raw_policy_jump_deg = float(
                        np.max(np.abs(raw_policy_delta))
                    )
                    policy_jump_scale = min(
                        1.0,
                        args.hard_arm_jump_deg / max_raw_policy_jump_deg,
                    ) if max_raw_policy_jump_deg > 0.0 else 1.0
                    policy_jump_projected = policy_jump_scale < 1.0
                    last_accepted_action[:14] = action[:14]
                    # Track the command the controller has actually received,
                    # not a possibly distant policy goal that is still being
                    # approached by the continuous setpoint limiter.
                    raw_arm_tracking_error = (
                        np.asarray(last_sent_action[:14], dtype=np.float64)
                        - np.asarray(current[:14], dtype=np.float64)
                    )
                    tracking_error_mask = (
                        np.abs(raw_arm_tracking_error) > args.max_arm_step_deg
                    )
                    tracking_status = (
                        tracking_error_guard.update(tracking_error_mask)
                        if args.tracking_error_replan
                        else None
                    )
                    if tracking_status is not None and tracking_status.triggered:
                        # A brief target/feedback difference is normal and the
                        # robot controller is allowed to catch up. Only a
                        # sustained following failure holds measured position
                        # and invalidates the old policy phase.
                        command_limiter.reset(current[7:14])
                        last_accepted_action = current.copy()
                        if not args.dry_run:
                            execute_action(robot, gripper, current, enabled_arms)
                        last_sent_action = current.copy()
                        action_history.reset()
                        pending_history_command = None
                        latest_observation_rt = current_rt
                        latest_observation_state = current.copy()
                        dropped = actions.clear_pending()
                        invalidated_requests = len(pending_requests)
                        pending_requests.clear()
                        actions.latest_executed = max(
                            actions.latest_executed,
                            timestep,
                        ) + 1
                        min_action_observation_sequence = actions.latest_executed
                        latest_observation_timestep = actions.latest_executed
                        must_go = True
                        trajectory_recorder.record(
                            "tracking_error_replan",
                            policy_timestep=timestep,
                            active_joints=list(tracking_status.active_joints),
                            active_joint_labels=_arm_joint_labels(
                                tracking_status.active_joints
                            ),
                            timed_out_joints=list(tracking_status.timed_out_joints),
                            timed_out_joint_labels=_arm_joint_labels(
                                tracking_status.timed_out_joints
                            ),
                            replan_attempt=tracking_status.replan_attempt,
                            queued_actions_dropped=dropped,
                            pending_policy_requests_invalidated=invalidated_requests,
                            next_observation_timestep=actions.latest_executed,
                            raw_action=_right_vector_payload(raw_action),
                            validated_action=_right_vector_payload(action),
                            feedback_state=_right_vector_payload(current),
                            hold_command=_right_vector_payload(current),
                            right_arm_sdk=_right_arm_rt_payload(current_rt),
                            gripper=gripper.diagnostics(),
                        )
                        LOGGER.warning(
                            "Tracking error persisted on %s; "
                            "holding feedback position, dropped %d queued actions, "
                            "and requesting fresh observation %d "
                            "(continuous replan attempt %d)",
                            _arm_joint_labels(tracking_status.timed_out_joints),
                            dropped,
                            actions.latest_executed,
                            tracking_status.replan_attempt,
                        )
                    else:
                        decision_gripper = gripper.diagnostics()
                        held_action = gripper_command_processor.apply(
                            action,
                            enabled_arms,
                            contact_latched={
                                0: bool(decision_gripper["left"]["contact_latched"]),
                                1: bool(decision_gripper["right"]["contact_latched"]),
                            },
                        )
                        shaped_action = held_action.copy()
                        command_velocity_deg_s = (
                            np.asarray(action[7:14], dtype=np.float64)
                            - np.asarray(current[7:14], dtype=np.float64)
                        ) * args.action_hz
                        velocity_limited_joints: list[int] = []
                        acceleration_limited_joints: list[int] = []
                        tracking_error_joints: list[int] = []
                        # Publish exactly one arm setpoint per action period.
                        # Slew all enabled arm joints proportionally from the
                        # preceding published setpoint, so the bimanual vector
                        # stays continuous and no action-boundary feedback
                        # restart can reverse the command. Grippers retain the
                        # existing immediate direct/postprocess target semantics.
                        command = shaped_action.copy()
                        command_delta = (
                            np.asarray(shaped_action[enabled_joint_indices], dtype=np.float64)
                            - np.asarray(
                                last_sent_action[enabled_joint_indices], dtype=np.float64
                            )
                        )
                        max_setpoint_delta_deg = float(np.max(np.abs(command_delta)))
                        setpoint_step_scale = (
                            1.0
                            if args.arm_command_style == "ptp"
                            else (
                                min(
                                    1.0,
                                    args.interpolation_step_deg / max_setpoint_delta_deg,
                                )
                                if max_setpoint_delta_deg > 0.0
                                else 1.0
                            )
                        )
                        command[enabled_joint_indices] = (
                            np.asarray(
                                last_sent_action[enabled_joint_indices], dtype=np.float64
                            )
                            + command_delta * setpoint_step_scale
                        )
                        command_velocity_deg_s = (
                            np.asarray(command[7:14], dtype=np.float64)
                            - np.asarray(last_sent_action[7:14], dtype=np.float64)
                        ) * args.action_hz
                        trajectory_recorder.record(
                            "action_decision",
                            policy_timestep=timestep,
                            raw_action=_right_vector_payload(raw_action),
                            validated_action=_right_vector_payload(action),
                            safe_action=_right_vector_payload(shaped_action),
                            gripper_control=gripper_command_processor.diagnostics(),
                            feedback_state=_right_vector_payload(current),
                            right_arm_sdk=_right_arm_rt_payload(current_rt),
                            gripper=decision_gripper,
                            execution_mode="direct_guard",
                            policy_jump_projected=policy_jump_projected,
                            policy_jump_scale=policy_jump_scale,
                            max_raw_policy_jump_deg=max_raw_policy_jump_deg,
                            command_velocity_deg_s=command_velocity_deg_s.tolist(),
                            velocity_limited_joints=velocity_limited_joints,
                            acceleration_limited_joints=acceleration_limited_joints,
                            tracking_error_joints=tracking_error_joints,
                            interpolation_count=1,
                            interpolation_start="last_sent_action",
                            max_setpoint_delta_deg=max_setpoint_delta_deg,
                            setpoint_step_scale=setpoint_step_scale,
                        )
                        if (
                            args.dagger_mode
                            and dagger_pub is not None
                            and args.dagger_publish_mode == "setpoint"
                            and dagger_err_guard is not None
                        ):
                            # Micro-ramp from previous published cmd → this tick's
                            # slewed command. ERR_MAX freezes advance (start=end).
                            err_decision = dagger_err_guard.evaluate(
                                last_sent_action, current
                            )
                            publish_end = (
                                last_sent_action.copy()
                                if err_decision.hold
                                else command.copy()
                            )
                            if err_decision.hold:
                                command = publish_end.copy()
                            start_policy, end_policy = robot_commands_to_setpoint_policy(
                                last_sent_action, publish_end
                            )
                            fields = dagger_pub.publish_setpoint_ramp(
                                start_policy,
                                end_policy,
                                dt=micro_ramp_dt(args.model_action_hz),
                                policy_version=args.dagger_policy_version,
                                observation_sequence=latest_observation_timestep,
                            )
                            LOGGER.info(
                                "DAGGER setpoint PolicyActionChunk steps=%d "
                                "dt=%.5f err_rad=%.4f hold=%s trip=%d "
                                "stamp=%.3f interval_ms=%s",
                                fields["chunk_size"],
                                fields["dt"],
                                err_decision.err_rad,
                                err_decision.hold,
                                err_decision.trip_count,
                                fields.get("header_stamp_sec", float("nan")),
                                fields.get("publish_interval_ms"),
                            )
                        if args.dry_run:
                            LOGGER.info(
                                "dry-run action[%d]: %s",
                                timestep,
                                command.round(3).tolist(),
                            )
                            trajectory_recorder.record(
                                "command_preview",
                                policy_timestep=timestep,
                                interpolation_index=0,
                                interpolation_count=1,
                                command=_right_vector_payload(command),
                                gripper=gripper.diagnostics(),
                            )
                        else:
                            try:
                                execute_action(robot, gripper, command, enabled_arms)
                            except BaseException as exc:
                                trajectory_recorder.record(
                                    "command_failed",
                                    policy_timestep=timestep,
                                    interpolation_index=0,
                                    interpolation_count=1,
                                    command=_right_vector_payload(command),
                                    error_type=type(exc).__name__,
                                    message=str(exc),
                                    gripper=gripper.diagnostics(),
                                )
                                raise
                            trajectory_recorder.record(
                                "command_sent",
                                policy_timestep=timestep,
                                interpolation_index=0,
                                interpolation_count=1,
                                command=_right_vector_payload(command),
                                gripper=gripper.diagnostics(),
                            )
                        last_sent_action = command.copy()
                        pending_history_command = last_sent_action.copy()

            # Sample physical feedback once per control cycle while an operator
            # segment is active.  This is deliberately done in the main thread:
            # the Gento SDK connection is not shared with another polling thread.
            # A recorder-only read failure is captured but never changes motion.
            if trajectory_recorder.is_recording:
                try:
                    feedback_rt = robot.get_rt_dict()
                    feedback_state = state_from_rt(
                        feedback_rt,
                        gripper.state(args.gripper_feedback_max_age, enabled_arms),
                        enabled_arms,
                    )
                    feedback_payload = {
                        "feedback_state": _right_vector_payload(feedback_state),
                        "right_arm_sdk": _right_arm_rt_payload(feedback_rt),
                        "gripper": gripper.diagnostics(),
                    }
                except Exception as exc:
                    trajectory_recorder.record(
                        "feedback_sample_failed",
                        sample_index=feedback_sample_index,
                        error_type=type(exc).__name__,
                        message=str(exc),
                    )
                else:
                    trajectory_recorder.record(
                        "feedback_sample",
                        sample_index=feedback_sample_index,
                        action_queue_depth=len(actions),
                        pending_policy_requests=len(pending_requests),
                        **feedback_payload,
                    )
                feedback_sample_index += 1

            ready = len(actions) / max(1, actions.chunk_size) <= args.chunk_size_threshold
            observation_timestep = latest_observation_timestep
            request_clock = time.monotonic()
            request_due = (
                request_clock - last_policy_request_at >= 1.0 / args.observation_hz
            )
            # The inference server has one active worker and a size-one
            # latest-value pending queue. During accelerated full-chunk playback,
            # refresh that pending slot with newer observations instead of
            # waiting for the active inference to finish. The server explicitly
            # acknowledges replaced observations as observation_dropped, and the
            # normal timestep checks still reject late action chunks.
            should_send = (
                policy_active
                and ready
                and request_due
                and len(pending_requests) < args.max_pending_policy_requests
                and (history_advanced_this_cycle or (must_go and len(actions) == 0))
                and (must_go or observation_timestep > last_sent_timestep)
            )
            if should_send:
                observation_rt = latest_observation_rt
                state = latest_observation_state.copy()
                if ros_camera_bridge is not None:
                    try:
                        images = ros_camera_bridge.read_jpegs(
                            timeout_s=args.camera_read_timeout,
                            max_age_s=args.camera_max_age,
                            sync_tolerance_s=args.camera_sync_tolerance,
                        )
                    except TimeoutError as exc:
                        # Camera loss is also a motion hold.  Reset command
                        # shaping to measured position so recovery does not
                        # resume from a pre-outage software target.
                        command_limiter.reset(state[7:14])
                        tracking_error_guard.reset_active(reset_attempts=True)
                        last_accepted_action = state.copy()
                        consecutive_invalid_action_replans = 0
                        if not args.dry_run:
                            execute_action(robot, gripper, state, enabled_arms)
                        last_sent_action = state.copy()
                        action_history.reset()
                        pending_history_command = None
                        dropped = actions.clear_pending()
                        # Invalidate every reply based on a pre-fault
                        # observation. Advancing one logical timestep lets the
                        # recovered camera pair produce a uniquely newer
                        # observation before motion resumes.
                        actions.latest_executed = max(
                            actions.latest_executed,
                            observation_timestep,
                        ) + 1
                        min_action_observation_sequence = actions.latest_executed
                        latest_observation_timestep = actions.latest_executed
                        pending_requests.clear()
                        must_go = True
                        trajectory_recorder.record(
                            "camera_hold",
                            observation_timestep=observation_timestep,
                            reason=str(exc),
                            queued_actions_dropped=dropped,
                            next_observation_timestep=actions.latest_executed,
                            feedback_state=_right_vector_payload(state),
                            right_arm_sdk=_right_arm_rt_payload(observation_rt),
                            gripper=gripper.diagnostics(),
                            hold_command=_right_vector_payload(state),
                        )
                        LOGGER.warning(
                            "Camera temporarily unavailable; holding position: "
                            "%s; dropped %d queued actions and waiting for fresh frames",
                            exc,
                            dropped,
                        )
                        time.sleep(period)
                        continue
                else:
                    images = []
                    for camera_name, camera in opencv_cameras.items():
                        jpeg, width, height = camera.read_jpeg()
                        images.append((camera_name, jpeg, width, height))
                if args.trajectory_auto_start and not auto_trajectory_start_attempted:
                    # Start only after robot/camera initialization has completed and
                    # immediately before the first policy request. This excludes the
                    # initial-pose motion while retaining the complete inference run.
                    auto_trajectory_start_attempted = True
                    if trajectory_recorder.is_recording:
                        LOGGER.info(
                            "Trajectory recording was started manually before the "
                            "first policy request; keeping that segment active"
                        )
                    else:
                        try:
                            recorder_status = trajectory_recorder.start_segment(
                                label="auto_inference"
                            )
                        except Exception as exc:
                            # Recording is diagnostic-only and must never stop motion.
                            LOGGER.error(
                                "Could not automatically start trajectory recording; "
                                "continuing inference without a segment: %s",
                                exc,
                            )
                        else:
                            active_segment = recorder_status.get("active_segment", {})
                            LOGGER.info(
                                "Automatic trajectory recording started with first "
                                "policy request: %s",
                                active_segment.get("trajectory_path", "unknown path"),
                            )
                is_must_go = must_go and len(actions) == 0
                history_actions, history_actions_is_pad = action_history.snapshot()
                rtc_action_prefix = None
                rtc_action_prefix_is_pad = None
                if args.rtc_max_prefix > 0:
                    rtc_prefix_robot, rtc_action_prefix_is_pad = (
                        actions.snapshot_rtc_prefix(
                            action_horizon=args.rtc_action_horizon,
                            max_prefix=args.rtc_max_prefix,
                        )
                    )
                    rtc_action_prefix = rtc_prefix_robot.copy()
                    rtc_action_prefix[:, :14] = np.deg2rad(
                        rtc_action_prefix[:, :14]
                    )
                request_id, frames = _observation_frames(
                    state=state,
                    history_actions=history_actions,
                    history_actions_is_pad=history_actions_is_pad,
                    rtc_action_prefix=rtc_action_prefix,
                    rtc_action_prefix_is_pad=rtc_action_prefix_is_pad,
                    episode_id=episode_id,
                    images=images,
                    timestep=observation_timestep,
                    must_go=is_must_go,
                    task=current_task,
                )
                replaced = network.send_latest(frames)
                replaced_request_id = None
                if replaced is not None:
                    replaced_meta = parse_json(replaced[0])
                    replaced_request_id = str(replaced_meta.get("request_id", ""))
                    pending_requests.pop(replaced_request_id, None)
                pending_requests[request_id] = time.monotonic()
                last_policy_request_at = request_clock
                last_sent_timestep = observation_timestep
                if is_must_go:
                    must_go = False
                recorded_cameras = (
                    trajectory_recorder.record_camera_frames(
                        observation_timestep=observation_timestep,
                        request_id=request_id,
                        images=images,
                    )
                    if args.record_camera_frames
                    else []
                )
                recorded_by_name = {
                    str(camera["name"]): camera for camera in recorded_cameras
                }
                trajectory_recorder.record(
                    "observation_sent",
                    request_id=request_id,
                    observation_timestep=observation_timestep,
                    must_go=is_must_go,
                    replaced_unsent_request_id=replaced_request_id,
                    action_queue_depth=len(actions),
                    pending_policy_requests=len(pending_requests),
                    feedback_state=_right_vector_payload(state),
                    policy_state_radian_16d=robot_state_to_policy(state).tolist(),
                    history_actions_radian_16d=history_actions.tolist(),
                    history_actions_is_pad=history_actions_is_pad.tolist(),
                    rtc_action_prefix_radian_16d=(
                        None
                        if rtc_action_prefix is None
                        else rtc_action_prefix.tolist()
                    ),
                    rtc_action_prefix_is_pad=(
                        None
                        if rtc_action_prefix_is_pad is None
                        else rtc_action_prefix_is_pad.tolist()
                    ),
                    episode_id=episode_id,
                    right_arm_sdk=_right_arm_rt_payload(observation_rt),
                    gripper=gripper.diagnostics(),
                    cameras=[
                        {
                            "name": name,
                            "width": width,
                            "height": height,
                            **(
                                {
                                    "jpeg_bytes": recorded_by_name[name]["jpeg_bytes"],
                                    "recording_path": recorded_by_name[name][
                                        "recording_path"
                                    ],
                                }
                                if name in recorded_by_name
                                else {}
                            ),
                        }
                        for name, _jpeg, width, height in images
                    ],
                )
                LOGGER.debug(
                    "Sent observation #%d (must_go=%s, queue=%d)",
                    observation_timestep,
                    is_must_go,
                    len(actions),
                )

            if len(actions) == 0 and pending_requests:
                oldest_request_id, oldest_request_at = min(
                    pending_requests.items(),
                    key=lambda item: item[1],
                )
                oldest_age_ms = (time.monotonic() - oldest_request_at) * 1000
                watchdog_timeout_ms = (
                    args.timeout_ms
                    if received_first_action_chunk
                    else args.startup_timeout_ms
                )
                if oldest_age_ms > watchdog_timeout_ms:
                    timeout_phase = (
                        "runtime"
                        if received_first_action_chunk
                        else "first action (policy cold start)"
                    )
                    trajectory_recorder.record(
                        "watchdog_timeout",
                        phase=timeout_phase,
                        timeout_ms=watchdog_timeout_ms,
                        oldest_request_id=oldest_request_id,
                        oldest_request_age_ms=oldest_age_ms,
                        pending_policy_requests=len(pending_requests),
                    )
                    raise TimeoutError(
                        f"policy server did not return {timeout_phase} actions within "
                        f"{watchdog_timeout_ms} ms "
                        f"(oldest_request={oldest_request_id}, "
                        f"age={oldest_age_ms:.0f} ms, "
                        f"outstanding={len(pending_requests)})"
                    )

            runtime_control.update_status(
                state="policy" if policy_active else "hold",
                ready=True,
                task=current_task,
                generation=runtime_generation,
                action_queue_depth=len(actions),
                pending_policy_requests=len(pending_requests),
            )

            elapsed = time.perf_counter() - started
            # Pace the complete loop, including SDK reads, command publication,
            # recording and network work, to the configured action frequency.
            # Sleeping a full period immediately after sending would add all of
            # that processing time and previously stretched nominal 30 Hz to
            # roughly 23 Hz in recorded runs.
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        # Initial-pose/gripper interpolation raises KeyboardInterrupt after
        # the signal handler sets stop_event. This is still a requested,
        # controlled shutdown, not a robot fault.
        requested_signal = requested_signal or signal.SIGINT
        trajectory_recorder.record(
            "client_stop_requested",
            signal=requested_signal,
        )
        LOGGER.info("Stop requested; stopping commands and returning arm to IDLE")
        return 0
    except InitialPosePlanRejected as exc:
        # No command was sent. Return the arms to IDLE instead of emergency stop.
        plan_rejected = True
        trajectory_recorder.record(
            "client_error",
            error_type=type(exc).__name__,
            message=str(exc),
        )
        LOGGER.error("Initial pose plan rejected; returning arms to IDLE: %s", exc)
        return 1
    except (TimeoutError, zmq.ZMQError) as exc:
        if requested_signal is not None:
            trajectory_recorder.record(
                "client_stop_requested",
                signal=requested_signal,
                message=str(exc),
            )
            LOGGER.info("Stop requested; stopping commands and returning arm to IDLE")
            return 0
        fatal_shutdown = True
        trajectory_recorder.record(
            "client_error",
            error_type=type(exc).__name__,
            message=str(exc),
        )
        LOGGER.error("Stopped: %s", exc)
        return 1
    except BaseException as exc:
        fatal_shutdown = True
        trajectory_recorder.record(
            "client_error",
            error_type=type(exc).__name__,
            message=str(exc),
        )
        raise
    finally:
        runtime_control.update_status(state="stopping", ready=False)
        trajectory_recorder.record(
            "client_shutdown_started",
            requested_signal=requested_signal,
            fatal_shutdown=fatal_shutdown,
        )
        gripper_sampler_stop.set()
        if gripper_sampler_thread is not None:
            gripper_sampler_thread.join(timeout=2.0)
        shutdown_reason = (
            f"signal_{requested_signal}"
            if requested_signal is not None and not fatal_shutdown
            else "fatal_shutdown"
            if fatal_shutdown
            else "client_shutdown"
        )
        try:
            network.close()
            if gripper is not None:
                gripper.close()
                gripper = None
            if args.dagger_mode:
                if dagger_pub is not None:
                    dagger_pub.close()
                    dagger_pub = None
                if getattr(robot, "_connected", False):
                    robot.unlink()
            elif robot._connected:
                if not args.dry_run:
                    graceful_stop = (
                        not fatal_shutdown
                        and (requested_signal is not None or plan_rejected)
                    )
                    if graceful_stop:
                        idle_failed = False
                        for arm_idx, obj in (
                            (0, FXObjType.OBJ_ARM0),
                            (1, FXObjType.OBJ_ARM1),
                        ):
                            if arm_idx not in enabled_arms:
                                continue
                            ret = robot.switch_to_idle(obj, 3000)
                            if ret != 0:
                                LOGGER.error(
                                    "Controlled stop could not switch arm %d to "
                                    "IDLE: ret=%d (%s); using emergency stop",
                                    arm_idx,
                                    ret,
                                    robot._get_operate_error_msg(ret),
                                )
                                idle_failed = True
                                break
                        if idle_failed:
                            robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
                        else:
                            LOGGER.info(
                                "Controlled stop complete: enabled arms are IDLE; "
                                "controller will not be left in software emergency"
                            )
                    else:
                        robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
                robot.unlink()
            if ros_camera_bridge is not None:
                ros_camera_bridge.close()
            for camera in opencv_cameras.values():
                camera.close()
        finally:
            runtime_control.close()
            trajectory_recorder.close(reason=shutdown_reason)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
