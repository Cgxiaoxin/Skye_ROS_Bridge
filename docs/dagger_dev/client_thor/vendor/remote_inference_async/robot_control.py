"""Tianji right-arm SDK boundary used by the async robot client."""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Protocol

import os

import cv2
import numpy as np

# DAgger/HITL path never links Gento; avoid requiring PYTHON_SDK (and loading
# libGentoSDKPY) while skye_robot_driver already owns the controller.
if os.environ.get("DAGGER_MODE", "0") == "1":
    class FXObjType:  # noqa: D101 — stub for annotations / defaults only
        OBJ_ARM0 = 0
        OBJ_ARM1 = 1

    class FXStateType:  # noqa: D101
        FX_STATE_IDLE = 0
        FX_STATE_POSITION = 1
        FX_STATE_IMP_JOINT = 2
        FX_STATE_ERROR = 100
        FX_STATE_UNKNOWN = 200

    GentoRobot = None  # type: ignore
    error_dict: dict = {}
else:
    from PYTHON_SDK.GentoRobot import (  # type: ignore
        FXObjType,
        FXStateType,
        GentoRobot,
        error_dict,
    )
from remote_inference_async.protocol import ACTION_DIM, STATE_DIM, split_action


LOGGER = logging.getLogger("tianji_async_client")
GRIPPER_COMMAND_MAX = 0.9
JOINT_IMPEDANCE_K = [2.0, 2.0, 2.0, 1.6, 1.0, 2.0, 1.0]
JOINT_IMPEDANCE_D = [0.6, 0.6, 0.6, 0.6, 0.6, 0.8, 0.6]
TOOL_DYNAMICS = [1.3, 0.0, 0.0, 80.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
INITIAL_LEFT_ARM_POSE = np.array(
    [-166.551, -27.824, 142.332, -125.861, 91.891, -33.642, 38.912],
    dtype=np.float32,
)

INITIAL_RIGHT_ARM_POSE = np.array(
    [-138.486, -24.946, 167.126, -130.223, -97.531, -36.204, -12.739],
    dtype=np.float32,
)
INITIAL_GRIPPER_SETTLE_SAMPLES = 3
MODE_TRANSITION_STABLE_S = 1.0
INVALID_CONDITION_RET = -6


class GripperBackend(Protocol):
    def state(
        self,
        max_age_s: float,
        enabled_arms: set[int] | None = None,
    ) -> np.ndarray: ...

    def command(
        self,
        values: np.ndarray,
        enabled_arms: set[int] | None = None,
    ) -> None: ...


class GripperCommandHold:
    """Suppress small policy open-command oscillations during an active grasp.

    Once a policy command crosses ``enter_command``, retain the largest close
    request seen for that arm.  Release it only when the policy makes an
    unambiguous open request at or below ``release_command``.  This state is
    deliberately applied only to online policy actions, not initialization or
    shutdown commands sent directly to the gripper backend.
    """

    def __init__(
        self,
        *,
        enter_command: float,
        release_command: float,
        contact_release_command: float | None = None,
        open_hold_cycles: int = 0,
    ) -> None:
        if contact_release_command is None:
            contact_release_command = release_command
        if not (
            math.isfinite(enter_command)
            and math.isfinite(release_command)
            and math.isfinite(contact_release_command)
            and 0.0
            <= release_command
            <= contact_release_command
            < enter_command
            <= GRIPPER_COMMAND_MAX
        ):
            raise ValueError(
                "gripper hold thresholds must satisfy "
                "0 <= release <= contact release < enter <= 0.9"
            )
        if open_hold_cycles < 0:
            raise ValueError("gripper open hold cycles must be non-negative")
        self.enter_command = float(enter_command)
        self.release_command = float(release_command)
        self.contact_release_command = float(contact_release_command)
        self.open_hold_cycles = int(open_hold_cycles)
        self._latched = [False, False]
        self._held_command = [0.0, 0.0]
        self._last_policy_command = [0.0, 0.0]
        self._open_cycles_remaining = [0, 0]
        self._contact_latched = [False, False]

    def reset(self) -> None:
        """Forget policy-phase latches when a new motion phase starts."""
        self._latched = [False, False]
        self._held_command = [0.0, 0.0]
        self._last_policy_command = [0.0, 0.0]
        self._open_cycles_remaining = [0, 0]
        self._contact_latched = [False, False]

    def apply(
        self,
        action: np.ndarray,
        enabled_arms: set[int],
        contact_latched: dict[int, bool] | None = None,
    ) -> np.ndarray:
        held = np.asarray(action, dtype=np.float32).copy()
        if held.shape != (ACTION_DIM,) or not np.isfinite(held).all():
            raise ValueError(f"invalid action for gripper hold: {held.shape}")
        for arm_idx in enabled_arms:
            gripper_idx = 14 + arm_idx
            command = float(np.clip(held[gripper_idx], 0.0, GRIPPER_COMMAND_MAX))
            self._last_policy_command[arm_idx] = command
            contact = bool(contact_latched and contact_latched.get(arm_idx, False))
            self._contact_latched[arm_idx] = contact
            if self._open_cycles_remaining[arm_idx] > 0:
                self._open_cycles_remaining[arm_idx] -= 1
                self._latched[arm_idx] = False
                self._held_command[arm_idx] = 0.0
                command = 0.0
            elif self._latched[arm_idx]:
                release_threshold = (
                    self.contact_release_command if contact else self.release_command
                )
                if command <= release_threshold:
                    self._latched[arm_idx] = False
                    self._open_cycles_remaining[arm_idx] = self.open_hold_cycles
                    self._held_command[arm_idx] = 0.0
                    command = 0.0
                else:
                    self._held_command[arm_idx] = max(
                        self._held_command[arm_idx], command
                    )
                    command = self._held_command[arm_idx]
            elif command >= self.enter_command:
                self._latched[arm_idx] = True
                self._held_command[arm_idx] = command
            else:
                self._held_command[arm_idx] = command
            held[gripper_idx] = command
        return held

    def diagnostics(self) -> dict[str, object]:
        return {
            label: {
                "latched": self._latched[arm_idx],
                "policy_command": self._last_policy_command[arm_idx],
                "held_command": self._held_command[arm_idx],
                "open_cycles_remaining": self._open_cycles_remaining[arm_idx],
                "contact_latched": self._contact_latched[arm_idx],
                "active_release_threshold": (
                    self.contact_release_command
                    if self._contact_latched[arm_idx]
                    else self.release_command
                ),
            }
            for arm_idx, label in ((0, "left"), (1, "right"))
        }


class GripperCommandProcessor:
    """Select direct policy commands or the optional grasp postprocessor."""

    def __init__(
        self,
        *,
        mode: str,
        enter_command: float,
        release_command: float,
        contact_release_command: float,
        open_hold_cycles: int,
    ) -> None:
        if mode not in ("direct", "postprocess"):
            raise ValueError("gripper control mode must be direct or postprocess")
        self.mode = mode
        self._hold = (
            GripperCommandHold(
                enter_command=enter_command,
                release_command=release_command,
                contact_release_command=contact_release_command,
                open_hold_cycles=open_hold_cycles,
            )
            if mode == "postprocess"
            else None
        )

    def apply(
        self,
        action: np.ndarray,
        enabled_arms: set[int],
        contact_latched: dict[int, bool] | None = None,
    ) -> np.ndarray:
        if self._hold is not None:
            return self._hold.apply(action, enabled_arms, contact_latched)
        direct = np.asarray(action, dtype=np.float32).copy()
        if direct.shape != (ACTION_DIM,) or not np.isfinite(direct).all():
            raise ValueError(f"invalid direct gripper action: {direct.shape}")
        return direct

    def diagnostics(self) -> dict[str, object]:
        return {
            "mode": self.mode,
            "postprocess_enabled": self._hold is not None,
            "hold": self._hold.diagnostics() if self._hold is not None else None,
        }

    def reset(self) -> None:
        """Reset state that must not cross policy-task boundaries."""
        if self._hold is not None:
            self._hold.reset()


def arm_diagnostics(robot: GentoRobot, obj: int) -> str:
    """Capture controller and per-axis details before cleanup changes state."""
    parts: list[str] = []
    try:
        rt = robot.get_rt_dict()
        if "error" in rt:
            parts.append(f"rt_error={rt['error']}")
        else:
            arm = rt["arms"][int(obj)]
            system_error = int(arm["state"]["err"])
            error_name = error_dict.get(system_error, "Unknown")
            parts.append(
                "controller_state="
                f"{arm['state']['cur']}, commanded_state={arm['state']['cmd']}, "
                f"system_error=0x{system_error:08X} ({error_name})"
            )
            parts.append(f"joint_feedback={arm['fb']['fb_pos']}")
            parts.append(f"joint_sensor_torque={arm['fb']['fb_sensor']}")
            parts.append(f"joint_external_torque={arm['fb']['fb_ext_torque']}")
    except Exception as exc:
        parts.append(f"rt_diagnostics_failed={exc}")
    try:
        physical_ret, physical_state = robot.get_ctrl_obj_physical_state(obj)
        parts.append(f"physical_state={physical_state} (ret={physical_ret})")
    except Exception as exc:
        parts.append(f"physical_state_failed={exc}")
    try:
        servo_ret, servo_errors = robot.get_servo_error_codes(obj)
        parts.append(
            f"servo_errors(ret={servo_ret}):\n"
            f"{servo_errors if servo_errors else 'unavailable'}"
        )
    except Exception as exc:
        parts.append(f"servo_errors_failed={exc}")
    try:
        sensor_ret, sensor_versions, sensor_serials = (
            robot.get_ctrl_obj_sensor_version_and_serial(obj)
        )
        parts.append(
            f"sensor_versions(ret={sensor_ret})={sensor_versions}, "
            f"sensor_serials={sensor_serials}"
        )
    except Exception as exc:
        parts.append(f"sensor_diagnostics_failed={exc}")
    return "; ".join(parts)


def _servo_errors_are_clear(robot: GentoRobot, obj: int) -> bool:
    servo_ret, servo_errors = robot.get_servo_error_codes(obj)
    servo_lines = [line for line in servo_errors.splitlines() if line.strip()]
    return (
        servo_ret == 0
        and len(servo_lines) == 7
        and all("0x0000" in line for line in servo_lines)
    )


def _arm_system_error(robot: GentoRobot, obj: int) -> int:
    rt = robot.get_rt_dict()
    if "error" in rt:
        raise RuntimeError(f"cannot inspect arm {obj} error: {rt['error']}")
    return int(rt["arms"][int(obj)]["state"]["err"])


def _reset_recoverable_arm_error(robot: GentoRobot, obj: int) -> None:
    """Clear a known recoverable controller error when every servo is healthy."""
    system_error = _arm_system_error(robot, obj)
    # 101: software emergency left by a shutdown path.
    # 106: controller failed to request the torque mode used by impedance.
    # 110: controller did not receive the expected disable-servo response
    #      while handling the preceding emergency stop.  The SDK names this
    #      ERR_ResponseDisableServo and its official ResetError example uses
    #      reset_error() to return an ERROR arm to IDLE.
    # These are controller state-machine errors and are safe to reset only
    # when all seven servo drives explicitly report no underlying error.
    if system_error not in (101, 106, 110) or not _servo_errors_are_clear(robot, obj):
        raise RuntimeError(
            f"arm {obj} starts in ERROR and is not a known recoverable "
            f"controller error with all servos healthy; automatic reset "
            f"refused; {arm_diagnostics(robot, obj)}"
        )

    LOGGER.warning(
        "Arm %d starts with recoverable controller error %d (%s) and all "
        "servo errors clear; resetting to IDLE",
        int(obj),
        system_error,
        error_dict.get(system_error, "Unknown"),
    )
    ret, remaining_error = robot.reset_error(obj, 3000)
    if ret != 0:
        raise RuntimeError(
            f"failed to clear arm {obj} recoverable controller error "
            f"{system_error}: ret={ret}, "
            f"error=0x{remaining_error:08X}; {arm_diagnostics(robot, obj)}"
        )
    state = robot.current_state(obj)
    if state != FXStateType.FX_STATE_IDLE:
        raise RuntimeError(
            f"arm {obj} error reset returned success but state={state}; "
            f"{arm_diagnostics(robot, obj)}"
        )


def _wait_for_idle_and_low_speed(
    robot: GentoRobot,
    obj: int,
    settle_timeout_s: float,
    stable_duration_s: float = MODE_TRANSITION_STABLE_S,
) -> None:
    """Require a continuous IDLE/low-speed window before changing modes."""

    if settle_timeout_s <= 0 or stable_duration_s < 0:
        raise ValueError("mode-transition timing values must be non-negative")
    deadline = time.monotonic() + settle_timeout_s
    last_state = robot.current_state(obj)
    last_low_speed_flag: int | None = None
    stable_since: float | None = None
    while time.monotonic() < deadline:
        last_state = robot.current_state(obj)
        sg = robot.get_sg_dict()
        last_low_speed_flag = int(sg["arms"][obj]["get"]["low_speed_flag"])
        if last_state == FXStateType.FX_STATE_ERROR:
            raise RuntimeError(
                f"arm {obj} entered ERROR while waiting to settle; "
                f"{arm_diagnostics(robot, obj)}"
            )
        now = time.monotonic()
        if last_state == FXStateType.FX_STATE_IDLE and last_low_speed_flag == 1:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= stable_duration_s:
                return
        else:
            stable_since = None
        time.sleep(0.02)
    raise TimeoutError(
        f"arm {obj} did not settle within {settle_timeout_s:.1f}s: "
        f"state={last_state}, low_speed_flag={last_low_speed_flag}, "
        f"required_stable_duration={stable_duration_s:.1f}s"
    )


class Camera:
    """Compatibility OpenCV source; the deployed setup uses ROS 2 cameras."""

    def __init__(self, device: str, width: int, height: int, jpeg_quality: int):
        source = int(device) if device.isdigit() else device
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.jpeg_quality = jpeg_quality
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera {device}")

    def read_jpeg(self) -> tuple[bytes, int, int]:
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("camera read failed")
        height, width = frame.shape[:2]
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        return encoded.tobytes(), width, height

    def close(self) -> None:
        self.cap.release()


def state_from_rt(
    rt: dict,
    grippers: np.ndarray,
    enabled_arms: set[int] | None = None,
) -> np.ndarray:
    if "error" in rt:
        raise RuntimeError(rt["error"])
    enabled = {0, 1} if enabled_arms is None else enabled_arms
    grippers = np.asarray(grippers, dtype=np.float32)
    if grippers.shape != (2,) or not np.isfinite(grippers).all():
        raise RuntimeError(f"invalid gripper state shape/value: {grippers.shape}")
    state = np.zeros(STATE_DIM, dtype=np.float32)
    for arm_idx, offset in ((0, 0), (1, 7)):
        if arm_idx in enabled:
            state[offset : offset + 7] = np.asarray(
                rt["arms"][arm_idx]["fb"]["fb_pos"], dtype=np.float32
            )
            state[14 + arm_idx] = grippers[arm_idx]
    if not np.isfinite(state).all():
        raise RuntimeError("robot state contains NaN or infinity")
    return state


def prepare_arm_position(
    robot: GentoRobot,
    obj: int,
    velocity: float,
    acceleration: float,
    settle_timeout_s: float = 10.0,
) -> None:
    """Recover ARM to a current-pose hold in position mode."""
    state = robot.current_state(obj)
    if state == FXStateType.FX_STATE_ERROR:
        _reset_recoverable_arm_error(robot, obj)
        time.sleep(1.0)
        state = robot.current_state(obj)
    if state not in (FXStateType.FX_STATE_IDLE, FXStateType.FX_STATE_POSITION):
        ret = robot.switch_to_idle(obj, 2000)
        if ret != 0:
            raise RuntimeError(
                f"switch_to_idle({obj}) before initial position move failed: "
                f"{ret} ({robot._get_operate_error_msg(ret)}); "
                f"{arm_diagnostics(robot, obj)}"
            )
        state = robot.current_state(obj)
    if state == FXStateType.FX_STATE_IDLE:
        _wait_for_idle_and_low_speed(robot, obj, settle_timeout_s)

    rt = robot.get_rt_dict()
    if "error" in rt:
        raise RuntimeError(
            f"cannot read arm {obj} before initial position mode: {rt['error']}"
        )
    current = np.asarray(
        rt["arms"][int(obj)]["fb"]["fb_pos"],
        dtype=np.float64,
    )
    if current.shape != (7,) or not np.isfinite(current).all():
        raise RuntimeError(
            f"invalid arm {obj} feedback before initial position mode: {current}"
        )

    # Preload the current feedback as the position target before changing
    # modes. This prevents an old position command from causing a jump while
    # the arm is still in contact with its environment.
    ret = robot.runtime_set_joint_pos_cmd(obj, current.tolist())
    if ret != 0:
        raise RuntimeError(
            f"failed to preload arm {obj} current position: {ret} "
            f"({robot._get_operate_error_msg(ret)})"
        )
    ret = robot.switch_to_position_mode(
        obj,
        3000,
        velocity,
        acceleration,
    )
    if ret != 0:
        raise RuntimeError(
            f"switch_to_position_mode({obj}) failed: {ret} "
            f"({robot._get_operate_error_msg(ret)}); "
            f"{arm_diagnostics(robot, obj)}"
        )
    ret = robot.runtime_set_joint_pos_cmd(obj, current.tolist())
    if ret != 0:
        raise RuntimeError(
            f"failed to hold arm {obj} after entering position mode: {ret} "
            f"({robot._get_operate_error_msg(ret)})"
        )
    LOGGER.info(
        "Arm %d ready in position mode, holding current pose: %s",
        int(obj),
        current.round(3).tolist(),
    )


def prepare_arm(
    robot: GentoRobot,
    obj: int,
    velocity: float,
    acceleration: float,
    settle_timeout_s: float = 10.0,
) -> None:
    state = robot.current_state(obj)
    if state == FXStateType.FX_STATE_ERROR:
        _reset_recoverable_arm_error(robot, obj)
        # reset_error() reports IDLE before every controller/servo mode state
        # machine is necessarily ready for a new torque-mode request. Position
        # mode tolerated this race; impedance mode may otherwise enter error
        # 106 (RequestTorqueMode).
        time.sleep(1.0)
        state = robot.current_state(obj)
    if state != FXStateType.FX_STATE_IDLE:
        ret = robot.switch_to_idle(obj, 2000)
        if ret != 0:
            raise RuntimeError(f"switch_to_idle({obj}) failed: {ret}")

    # switch_to_idle() returns as soon as the controller reports IDLE, but the
    # SDK rejects a subsequent position/impedance-mode transition with -6
    # until its low-speed feedback flag also becomes 1. Wait for both
    # conditions instead of racing the arm while it is still decelerating.
    _wait_for_idle_and_low_speed(robot, obj, settle_timeout_s)

    rt = robot.get_rt_dict()
    if "error" in rt:
        raise RuntimeError(f"cannot read arm {obj} before JointImp: {rt['error']}")
    current = np.asarray(
        rt["arms"][int(obj)]["fb"]["fb_pos"],
        dtype=np.float64,
    )
    if current.shape != (7,) or not np.isfinite(current).all():
        raise RuntimeError(f"invalid arm {obj} feedback before JointImp: {current}")

    # Preload current feedback before requesting torque mode. This prevents a
    # stale command from becoming active during the position-to-impedance handoff.
    ret = robot.runtime_set_joint_pos_cmd(obj, current.tolist())
    if ret != 0:
        detail = robot._get_operate_error_msg(ret)
        raise RuntimeError(
            f"failed to preload arm {obj} current position before JointImp: "
            f"{ret} ({detail})"
        )

    # Tool dynamics must be installed before requesting torque mode so the
    # impedance controller compensates the payload from its first cycle.
    ret = robot.runtime_set_tool_d(obj, TOOL_DYNAMICS)
    if ret != 0:
        detail = robot._get_operate_error_msg(ret)
        raise RuntimeError(
            f"runtime_set_tool_d({obj}) failed before impedance mode: "
            f"{ret} ({detail})"
        )

    # Keep the existing joint-position command path, but track those targets
    # with the stable K/D set validated for high-rate bimanual streaming.
    for attempt in range(2):
        ret = robot.switch_to_imp_joint_mode(
            obj,
            3000,
            velocity,
            acceleration,
            JOINT_IMPEDANCE_K,
            JOINT_IMPEDANCE_D,
        )
        if ret == 0:
            hold_ret = robot.runtime_set_joint_pos_cmd(obj, current.tolist())
            if hold_ret != 0:
                detail = robot._get_operate_error_msg(hold_ret)
                raise RuntimeError(
                    f"failed to hold arm {obj} after entering JointImp: "
                    f"{hold_ret} ({detail})"
                )

            # FXPlatform's Confirm Speed button applies these settings only
            # after JointImp is active, using the two runtime APIs in this
            # order.  Passing the same values to the mode-transition API alone
            # updates SG feedback but does not engage the controller's
            # velocity/acceleration trajectory scaling on this firmware.
            vel_ret = robot.runtime_set_vel_ratio(obj, velocity)
            if vel_ret != 0:
                detail = robot._get_operate_error_msg(vel_ret)
                raise RuntimeError(
                    f"runtime_set_vel_ratio({obj}, {velocity}) failed after "
                    f"JointImp transition: {vel_ret} ({detail})"
                )
            acc_ret = robot.runtime_set_acc_ratio(obj, acceleration)
            if acc_ret != 0:
                detail = robot._get_operate_error_msg(acc_ret)
                raise RuntimeError(
                    f"runtime_set_acc_ratio({obj}, {acceleration}) failed after "
                    f"JointImp transition: {acc_ret} ({detail})"
                )

            setting = robot.get_sg_dict()["arms"][int(obj)]["set"]
            actual_velocity = float(setting["vel_ratio"])
            actual_acceleration = float(setting["acc_ratio"])
            if (
                abs(actual_velocity - float(velocity)) > 0.05
                or abs(actual_acceleration - float(acceleration)) > 0.05
            ):
                raise RuntimeError(
                    "JointImp runtime speed readback mismatch for arm "
                    f"{int(obj)}: requested velocity/acceleration="
                    f"{velocity}/{acceleration}, actual="
                    f"{actual_velocity}/{actual_acceleration}"
                )
            LOGGER.info(
                "Arm %d JointImp runtime speed confirmed: velocity=%.1f%%, "
                "acceleration=%.1f%%",
                int(obj),
                actual_velocity,
                actual_acceleration,
            )
            return

        # Error 106 is a controller-side failure to request the servo torque
        # mode used by every impedance controller. It can be transient just
        # after an emergency reset. Clear and retry it once only when all
        # seven drives explicitly report no fault. Never fall back to rigid
        # position mode, because that would defeat the requested safety model.
        system_error = _arm_system_error(robot, obj)
        if (
            attempt == 0
            and ret == INVALID_CONDITION_RET
            and system_error == 0
            and _servo_errors_are_clear(robot, obj)
        ):
            LOGGER.warning(
                "Arm %d impedance request returned transient Invalid condition "
                "with controller and all servos healthy; waiting for another "
                "continuous IDLE/low-speed window and retrying once",
                int(obj),
            )
            _wait_for_idle_and_low_speed(robot, obj, settle_timeout_s)
            continue
        if (
            attempt == 0
            and system_error == 106
            and _servo_errors_are_clear(robot, obj)
        ):
            LOGGER.warning(
                "Arm %d torque-mode request failed with error 106 and all "
                "servo errors clear; resetting, settling, and retrying once",
                int(obj),
            )
            reset_ret, remaining_error = robot.reset_error(obj, 3000)
            if reset_ret != 0:
                raise RuntimeError(
                    f"failed to reset arm {obj} torque-mode request error: "
                    f"ret={reset_ret}, error=0x{remaining_error:08X}; "
                    f"{arm_diagnostics(robot, obj)}"
                )
            # Give the controller/drive mode state machines time to settle
            # after reset_error() reports IDLE.
            time.sleep(1.0)
            _wait_for_idle_and_low_speed(robot, obj, settle_timeout_s)
            continue

        detail = robot._get_operate_error_msg(ret)
        raise RuntimeError(
            f"switch_to_imp_joint_mode({obj}) failed: {ret} ({detail}); "
            f"controller refused the torque mode required by joint "
            f"impedance after {attempt + 1} attempt(s); inference was not "
            f"started; low_speed_flag="
            f"{robot.get_sg_dict()['arms'][obj]['get']['low_speed_flag']}; "
            f"{arm_diagnostics(robot, obj)}"
        )

    raise AssertionError("unreachable impedance-mode retry loop")


def set_arm_impedance_speed(
    robot: GentoRobot,
    obj: int,
    velocity: float,
    acceleration: float,
) -> None:
    """Apply and verify runtime speed ratios while an arm stays in JointImp."""
    if robot.current_state(obj) != FXStateType.FX_STATE_IMP_JOINT:
        raise RuntimeError(
            f"arm {int(obj)} must be in joint-impedance mode before changing speed"
        )
    # FXPlatform's Confirm Speed button applies these settings only after
    # JointImp is active, using the two runtime APIs in this order.
    vel_ret = robot.runtime_set_vel_ratio(obj, velocity)
    if vel_ret != 0:
        detail = robot._get_operate_error_msg(vel_ret)
        raise RuntimeError(
            f"runtime_set_vel_ratio({obj}, {velocity}) failed in JointImp: "
            f"{vel_ret} ({detail})"
        )
    acc_ret = robot.runtime_set_acc_ratio(obj, acceleration)
    if acc_ret != 0:
        detail = robot._get_operate_error_msg(acc_ret)
        raise RuntimeError(
            f"runtime_set_acc_ratio({obj}, {acceleration}) failed in JointImp: "
            f"{acc_ret} ({detail})"
        )

    time.sleep(1.0)
    setting = robot.get_sg_dict()["arms"][int(obj)]["set"]
    actual_velocity = float(setting["vel_ratio"])
    actual_acceleration = float(setting["acc_ratio"])
    if (
        abs(actual_velocity - float(velocity)) > 0.05
        or abs(actual_acceleration - float(acceleration)) > 0.05
    ):
        raise RuntimeError(
            "JointImp runtime speed readback mismatch for arm "
            f"{int(obj)}: requested velocity/acceleration="
            f"{velocity}/{acceleration}, actual="
            f"{actual_velocity}/{actual_acceleration}"
        )
    LOGGER.info(
        "Arm %d JointImp runtime speed confirmed: velocity=%.1f%%, "
        "acceleration=%.1f%%",
        int(obj),
        actual_velocity,
        actual_acceleration,
    )


def move_arm_to_joint_pose(
    robot: GentoRobot,
    obj: int,
    target: np.ndarray,
    *,
    step_deg: float,
    command_hz: float,
    tolerance_deg: float,
    timeout_s: float,
    stop_event: threading.Event,
    required_state: int = FXStateType.FX_STATE_IMP_JOINT,
) -> np.ndarray:
    """Slowly approach a joint pose using feedback-limited position targets."""
    target = np.asarray(target, dtype=np.float32)
    if target.shape != (7,) or not np.isfinite(target).all():
        raise ValueError(f"initial arm target must be finite shape (7,), got {target.shape}")
    if min(step_deg, command_hz, tolerance_deg, timeout_s) <= 0:
        raise ValueError("initial arm trajectory parameters must be positive")

    arm_idx = int(obj)
    period = 1.0 / command_hz
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if stop_event.is_set():
            raise KeyboardInterrupt
        state = robot.current_state(obj)
        if state != required_state:
            raise RuntimeError(
                f"arm {arm_idx} left required state {required_state} during "
                f"initial move: state={state}"
            )
        rt = robot.get_rt_dict()
        if "error" in rt:
            raise RuntimeError(rt["error"])
        current = np.asarray(rt["arms"][arm_idx]["fb"]["fb_pos"], dtype=np.float32)
        if current.shape != (7,) or not np.isfinite(current).all():
            raise RuntimeError(f"invalid arm {arm_idx} feedback during initial move: {current}")
        delta = target - current
        if float(np.max(np.abs(delta))) <= tolerance_deg:
            return current
        command = current + np.clip(delta, -step_deg, step_deg)
        ret = robot.runtime_set_joint_pos_cmd(obj, command.tolist())
        if ret != 0:
            detail = robot._get_operate_error_msg(ret)
            raise RuntimeError(f"initial arm position command failed: {ret} ({detail})")
        time.sleep(period)

    rt = robot.get_rt_dict()
    current = np.asarray(rt["arms"][arm_idx]["fb"]["fb_pos"], dtype=np.float32)
    error = target - current
    raise TimeoutError(
        f"arm {arm_idx} did not reach initial pose within {timeout_s:.1f}s: "
        f"max_error={float(np.max(np.abs(error))):.3f} deg, "
        f"current={current.round(3).tolist()}"
    )


class InitialPosePlanRejected(RuntimeError):
    """Joint-space return exceeds the hard duration cap. No command was sent."""


def _initial_joint_delta_report(
    starts: dict[int, np.ndarray],
    targets: dict[int, np.ndarray],
) -> tuple[list[str], tuple[float, str, int, float, float, float] | None]:
    lines: list[str] = []
    worst = None
    for obj, target in targets.items():
        current = starts[obj]
        arm = int(obj)
        name = "left" if arm == 0 else "right" if arm == 1 else f"arm{arm}"
        parts = []
        for index, (cur, tgt) in enumerate(zip(current, target)):
            err = float(tgt) - float(cur)
            parts.append(
                f"j{index}: cur={float(cur):.3f} tgt={float(tgt):.3f} delta={err:+.3f}"
            )
            if worst is None or abs(err) > worst[0]:
                worst = (abs(err), name, index, float(cur), float(tgt), err)
        lines.append(name + ": " + "; ".join(parts))
    return lines, worst


def move_arms_to_joint_poses(
    robot: GentoRobot,
    targets: dict[int, np.ndarray],
    *,
    step_deg: float,
    max_lead_deg: float,
    command_hz: float,
    tolerance_deg: float,
    timeout_s: float,
    stop_event: threading.Event,
    required_state: int = FXStateType.FX_STATE_IMP_JOINT,
) -> dict[int, np.ndarray]:
    """Synchronously ease both arms to fixed joint targets in JointImp.

    The path is generated locally from one feedback snapshot, so it does not
    depend on the SDK kinematics planner or restart every cycle from lagging
    feedback. A quintic smoothstep gives zero velocity and acceleration at
    both endpoints. Progress pauses for both arms when either arm's command is
    too far ahead of feedback, preserving synchronization and bounded contact
    force while the impedance controller catches up.
    """
    if not targets:
        return {}
    normalized_targets = {
        int(obj): np.asarray(target, dtype=np.float64) for obj, target in targets.items()
    }
    if any(
        target.shape != (7,) or not np.isfinite(target).all()
        for target in normalized_targets.values()
    ):
        raise ValueError("initial arm targets must all be finite shape (7,)")
    if min(
        step_deg,
        max_lead_deg,
        command_hz,
        tolerance_deg,
        timeout_s,
    ) <= 0:
        raise ValueError("initial arm trajectory parameters must be positive")
    if max_lead_deg < step_deg:
        raise ValueError("initial arm max lead must be at least one command step")

    for obj in normalized_targets:
        state = robot.current_state(obj)
        if state != required_state:
            raise RuntimeError(
                f"arm {obj} must be in required state {required_state} before "
                f"the impedance initial move: state={state}"
            )

    rt = robot.get_rt_dict()
    if "error" in rt:
        raise RuntimeError(rt["error"])
    starts: dict[int, np.ndarray] = {}
    for obj in normalized_targets:
        current = np.asarray(rt["arms"][obj]["fb"]["fb_pos"], dtype=np.float64)
        if current.shape != (7,) or not np.isfinite(current).all():
            raise RuntimeError(
                f"invalid arm {obj} feedback before impedance initial move: {current}"
            )
        starts[obj] = current.copy()

    max_distance_deg = max(
        float(np.max(np.abs(normalized_targets[obj] - starts[obj])))
        for obj in normalized_targets
    )
    detail_lines, worst = _initial_joint_delta_report(starts, normalized_targets)
    for line in detail_lines:
        LOGGER.info("Initial joint delta %s", line)
    if worst is not None:
        LOGGER.info(
            "Largest initial joint error: %s j%d current=%.3f target=%.3f "
            "delta=%+.3f deg",
            worst[1],
            worst[2],
            worst[3],
            worst[4],
            worst[5],
        )
    # A quintic smoothstep has a peak normalized slope of 1.875. Choose the
    # number of samples from that peak so every command increment is bounded
    # by step_deg, including the middle of the trajectory.
    # The caller timeout is an execution floor, not a reason to refuse a long
    # but still physical return. Only a plan beyond PLAN_LIMIT_S is rejected,
    # and that rejection sends no command.
    plan_limit_s = 180.0
    exec_margin_s = 45.0
    trajectory_steps = max(1, math.ceil(1.875 * max_distance_deg / step_deg))
    planned_duration = trajectory_steps / command_hz
    if planned_duration > plan_limit_s:
        largest = ""
        if worst is not None:
            largest = (
                f"; largest {worst[1]} j{worst[2]} current={worst[3]:.3f} "
                f"target={worst[4]:.3f} delta={worst[5]:+.3f} deg"
            )
        raise InitialPosePlanRejected(
            f"smooth impedance initial trajectory requires {planned_duration:.2f}s, "
            f"which exceeds the {plan_limit_s:.0f}s cap; max_distance="
            f"{max_distance_deg:.3f} deg, step={step_deg:.3f} deg"
            f"{largest}"
        )
    execution_timeout_s = max(timeout_s, planned_duration + exec_margin_s)
    LOGGER.info(
        "Synchronized quintic JointImp initial trajectory: steps=%d, "
        "duration=%.2fs, execution_timeout=%.2fs, max_distance=%.3f deg, "
        "max_step=%.3f deg, max_feedback_lead=%.3f deg",
        trajectory_steps,
        planned_duration,
        execution_timeout_s,
        max_distance_deg,
        step_deg,
        max_lead_deg,
    )

    period = 1.0 / command_hz
    deadline = time.monotonic() + execution_timeout_s
    last_commands = {obj: start.copy() for obj, start in starts.items()}
    latest = {obj: start.copy() for obj, start in starts.items()}
    trajectory_index = 0
    blocked_cycles = 0
    next_command_at = time.monotonic()
    while trajectory_index <= trajectory_steps:
        if stop_event.is_set():
            raise KeyboardInterrupt
        if time.monotonic() >= deadline:
            errors = {
                obj: float(np.max(np.abs(normalized_targets[obj] - latest[obj])))
                for obj in normalized_targets
            }
            raise TimeoutError(
                "arms did not finish the smooth impedance initial trajectory "
                f"within {execution_timeout_s:.1f}s: max_errors_deg={errors}"
            )

        rt = robot.get_rt_dict()
        if "error" in rt:
            raise RuntimeError(rt["error"])
        progress = trajectory_index / trajectory_steps
        blend = progress**3 * (10.0 + progress * (-15.0 + 6.0 * progress))
        proposed: dict[int, np.ndarray] = {}
        max_proposed_lead = 0.0
        blocked_arm = -1
        for obj, target in normalized_targets.items():
            state = robot.current_state(obj)
            if state != required_state:
                raise RuntimeError(
                    f"arm {obj} left required state {required_state} during "
                    f"the smooth impedance initial move: state={state}"
                )
            current = np.asarray(rt["arms"][obj]["fb"]["fb_pos"], dtype=np.float64)
            if current.shape != (7,) or not np.isfinite(current).all():
                raise RuntimeError(
                    f"invalid arm {obj} feedback during impedance initial move: "
                    f"{current}"
                )
            latest[obj] = current
            command = starts[obj] + blend * (target - starts[obj])
            command_step = float(np.max(np.abs(command - last_commands[obj])))
            if command_step > step_deg + 1e-6:
                raise RuntimeError(
                    f"arm {obj} generated initial command step {command_step:.6f} "
                    f"deg exceeds limit {step_deg:.6f} deg"
                )
            proposed[obj] = command
            proposed_lead = float(np.max(np.abs(command - current)))
            if proposed_lead > max_proposed_lead:
                max_proposed_lead = proposed_lead
                blocked_arm = obj

        advance = max_proposed_lead <= max_lead_deg
        commands = proposed if advance else last_commands
        for obj, command in commands.items():
            ret = robot.runtime_set_joint_pos_cmd(obj, command.tolist())
            if ret != 0:
                detail = robot._get_operate_error_msg(ret)
                raise RuntimeError(
                    f"arm {obj} smooth impedance initial command failed: "
                    f"{ret} ({detail})"
                )
        if advance:
            last_commands = {obj: command.copy() for obj, command in proposed.items()}
            trajectory_index += 1
            blocked_cycles = 0
        else:
            blocked_cycles += 1
            log_interval_cycles = max(1, int(round(command_hz)))
            if blocked_cycles == 1 or blocked_cycles % log_interval_cycles == 0:
                LOGGER.warning(
                    "Pausing synchronized initial trajectory for feedback: "
                    "arm=%d, command_lead=%.3f deg, limit=%.3f deg, step=%d/%d",
                    blocked_arm,
                    max_proposed_lead,
                    max_lead_deg,
                    trajectory_index,
                    trajectory_steps,
                )
        next_command_at += period
        remaining = next_command_at - time.monotonic()
        if remaining > 0 and stop_event.wait(remaining):
            raise KeyboardInterrupt
        if remaining <= 0:
            next_command_at = time.monotonic()

    while time.monotonic() < deadline:
        if stop_event.is_set():
            raise KeyboardInterrupt
        rt = robot.get_rt_dict()
        if "error" in rt:
            raise RuntimeError(rt["error"])
        all_reached = True
        for obj, target in normalized_targets.items():
            state = robot.current_state(obj)
            if state != required_state:
                raise RuntimeError(
                    f"arm {obj} left required state {required_state} while "
                    f"settling at the impedance initial endpoint: state={state}"
                )
            current = np.asarray(rt["arms"][obj]["fb"]["fb_pos"], dtype=np.float64)
            if current.shape != (7,) or not np.isfinite(current).all():
                raise RuntimeError(
                    f"invalid arm {obj} feedback at impedance initial endpoint: "
                    f"{current}"
                )
            latest[obj] = current
            all_reached = (
                float(np.max(np.abs(target - current))) <= tolerance_deg
                and all_reached
            )
            ret = robot.runtime_set_joint_pos_cmd(obj, target.tolist())
            if ret != 0:
                detail = robot._get_operate_error_msg(ret)
                raise RuntimeError(
                    f"arm {obj} endpoint hold failed: {ret} ({detail})"
                )
        if all_reached:
            return latest
        if stop_event.wait(period):
            raise KeyboardInterrupt

    errors = {
        obj: float(np.max(np.abs(target - latest[obj])))
        for obj, target in normalized_targets.items()
    }
    raise TimeoutError(
        f"arms did not reach smooth impedance initial poses within {execution_timeout_s:.1f}s: "
        f"max_errors_deg={errors}"
    )


def move_grippers_to_position(
    gripper: GripperBackend,
    arm_indices: set[int],
    target: float,
    *,
    step: float,
    command_hz: float,
    tolerance: float,
    timeout_s: float,
    feedback_max_age_s: float,
    stop_event: threading.Event,
) -> dict[int, float]:
    """Move all selected grippers on one clock and verify them together.

    Always publish the exact target before reporting success.  In particular,
    a startup feedback value already equal to zero must not skip the explicit
    ``0.0`` open command.
    """
    enabled = set(arm_indices)
    if not enabled or not enabled.issubset({0, 1}):
        raise ValueError(f"invalid gripper indices {sorted(enabled)}")
    if not np.isfinite(target) or not 0.0 <= target <= GRIPPER_COMMAND_MAX:
        raise ValueError(f"initial gripper target must be in [0,0.9], got {target}")
    if min(step, command_hz, tolerance, timeout_s, feedback_max_age_s) <= 0:
        raise ValueError("initial gripper trajectory parameters must be positive")

    period = 1.0 / command_hz
    deadline = time.monotonic() + timeout_s
    expected_state = target / GRIPPER_COMMAND_MAX
    settled_samples = {arm_idx: 0 for arm_idx in enabled}
    command_calibrated = getattr(gripper, "command_calibrated", gripper.command)
    while time.monotonic() < deadline:
        if stop_event.is_set():
            raise KeyboardInterrupt
        current_values = gripper.state(feedback_max_age_s, enabled)
        command = np.zeros(2, dtype=np.float32)
        for arm_idx in enabled:
            current_command = float(current_values[arm_idx]) * GRIPPER_COMMAND_MAX
            delta = target - current_command
            if delta < -tolerance:
                # Opening is release-safe, so both sides receive the final
                # endpoint immediately in the same dual-gripper call.
                command[arm_idx] = target
                settled_samples[arm_idx] = 0
            elif abs(delta) <= tolerance:
                command[arm_idx] = target
                settled_samples[arm_idx] += 1
            else:
                # Closing remains gradual because it can establish contact.
                command[arm_idx] = current_command + min(delta, step)
                settled_samples[arm_idx] = 0

        # Initialization must use the measured endpoint mapping. Policy-only
        # close gain/overtravel can otherwise reverse a small opening step.
        command_calibrated(command, enabled_arms=enabled)

        if all(
            settled_samples[arm_idx] >= INITIAL_GRIPPER_SETTLE_SAMPLES
            for arm_idx in enabled
        ):
            if stop_event.wait(period):
                raise KeyboardInterrupt
            confirmed = gripper.state(feedback_max_age_s, enabled)
            if all(
                abs(expected_state - float(confirmed[arm_idx]))
                <= tolerance / GRIPPER_COMMAND_MAX
                for arm_idx in enabled
            ):
                return {arm_idx: float(confirmed[arm_idx]) for arm_idx in enabled}
            for arm_idx in enabled:
                settled_samples[arm_idx] = 0
        if stop_event.wait(period):
            raise KeyboardInterrupt

    current = gripper.state(feedback_max_age_s, enabled)
    raise TimeoutError(
        f"grippers did not reach initial position within {timeout_s:.1f}s: "
        f"target={target:.3f}, current="
        f"{dict((arm_idx, round(float(current[arm_idx]), 3)) for arm_idx in enabled)}"
    )


def move_gripper_to_position(
    gripper: GripperBackend,
    arm_idx: int,
    target: float,
    *,
    step: float,
    command_hz: float,
    tolerance: float,
    timeout_s: float,
    feedback_max_age_s: float,
    stop_event: threading.Event,
) -> float:
    """Compatibility wrapper for the single-gripper dry-run path."""
    return move_grippers_to_position(
        gripper,
        {arm_idx},
        target,
        step=step,
        command_hz=command_hz,
        tolerance=tolerance,
        timeout_s=timeout_s,
        feedback_max_age_s=feedback_max_age_s,
        stop_event=stop_event,
    )[arm_idx]


def validate_action(
    action: np.ndarray,
    current: np.ndarray,
    hard_arm_jump_deg: float,
    max_gripper_step: float,
    enabled_arms: set[int],
    arm_reference: np.ndarray | None = None,
) -> np.ndarray:
    if action.shape != (ACTION_DIM,) or not np.isfinite(action).all():
        raise ValueError(f"invalid action shape/value: {action.shape}")
    reference = current if arm_reference is None else np.asarray(arm_reference)
    if reference.shape != (ACTION_DIM,) or not np.isfinite(reference).all():
        raise ValueError(f"invalid arm jump reference shape/value: {reference.shape}")
    if not np.isfinite(hard_arm_jump_deg) or hard_arm_jump_deg <= 0:
        raise ValueError("hard_arm_jump_deg must be finite and positive")
    safe = action.copy()
    enabled_joint_indices: list[int] = []
    for arm_idx, offset in ((0, 0), (1, 7)):
        if arm_idx not in enabled_arms:
            safe[offset : offset + 7] = 0.0
            safe[14 + arm_idx] = 0.0
            continue
        enabled_joint_indices.extend(range(offset, offset + 7))

    # Project one complete bimanual policy delta, rather than clipping joints
    # independently. This keeps every enabled arm moving in the model-selected
    # direction and preserves the relative joint increments while guaranteeing
    # that no component advances by more than the configured envelope.
    if enabled_joint_indices:
        indices = np.asarray(enabled_joint_indices, dtype=np.int64)
        policy_delta = safe[indices] - reference[indices]
        largest_jump = float(np.max(np.abs(policy_delta)))
        if largest_jump > hard_arm_jump_deg:
            safe[indices] = (
                reference[indices]
                + policy_delta * (hard_arm_jump_deg / largest_jump)
            )

    for arm_idx, offset in ((0, 0), (1, 7)):
        if arm_idx not in enabled_arms:
            continue
        # Do not introduce a discontinuity at an arbitrary tracking-error
        # threshold. Every finite target inside the configured policy-jump
        # envelope is preserved here; target-versus-feedback lag is a separate
        # decision in the execution loop.
        gripper_idx = 14 + arm_idx
        # A large gripper target must not reject the entire arm action: doing
        # so leaves the gripper feedback unchanged, the policy predicts the
        # same target again, and the client deadlocks in reject/replan. Apply
        # a per-action slew limit instead; interpolate_action() will further
        # split this bounded change into small gripper commands.
        bounded_target = float(np.clip(safe[gripper_idx], 0.0, GRIPPER_COMMAND_MAX))
        current_command = float(current[gripper_idx]) * GRIPPER_COMMAND_MAX
        gripper_delta = bounded_target - current_command
        safe[gripper_idx] = np.clip(
            current_command
            + float(np.clip(gripper_delta, -max_gripper_step, max_gripper_step)),
            0.0,
            GRIPPER_COMMAND_MAX,
        )
    return safe


def execute_action(
    robot: GentoRobot,
    gripper: GripperBackend,
    action: np.ndarray,
    enabled_arms: set[int],
) -> None:
    left_arm, right_arm, grippers = split_action(action)
    calls = []
    if 0 in enabled_arms:
        calls.append(
            (
                "left arm",
                robot.runtime_set_joint_pos_cmd(
                    FXObjType.OBJ_ARM0, left_arm.tolist()
                ),
            )
        )
    if 1 in enabled_arms:
        calls.append(
            (
                "right arm",
                robot.runtime_set_joint_pos_cmd(
                    FXObjType.OBJ_ARM1, right_arm.tolist()
                ),
            )
        )
    failed = [(name, ret) for name, ret in calls if ret != 0]
    if failed:
        raise RuntimeError(f"robot command failed: {failed}")
    gripper.command(grippers, enabled_arms=enabled_arms)


def interpolate_action(
    current: np.ndarray,
    target: np.ndarray,
    enabled_arms: set[int],
    arm_step_deg: float,
    gripper_step: float,
) -> list[np.ndarray]:
    required = 1
    for arm_idx, offset in ((0, 0), (1, 7)):
        if arm_idx in enabled_arms:
            delta = np.max(
                np.abs(target[offset : offset + 7] - current[offset : offset + 7])
            )
            required = max(required, math.ceil(float(delta) / arm_step_deg))
            gripper_idx = 14 + arm_idx
            required = max(
                required,
                math.ceil(
                    abs(float(target[gripper_idx] - current[gripper_idx] * GRIPPER_COMMAND_MAX))
                    / gripper_step
                ),
            )
    start = current.copy()
    goal = target.copy()
    for arm_idx in enabled_arms:
        start[14 + arm_idx] *= GRIPPER_COMMAND_MAX
    for arm_idx, offset in ((0, 0), (1, 7)):
        if arm_idx not in enabled_arms:
            start[offset : offset + 7] = 0.0
            goal[offset : offset + 7] = 0.0
            start[14 + arm_idx] = 0.0
            goal[14 + arm_idx] = 0.0
    return [
        start + (goal - start) * (index / required)
        for index in range(1, required + 1)
    ]
