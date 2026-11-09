"""Direct DM4310 gripper control through the Gento ARM terminal CAN-FD API.

This backend intentionally bypasses dm_gripper_py, ROS gripper topics and
SocketCAN.  It shares the already-exclusive GentoRobot connection with the arm
client and follows the initialization order used by the vendor C++ example:
disable, request MIT mode, enable, then continuously send MIT position frames.
"""

from __future__ import annotations

import logging
import math
import struct
import threading
import time
from typing import Protocol

import numpy as np


LOGGER = logging.getLogger("tianji_direct_gripper")

TERMINAL_ARM1 = 1
TERMINAL_ARM0 = 0
CHANNEL_CANFD = 1
LEFT_SLAVE_ID = 0x01
LEFT_MASTER_ID = 0x11
RIGHT_SLAVE_ID = 0x02
RIGHT_MASTER_ID = 0x12
PARAMETER_CAN_ID = 0x7FF
CONTROL_MODE_RID = 10
MIT_MODE = 1
DM_ENABLED = 0x1
ENABLE_RETRY_INTERVAL_S = 0.10

DM_Q_MAX = 12.5
DM_DQ_MAX = 30.0
DM_TAU_MAX = 10.0
GRIPPER_COMMAND_MAX = 0.9

_ENABLE = b"\xff" * 7 + b"\xfc"
_DISABLE = b"\xff" * 7 + b"\xfd"


class ContactGripController:
    """Convert a [0, 0.9] grasp request into a force-limited position target."""

    def __init__(
        self,
        *,
        open_rad: float,
        closed_rad: float,
        calibrated_closed_rad: float | None = None,
        command_hz: float,
        close_rate_rad_s: float,
        contact_torque_nm: float,
        contact_velocity_rad_s: float,
        contact_cycles: int,
        regrip_torque_nm: float,
        regrip_cycles: int,
        close_scale: float = 1.0,
        opening_deadband_rad: float = 0.02,
        teleop_range_rad: float = 1.6,
        contact_stop_enabled: bool = True,
    ) -> None:
        if closed_rad <= open_rad:
            raise ValueError("closed_rad must exceed open_rad")
        if calibrated_closed_rad is None:
            calibrated_closed_rad = closed_rad
        if not open_rad < calibrated_closed_rad <= closed_rad:
            raise ValueError(
                "calibrated_closed_rad must be above open_rad and no greater "
                "than closed_rad"
            )
        if command_hz <= 0 or close_rate_rad_s <= 0:
            raise ValueError("gripper command frequency and close rate must be positive")
        if not 0 <= regrip_torque_nm < contact_torque_nm:
            raise ValueError("regrip torque must be non-negative and below contact torque")
        if contact_velocity_rad_s < 0 or contact_cycles <= 0 or regrip_cycles <= 0:
            raise ValueError("invalid gripper contact confirmation parameters")
        if not math.isfinite(close_scale) or close_scale <= 0:
            raise ValueError("gripper close scale must be finite and positive")
        if not math.isfinite(teleop_range_rad) or not 0 < teleop_range_rad <= 1.6:
            raise ValueError("gripper teleoperation range must be in (0,1.6] rad")
        self.open_rad = float(open_rad)
        self.closed_rad = float(closed_rad)
        self.calibrated_closed_rad = float(calibrated_closed_rad)
        self.close_step_rad = float(close_rate_rad_s) / float(command_hz)
        self.contact_torque_nm = float(contact_torque_nm)
        self.contact_velocity_rad_s = float(contact_velocity_rad_s)
        self.contact_cycles = int(contact_cycles)
        self.regrip_torque_nm = float(regrip_torque_nm)
        self.regrip_cycles = int(regrip_cycles)
        self.close_scale = float(close_scale)
        self.opening_deadband_rad = float(opening_deadband_rad)
        self.teleop_range_rad = float(teleop_range_rad)
        self.teleop_max_target_rad = GRIPPER_COMMAND_MAX * self.teleop_range_rad
        self.hard_closed_rad = max(self.closed_rad, self.teleop_max_target_rad)
        self.contact_stop_enabled = bool(contact_stop_enabled)
        self.input_command = 0.0
        self.effective_command = 0.0
        self.enhanced_mapping = False
        self.mapping_mode = "calibrated"
        self.requested_raw = self.open_rad
        self.target_raw: float | None = None
        self.contact_latched = False
        self._contact_count = 0
        self._regrip_count = 0

    def set_command(
        self,
        command: float,
        *,
        enhanced: bool = True,
        teleop: bool = False,
    ) -> None:
        if enhanced and teleop:
            raise ValueError("enhanced and teleop gripper mappings are mutually exclusive")
        command = _clip_float(command, 0.0, GRIPPER_COMMAND_MAX)
        if teleop:
            effective_command = command
            # Match dm_gripper_py teleoperation semantics exactly: its
            # normalized [0,1] command is multiplied by the 1.6-rad motor
            # range. The policy itself remains capped at 0.9, hence 1.44 rad.
            requested = _clip_float(
                command * self.teleop_range_rad,
                self.open_rad,
                self.hard_closed_rad,
            )
            mapping_mode = "teleop"
        else:
            effective_command = (
                _clip_float(command * self.close_scale, 0.0, GRIPPER_COMMAND_MAX)
                if enhanced
                else command
            )
            closed_rad = self.closed_rad if enhanced else self.calibrated_closed_rad
            requested = self.open_rad + (effective_command / GRIPPER_COMMAND_MAX) * (
                closed_rad - self.open_rad
            )
            mapping_mode = "postprocess" if enhanced else "calibrated"
        # A placement/opening request must always override a contact latch.
        if requested < self.requested_raw - self.opening_deadband_rad:
            self.contact_latched = False
            self._contact_count = 0
            self._regrip_count = 0
        self.input_command = command
        self.effective_command = effective_command
        self.enhanced_mapping = bool(enhanced)
        self.mapping_mode = mapping_mode
        self.requested_raw = requested

    def update(self, *, position: float, velocity: float, torque: float) -> float:
        if self.target_raw is None:
            self.target_raw = _clip_float(position, self.open_rad, self.hard_closed_rad)

        if self.mapping_mode == "teleop" and not self.contact_stop_enabled:
            # True teleoperation dynamics: publish the mapped motor target on
            # the next 100-Hz MIT cycle. Do not add the software close-rate
            # ramp used by the force-limited postprocess controller.
            self.target_raw = self.requested_raw
            self.contact_latched = False
            self._contact_count = 0
            self._regrip_count = 0
            return self.target_raw

        if self.requested_raw < self.target_raw - self.opening_deadband_rad:
            # Opening is intentionally not torque-gated, so placement can release promptly.
            self.contact_latched = False
            self._contact_count = 0
            self._regrip_count = 0
            self.target_raw = self.requested_raw
            return self.target_raw

        if self.contact_stop_enabled and self.contact_latched:
            if abs(torque) < self.regrip_torque_nm:
                self._regrip_count += 1
                if self._regrip_count >= self.regrip_cycles:
                    self.contact_latched = False
                    self._regrip_count = 0
            else:
                self._regrip_count = 0
            if self.contact_latched:
                return self.target_raw

        if self.requested_raw > self.target_raw:
            self.target_raw = min(
                self.requested_raw,
                self.target_raw + self.close_step_rad,
                self.hard_closed_rad,
            )

        if self.contact_stop_enabled:
            contacting = (
                self.requested_raw > position + self.opening_deadband_rad
                and abs(torque) >= self.contact_torque_nm
                and abs(velocity) <= self.contact_velocity_rad_s
            )
            self._contact_count = self._contact_count + 1 if contacting else 0
            if self._contact_count >= self.contact_cycles:
                self.contact_latched = True
                self._contact_count = 0
        else:
            # Teleoperation keeps sending the commanded motor position even
            # after contact; torque is limited by the MIT gains rather than by
            # freezing the target at the first contact sample.
            self.contact_latched = False
            self._contact_count = 0
            self._regrip_count = 0
        return self.target_raw


class _TerminalRobot(Protocol):
    def terminal_clear(self, terminal_type: int) -> int: ...

    def terminal_get(
        self, terminal_type: int, timeout: int = 1000
    ) -> tuple[int, bytes, int]: ...

    def terminal_set(
        self,
        terminal_type: int,
        chn_type: int,
        data: bytes,
        timeout: int = 1000,
    ) -> tuple[int, int]: ...


def _clip_float(value: float, low: float, high: float) -> float:
    return min(max(float(value), low), high)


def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    value = _clip_float(value, low, high)
    return int((value - low) * ((1 << bits) - 1) / (high - low))


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    return float(value) * (high - low) / ((1 << bits) - 1) + low


def terminal_can_frame(can_id: int, data: bytes) -> bytes:
    """Encode the vendor terminal format: uint32 LE ID plus eight data bytes."""

    if not 0 <= int(can_id) <= 0x1FFFFFFF:
        raise ValueError(f"invalid CAN ID: {can_id}")
    if len(data) != 8:
        raise ValueError(f"DM CAN payload must contain 8 bytes, got {len(data)}")
    return int(can_id).to_bytes(4, "little") + bytes(data)


def parse_terminal_can_frame(frame: bytes) -> tuple[int, bytes] | None:
    if len(frame) < 12:
        return None
    return int.from_bytes(frame[:4], "little"), bytes(frame[4:12])


def encode_mit_command(
    position: float,
    *,
    kp: float,
    kd: float,
    velocity: float = 0.0,
    torque: float = 0.0,
) -> bytes:
    """Encode the 8-byte DM MIT command used by the vendor C++ example."""

    q = float_to_uint(position, -DM_Q_MAX, DM_Q_MAX, 16)
    dq = float_to_uint(velocity, -DM_DQ_MAX, DM_DQ_MAX, 12)
    kp_u = float_to_uint(kp, 0.0, 500.0, 12)
    kd_u = float_to_uint(kd, 0.0, 5.0, 12)
    tau = float_to_uint(torque, -DM_TAU_MAX, DM_TAU_MAX, 12)
    return bytes(
        (
            (q >> 8) & 0xFF,
            q & 0xFF,
            (dq >> 4) & 0xFF,
            ((dq & 0xF) << 4) | ((kp_u >> 8) & 0xF),
            kp_u & 0xFF,
            (kd_u >> 4) & 0xFF,
            ((kd_u & 0xF) << 4) | ((tau >> 8) & 0xF),
            tau & 0xFF,
        )
    )


def decode_dm_feedback(
    can_id: int,
    data: bytes,
    *,
    slave_id: int,
    master_id: int,
) -> dict[str, float | int] | None:
    """Decode one DM motor MIT feedback frame."""

    if len(data) != 8:
        return None
    # Firmware variants observed in the vendor drivers respond on either the
    # configured MasterID, the motor's own SlaveID, or CAN ID 0.  The previous
    # implementation omitted the SlaveID form, so valid enable feedback from
    # classical DM4310 firmware was silently discarded until startup timed out.
    if can_id in (slave_id, master_id):
        pass
    elif can_id == 0 and (data[0] & 0x0F) == slave_id:
        pass
    else:
        return None
    status = (data[0] >> 4) & 0x0F
    q_u = (data[1] << 8) | data[2]
    dq_u = (data[3] << 4) | (data[4] >> 4)
    tau_u = ((data[4] & 0x0F) << 8) | data[5]
    return {
        "status": status,
        "position": uint_to_float(q_u, -DM_Q_MAX, DM_Q_MAX, 16),
        "velocity": uint_to_float(dq_u, -DM_DQ_MAX, DM_DQ_MAX, 12),
        "torque": uint_to_float(tau_u, -DM_TAU_MAX, DM_TAU_MAX, 12),
        "mos_temperature": int(data[6]),
        "rotor_temperature": int(data[7]),
    }


def _is_mit_mode_response(can_id: int, data: bytes, *, slave_id: int) -> bool:
    del can_id  # Firmware variants use different response CAN IDs.
    return (
        len(data) == 8
        and data[0] == slave_id
        and data[1] == 0
        and data[2] in (0x33, 0x55)
        and data[3] == CONTROL_MODE_RID
        and int.from_bytes(data[4:8], "little") == MIT_MODE
    )


def _is_parameter_response(data: bytes, *, slave_id: int) -> bool:
    return (
        len(data) == 8
        and data[0] == slave_id
        and data[1] == 0
        and data[2] in (0x33, 0x55)
    )


class GentoDmGripper:
    """One DM gripper connected to an ARM0/ARM1 terminal."""

    def __init__(
        self,
        robot: _TerminalRobot,
        *,
        limits: tuple[float, float],
        terminal_type: int = TERMINAL_ARM1,
        arm_label: str = "right",
        slave_id: int = RIGHT_SLAVE_ID,
        master_id: int = RIGHT_MASTER_ID,
        io_lock: threading.RLock | None = None,
        command_hz: float = 100.0,
        kp: float = 3.0,
        kd: float = 0.12,
        initialization_timeout_s: float = 3.0,
        trace_interval_s: float = 2.0,
        close_rate_rad_s: float = 0.40,
        contact_torque_nm: float = 0.35,
        contact_velocity_rad_s: float = 0.12,
        contact_cycles: int = 5,
        regrip_torque_nm: float = 0.20,
        regrip_cycles: int = 10,
        close_scale: float = 1.0,
        close_overtravel_rad: float = 0.0,
        policy_postprocess: bool = True,
        teleop_range_rad: float = 1.6,
    ) -> None:
        low, high = limits
        if not math.isfinite(low) or not math.isfinite(high) or high <= low:
            raise ValueError(f"invalid {arm_label} gripper limits: {limits}")
        if command_hz <= 0 or initialization_timeout_s <= 0:
            raise ValueError("gripper frequency/timeouts must be positive")
        if kp < 0 or kd < 0:
            raise ValueError("gripper MIT gains must be non-negative")
        if (
            not math.isfinite(close_overtravel_rad)
            or not 0.0 <= close_overtravel_rad <= 0.20
        ):
            raise ValueError("gripper close overtravel must be in [0,0.20] rad")
        self.robot = robot
        if terminal_type not in (TERMINAL_ARM0, TERMINAL_ARM1):
            raise ValueError(f"invalid arm terminal: {terminal_type}")
        self.terminal_type = terminal_type
        self.arm_label = arm_label
        if not 0 < int(slave_id) <= 0x7FF or not 0 < int(master_id) <= 0x7FF:
            raise ValueError(
                f"invalid {arm_label} gripper CAN IDs: "
                f"slave=0x{int(slave_id):X}, master=0x{int(master_id):X}"
            )
        self.slave_id = int(slave_id)
        self.master_id = int(master_id)
        self._io_lock = io_lock or threading.RLock()
        self.low = float(low)
        self.high = float(high)
        self.period_s = 1.0 / float(command_hz)
        self.kp = float(kp)
        self.kd = float(kd)
        self.trace_interval_s = float(trace_interval_s)
        self.policy_postprocess = bool(policy_postprocess)
        self._grip = ContactGripController(
            open_rad=self.low,
            # Keep feedback normalized against the measured empty-gripper
            # endpoint, but permit a small force-limited target beyond it.
            # ContactGripController still stops advancing after sustained
            # low-speed/high-torque contact.
            closed_rad=self.high + float(close_overtravel_rad),
            calibrated_closed_rad=self.high,
            command_hz=command_hz,
            close_rate_rad_s=close_rate_rad_s,
            contact_torque_nm=contact_torque_nm,
            contact_velocity_rad_s=contact_velocity_rad_s,
            contact_cycles=contact_cycles,
            regrip_torque_nm=regrip_torque_nm,
            regrip_cycles=regrip_cycles,
            close_scale=close_scale,
            teleop_range_rad=teleop_range_rad,
            contact_stop_enabled=self.policy_postprocess,
        )

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fatal_error: BaseException | None = None
        self._target_raw: float | None = None
        self._position_raw: float | None = None
        self._velocity = 0.0
        self._torque = 0.0
        self._status: int | None = None
        self._last_feedback_ns = 0
        self._last_trace_at = 0.0
        self._last_logged_target: float | None = None
        self.tx_frames = 0
        self.rx_frames = 0

        self._initialize_motor(initialization_timeout_s)
        self._thread = threading.Thread(
            target=self._run,
            name=f"gento-direct-{arm_label}-gripper",
            daemon=True,
        )
        self._thread.start()

    def _send(self, can_id: int, data: bytes, timeout_ms: int = 20) -> None:
        with self._io_lock:
            ret, _sending_time = self.robot.terminal_set(
                self.terminal_type, CHANNEL_CANFD, terminal_can_frame(can_id, data),
                timeout=timeout_ms,
            )
        if ret != 0:
            raise RuntimeError(
                f"FX_L1_Terminal_SetData failed: can_id=0x{can_id:X}, ret={ret}"
            )
        self.tx_frames += 1

    def _receive_once(self, timeout_ms: int) -> tuple[int, bytes] | None:
        with self._io_lock:
            channel, frame, _receiving_time = self.robot.terminal_get(
                self.terminal_type, timeout=timeout_ms,
            )
        if not frame or channel != CHANNEL_CANFD:
            return None
        return parse_terminal_can_frame(frame)

    def _accept_feedback(self, can_id: int, data: bytes) -> bool:
        decoded = decode_dm_feedback(
            can_id,
            data,
            slave_id=self.slave_id,
            master_id=self.master_id,
        )
        if decoded is None:
            return False
        with self._lock:
            self._position_raw = float(decoded["position"])
            self._velocity = float(decoded["velocity"])
            self._torque = float(decoded["torque"])
            self._status = int(decoded["status"])
            self._last_feedback_ns = time.monotonic_ns()
            if self._target_raw is None:
                # Hold the measured pose after enabling.  Never snap to 0 just
                # because the process started before the first policy command.
                self._target_raw = self._position_raw
                self._grip.target_raw = self._position_raw
                self._grip.requested_raw = self._position_raw
        self.rx_frames += 1
        return True

    def _drain_for(self, duration_s: float, *, seek_mode: bool = False) -> bool:
        deadline = time.monotonic() + duration_s
        mode_confirmed = False
        while time.monotonic() < deadline:
            received = self._receive_once(min(20, max(1, int(duration_s * 1000))))
            if received is None:
                continue
            can_id, data = received
            if seek_mode and _is_mit_mode_response(
                can_id, data, slave_id=self.slave_id
            ):
                mode_confirmed = True
            if _is_parameter_response(data, slave_id=self.slave_id):
                continue
            self._accept_feedback(can_id, data)
        return mode_confirmed

    def _initialize_motor(self, timeout_s: float) -> None:
        with self._io_lock:
            clear_ret = self.robot.terminal_clear(self.terminal_type)
        if clear_ret != 0:
            raise RuntimeError(
                f"FX_L1_Terminal_ClearData({self.arm_label}) failed: {clear_ret}"
            )

        self._send(self.slave_id, _DISABLE)
        self._drain_for(0.10)

        # DM register write: slave ID, 0x55 write marker, RID=CTRL_MODE,
        # uint32 little-endian value MIT_MODE=1.
        mode_payload = bytes(
            (self.slave_id, 0x00, 0x55, CONTROL_MODE_RID)
        ) + struct.pack("<I", MIT_MODE)
        self._send(PARAMETER_CAN_ID, mode_payload)
        mode_confirmed = self._drain_for(0.50, seek_mode=True)
        if mode_confirmed:
            LOGGER.info("%s DM gripper confirmed MIT control mode", self.arm_label)
        else:
            # This is the same compatibility behavior as the C++ example:
            # classical firmware may reject register access but already use
            # MIT mode, so proceed to standard MIT enable and verify feedback.
            LOGGER.warning(
                "%s DM gripper did not confirm CTRL_MODE=MIT; "
                "trying classical-firmware MIT enable",
                self.arm_label,
            )

        deadline = time.monotonic() + timeout_s
        next_enable_at = 0.0
        enable_attempts = 0
        received_frames = 0
        seen_can_ids: set[int] = set()
        decoded_statuses: set[int] = set()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_enable_at:
                # DM feedback is request/response. If the one enable command
                # or its reply is dropped, passively waiting cannot recover.
                # Reasserting this idempotent command makes startup tolerant
                # of transient scheduling and terminal-buffer timing.
                self._send(self.slave_id, _ENABLE)
                enable_attempts += 1
                next_enable_at = now + ENABLE_RETRY_INTERVAL_S
            received = self._receive_once(20)
            if received is None:
                continue
            can_id, data = received
            if _is_parameter_response(data, slave_id=self.slave_id):
                continue
            received_frames += 1
            seen_can_ids.add(can_id)
            accepted = self._accept_feedback(can_id, data)
            with self._lock:
                if accepted and self._status is not None:
                    decoded_statuses.add(self._status)
                if self._status == DM_ENABLED and self._position_raw is not None:
                    LOGGER.info(
                        "%s DM gripper enabled: raw_position=%.4f rad, "
                        "normalized=%.3f",
                        self.arm_label,
                        self._position_raw,
                        self._normalize(self._position_raw),
                    )
                    return
        raise RuntimeError(
            f"{self.arm_label} DM gripper did not return enabled feedback after "
            "direct FX_L1_Terminal_SetData; "
            f"enable_attempts={enable_attempts}, "
            f"received_frames={received_frames}, "
            f"can_ids={[hex(value) for value in sorted(seen_can_ids)]}, "
            f"decoded_statuses={sorted(decoded_statuses)}"
        )

    def _normalize(self, raw: float) -> float:
        return float(np.clip((raw - self.low) / (self.high - self.low), 0.0, 1.0))

    def diagnostics(self) -> dict[str, float | int | bool | None]:
        """Return the latest command and physical feedback without CAN I/O.

        This method only copies fields already maintained by the 100 Hz control
        thread.  It is safe for the trajectory recorder to poll and never sends
        a motor command or waits for a new feedback frame.
        """

        sampled_at_ns = time.monotonic_ns()
        with self._lock:
            target = self._target_raw
            position = self._position_raw
            velocity = self._velocity
            torque = self._torque
            status = self._status
            feedback_at_ns = self._last_feedback_ns
        feedback_age_s = (
            (sampled_at_ns - feedback_at_ns) / 1e9 if feedback_at_ns else None
        )
        return {
            "sample_monotonic_ns": sampled_at_ns,
            "feedback_monotonic_ns": feedback_at_ns or None,
            "feedback_age_seconds": feedback_age_s,
            "command_normalized": self._normalize(target) if target is not None else None,
            "command_position_rad": target,
            "feedback_normalized": (
                self._normalize(position) if position is not None else None
            ),
            "feedback_position_rad": position,
            "position_error_rad": (
                target - position
                if target is not None and position is not None
                else None
            ),
            "feedback_velocity_rad_s": velocity,
            "feedback_torque_nm": torque,
            "status": status,
            "enabled": status == DM_ENABLED,
            "kp": self.kp,
            "kd": self.kd,
            "contact_latched": self._grip.contact_latched,
            "input_command": self._grip.input_command,
            "effective_command": self._grip.effective_command,
            "enhanced_mapping": self._grip.enhanced_mapping,
            "mapping_mode": self._grip.mapping_mode,
            "close_scale": self._grip.close_scale,
            "calibrated_closed_position_rad": self.high,
            "close_overtravel_rad": self._grip.closed_rad - self.high,
            "command_closed_position_rad": self._grip.closed_rad,
            "hard_closed_position_rad": self._grip.hard_closed_rad,
            "teleop_range_rad": self._grip.teleop_range_rad,
            "teleop_max_target_rad": self._grip.teleop_max_target_rad,
            "contact_stop_enabled": self._grip.contact_stop_enabled,
            "policy_target_update_mode": (
                "rate_limited" if self.policy_postprocess else "immediate"
            ),
            "policy_postprocess": self.policy_postprocess,
            "requested_position_rad": self._grip.requested_raw,
            "terminal_tx_frames": self.tx_frames,
            "terminal_rx_frames": self.rx_frames,
        }

    def _run(self) -> None:
        consecutive_send_errors = 0
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                with self._lock:
                    if self._position_raw is not None:
                        self._target_raw = self._grip.update(
                            position=self._position_raw,
                            velocity=self._velocity,
                            torque=self._torque,
                        )
                    target_raw = self._target_raw
                if target_raw is not None:
                    try:
                        self._send(
                            self.slave_id,
                            encode_mit_command(target_raw, kp=self.kp, kd=self.kd),
                            timeout_ms=10,
                        )
                    except RuntimeError:
                        consecutive_send_errors += 1
                        if consecutive_send_errors >= 20:
                            raise
                    else:
                        consecutive_send_errors = 0
                received = self._receive_once(2)
                if received is not None:
                    can_id, data = received
                    if not _is_parameter_response(data, slave_id=self.slave_id):
                        self._accept_feedback(can_id, data)
                self._trace_if_needed()
                remaining = self.period_s - (time.monotonic() - started)
                if remaining > 0:
                    self._stop.wait(remaining)
        except BaseException as exc:
            self._fatal_error = exc
            self._stop.set()
            LOGGER.exception("Direct %s-gripper control thread stopped", self.arm_label)

    def _trace_if_needed(self) -> None:
        now = time.monotonic()
        with self._lock:
            target = self._target_raw
            position = self._position_raw
            velocity = self._velocity
            torque = self._torque
            status = self._status
            input_command = self._grip.input_command
            effective_command = self._grip.effective_command
        target_changed = (
            target is not None
            and (
                self._last_logged_target is None
                or abs(target - self._last_logged_target) >= 0.08
            )
        )
        periodic = self.trace_interval_s > 0 and now - self._last_trace_at >= self.trace_interval_s
        if not target_changed and not periodic:
            return
        if target is not None and position is not None:
            LOGGER.info(
                "%s gripper direct trace: policy=%.3f, mapped=%.3f, "
                "command=%.3f (%.4f rad), "
                "feedback=%.3f (%.4f rad), error=%+.4f rad, "
                "velocity=%+.4f rad/s, torque=%+.4f Nm, status=%s",
                self.arm_label,
                input_command,
                effective_command,
                self._normalize(target),
                target,
                self._normalize(position),
                position,
                target - position,
                velocity,
                torque,
                status,
            )
            self._last_logged_target = target
            self._last_trace_at = now

    def _snapshot(self, max_age_s: float) -> float:
        if self._fatal_error is not None:
            raise RuntimeError(
                f"direct {self.arm_label}-gripper thread failed"
            ) from self._fatal_error
        with self._lock:
            position = self._position_raw
            status = self._status
            stamp = self._last_feedback_ns
        if position is None or stamp == 0:
            raise RuntimeError(f"waiting for direct {self.arm_label}-gripper feedback")
        age_s = (time.monotonic_ns() - stamp) / 1e9
        if age_s > max_age_s:
            raise RuntimeError(
                f"direct {self.arm_label}-gripper feedback is stale: age={age_s:.3f}s, "
                f"limit={max_age_s:.3f}s"
            )
        if status != DM_ENABLED:
            raise RuntimeError(
                f"{self.arm_label} DM gripper is not enabled: status={status}"
            )
        return self._normalize(position)

    def wait_for_feedback(
        self,
        timeout_s: float,
        max_age_s: float,
        enabled_arms: set[int] | None = None,
    ) -> np.ndarray:
        del enabled_arms
        deadline = time.monotonic() + timeout_s
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                value = self._snapshot(max_age_s)
                return np.asarray((0.0, value), dtype=np.float32)
            except RuntimeError as exc:
                last_error = exc
                time.sleep(0.02)
        raise RuntimeError(f"direct gripper feedback timeout: {last_error}")

    def state(
        self,
        max_age_s: float,
        enabled_arms: set[int] | None = None,
    ) -> np.ndarray:
        del enabled_arms
        return np.asarray((0.0, self._snapshot(max_age_s)), dtype=np.float32)

    def command(
        self,
        values: np.ndarray,
        enabled_arms: set[int] | None = None,
    ) -> None:
        enabled = {1} if enabled_arms is None else enabled_arms
        if 1 not in enabled:
            return
        values = np.asarray(values, dtype=np.float32)
        if values.shape != (2,) or not np.isfinite(values).all():
            raise ValueError(f"gripper command must be finite shape (2,), got {values.shape}")
        command = float(np.clip(values[1], 0.0, GRIPPER_COMMAND_MAX))
        with self._lock:
            self._grip.set_command(
                command,
                enhanced=self.policy_postprocess,
                teleop=not self.policy_postprocess,
            )

    def command_calibrated(
        self,
        values: np.ndarray,
        enabled_arms: set[int] | None = None,
    ) -> None:
        """Command the measured endpoint mapping without policy enhancements."""

        enabled = {1} if enabled_arms is None else enabled_arms
        if 1 not in enabled:
            return
        values = np.asarray(values, dtype=np.float32)
        if values.shape != (2,) or not np.isfinite(values).all():
            raise ValueError(f"gripper command must be finite shape (2,), got {values.shape}")
        command = float(np.clip(values[1], 0.0, GRIPPER_COMMAND_MAX))
        with self._lock:
            self._grip.set_command(command, enhanced=False)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        LOGGER.info(
            "Stopped direct %s-gripper control: terminal_tx=%d, terminal_rx=%d",
            self.arm_label,
            self.tx_frames,
            self.rx_frames,
        )


class DualGentoDmGripper:
    """Two-arm gripper facade matching the robot-control GripperBackend protocol."""

    def __init__(
        self,
        robot: _TerminalRobot,
        *,
        left_limits: tuple[float, float],
        right_limits: tuple[float, float],
        command_hz: float = 100.0,
        kp: float = 3.0,
        kd: float = 0.12,
        initialization_timeout_s: float = 3.0,
        trace_interval_s: float = 2.0,
        close_rate_rad_s: float = 0.40,
        contact_torque_nm: float = 0.35,
        contact_velocity_rad_s: float = 0.12,
        contact_cycles: int = 5,
        regrip_torque_nm: float = 0.20,
        regrip_cycles: int = 10,
        close_scale: float = 1.0,
        close_overtravel_rad: float = 0.0,
        policy_postprocess: bool = True,
        teleop_range_rad: float = 1.6,
    ) -> None:
        io_lock = threading.RLock()
        common = dict(
            robot=robot,
            command_hz=command_hz,
            kp=kp,
            kd=kd,
            initialization_timeout_s=initialization_timeout_s,
            trace_interval_s=trace_interval_s,
            close_rate_rad_s=close_rate_rad_s,
            contact_torque_nm=contact_torque_nm,
            contact_velocity_rad_s=contact_velocity_rad_s,
            contact_cycles=contact_cycles,
            regrip_torque_nm=regrip_torque_nm,
            regrip_cycles=regrip_cycles,
            close_scale=close_scale,
            close_overtravel_rad=close_overtravel_rad,
            policy_postprocess=policy_postprocess,
            teleop_range_rad=teleop_range_rad,
            io_lock=io_lock,
        )
        left = GentoDmGripper(
            **common,
            limits=left_limits,
            terminal_type=TERMINAL_ARM0,
            arm_label="left",
            slave_id=LEFT_SLAVE_ID,
            master_id=LEFT_MASTER_ID,
        )
        try:
            right = GentoDmGripper(
                **common,
                limits=right_limits,
                terminal_type=TERMINAL_ARM1,
                arm_label="right",
                slave_id=RIGHT_SLAVE_ID,
                master_id=RIGHT_MASTER_ID,
            )
        except BaseException:
            left.close()
            raise
        self._motors = (left, right)

    @staticmethod
    def _enabled(enabled_arms: set[int] | None) -> set[int]:
        return {0, 1} if enabled_arms is None else enabled_arms

    def wait_for_feedback(self, timeout_s: float, max_age_s: float,
                          enabled_arms: set[int] | None = None) -> np.ndarray:
        result = np.zeros(2, dtype=np.float32)
        for arm in self._enabled(enabled_arms):
            result[arm] = self._motors[arm]._snapshot(max_age_s)
        return result

    def state(self, max_age_s: float, enabled_arms: set[int] | None = None) -> np.ndarray:
        result = np.zeros(2, dtype=np.float32)
        for arm in self._enabled(enabled_arms):
            result[arm] = self._motors[arm]._snapshot(max_age_s)
        return result

    def command(self, values: np.ndarray, enabled_arms: set[int] | None = None) -> None:
        values = np.asarray(values, dtype=np.float32)
        if values.shape != (2,) or not np.isfinite(values).all():
            raise ValueError(f"gripper command must be finite shape (2,), got {values.shape}")
        for arm in self._enabled(enabled_arms):
            # Each single backend uses index 1, so adapt the selected arm value.
            self._motors[arm].command(np.asarray((0.0, values[arm]), dtype=np.float32), {1})

    def command_calibrated(
        self,
        values: np.ndarray,
        enabled_arms: set[int] | None = None,
    ) -> None:
        values = np.asarray(values, dtype=np.float32)
        if values.shape != (2,) or not np.isfinite(values).all():
            raise ValueError(f"gripper command must be finite shape (2,), got {values.shape}")
        for arm in self._enabled(enabled_arms):
            self._motors[arm].command_calibrated(
                np.asarray((0.0, values[arm]), dtype=np.float32), {1}
            )

    def diagnostics(self) -> dict[str, object]:
        return {"left": self._motors[0].diagnostics(), "right": self._motors[1].diagnostics()}

    def close(self) -> None:
        for motor in self._motors:
            motor.close()
