"""Stateful command shaping for the Tianji policy execution loop.

The limiter deliberately evolves from the previous *software command*, not
from physical feedback.  Using feedback as the base on every cycle recreates
the old global slew limiter: the robot falls behind the policy phase and the
policy then produces increasingly aggressive catch-up targets.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


RIGHT_ARM_DOF = 7


@dataclass(frozen=True)
class JointLimitResult:
    command_deg: np.ndarray
    command_velocity_deg_s: np.ndarray
    velocity_limited: np.ndarray
    acceleration_limited: np.ndarray


class JointCommandLimiter:
    """Apply per-joint velocity and acceleration limits to policy targets."""

    def __init__(
        self,
        *,
        command_hz: float,
        velocity_limits_deg_s: np.ndarray,
        acceleration_limits_deg_s2: np.ndarray,
    ) -> None:
        self.command_hz = float(command_hz)
        if not math.isfinite(self.command_hz) or self.command_hz <= 0:
            raise ValueError("command_hz must be finite and positive")
        self.dt = 1.0 / self.command_hz
        self.velocity_limits = self._limit_vector(
            velocity_limits_deg_s, "velocity_limits_deg_s"
        )
        self.acceleration_limits = self._limit_vector(
            acceleration_limits_deg_s2, "acceleration_limits_deg_s2"
        )
        self._last_command: np.ndarray | None = None
        self._last_velocity = np.zeros(RIGHT_ARM_DOF, dtype=np.float64)

    @staticmethod
    def _limit_vector(values: np.ndarray, name: str) -> np.ndarray:
        array = np.asarray(values, dtype=np.float64)
        if array.shape != (RIGHT_ARM_DOF,):
            raise ValueError(f"{name} must have shape ({RIGHT_ARM_DOF},), got {array.shape}")
        if not np.isfinite(array).all() or np.any(array <= 0):
            raise ValueError(f"{name} must contain finite positive values")
        return array.copy()

    def reset(self, position_deg: np.ndarray) -> None:
        position = np.asarray(position_deg, dtype=np.float64)
        if position.shape != (RIGHT_ARM_DOF,) or not np.isfinite(position).all():
            raise ValueError("limiter reset position must be a finite 7-D vector")
        self._last_command = position.copy()
        self._last_velocity.fill(0.0)

    def step(self, target_deg: np.ndarray) -> JointLimitResult:
        target = np.asarray(target_deg, dtype=np.float64)
        if target.shape != (RIGHT_ARM_DOF,) or not np.isfinite(target).all():
            raise ValueError("limiter target must be a finite 7-D vector")
        if self._last_command is None:
            raise RuntimeError("JointCommandLimiter.reset() must be called before step()")

        previous = self._last_command
        target_delta = target - previous
        desired_velocity = target_delta / self.dt
        velocity_limited = np.abs(desired_velocity) > self.velocity_limits + 1e-9
        # Start decelerating before a nearby target.  This preserves a strict
        # acceleration bound without relying on an abrupt endpoint snap.
        braking_velocity = np.sqrt(
            2.0 * self.acceleration_limits * np.abs(target_delta)
        )
        velocity_target = np.sign(target_delta) * np.minimum(
            np.minimum(np.abs(desired_velocity), self.velocity_limits),
            braking_velocity,
        )

        max_velocity_change = self.acceleration_limits * self.dt
        velocity_change = velocity_target - self._last_velocity
        acceleration_limited = np.abs(velocity_change) > max_velocity_change + 1e-9
        command_velocity = self._last_velocity + np.clip(
            velocity_change,
            -max_velocity_change,
            max_velocity_change,
        )
        command = previous + command_velocity * self.dt

        # Land exactly on a target only when that final step also satisfies
        # the acceleration bound.  During a sharp reversal, the virtual
        # trajectory may pass the target while it decelerates; that bounded
        # behavior is preferable to hiding an acceleration spike in a snap.
        crossed_target = (
            (target_delta != 0.0)
            & (np.sign(target - command) != np.sign(target_delta))
        )
        already_at_target = np.isclose(target_delta, 0.0, atol=1e-9)
        snap_velocity = target_delta / self.dt
        snap_respects_acceleration = (
            np.abs(snap_velocity - self._last_velocity)
            <= max_velocity_change + 1e-9
        )
        snap_to_target = (crossed_target | already_at_target) & snap_respects_acceleration
        if np.any(snap_to_target):
            command[snap_to_target] = target[snap_to_target]
            command_velocity[snap_to_target] = (
                command[snap_to_target] - previous[snap_to_target]
            ) / self.dt

        self._last_command = command.copy()
        self._last_velocity = command_velocity.copy()
        return JointLimitResult(
            command_deg=command.astype(np.float32),
            command_velocity_deg_s=command_velocity.astype(np.float32),
            velocity_limited=velocity_limited,
            acceleration_limited=acceleration_limited,
        )


@dataclass(frozen=True)
class TrackingErrorResult:
    active_joints: tuple[int, ...]
    timed_out_joints: tuple[int, ...]
    replan_attempt: int

    @property
    def triggered(self) -> bool:
        return bool(self.timed_out_joints)


class TrackingErrorGuard:
    """Detect a sustained model-target versus feedback tracking error.

    This monitor never modifies a target.  Its only job is to distinguish a
    brief, recoverable lag from a persistent following failure that requires
    holding position and replanning.
    """

    def __init__(
        self,
        *,
        command_hz: float,
        timeout_s: float,
        reset_s: float,
        dof: int = RIGHT_ARM_DOF,
    ) -> None:
        if min(command_hz, timeout_s, reset_s) <= 0:
            raise ValueError("tracking guard frequencies/durations must be positive")
        if dof <= 0:
            raise ValueError("tracking guard dof must be positive")
        self.timeout_cycles = max(1, math.ceil(command_hz * timeout_s))
        self.reset_cycles = max(1, math.ceil(command_hz * reset_s))
        self.dof = int(dof)
        self._active_cycles = np.zeros(self.dof, dtype=np.int64)
        self._clear_cycles = 0
        self._replan_attempts = 0

    def reset_active(self, *, reset_attempts: bool = False) -> None:
        self._active_cycles.fill(0)
        self._clear_cycles = 0
        if reset_attempts:
            self._replan_attempts = 0

    def update(self, tracking_error_mask: np.ndarray) -> TrackingErrorResult:
        mask = np.asarray(tracking_error_mask, dtype=bool)
        if mask.shape != (self.dof,):
            raise ValueError(f"tracking error mask must have shape ({self.dof},)")

        self._active_cycles[mask] += 1
        self._active_cycles[~mask] = 0
        active_joints = tuple(int(index) for index in np.flatnonzero(mask))

        if active_joints:
            self._clear_cycles = 0
        else:
            self._clear_cycles += 1
            if self._clear_cycles >= self.reset_cycles:
                self._replan_attempts = 0

        timed_out = self._active_cycles >= self.timeout_cycles
        timed_out_joints = tuple(int(index) for index in np.flatnonzero(timed_out))
        if timed_out_joints:
            self._replan_attempts += 1
            # A replan gets a complete new observation window.  If the new
            # policy output returns to the same band, it must persist for a
            # full timeout again before another attempt is consumed.
            self._active_cycles.fill(0)

        return TrackingErrorResult(
            active_joints=active_joints,
            timed_out_joints=timed_out_joints,
            replan_attempt=self._replan_attempts,
        )
