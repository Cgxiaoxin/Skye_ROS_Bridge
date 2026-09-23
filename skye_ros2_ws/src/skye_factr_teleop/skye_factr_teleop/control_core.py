# Copyright (c) 2026 lyy
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li
# SPDX-License-Identifier: Apache-2.0

"""ROS-free, non-blocking leader control core with explicit calibration."""

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .config import ARM_JOINT_COUNT, MOTOR_COUNT, TeleopConfig
from .dynamixel_driver import DynamixelIO, get_servo_spec, torque_nm_to_current_ma


class DynamicsIO(Protocol):
    """Implemented by a runtime adapter; no Pinocchio dependency here."""

    def gravity(self, q: np.ndarray) -> np.ndarray: ...
    def jacobian(self, q: np.ndarray) -> np.ndarray: ...
    def forward_kinematics(self, q: np.ndarray) -> object: ...


class FeedbackIO(Protocol):
    """Reserved for stage-8 follower torque and gripper feedback adapters."""

    def external_joint_torque(self) -> np.ndarray: ...
    def gripper_feedback(self) -> float: ...


@dataclass(frozen=True)
class ControlTerms:
    joint_limits: bool = True
    gravity: bool = True
    static_friction: bool = True
    nullspace: bool = False


@dataclass(frozen=True)
class LeaderState:
    arm_position: np.ndarray
    arm_velocity: np.ndarray
    gripper_position: float
    gripper_velocity: float


@dataclass(frozen=True)
class CycleResult:
    state: LeaderState
    joint_limit_torque: np.ndarray
    gravity_torque: np.ndarray
    friction_torque: np.ndarray
    nullspace_torque: np.ndarray
    logical_torque: np.ndarray
    physical_current_ma: np.ndarray
    wrote_current: bool


def _vector(value: object, size: int, name: str) -> np.ndarray:
    try:
        result = np.array(value, dtype=float, copy=True)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain real numbers") from exc
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite vector of length {size}")
    return result


def _scalar(value: object, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not np.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def select_joint_offsets(
    raw_position: object, joint_signs: object, calibration_joint_pos: object,
) -> np.ndarray:
    """Choose nearest M6 quarter-turn zero in -20π..20π; zero the gripper here.

    Acquiring the calibration pose and checking physical alignment are external.
    """
    raw = _vector(raw_position, MOTOR_COUNT, "raw_position")
    signs = _vector(joint_signs, MOTOR_COUNT, "joint_signs")
    target = _vector(calibration_joint_pos, ARM_JOINT_COUNT, "calibration_joint_pos")
    if not np.all(np.isin(signs, (-1.0, 1.0))):
        raise ValueError("joint_signs must contain only -1 or 1")
    choices = np.arange(-40, 41, dtype=float) * (np.pi / 2.0)
    errors = np.abs((raw[:ARM_JOINT_COUNT, None] - choices) * signs[:ARM_JOINT_COUNT, None] - target[:, None])
    offsets = np.empty(MOTOR_COUNT)
    offsets[:ARM_JOINT_COUNT] = choices[np.argmin(errors, axis=1)]
    offsets[-1] = raw[-1]
    return offsets


def transform_leader_state(
    raw_position: object, raw_velocity: object, offsets: object, joint_signs: object,
) -> LeaderState:
    """Split seven logical arm joints and one gripper from physical servos."""
    position = _vector(raw_position, MOTOR_COUNT, "raw_position")
    velocity = _vector(raw_velocity, MOTOR_COUNT, "raw_velocity")
    zero = _vector(offsets, MOTOR_COUNT, "offsets")
    signs = _vector(joint_signs, MOTOR_COUNT, "joint_signs")
    if not np.all(np.isin(signs, (-1.0, 1.0))):
        raise ValueError("joint_signs must contain only -1 or 1")
    return LeaderState(
        (position[:ARM_JOINT_COUNT] - zero[:ARM_JOINT_COUNT]) * signs[:ARM_JOINT_COUNT],
        velocity[:ARM_JOINT_COUNT] * signs[:ARM_JOINT_COUNT],
        float((position[-1] - zero[-1]) * signs[-1]),
        float(velocity[-1] * signs[-1]),
    )


def joint_limit_barrier(
    state: LeaderState, lower: object, upper: object, kp: float, kd: float,
    gripper_lower: float, gripper_upper: float, *, margin: float = 0.0,
) -> np.ndarray:
    """Repulsive torque only outside the safe interval, including gripper."""
    q = _vector(state.arm_position, ARM_JOINT_COUNT, "arm_position")
    dq = _vector(state.arm_velocity, ARM_JOINT_COUNT, "arm_velocity")
    safety_margin = _scalar(margin, "margin")
    minimum = _vector(lower, ARM_JOINT_COUNT, "lower") + safety_margin
    maximum = _vector(upper, ARM_JOINT_COUNT, "upper") - safety_margin
    gain_p, gain_d = _scalar(kp, "kp"), _scalar(kd, "kd")
    if np.any(minimum >= maximum) or min(gain_p, gain_d, safety_margin) < 0:
        raise ValueError("invalid joint limit interval or gains")
    result = np.zeros(MOTOR_COUNT)
    outside = (q < minimum) | (q > maximum)
    bound = np.clip(q, minimum, maximum)
    result[:ARM_JOINT_COUNT] = np.where(outside, -gain_p * (q - bound) - gain_d * dq, 0.0)
    grip = _scalar(state.gripper_position, "gripper_position")
    grip_vel = _scalar(state.gripper_velocity, "gripper_velocity")
    grip_min = _scalar(gripper_lower, "gripper_lower")
    grip_max = _scalar(gripper_upper, "gripper_upper")
    if grip_min >= grip_max:
        raise ValueError("invalid gripper limits")
    if grip < grip_min or grip > grip_max:
        result[-1] = -gain_p * (grip - np.clip(grip, grip_min, grip_max)) - gain_d * grip_vel
    return result


def gravity_compensation(q: object, dynamics: DynamicsIO, gain: object) -> np.ndarray:
    position = _vector(q, ARM_JOINT_COUNT, "arm_position")
    weights = _vector(gain, ARM_JOINT_COUNT, "gravity_gain")
    return _vector(dynamics.gravity(position.copy()), ARM_JOINT_COUNT, "gravity") * weights


def static_friction_compensation(
    velocity: object, gravity_torque: object, enable_speed: object,
    gain: object, dither_positive: object,
) -> tuple[np.ndarray, np.ndarray]:
    """Alternate the M6 stiction dither only on joints below speed threshold."""
    dq = _vector(velocity, ARM_JOINT_COUNT, "arm_velocity")
    gravity = _vector(gravity_torque, ARM_JOINT_COUNT, "gravity_torque")
    speed = _vector(enable_speed, ARM_JOINT_COUNT, "enable_speed")
    weight = _vector(gain, ARM_JOINT_COUNT, "friction_gain")
    flags = np.array(dither_positive, dtype=bool, copy=True)
    if flags.shape != (ARM_JOINT_COUNT,) or np.any(speed < 0) or np.any(weight < 0):
        raise ValueError("invalid static friction parameters")
    active = np.abs(dq) < speed
    torque = np.where(active, np.where(flags, 1.0, -1.0) * weight * np.abs(gravity), 0.0)
    flags[active] = ~flags[active]
    return torque, flags


def nullspace_regulation(
    q: object, dq: object, dynamics: DynamicsIO, target: object,
    kp: object, kd: object,
) -> np.ndarray:
    position = _vector(q, ARM_JOINT_COUNT, "arm_position")
    velocity = _vector(dq, ARM_JOINT_COUNT, "arm_velocity")
    setpoint = _vector(target, ARM_JOINT_COUNT, "nullspace_target")
    gain_p = _vector(kp, ARM_JOINT_COUNT, "nullspace_kp")
    gain_d = _vector(kd, ARM_JOINT_COUNT, "nullspace_kd")
    jacobian = np.array(dynamics.jacobian(position.copy()), dtype=float, copy=True)
    if jacobian.ndim != 2 or jacobian.shape[1] != ARM_JOINT_COUNT or not np.all(np.isfinite(jacobian)):
        raise ValueError("jacobian must be a finite matrix with seven columns")
    projector = np.eye(ARM_JOINT_COUNT) - np.linalg.pinv(jacobian) @ jacobian
    return projector @ (-gain_p * (position - setpoint) - gain_d * velocity)


class ControlCore:
    """Exactly one read / compute / optional write per call; never enables torque."""

    def __init__(
        self, config: TeleopConfig, hardware: DynamixelIO, dynamics: DynamicsIO,
        joint_offsets: object, current_limit_ma: object,
        terms: ControlTerms | None = None,
    ) -> None:
        self.config, self.hardware, self.dynamics = config, hardware, dynamics
        self.joint_offsets = _vector(joint_offsets, MOTOR_COUNT, "joint_offsets")
        self.joint_signs = _vector(config.dynamixel.joint_signs, MOTOR_COUNT, "joint_signs")
        if np.isscalar(current_limit_ma):
            limits = np.full(MOTOR_COUNT, _scalar(current_limit_ma, "current_limit_ma"))
        else:
            limits = _vector(current_limit_ma, MOTOR_COUNT, "current_limit_ma")
        if np.any(limits <= 0):
            raise ValueError("current_limit_ma must be positive")
        self.current_limit_ma = np.minimum(
            limits,
            [get_servo_spec(model).current_limit_ma for model in config.dynamixel.servo_types],
        )
        self.terms = terms or ControlTerms(
            joint_limits=config.arm_teleop.arm_limit_barrier_enable,
            gravity=config.controller.gravity_comp.enable,
            static_friction=config.controller.gravity_comp.enable,
            nullspace=config.controller.null_space_regulation.enable,
        )
        self._dither_positive = np.ones(ARM_JOINT_COUNT, dtype=bool)

    def step(self) -> CycleResult:
        """Run one cycle; any fault attempts zero current and torque disable."""
        try:
            raw_position, raw_velocity = self.hardware.read_state()
            state = transform_leader_state(raw_position, raw_velocity, self.joint_offsets, self.joint_signs)
            limit = np.zeros(MOTOR_COUNT)
            gravity = np.zeros(MOTOR_COUNT)
            friction = np.zeros(MOTOR_COUNT)
            nullspace = np.zeros(MOTOR_COUNT)
            next_flags = self._dither_positive
            arm, ctrl = self.config.arm_teleop, self.config.controller
            if self.hardware.torque_enabled:
                if self.terms.joint_limits:
                    limit = joint_limit_barrier(
                        state, arm.arm_joint_limits_min, arm.arm_joint_limits_max,
                        ctrl.joint_limit_barrier.kp, ctrl.joint_limit_barrier.kd,
                        self.config.gripper_teleop.limit_min, self.config.gripper_teleop.limit_max,
                        margin=arm.arm_joint_limits_safety_margin,
                    )
                if self.terms.gravity or self.terms.static_friction:
                    raw_gravity = _vector(
                        self.dynamics.gravity(state.arm_position.copy()), ARM_JOINT_COUNT, "gravity",
                    )
                    if self.terms.gravity:
                        gravity[:ARM_JOINT_COUNT] = raw_gravity * ctrl.gravity_comp.gain
                    if self.terms.static_friction:
                        friction[:ARM_JOINT_COUNT], next_flags = static_friction_compensation(
                            state.arm_velocity, raw_gravity * ctrl.gravity_comp.gain,
                            ctrl.static_friction_comp.enable_speed,
                            ctrl.static_friction_comp.gain, self._dither_positive,
                        )
                if self.terms.nullspace:
                    n = ctrl.null_space_regulation
                    nullspace[:ARM_JOINT_COUNT] = nullspace_regulation(
                        state.arm_position, state.arm_velocity, self.dynamics,
                        n.null_space_joint_target, n.kp, n.kd,
                    )
            logical = limit + gravity + friction + nullspace
            _vector(logical, MOTOR_COUNT, "logical_torque")
            physical_torque = logical * self.joint_signs
            currents = np.array([
                torque_nm_to_current_ma(torque, model)
                for torque, model in zip(physical_torque, self.config.dynamixel.servo_types)
            ])
            currents = np.clip(currents, -self.current_limit_ma, self.current_limit_ma)
            wrote = bool(self.hardware.torque_enabled)
            if wrote:
                self.hardware.write_currents(currents)
                self._dither_positive = next_flags
            return CycleResult(state, limit, gravity, friction, nullspace, logical, currents, wrote)
        except Exception:
            if self.hardware.torque_enabled:
                try:
                    self.hardware.write_currents(np.zeros(MOTOR_COUNT))
                finally:
                    self.hardware.set_torque_enabled(False)
            raise
