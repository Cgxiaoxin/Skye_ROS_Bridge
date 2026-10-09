# Copyright (c) 2026 lyy
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li
# SPDX-License-Identifier: Apache-2.0

"""ROS-free, non-blocking leader control core with explicit calibration."""

from dataclasses import dataclass
from collections.abc import Callable
from typing import Protocol

import numpy as np

from .config import ARM_JOINT_COUNT, MOTOR_COUNT, TeleopConfig
from .dynamixel_driver import DynamixelIO, get_servo_spec, torque_nm_to_current_ma
from .state_machine import Action, ControlMode, Event, StateMachine, TransitionResult


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
class TrajectorySample:
    position: np.ndarray
    velocity: np.ndarray
    finished: bool

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
    tracking_torque: np.ndarray
    mode: ControlMode
    target_position: np.ndarray | None
    target_velocity: np.ndarray | None

@dataclass(frozen=True)
class _TorqueCalculation:
    joint_limit: np.ndarray
    gravity: np.ndarray
    friction: np.ndarray
    nullspace: np.ndarray
    tracking: np.ndarray
    target_position: np.ndarray | None
    target_velocity: np.ndarray | None
    next_dither_positive: np.ndarray

class DynamicsIO(Protocol):
    """Implemented by a runtime adapter; no Pinocchio dependency here."""

    def gravity(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """Return unscaled RNEA(q, dq, 0) torque in Nm, matching FACTR.

        Inputs are seven logical arm joints in rad and rad/s, in LeaderState
        order. Despite the upstream name, this includes velocity-dependent
        torque C(q, dq) @ dq as well as gravity g(q). Apply gains in the core.
        """
        ...

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """Match pin.computeJointJacobian(model, data, q, num_arm_joints).

        Return the last arm joint's 6-by-7 Jacobian with columns in LeaderState
        order, using the same frame convention as that upstream call.
        """
        ...

    def forward_kinematics(self, q: np.ndarray) -> object: ...

class FeedbackIO(Protocol):
    """Reserved for stage-8 follower torque and gripper feedback adapters."""

    def external_joint_torque(self) -> np.ndarray: ...
    def gripper_feedback(self) -> float: ...

class TrajectoryIO(Protocol):
    """One synchronous trajectory per sync; no queued completion callbacks."""

    def start(self, position: np.ndarray, velocity: np.ndarray, target: np.ndarray) -> None: ...
    def update(self) -> TrajectorySample: ...

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
    gripper_kp: float, gripper_kd: float,
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
    grip_gain_p = _scalar(gripper_kp, "gripper_kp")
    grip_gain_d = _scalar(gripper_kd, "gripper_kd")
    if grip_min >= grip_max or min(grip_gain_p, grip_gain_d) < 0:
        raise ValueError("invalid gripper limits or gains")
    if grip < grip_min or grip > grip_max:
        result[-1] = -grip_gain_p * (grip - np.clip(grip, grip_min, grip_max)) - grip_gain_d * grip_vel
    return result

def gravity_compensation(
    q: object, dq: object, dynamics: DynamicsIO, gain: object,
    *, mode: str = "position_velocity",
) -> np.ndarray:
    """Scale RNEA(q, dq, 0), or RNEA(q, 0, 0) for position_only.

    Zero only the model's velocity input; other control terms use measured dq.
    """
    position = _vector(q, ARM_JOINT_COUNT, "arm_position")
    velocity = _vector(dq, ARM_JOINT_COUNT, "arm_velocity")
    weights = _vector(gain, ARM_JOINT_COUNT, "gravity_gain")
    if mode == "position_only":
        velocity = np.zeros_like(velocity)
    elif mode != "position_velocity":
        raise ValueError("gravity mode must be position_velocity or position_only")
    return _vector(
        dynamics.gravity(position.copy(), velocity.copy()), ARM_JOINT_COUNT, "gravity",
    ) * weights

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
    """Single-caller control loop; never enables torque or publishes ROS data.

    handle() and step() must be serialized by the caller. A factory creates a
    fresh trajectory for every accepted sync. All targets use logical leader
    coordinates; follower mapping/freshness checks belong to the ROS adapter.
    """

    def __init__(
        self, config: TeleopConfig, hardware: DynamixelIO, dynamics: DynamicsIO,
        joint_offsets: object, current_limit_ma: object,
        terms: ControlTerms | None = None,
        *, trajectory_factory: Callable[[], TrajectoryIO] | None = None,
        now: float = 0.0,
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
        # 显式传入 terms 时，优先使用该对象，实现各控制项独立开关。
        # 未传入时，按配置生成开关；静摩擦跟随重力开关，保持上游默认行为。
        # 此处显式指定各字段，因此修改 ControlTerms 类的字段默认值不会影响此分支。
        self.terms = terms or ControlTerms(
            joint_limits=config.controller.limit_barrier_control.enable,
            gravity=config.controller.joint_gravity_comp.enable,
            static_friction=config.controller.joint_gravity_comp.enable,
            nullspace=config.controller.null_space_regulation.enable,
        )
        self._dither_positive = np.ones(ARM_JOINT_COUNT, dtype=bool)
        self._now = _scalar(now, "now")
        self._machine = StateMachine(now=self._now)
        self._trajectory_factory = trajectory_factory
        self._trajectory: TrajectoryIO | None = None
        self._target: np.ndarray | None = None
        self._hold_integral = np.zeros(MOTOR_COUNT)
        self._settle_since: float | None = None
        self._last_step_at = self._now
        self._last_transition: TransitionResult | None = None

    @property
    def mode(self) -> ControlMode:
        return self._machine.state

    @property
    def last_transition(self) -> TransitionResult | None:
        return self._last_transition

    @property
    def hold_integral(self) -> np.ndarray:
        return self._hold_integral.copy()

    def step(self, *, now: float | None = None) -> CycleResult:
        """Run one cycle; any fault attempts zero current and torque disable."""
        try:
            dt = self._prepare_cycle(now)
            state = self._read_state()
            torques = self._calculate_torques(state, dt)
            logical = (torques.joint_limit + torques.gravity + torques.friction
                       + torques.nullspace + torques.tracking)
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
                self._dither_positive = torques.next_dither_positive
            return CycleResult(
                state=state,
                joint_limit_torque=torques.joint_limit,
                gravity_torque=torques.gravity,
                friction_torque=torques.friction,
                nullspace_torque=torques.nullspace,
                logical_torque=logical,
                physical_current_ma=currents,
                wrote_current=wrote,
                tracking_torque=torques.tracking,
                mode=self.mode,
                target_position=torques.target_position,
                target_velocity=torques.target_velocity,
            )
        except Exception:
            self.stop()
            raise

    def handle(
        self, event: Event | str, *, now: float,
        follower_position: object = None, gripper_setpoint: float | None = None,
    ) -> TransitionResult:
        """Handle user switches; completion/timeout come only from step().

        Starting sync captures one fresh eight-axis hardware sample. On any
        setup/action failure, stop and disable torque before propagating it.
        Rejected/ignored requests do not read hardware or replace the target.
        """
        if event == Event.SWITCH_STOP:
            try:
                self._time(now)
            except (TypeError, ValueError):
                pass  # A bad timestamp must not prevent an explicit stop.
            return self.stop()
        try:
            self._time(now)
            if event in (Event.SYNC_COMPLETE, Event.SYNC_TIMEOUT,
                         Event.TRAJECTORY_COMPLETE, Event.SETTLE_TIMEOUT):
                raise ValueError("trajectory events are internal to step()")
            result = self._machine.handle(event, now=self._now)
            self._apply_actions(result, follower_position, gripper_setpoint)
            self._last_transition = result
            return result
        except Exception:
            self.stop()
            raise

    def stop(self) -> TransitionResult:
        """Stop even without a valid clock; clear state before hardware cleanup."""
        result = self._machine.handle(Event.SWITCH_STOP, now=self._now)
        self._last_transition = result
        self._clear_sync()
        self._zero_and_disable()
        return result

    def _time(self, now: float) -> float:
        value = _scalar(now, "now")
        if value < self._now:
            raise ValueError("now must be nondecreasing")
        self._now = value
        return value

    def _read_state(self) -> LeaderState:
        position, velocity = self.hardware.read_state()
        return transform_leader_state(position, velocity, self.joint_offsets, self.joint_signs)

    def _prepare_cycle(self, now: float | None) -> float:
        if now is None and self.mode in (ControlMode.TELEOP_SYNCING, ControlMode.HOLD_SETTLING, ControlMode.SYNCED):
            raise ValueError("now is required during synchronization and hold")
        if now is not None:
            self._time(now)
        dt = self._now - self._last_step_at
        self._last_step_at = self._now
        if not self.hardware.torque_enabled and self.mode != ControlMode.IDLE:
            self.stop()
        if (self.mode == ControlMode.TELEOP_SYNCING
                and self._now - self._machine.entered_at
                >= self.config.controller.sync_hold_control.ruckig.trajectory_timeout):
            self._trajectory_event(Event.SYNC_TIMEOUT)
            self._zero_and_disable()
        if (self.mode == ControlMode.HOLD_SETTLING
                and self._now - self._machine.entered_at
                >= self.config.controller.sync_hold_control.hold.settle_timeout):
            self._trajectory_event(Event.SETTLE_TIMEOUT)
            self._zero_and_disable()
        return dt

    def _calculate_torques(self, state: LeaderState, dt: float) -> _TorqueCalculation:
        limit = np.zeros(MOTOR_COUNT)
        gravity = np.zeros(MOTOR_COUNT)
        friction = np.zeros(MOTOR_COUNT)
        nullspace = np.zeros(MOTOR_COUNT)
        tracking = np.zeros(MOTOR_COUNT)
        target = target_velocity = None
        next_flags = self._dither_positive
        ctrl = self.config.controller
        if self.hardware.torque_enabled:
            tracking, target, target_velocity = self._update_tracking(state, dt)
            motion = self.mode in (ControlMode.TELEOP_SYNCING, ControlMode.HOLD_SETTLING, ControlMode.SYNCED)
            if self.terms.joint_limits:
                barrier = ctrl.limit_barrier_control
                joint = barrier.joint
                gripper = barrier.gripper
                limit = joint_limit_barrier(
                    state, joint.limit_min, joint.limit_max,
                    joint.kp, joint.kd,
                    gripper.limit_min, gripper.limit_max,
                    margin=joint.safety_margin,
                    gripper_kp=gripper.kp, gripper_kd=gripper.kd,
                )
            if self.terms.gravity or self.terms.static_friction:
                gain = ctrl.joint_gravity_comp.gain
                if self.mode == ControlMode.TELEOP_SYNCING:
                    gain = ctrl.sync_hold_control.ruckig.gravity_comp_gain
                elif self.mode in (ControlMode.HOLD_SETTLING, ControlMode.SYNCED):
                    gain = ctrl.sync_hold_control.hold.gravity_comp_gain
                compensation = gravity_compensation(
                    state.arm_position, state.arm_velocity,
                    self.dynamics, gain,
                    mode=ctrl.joint_gravity_comp.mode,
                )
                if self.terms.gravity:
                    gravity[:ARM_JOINT_COUNT] = compensation
                if self.terms.static_friction:
                    friction[:ARM_JOINT_COUNT], next_flags = static_friction_compensation(
                        state.arm_velocity, compensation,
                        ctrl.static_friction_comp.enable_speed,
                        ctrl.static_friction_comp.gain, self._dither_positive,
                    )
            if self.terms.nullspace and not motion:
                n = ctrl.null_space_regulation
                nullspace[:ARM_JOINT_COUNT] = nullspace_regulation(
                    state.arm_position, state.arm_velocity, self.dynamics,
                    n.null_space_joint_target, n.kp, n.kd,
                )
        return _TorqueCalculation(
            joint_limit=limit,
            gravity=gravity,
            friction=friction,
            nullspace=nullspace,
            tracking=tracking,
            target_position=target,
            target_velocity=target_velocity,
            next_dither_positive=next_flags,
        )

    def _update_tracking(
        self, state: LeaderState, dt: float,
    ) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
        """Advance the trajectory, then hold until measured state settles."""
        if self.mode not in (ControlMode.TELEOP_SYNCING, ControlMode.HOLD_SETTLING, ControlMode.SYNCED):
            return np.zeros(MOTOR_COUNT), None, None

        ruckig = self.config.controller.sync_hold_control.ruckig
        hold = self.config.controller.sync_hold_control.hold
        position = np.append(state.arm_position, state.gripper_position)
        velocity = np.append(state.arm_velocity, state.gripper_velocity)

        if self.mode == ControlMode.TELEOP_SYNCING:
            if self._trajectory is None:
                raise RuntimeError("missing synchronization trajectory")

            sample = self._trajectory.update()
            target = _vector(sample.position, MOTOR_COUNT, "trajectory position")
            target_velocity = _vector(sample.velocity, MOTOR_COUNT, "trajectory velocity")
            if not sample.finished:
                torque = np.asarray(ruckig.kp) * (target - position)
                torque += np.asarray(ruckig.kd) * (target_velocity - velocity)
                return torque, target, target_velocity

            self._trajectory_event(Event.TRAJECTORY_COMPLETE)
            # Continue with hold control this cycle, without integrating trajectory time.
            dt = 0.0

        target = _vector(self._target, MOTOR_COUNT, "hold target")
        error = target - position
        if self.mode == ControlMode.HOLD_SETTLING:
            settled = (np.all(np.abs(error) <= hold.sync_position_tolerance)
                       and np.all(np.abs(velocity) <= hold.sync_velocity_tolerance))
            if settled:
                if self._settle_since is None:
                    self._settle_since = self._now
                elif self._now - self._settle_since >= hold.settle_duration:
                    self._trajectory_event(Event.SYNC_COMPLETE)
            else:
                self._settle_since = None

        # Integrate only near the target; reset each axis outside its threshold.
        self._hold_integral = np.where(
            np.abs(error) <= hold.i_thresh,
            np.clip(
                self._hold_integral + error * dt,
                -np.asarray(hold.i_max),
                hold.i_max,
            ),
            0.0,
        )
        torque = (
            np.asarray(hold.kp) * error
            - np.asarray(hold.kd) * velocity
            + np.asarray(hold.ki) * self._hold_integral
        )
        return torque, target, np.zeros(MOTOR_COUNT)

    def _trajectory_event(self, event: Event) -> None:
        result = self._machine.handle(event, now=self._now)
        self._apply_actions(result)
        self._last_transition = result

    def _apply_actions(
        self, result: TransitionResult, follower_position: object = None,
        gripper_setpoint: float | None = None,
    ) -> None:
        for action in result.actions:
            if action == Action.STOP_MOTION:
                self._trajectory = None
            elif action == Action.CLEAR_SYNC:
                self._clear_sync()
            elif action == Action.START_SYNC:
                if not self.hardware.torque_enabled:
                    raise RuntimeError("synchronization requires enabled torque")
                if self._trajectory_factory is None:
                    raise RuntimeError("synchronization requires a trajectory_factory with explicit limits")
                arm = _vector(follower_position, ARM_JOINT_COUNT, "follower_position")
                grip = _scalar(
                    self.config.controller.sync_hold_control.gripper_setpoint
                    if gripper_setpoint is None else gripper_setpoint, "gripper_setpoint",
                )
                self._target = np.append(arm, grip)
                state = self._read_state()
                self._trajectory = self._trajectory_factory()
                self._trajectory.start(
                    np.append(state.arm_position, state.gripper_position),
                    np.append(state.arm_velocity, state.gripper_velocity),
                    self._target.copy(),
                )
                self._last_step_at = self._now
            elif action == Action.START_HOLD:
                if self._target is None:
                    raise RuntimeError("hold requires a synchronization target")
                self._hold_integral.fill(0.0)
                self._settle_since = None
            elif action == Action.START_TELEOP:
                if not self.hardware.torque_enabled:
                    raise RuntimeError("teleoperation requires enabled torque")
                # Publication belongs to the future ROS adapter; the preceding
                # actions have already removed leader automatic motion.

    def _clear_sync(self) -> None:
        self._trajectory = None
        self._target = None
        self._hold_integral.fill(0.0)
        self._settle_since = None

    def _zero_and_disable(self) -> None:
        if self.hardware.torque_enabled:
            try:
                self.hardware.write_currents(np.zeros(MOTOR_COUNT))
            finally:
                self.hardware.set_torque_enabled(False)
