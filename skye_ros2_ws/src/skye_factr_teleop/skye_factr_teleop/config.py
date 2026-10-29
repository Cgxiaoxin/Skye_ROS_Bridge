# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Immutable, validated configuration for Skye FACTR teleoperation."""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path, PurePosixPath
from typing import Any, Union

import yaml


ARM_JOINT_COUNT = 7
MOTOR_COUNT = 8
SUPPORTED_SERVO_TYPES = frozenset(
    {
        "XM430-W350", "XM430-W350-T", "XM430_W350", "XM430_W350_T",
        "XC330-T288", "XC330-T288-T", "XC330_T288", "XC330_T288_T",
    }
)


class ConfigError(ValueError):
    """The YAML document does not satisfy the teleoperation schema."""


@dataclass(frozen=True)
class DynamixelConfig:
    dynamixel_port: str
    servo_types: tuple[str, ...]
    baudrate: int
    current_limit_ma: tuple[float, ...] | None
    joint_offsets: tuple[float, ...] | None
    joint_signs: tuple[float, ...]


@dataclass(frozen=True)
class LeaderDynamicsConfig:
    leader_urdf_package: str
    leader_urdf: str


@dataclass(frozen=True)
class FollowerDynamicsConfig:
    follower_urdf_package: str
    follower_urdf: str
    follower_joint_offset: int
    follower_gravity_in_world: tuple[float, ...]


@dataclass(frozen=True)
class JointFeedbackConfig:
    enable: bool
    follower_gravity_comp_enable: bool
    dynamics: FollowerDynamicsConfig
    gain: tuple[float, ...]
    damping: tuple[float, ...]
    motor_scalar: tuple[float, ...]


@dataclass(frozen=True)
class JointGravityCompConfig:
    enable: bool
    gain: tuple[float, ...]
    dynamics: LeaderDynamicsConfig
    mode: str = "position_velocity"


@dataclass(frozen=True)
class NullSpaceConfig:
    enable: bool
    null_space_joint_target: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]


@dataclass(frozen=True)
class StaticFrictionConfig:
    enable: bool
    enable_speed: tuple[float, ...]
    gain: tuple[float, ...]


@dataclass(frozen=True)
class JointBarrierConfig:
    limit_max: tuple[float, ...]
    limit_min: tuple[float, ...]
    safety_margin: float
    kp: float
    kd: float


@dataclass(frozen=True)
class GripperBarrierConfig:
    limit_max: float
    limit_min: float
    gripper_setpoint: float
    kp: float
    kd: float


@dataclass(frozen=True)
class LimitBarrierControlConfig:
    enable: bool
    joint: JointBarrierConfig
    gripper: GripperBarrierConfig


@dataclass(frozen=True)
class GripperFeedbackConfig:
    enable: bool
    gain: float
    ema_beta: float
    effort_threshold: float
    damping: float
    ema_fast_alpha: float


@dataclass(frozen=True)
class ForceFeedbackControlConfig:
    joint: JointFeedbackConfig
    gripper: GripperFeedbackConfig


@dataclass(frozen=True)
class RuckigControlConfig:
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    gravity_comp_gain: tuple[float, ...]
    max_velocity: tuple[float, ...] | None
    max_acceleration: tuple[float, ...] | None
    max_jerk: tuple[float, ...] | None
    trajectory_timeout: float


@dataclass(frozen=True)
class HoldControlConfig:
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    ki: tuple[float, ...]
    i_max: tuple[float, ...]
    i_thresh: tuple[float, ...]
    gravity_comp_gain: tuple[float, ...]
    sync_position_tolerance: tuple[float, ...]
    sync_velocity_tolerance: tuple[float, ...]
    settle_duration: float
    settle_timeout: float


@dataclass(frozen=True)
class SyncHoldControlConfig:
    ruckig: RuckigControlConfig
    hold: HoldControlConfig


@dataclass(frozen=True)
class ControllerConfig:
    frequency: float
    force_feedback_control: ForceFeedbackControlConfig
    joint_gravity_comp: JointGravityCompConfig
    null_space_regulation: NullSpaceConfig
    static_friction_comp: StaticFrictionConfig
    limit_barrier_control: LimitBarrierControlConfig
    sync_hold_control: SyncHoldControlConfig


@dataclass(frozen=True)
class RuntimeConfig:
    state_timeout: float = 0.25
    future_tolerance: float = 0.05
    allow_unnamed_joint_states: bool = False


@dataclass(frozen=True)
class TeleopConfig:
    name: str
    dynamixel: DynamixelConfig
    controller: ControllerConfig
    runtime: RuntimeConfig

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "TeleopConfig":
        data = _map(value, "config")
        _only(data, {"name", "dynamixel", "controller", "runtime"}, "config")
        return cls(
            _str(_req(data, "name", "config"), "name"),
            _dynamixel(_req(data, "dynamixel", "config")),
            _controller(_req(data, "controller", "config")),
            _runtime(data.get("runtime", {})),
        )


def load_config(path: Union[str, Path], *, require_runtime: bool = False) -> TeleopConfig:
    """Load validated config; teleop startup also requires calibration and trajectory limits."""
    config_path = Path(path)
    try:
        with config_path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"failed to load {config_path}: {exc}") from exc
    if document is None:
        raise ConfigError(f"{config_path}: configuration is empty")
    try:
        config = TeleopConfig.from_mapping(document)
        if require_runtime:
            for name in ("joint_offsets", "current_limit_ma"):
                if getattr(config.dynamixel, name) is None:
                    raise ConfigError(f"dynamixel.{name} is required for teleop startup")
            ruckig = config.controller.sync_hold_control.ruckig
            for name in ("max_velocity", "max_acceleration", "max_jerk"):
                if getattr(ruckig, name) is None:
                    raise ConfigError(f"controller.sync_hold_control.ruckig.{name} is required for teleop startup")
        return config
    except ConfigError as exc:
        raise ConfigError(f"{config_path}: {exc}") from exc


def _dynamixel(value: Any) -> DynamixelConfig:
    path = "dynamixel"
    data = _map(value, path)
    _only(data, {"dynamixel_port", "baudrate", "servo_types", "joint_signs",
                 "joint_offsets", "current_limit_ma"}, path)
    baudrate = _int(_req(data, "baudrate", path), f"{path}.baudrate")
    if baudrate <= 0:
        raise ConfigError(f"{path}.baudrate must be positive")
    raw_types = _seq(_req(data, "servo_types", path), f"{path}.servo_types")
    if len(raw_types) != MOTOR_COUNT:
        raise ConfigError(f"{path}.servo_types must contain {MOTOR_COUNT} values")
    types = tuple(_str(item, f"{path}.servo_types[{i}]") for i, item in enumerate(raw_types))
    unsupported = sorted(set(types) - SUPPORTED_SERVO_TYPES)
    if unsupported:
        raise ConfigError(f"{path}.servo_types contains unsupported types: {unsupported}")
    signs = _vec(_req(data, "joint_signs", path), MOTOR_COUNT, f"{path}.joint_signs", False)
    if any(sign not in (-1.0, 1.0) for sign in signs):
        raise ConfigError(f"{path}.joint_signs values must be -1 or 1")
    current_limits = (
        _positive_vec(data["current_limit_ma"], MOTOR_COUNT, f"{path}.current_limit_ma", False)
        if "current_limit_ma" in data else None
    )
    if current_limits is not None:
        from .dynamixel_driver import get_servo_spec

        for index, (limit, model) in enumerate(zip(current_limits, types)):
            model_limit = get_servo_spec(model).current_limit_ma
            if limit > model_limit:
                raise ConfigError(
                    f"{path}.current_limit_ma[{index}] exceeds {model} current limit {model_limit} mA"
                )
    return DynamixelConfig(
        _str(_req(data, "dynamixel_port", path), f"{path}.dynamixel_port"),
        types,
        baudrate,
        current_limits,
        _vec(data["joint_offsets"], MOTOR_COUNT, f"{path}.joint_offsets", False)
        if "joint_offsets" in data else None,
        signs,
    )


def _controller(value: Any) -> ControllerConfig:
    path = "controller"
    data = _map(value, path)
    _only(data, {
        "frequency", "force_feedback_control", "joint_gravity_comp",
        "null_space_regulation",
        "static_friction_comp", "limit_barrier_control",
        "sync_hold_control",
    }, path)
    frequency = _positive(_req(data, "frequency", path), f"{path}.frequency")
    gravity = _joint_gravity_comp(_req(data, "joint_gravity_comp", path))
    return ControllerConfig(
        frequency,
        _force_feedback_control(_req(data, "force_feedback_control", path)),
        gravity,
        _null_space(_req(data, "null_space_regulation", path)),
        _static_friction(_req(data, "static_friction_comp", path), gravity.enable),
        _limit_barrier_control(_req(data, "limit_barrier_control", path)),
        _sync_hold_control(_req(data, "sync_hold_control", path)),
    )


def _runtime(value: Any) -> RuntimeConfig:
    path = "runtime"
    data = _map(value, path)
    _only(data, {"state_timeout", "future_tolerance", "allow_unnamed_joint_states"}, path)
    return RuntimeConfig(
        _positive(data.get("state_timeout", 0.25), f"{path}.state_timeout"),
        _positive(data.get("future_tolerance", 0.05), f"{path}.future_tolerance"),
        _bool(data.get("allow_unnamed_joint_states", False), f"{path}.allow_unnamed_joint_states"),
    )


def _force_feedback_control(value: Any) -> ForceFeedbackControlConfig:
    path = "controller.force_feedback_control"
    data = _map(value, path)
    _only(data, {"joint", "gripper"}, path)
    return ForceFeedbackControlConfig(
        _joint_feedback(_req(data, "joint", path)),
        _gripper_feedback(_req(data, "gripper", path)),
    )


def _joint_feedback(value: Any) -> JointFeedbackConfig:
    path = "controller.force_feedback_control.joint"
    data = _map(value, path)
    _only(data, {"enable", "follower_gravity_comp_enable", "dynamics", "gain", "damping", "motor_scalar"}, path)
    return JointFeedbackConfig(
        _bool(_req(data, "enable", path), f"{path}.enable"),
        _bool(_req(data, "follower_gravity_comp_enable", path), f"{path}.follower_gravity_comp_enable"),
        _follower_dynamics(_req(data, "dynamics", path)),
        _nonnegative_vec(_req(data, "gain", path), 7, f"{path}.gain"),
        _nonnegative_vec(_req(data, "damping", path), 7, f"{path}.damping"),
        _positive_vec(_req(data, "motor_scalar", path), 7, f"{path}.motor_scalar"),
    )


def _leader_dynamics(value: Any) -> LeaderDynamicsConfig:
    path = "controller.joint_gravity_comp.dynamics"
    data = _map(value, path)
    _only(data, {"leader_urdf_package", "leader_urdf"}, path)
    return LeaderDynamicsConfig(
        _package(_req(data, "leader_urdf_package", path), f"{path}.leader_urdf_package"),
        _resource(_req(data, "leader_urdf", path), f"{path}.leader_urdf"),
    )


def _follower_dynamics(value: Any) -> FollowerDynamicsConfig:
    path = "controller.force_feedback_control.joint.dynamics"
    data = _map(value, path)
    _only(data, {"follower_urdf_package", "follower_urdf", "follower_joint_offset", "follower_gravity_in_world"}, path)
    offset = _int(_req(data, "follower_joint_offset", path), f"{path}.follower_joint_offset")
    if offset not in (0, ARM_JOINT_COUNT):
        raise ConfigError(f"{path}.follower_joint_offset must be 0 or {ARM_JOINT_COUNT}")
    return FollowerDynamicsConfig(
        _package(_req(data, "follower_urdf_package", path), f"{path}.follower_urdf_package"),
        _resource(_req(data, "follower_urdf", path), f"{path}.follower_urdf"),
        offset,
        _vec(_req(data, "follower_gravity_in_world", path), 3, f"{path}.follower_gravity_in_world", False),
    )


def _joint_gravity_comp(value: Any) -> JointGravityCompConfig:
    path = "controller.joint_gravity_comp"
    data = _map(value, path)
    _only(data, {"enable", "gain", "mode", "dynamics"}, path)
    mode = _str(data.get("mode", "position_velocity"), f"{path}.mode")
    if mode not in ("position_velocity", "position_only"):
        raise ConfigError(f"{path}.mode must be position_velocity or position_only")
    return JointGravityCompConfig(
        _bool(_req(data, "enable", path), f"{path}.enable"),
        _nonnegative_vec(_req(data, "gain", path), 7, f"{path}.gain"),
        _leader_dynamics(_req(data, "dynamics", path)),
        mode,
    )


def _null_space(value: Any) -> NullSpaceConfig:
    path = "controller.null_space_regulation"
    data = _map(value, path)
    _only(data, {"enable", "null_space_joint_target", "kp", "kd"}, path)
    return NullSpaceConfig(
        _bool(_req(data, "enable", path), f"{path}.enable"),
        _vec(_req(data, "null_space_joint_target", path), 7, f"{path}.null_space_joint_target", False),
        _nonnegative_vec(_req(data, "kp", path), 7, f"{path}.kp"),
        _nonnegative_vec(_req(data, "kd", path), 7, f"{path}.kd"),
    )


def _static_friction(value: Any, legacy_enable: bool) -> StaticFrictionConfig:
    path = "controller.static_friction_comp"
    data = _map(value, path)
    _only(data, {"enable", "enable_speed", "gain"}, path)
    return StaticFrictionConfig(
        _bool(data.get("enable", legacy_enable), f"{path}.enable"),
        _nonnegative_vec(_req(data, "enable_speed", path), 7, f"{path}.enable_speed"),
        _nonnegative_vec(_req(data, "gain", path), 7, f"{path}.gain"),
    )


def _limit_barrier_control(value: Any) -> LimitBarrierControlConfig:
    path = "controller.limit_barrier_control"
    data = _map(value, path)
    _only(data, {"enable", "joint", "gripper"}, path)
    return LimitBarrierControlConfig(
        _bool(_req(data, "enable", path), f"{path}.enable"),
        _joint_barrier(_req(data, "joint", path)),
        _gripper_barrier(_req(data, "gripper", path)),
    )


def _joint_barrier(value: Any) -> JointBarrierConfig:
    path = "controller.limit_barrier_control.joint"
    data = _map(value, path)
    _only(data, {"limit_max", "limit_min", "safety_margin", "kp", "kd"}, path)
    lower = _vec(_req(data, "limit_min", path), ARM_JOINT_COUNT,
                 f"{path}.limit_min", False)
    upper = _vec(_req(data, "limit_max", path), ARM_JOINT_COUNT,
                 f"{path}.limit_max", False)
    margin = _nonnegative(_req(data, "safety_margin", path),
                          f"{path}.safety_margin")
    for index, (minimum, maximum) in enumerate(zip(lower, upper), 1):
        if minimum >= maximum:
            raise ConfigError(f"{path} joint {index} lower limit must be below upper limit")
        if minimum + margin >= maximum - margin:
            raise ConfigError(f"{path} joint {index} safety margin leaves no valid range")
    return JointBarrierConfig(
        upper, lower, margin,
        _nonnegative(_req(data, "kp", path), f"{path}.kp"),
        _nonnegative(_req(data, "kd", path), f"{path}.kd"),
    )


def _gripper_barrier(value: Any) -> GripperBarrierConfig:
    path = "controller.limit_barrier_control.gripper"
    data = _map(value, path)
    _only(data, {"limit_max", "limit_min", "gripper_setpoint", "kp", "kd"}, path)
    grip_max = _num(_req(data, "limit_max", path), f"{path}.limit_max")
    grip_min = _num(_req(data, "limit_min", path), f"{path}.limit_min")
    if grip_min >= grip_max:
        raise ConfigError(f"{path}.limit_min must be below limit_max")
    return GripperBarrierConfig(
        grip_max, grip_min,
        _num(_req(data, "gripper_setpoint", path), f"{path}.gripper_setpoint"),
        _nonnegative(_req(data, "kp", path), f"{path}.kp"),
        _nonnegative(_req(data, "kd", path), f"{path}.kd"),
    )


def _gripper_feedback(value: Any) -> GripperFeedbackConfig:
    path = "controller.force_feedback_control.gripper"
    data = _map(value, path)
    _only(data, {"enable", "gain", "ema_beta", "effort_threshold", "damping", "ema_fast_alpha"}, path)
    beta = _num(_req(data, "ema_beta", path), f"{path}.ema_beta")
    alpha = _num(_req(data, "ema_fast_alpha", path), f"{path}.ema_fast_alpha")
    if not 0 <= beta <= 1 or not 0 <= alpha <= 1:
        raise ConfigError(f"{path} EMA coefficients must be within [0, 1]")
    return GripperFeedbackConfig(
        _bool(_req(data, "enable", path), f"{path}.enable"),
        _nonnegative(_req(data, "gain", path), f"{path}.gain"), beta,
        _num(_req(data, "effort_threshold", path), f"{path}.effort_threshold"),
        _nonnegative(_req(data, "damping", path), f"{path}.damping"), alpha,
    )


def _sync_hold_control(value: Any) -> SyncHoldControlConfig:
    path = "controller.sync_hold_control"
    data = _map(value, path)
    _only(data, {"ruckig", "hold"}, path)
    return SyncHoldControlConfig(
        _ruckig_control(_req(data, "ruckig", path)),
        _hold_control(_req(data, "hold", path)),
    )


def _ruckig_control(value: Any) -> RuckigControlConfig:
    path = "controller.sync_hold_control.ruckig"
    data = _map(value, path)
    _only(data, {
        "kp", "kd", "gravity_comp_gain", "max_velocity", "max_acceleration",
        "max_jerk", "trajectory_timeout",
    }, path)
    limits = {
        name: _positive_vec(data[name], MOTOR_COUNT, f"{path}.{name}", False)
        if name in data else None
        for name in ("max_velocity", "max_acceleration", "max_jerk")
    }
    return RuckigControlConfig(
        _nonnegative_vec(_req(data, "kp", path), MOTOR_COUNT, f"{path}.kp", False),
        _nonnegative_vec(_req(data, "kd", path), MOTOR_COUNT, f"{path}.kd", False),
        _nonnegative_vec(_req(data, "gravity_comp_gain", path), ARM_JOINT_COUNT,
                         f"{path}.gravity_comp_gain", False),
        limits["max_velocity"], limits["max_acceleration"], limits["max_jerk"],
        _positive(_req(data, "trajectory_timeout", path), f"{path}.trajectory_timeout"),
    )


def _hold_control(value: Any) -> HoldControlConfig:
    path = "controller.sync_hold_control.hold"
    data = _map(value, path)
    _only(data, {
        "kp", "kd", "ki", "i_max", "i_thresh", "gravity_comp_gain",
        "sync_position_tolerance", "sync_velocity_tolerance",
        "settle_duration", "settle_timeout",
    }, path)
    gains = {
        name: _nonnegative_vec(_req(data, name, path), MOTOR_COUNT, f"{path}.{name}", False)
        for name in ("kp", "kd", "ki", "i_max", "i_thresh")
    }
    for name in ("i_max", "i_thresh"):
        if any(item <= 0 for item in gains[name]):
            raise ConfigError(f"{path}.{name} values must be positive")
    return HoldControlConfig(
        gains["kp"], gains["kd"], gains["ki"], gains["i_max"], gains["i_thresh"],
        _nonnegative_vec(_req(data, "gravity_comp_gain", path), ARM_JOINT_COUNT,
                         f"{path}.gravity_comp_gain", False),
        _positive_vec(_req(data, "sync_position_tolerance", path), MOTOR_COUNT,
                      f"{path}.sync_position_tolerance"),
        _positive_vec(_req(data, "sync_velocity_tolerance", path), MOTOR_COUNT,
                      f"{path}.sync_velocity_tolerance"),
        _positive(_req(data, "settle_duration", path), f"{path}.settle_duration"),
        _positive(_req(data, "settle_timeout", path), f"{path}.settle_timeout"),
    )


def _map(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigError(f"{path} must be a mapping")
    return value


def _req(data: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in data:
        raise ConfigError(f"{path}.{key} is required")
    return data[key]


def _only(data: Mapping[str, Any], expected: set[str], path: str) -> None:
    unknown = sorted(set(data) - expected)
    if unknown:
        raise ConfigError(f"{path} contains unknown fields: {unknown}")


def _seq(value: Any, path: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ConfigError(f"{path} must be an array")
    return value


def _str(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"{path} must be a non-empty string")
    return value.strip()


def _bool(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f"{path} must be a boolean")
    return value


def _num(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ConfigError(f"{path} must be finite")
    return result


def _int(value: Any, path: str) -> int:
    number = _num(value, path)
    if not number.is_integer():
        raise ConfigError(f"{path} must be an integer")
    return int(number)


def _positive(value: Any, path: str) -> float:
    number = _num(value, path)
    if number <= 0:
        raise ConfigError(f"{path} must be positive")
    return number


def _nonnegative(value: Any, path: str) -> float:
    number = _num(value, path)
    if number < 0:
        raise ConfigError(f"{path} must be non-negative")
    return number


def _vec(value: Any, size: int, path: str, scalar: bool = True) -> tuple[float, ...]:
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        if len(value) != size:
            raise ConfigError(f"{path} must contain {size} values")
        return tuple(_num(item, f"{path}[{i}]") for i, item in enumerate(value))
    if not scalar:
        raise ConfigError(f"{path} must contain {size} values")
    return (_num(value, path),) * size


def _nonnegative_vec(value: Any, size: int, path: str, scalar: bool = True) -> tuple[float, ...]:
    result = _vec(value, size, path, scalar)
    if any(item < 0 for item in result):
        raise ConfigError(f"{path} values must be non-negative")
    return result


def _positive_vec(value: Any, size: int, path: str, scalar: bool = True) -> tuple[float, ...]:
    result = _vec(value, size, path, scalar)
    if any(item <= 0 for item in result):
        raise ConfigError(f"{path} values must be positive")
    return result


def _package(value: Any, path: str) -> str:
    name = _str(value, path)
    if "/" in name or "\\" in name:
        raise ConfigError(f"{path} must be a ROS package name")
    return name


def _resource(value: Any, path: str) -> str:
    resource = _str(value, path).replace("\\", "/")
    pure = PurePosixPath(resource)
    if pure.is_absolute() or ".." in pure.parts:
        raise ConfigError(f"{path} must be a package-relative resource path")
    return resource
