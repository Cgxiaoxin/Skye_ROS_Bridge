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
    joint_signs: tuple[float, ...]


@dataclass(frozen=True)
class InitializationConfig:
    calibration_joint_pos: tuple[float, ...]
    initial_match_joint_pos: tuple[float, ...]


@dataclass(frozen=True)
class ArmTeleopConfig:
    leader_urdf_package: str
    leader_urdf: str
    leader_name: str
    baudrate: int
    num_arm_joints: int
    use_suction_cup: bool
    arm_limit_barrier_enable: bool
    arm_joint_limits_max: tuple[float, ...]
    arm_joint_limits_min: tuple[float, ...]
    arm_joint_limits_safety_margin: float
    initialization: InitializationConfig
    enable_follower_gravity_comp: bool
    follower_urdf_package: str
    follower_urdf: str
    follower_joint_offset: int
    follower_gravity_in_world: tuple[float, ...]


@dataclass(frozen=True)
class GripperTeleopConfig:
    limit_min: float
    limit_setpoint: float
    limit_max: float
    kp: float
    kd: float


@dataclass(frozen=True)
class TorqueFeedbackConfig:
    enable: bool
    gain: tuple[float, ...]
    damping: tuple[float, ...]
    motor_scalar: tuple[float, ...]


@dataclass(frozen=True)
class GravityCompConfig:
    enable: bool
    gain: tuple[float, ...]


@dataclass(frozen=True)
class NullSpaceConfig:
    enable: bool
    null_space_joint_target: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]


@dataclass(frozen=True)
class StaticFrictionConfig:
    enable_speed: tuple[float, ...]
    gain: tuple[float, ...]


@dataclass(frozen=True)
class PdConfig:
    kp: float
    kd: float


@dataclass(frozen=True)
class JointPositionConfig:
    interpolation_step_size: float
    kp: float
    kd: float


@dataclass(frozen=True)
class GripperFeedbackConfig:
    enable: bool
    gain: float
    ema_beta: float
    effort_threshold: float
    damping: float
    ema_fast_alpha: float


@dataclass(frozen=True)
class HoldRuckigConfig:
    ruckig_kp: tuple[float, ...]
    ruckig_kd: tuple[float, ...]
    hold_kp: tuple[float, ...]
    hold_kd: tuple[float, ...]
    hold_ki: tuple[float, ...]
    hold_i_max: tuple[float, ...]
    hold_i_thresh: tuple[float, ...]
    ruckig_gravity_comp_gain: tuple[float, ...]
    hold_gravity_comp_gain: tuple[float, ...]


@dataclass(frozen=True)
class ControllerConfig:
    frequency: float
    teleop_sync_timeout: float
    torque_feedback: TorqueFeedbackConfig
    gravity_comp: GravityCompConfig
    null_space_regulation: NullSpaceConfig
    static_friction_comp: StaticFrictionConfig
    joint_limit_barrier: PdConfig
    joint_position_control: JointPositionConfig
    gripper_feedback: GripperFeedbackConfig
    hold_ruckig_control: HoldRuckigConfig


@dataclass(frozen=True)
class TeleopConfig:
    name: str
    dynamixel: DynamixelConfig
    arm_teleop: ArmTeleopConfig
    gripper_teleop: GripperTeleopConfig
    controller: ControllerConfig

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "TeleopConfig":
        data = _map(value, "config")
        _only(data, {"name", "dynamixel", "arm_teleop", "gripper_teleop", "controller"}, "config")
        return cls(
            _str(_req(data, "name", "config"), "name"),
            _dynamixel(_req(data, "dynamixel", "config")),
            _arm(_req(data, "arm_teleop", "config")),
            _gripper(_req(data, "gripper_teleop", "config")),
            _controller(_req(data, "controller", "config")),
        )


def load_config(path: Union[str, Path]) -> TeleopConfig:
    """Safely load *path*, validate it, and return frozen dataclasses."""
    config_path = Path(path)
    try:
        with config_path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"failed to load {config_path}: {exc}") from exc
    if document is None:
        raise ConfigError(f"{config_path}: configuration is empty")
    try:
        return TeleopConfig.from_mapping(document)
    except ConfigError as exc:
        raise ConfigError(f"{config_path}: {exc}") from exc


def _dynamixel(value: Any) -> DynamixelConfig:
    path = "dynamixel"
    data = _map(value, path)
    _only(data, {"dynamixel_port", "servo_types", "joint_signs"}, path)
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
    return DynamixelConfig(
        _str(_req(data, "dynamixel_port", path), f"{path}.dynamixel_port"),
        types,
        signs,
    )


def _arm(value: Any) -> ArmTeleopConfig:
    path = "arm_teleop"
    data = _map(value, path)
    _only(data, {
        "leader_urdf_package", "leader_urdf", "leader_name", "baudrate",
        "num_arm_joints", "use_suction_cup", "arm_limit_barrier_enable",
        "arm_joint_limits_max", "arm_joint_limits_min",
        "arm_joint_limits_safety_margin", "initialization",
        "enable_follower_gravity_comp", "follower_urdf_package",
        "follower_urdf", "follower_joint_offset", "follower_gravity_in_world",
    }, path)
    count = _int(_req(data, "num_arm_joints", path), f"{path}.num_arm_joints")
    if count != ARM_JOINT_COUNT:
        raise ConfigError(f"{path}.num_arm_joints must be {ARM_JOINT_COUNT}")
    lower = _vec(_req(data, "arm_joint_limits_min", path), count, f"{path}.arm_joint_limits_min", False)
    upper = _vec(_req(data, "arm_joint_limits_max", path), count, f"{path}.arm_joint_limits_max", False)
    margin = _num(_req(data, "arm_joint_limits_safety_margin", path), f"{path}.arm_joint_limits_safety_margin")
    if margin < 0:
        raise ConfigError(f"{path}.arm_joint_limits_safety_margin must be non-negative")
    for index, (minimum, maximum) in enumerate(zip(lower, upper), 1):
        if minimum >= maximum:
            raise ConfigError(f"{path} joint {index} lower limit must be below upper limit")
        if minimum + margin >= maximum - margin:
            raise ConfigError(f"{path} joint {index} safety margin leaves no valid range")
    init_path = f"{path}.initialization"
    init = _map(_req(data, "initialization", path), init_path)
    _only(init, {"calibration_joint_pos", "initial_match_joint_pos"}, init_path)
    initialization = InitializationConfig(
        _vec(_req(init, "calibration_joint_pos", init_path), count, f"{init_path}.calibration_joint_pos", False),
        _vec(_req(init, "initial_match_joint_pos", init_path), count, f"{init_path}.initial_match_joint_pos", False),
    )
    baudrate = _int(_req(data, "baudrate", path), f"{path}.baudrate")
    if baudrate <= 0:
        raise ConfigError(f"{path}.baudrate must be positive")
    offset = _int(_req(data, "follower_joint_offset", path), f"{path}.follower_joint_offset")
    if offset not in (0, ARM_JOINT_COUNT):
        raise ConfigError(f"{path}.follower_joint_offset must be 0 or {ARM_JOINT_COUNT}")
    return ArmTeleopConfig(
        _package(_req(data, "leader_urdf_package", path), f"{path}.leader_urdf_package"),
        _resource(_req(data, "leader_urdf", path), f"{path}.leader_urdf"),
        _str(_req(data, "leader_name", path), f"{path}.leader_name"),
        baudrate, count,
        _bool(_req(data, "use_suction_cup", path), f"{path}.use_suction_cup"),
        _bool(_req(data, "arm_limit_barrier_enable", path), f"{path}.arm_limit_barrier_enable"),
        upper, lower, margin, initialization,
        _bool(_req(data, "enable_follower_gravity_comp", path), f"{path}.enable_follower_gravity_comp"),
        _package(_req(data, "follower_urdf_package", path), f"{path}.follower_urdf_package"),
        _resource(_req(data, "follower_urdf", path), f"{path}.follower_urdf"),
        offset,
        _vec(_req(data, "follower_gravity_in_world", path), 3, f"{path}.follower_gravity_in_world", False),
    )


def _gripper(value: Any) -> GripperTeleopConfig:
    path = "gripper_teleop"
    data = _map(value, path)
    _only(data, {"limit_min", "limit_setpoint", "limit_max", "kp", "kd"}, path)
    minimum = _num(_req(data, "limit_min", path), f"{path}.limit_min")
    setpoint = _num(_req(data, "limit_setpoint", path), f"{path}.limit_setpoint")
    maximum = _num(_req(data, "limit_max", path), f"{path}.limit_max")
    if minimum >= maximum:
        raise ConfigError(f"{path}.limit_min must be below limit_max")
    return GripperTeleopConfig(minimum, setpoint, maximum,
        _nonnegative(_req(data, "kp", path), f"{path}.kp"),
        _nonnegative(_req(data, "kd", path), f"{path}.kd"))


def _controller(value: Any) -> ControllerConfig:
    path = "controller"
    data = _map(value, path)
    _only(data, {
        "frequency", "teleop_sync_timeout", "torque_feedback", "gravity_comp",
        "null_space_regulation", "static_friction_comp", "joint_limit_barrier",
        "joint_position_control", "gripper_feedback", "hold_ruckig_control",
    }, path)
    frequency = _positive(_req(data, "frequency", path), f"{path}.frequency")
    timeout = _positive(_req(data, "teleop_sync_timeout", path), f"{path}.teleop_sync_timeout")
    return ControllerConfig(
        frequency, timeout,
        _torque_feedback(_req(data, "torque_feedback", path)),
        _gravity_comp(_req(data, "gravity_comp", path)),
        _null_space(_req(data, "null_space_regulation", path)),
        _static_friction(_req(data, "static_friction_comp", path)),
        _pd(_req(data, "joint_limit_barrier", path), f"{path}.joint_limit_barrier"),
        _joint_position(_req(data, "joint_position_control", path)),
        _gripper_feedback(_req(data, "gripper_feedback", path)),
        _hold_ruckig(_req(data, "hold_ruckig_control", path)),
    )


def _torque_feedback(value: Any) -> TorqueFeedbackConfig:
    path = "controller.torque_feedback"
    data = _map(value, path)
    _only(data, {"enable", "gain", "damping", "motor_scalar"}, path)
    return TorqueFeedbackConfig(
        _bool(_req(data, "enable", path), f"{path}.enable"),
        _nonnegative_vec(_req(data, "gain", path), 7, f"{path}.gain"),
        _nonnegative_vec(_req(data, "damping", path), 7, f"{path}.damping"),
        _positive_vec(_req(data, "motor_scalar", path), 7, f"{path}.motor_scalar"),
    )


def _gravity_comp(value: Any) -> GravityCompConfig:
    path = "controller.gravity_comp"
    data = _map(value, path)
    _only(data, {"enable", "gain"}, path)
    return GravityCompConfig(
        _bool(_req(data, "enable", path), f"{path}.enable"),
        _nonnegative_vec(_req(data, "gain", path), 7, f"{path}.gain"),
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


def _static_friction(value: Any) -> StaticFrictionConfig:
    path = "controller.static_friction_comp"
    data = _map(value, path)
    _only(data, {"enable_speed", "gain"}, path)
    return StaticFrictionConfig(
        _nonnegative_vec(_req(data, "enable_speed", path), 7, f"{path}.enable_speed"),
        _nonnegative_vec(_req(data, "gain", path), 7, f"{path}.gain"),
    )


def _pd(value: Any, path: str) -> PdConfig:
    data = _map(value, path)
    _only(data, {"kp", "kd"}, path)
    return PdConfig(_nonnegative(_req(data, "kp", path), f"{path}.kp"),
                    _nonnegative(_req(data, "kd", path), f"{path}.kd"))


def _joint_position(value: Any) -> JointPositionConfig:
    path = "controller.joint_position_control"
    data = _map(value, path)
    _only(data, {"interpolation_step_size", "kp", "kd"}, path)
    return JointPositionConfig(
        _positive(_req(data, "interpolation_step_size", path), f"{path}.interpolation_step_size"),
        _nonnegative(_req(data, "kp", path), f"{path}.kp"),
        _nonnegative(_req(data, "kd", path), f"{path}.kd"),
    )


def _gripper_feedback(value: Any) -> GripperFeedbackConfig:
    path = "controller.gripper_feedback"
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


def _hold_ruckig(value: Any) -> HoldRuckigConfig:
    path = "controller.hold_ruckig_control"
    data = _map(value, path)
    eight = {"ruckig_kp", "ruckig_kd", "hold_kp", "hold_kd", "hold_ki", "hold_i_max", "hold_i_thresh"}
    seven = {"ruckig_gravity_comp_gain", "hold_gravity_comp_gain"}
    _only(data, eight | seven, path)
    values = {key: _nonnegative_vec(_req(data, key, path), 8, f"{path}.{key}", False) for key in eight}
    for key in ("hold_i_max", "hold_i_thresh"):
        if any(item <= 0 for item in values[key]):
            raise ConfigError(f"{path}.{key} values must be positive")
    return HoldRuckigConfig(
        values["ruckig_kp"], values["ruckig_kd"], values["hold_kp"],
        values["hold_kd"], values["hold_ki"], values["hold_i_max"],
        values["hold_i_thresh"],
        _nonnegative_vec(_req(data, "ruckig_gravity_comp_gain", path), 7, f"{path}.ruckig_gravity_comp_gain", False),
        _nonnegative_vec(_req(data, "hold_gravity_comp_gain", path), 7, f"{path}.hold_gravity_comp_gain", False),
    )


def _req(data: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in data:
        raise ConfigError(f"{path}.{key} is required")
    return data[key]


def _only(data: Mapping[str, Any], expected: set[str], path: str) -> None:
    unknown = sorted(set(data) - expected)
    if unknown:
        raise ConfigError(f"{path} contains unknown fields: {unknown}")


def _map(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigError(f"{path} must be a mapping")
    return value


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


def _positive_vec(value: Any, size: int, path: str) -> tuple[float, ...]:
    result = _vec(value, size, path)
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
