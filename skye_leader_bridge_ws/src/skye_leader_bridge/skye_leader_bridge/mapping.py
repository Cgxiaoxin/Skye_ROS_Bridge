from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Iterable, Sequence


DEFAULT_JOINT_FIELDS = (
    "joint_pos",
    "joint_position",
    "joint_positions",
    "positions",
    "position",
    "pos",
    "data",
)


@dataclass(frozen=True)
class ArmMappingConfig:
    joint_order: Sequence[int] = (0, 1, 2, 3, 4, 5, 6)
    signs: Sequence[float] | None = None
    offsets: Sequence[float] | None = None
    limits_min: Sequence[float] | None = None
    limits_max: Sequence[float] | None = None


@dataclass(frozen=True)
class GripperMappingConfig:
    input_min: float = 0.0
    input_max: float = 1.0
    output_min: float = 0.0
    output_max: float = 1.0
    invert: bool = False
    deadband: float = 0.0


@dataclass(frozen=True)
class BridgeSafetyConfig:
    leader_timeout_s: float = 0.2
    max_delta_per_cycle: float = 0.05
    require_enable: bool = True


@dataclass
class _ArmRuntimeState:
    latest_time: float | None = None
    last_published: list[float] | None = None


@dataclass
class LeaderBridgeState:
    safety: BridgeSafetyConfig = field(default_factory=BridgeSafetyConfig)
    enabled: bool = False
    arms: dict[str, _ArmRuntimeState] = field(default_factory=dict)

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = bool(enabled)

    def update_leader(self, arm_name: str, positions: Sequence[float], now: float | None = None) -> None:
        del positions
        state = self.arms.setdefault(arm_name, _ArmRuntimeState())
        state.latest_time = time.monotonic() if now is None else float(now)

    def command_for_arm(
        self,
        arm_name: str,
        target: Sequence[float],
        now: float | None = None,
    ) -> list[float] | None:
        current_time = time.monotonic() if now is None else float(now)
        state = self.arms.setdefault(arm_name, _ArmRuntimeState())

        if self.safety.require_enable and not self.enabled:
            return None
        if state.latest_time is None:
            return None
        if current_time - state.latest_time > self.safety.leader_timeout_s:
            return None

        target_values = _as_float_list(target)
        if state.last_published is None:
            state.last_published = target_values
            return list(target_values)

        max_delta = float(self.safety.max_delta_per_cycle)
        if max_delta <= 0.0:
            state.last_published = target_values
            return list(target_values)

        limited = []
        for previous, desired in zip(state.last_published, target_values):
            delta = desired - previous
            if delta > max_delta:
                limited.append(previous + max_delta)
            elif delta < -max_delta:
                limited.append(previous - max_delta)
            else:
                limited.append(desired)
        state.last_published = limited
        return list(limited)


def map_arm_positions(leader_positions: Sequence[float], cfg: ArmMappingConfig) -> list[float]:
    order = [int(idx) for idx in cfg.joint_order]
    leader = _as_float_list(leader_positions)
    if leader and max(order, default=-1) >= len(leader):
        raise ValueError(f"leader_positions must contain at least {max(order) + 1} values")

    signs = _expand_or_validate(cfg.signs, len(order), "signs", default=1.0)
    offsets = _expand_or_validate(cfg.offsets, len(order), "offsets", default=0.0)
    mins = _optional_values(cfg.limits_min, len(order), "limits_min")
    maxs = _optional_values(cfg.limits_max, len(order), "limits_max")

    mapped = []
    for out_idx, src_idx in enumerate(order):
        value = leader[src_idx] * signs[out_idx] + offsets[out_idx]
        if mins is not None:
            value = max(mins[out_idx], value)
        if maxs is not None:
            value = min(maxs[out_idx], value)
        mapped.append(float(value))
    return mapped


def map_gripper_value(value: float, cfg: GripperMappingConfig) -> float:
    input_min = float(cfg.input_min)
    input_max = float(cfg.input_max)
    if input_max <= input_min:
        raise ValueError("input_max must be greater than input_min")

    norm = (float(value) - input_min) / (input_max - input_min)
    norm = min(1.0, max(0.0, norm))
    deadband = max(0.0, float(cfg.deadband))
    if norm <= deadband:
        norm = 0.0
    elif norm >= 1.0 - deadband:
        norm = 1.0
    if cfg.invert:
        norm = 1.0 - norm

    return float(cfg.output_min + norm * (cfg.output_max - cfg.output_min))


def fill_joint_command_message(
    msg: object,
    positions: Sequence[float],
    field_candidates: Iterable[str] = DEFAULT_JOINT_FIELDS,
) -> object:
    values = _as_float_list(positions)
    for field_name in field_candidates:
        if hasattr(msg, field_name):
            setattr(msg, field_name, values)
            return msg
    if _fill_numbered_fields(msg, values):
        return msg
    raise AttributeError(
        "Could not find a joint position field on the Skye command message. "
        f"Tried: {', '.join(field_candidates)}"
    )


def _as_float_list(values: Sequence[float]) -> list[float]:
    return [float(value) for value in values]


def _fill_numbered_fields(msg: object, values: Sequence[float]) -> bool:
    prefixes = ("joint_", "joint", "j")
    for prefix in prefixes:
        names = [f"{prefix}{idx}" for idx in range(len(values))]
        if all(hasattr(msg, name) for name in names):
            for name, value in zip(names, values):
                setattr(msg, name, float(value))
            return True
    return False


def _expand_or_validate(
    values: Sequence[float] | None,
    expected_len: int,
    name: str,
    default: float,
) -> list[float]:
    if values is None:
        return [float(default)] * expected_len
    return _optional_values(values, expected_len, name) or []


def _optional_values(values: Sequence[float] | None, expected_len: int, name: str) -> list[float] | None:
    if values is None:
        return None
    if len(values) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} values")
    return [float(value) for value in values]

