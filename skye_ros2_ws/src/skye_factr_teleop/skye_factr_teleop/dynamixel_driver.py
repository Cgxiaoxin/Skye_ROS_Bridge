# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Dynamixel control-table constants and unit conversions.

The serial/SDK lifecycle is implemented in the next development-plan task.
The helpers in this module are pure and do not access Dynamixel hardware.
"""

from dataclasses import dataclass
import math
from numbers import Integral, Real
from types import MappingProxyType


PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
LEN_GOAL_CURRENT = 2
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
LEN_SYNC_STATE = LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION

TORQUE_DISABLE = 0
TORQUE_ENABLE = 1
CURRENT_CONTROL_MODE = 0

POSITION_TICKS_PER_REVOLUTION = 4096
VELOCITY_RPM_PER_UNIT = 0.229


@dataclass(frozen=True)
class ServoSpec:
    """Current-register and reference torque data for one servo model.

    ``torque_nm_per_amp`` is the official 12 V stall-point ratio. It is a
    linear conversion reference for the controller, not a continuous-output
    rating; it must still be validated on the M6 hardware in phase 2.5.
    """

    current_unit_ma: float
    current_limit_raw: int
    torque_nm_per_amp: float

    @property
    def current_limit_ma(self) -> float:
        return self.current_unit_ma * self.current_limit_raw


# Sources (ROBOTIS e-Manual, accessed 2026-09-17):
# https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
#   Current/Goal Current unit: 2.69 mA; Current Limit range: 0..1193.
#   12 V stall point: 4.1 Nm at 2.3 A, published as 1.783 Nm/A.
# https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/
#   Current/Goal Current unit: 1.0 mA; Current Limit range: 0..910.
#   12 V stall point: 1.00 Nm at 0.88 A, published as 1.136 Nm/A.
# These official values replace coefficients inferred only from the old binary
# module. The e-Manual also warns that stall torque is not continuous output.
SERVO_SPECS = MappingProxyType(
    {
        "XM430-W350": ServoSpec(
            current_unit_ma=2.69,
            current_limit_raw=1193,
            torque_nm_per_amp=1.783,
        ),
        "XC330-T288": ServoSpec(
            current_unit_ma=1.0,
            current_limit_raw=910,
            torque_nm_per_amp=1.136,
        ),
    }
)

SERVO_TYPE_ALIASES = MappingProxyType(
    {
        "XM430-W350": "XM430-W350",
        "XM430-W350-T": "XM430-W350",
        "XM430_W350": "XM430-W350",
        "XM430_W350_T": "XM430-W350",
        "XC330-T288": "XC330-T288",
        "XC330-T288-T": "XC330-T288",
        "XC330_T288": "XC330-T288",
        "XC330_T288_T": "XC330-T288",
    }
)

# Named read-only views let configuration and driver code share model data.
CURRENT_LIMIT_RAW = MappingProxyType(
    {model: spec.current_limit_raw for model, spec in SERVO_SPECS.items()}
)
CURRENT_LIMIT_MA = MappingProxyType(
    {model: spec.current_limit_ma for model, spec in SERVO_SPECS.items()}
)
TORQUE_TO_CURRENT_MA_PER_NM = MappingProxyType(
    {
        model: 1000.0 / spec.torque_nm_per_amp
        for model, spec in SERVO_SPECS.items()
    }
)


def uint32_to_int32(value: int) -> int:
    """Interpret an unsigned 32-bit register value as two's-complement."""
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise TypeError("32-bit register value must be an integer")
    value = int(value)
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("32-bit register value must be in 0..0xFFFFFFFF")
    return value - 0x100000000 if value & 0x80000000 else value


def encode_goal_current(value: int) -> list[int]:
    """Encode a signed raw Goal Current value as two little-endian bytes."""
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise TypeError("Goal Current must be an integer")
    value = int(value)
    if not -0x8000 <= value <= 0x7FFF:
        raise ValueError("Goal Current must fit in a signed 16-bit register")
    unsigned = value & 0xFFFF
    return [unsigned & 0xFF, (unsigned >> 8) & 0xFF]


def position_ticks_to_radians(ticks: Real) -> float:
    """Convert signed position ticks to radians (4096 ticks per revolution)."""
    ticks = _finite_number(ticks, "position ticks")
    return ticks * (2.0 * math.pi / POSITION_TICKS_PER_REVOLUTION)


def velocity_units_to_rad_s(units: Real) -> float:
    """Convert signed 0.229-rpm velocity units to radians per second."""
    units = _finite_number(units, "velocity units")
    return units * VELOCITY_RPM_PER_UNIT * (2.0 * math.pi / 60.0)


def normalize_servo_type(servo_type: str) -> str:
    """Return the canonical model name for a supported configuration alias."""
    if not isinstance(servo_type, str):
        raise TypeError("servo type must be a string")
    try:
        return SERVO_TYPE_ALIASES[servo_type.strip().upper()]
    except KeyError as exc:
        raise ValueError(f"unsupported servo type: {servo_type!r}") from exc


def get_servo_spec(servo_type: str) -> ServoSpec:
    """Return immutable conversion data for *servo_type*."""
    return SERVO_SPECS[normalize_servo_type(servo_type)]


def torque_nm_to_current_ma(torque_nm: Real, servo_type: str) -> float:
    """Convert output torque in Nm to the model's reference current in mA.

    This preserves sign and deliberately does not clamp. The hardware write
    path must reject commands outside ``ServoSpec.current_limit_ma``.
    """
    torque_nm = _finite_number(torque_nm, "torque")
    spec = get_servo_spec(servo_type)
    return torque_nm * 1000.0 / spec.torque_nm_per_amp


def current_ma_to_raw(current_ma: Real, servo_type: str) -> int:
    """Convert mA to a signed Goal Current unit, rejecting model over-current."""
    current_ma = _finite_number(current_ma, "current")
    spec = get_servo_spec(servo_type)
    if abs(current_ma) > spec.current_limit_ma:
        raise ValueError(
            f"current {current_ma} mA exceeds {normalize_servo_type(servo_type)} "
            f"current limit {spec.current_limit_ma} mA"
        )
    raw = int(round(current_ma / spec.current_unit_ma))
    if abs(raw) > spec.current_limit_raw:
        raise ValueError(
            f"rounded Goal Current {raw} exceeds current limit "
            f"{spec.current_limit_raw}"
        )
    return raw


def _finite_number(value: Real, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result
