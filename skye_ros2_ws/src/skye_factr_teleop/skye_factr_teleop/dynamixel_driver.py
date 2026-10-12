# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Protocol 2.0 hardware adapter for the M6 leader Dynamixel bus."""

from dataclasses import dataclass
import math
from numbers import Integral, Real
import os
from pathlib import Path
from threading import RLock
from types import MappingProxyType
from typing import Protocol, Sequence, runtime_checkable

import numpy as np
from dynamixel_sdk import (
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)


PROTOCOL_VERSION = 2.0

ADDR_RETURN_DELAY_TIME = 9
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
LEN_GOAL_CURRENT = 2
ADDR_PRESENT_CURRENT = 126
LEN_PRESENT_CURRENT = 2
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
LEN_SYNC_STATE = LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION
LEN_SYNC_STATE_WITH_CURRENT = LEN_PRESENT_CURRENT + LEN_SYNC_STATE

TORQUE_DISABLE = 0
TORQUE_ENABLE = 1
CURRENT_CONTROL_MODE = 0

POSITION_TICKS_PER_REVOLUTION = 4096
VELOCITY_RPM_PER_UNIT = 0.229
DEFAULT_BAUDRATE = 4_000_000
DEFAULT_SYNC_READ_ATTEMPTS = 3
SERIAL_BY_ID_DIRECTORY = Path("/dev/serial/by-id")
SYS_TTY_DIRECTORY = Path("/sys/class/tty")


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


@dataclass(frozen=True)
class DynamixelDeviceInfo:
    """Read-only identity and safety state captured while opening a bus."""

    device_id: int
    model_number: int
    return_delay_time: int
    torque_enabled: bool


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

# Official ROBOTIS control-table model numbers.  They are checked by the
# phase-2.5 read-only tool so an ID wired to the wrong joint is visible before
# any actuator register is written.
MODEL_NUMBERS = MappingProxyType(
    {
        "XM430-W350": 1020,
        "XC330-T288": 1220,
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


def uint16_to_int16(value: int) -> int:
    """Interpret an unsigned 16-bit register value as two's-complement."""
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise TypeError("16-bit register value must be an integer")
    value = int(value)
    if not 0 <= value <= 0xFFFF:
        raise ValueError("16-bit register value must be in 0..0xFFFF")
    return value - 0x10000 if value & 0x8000 else value


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


class DynamixelError(RuntimeError):
    """Base error raised by the Dynamixel hardware adapter."""


class DynamixelPortError(DynamixelError):
    """The configured serial port cannot be used safely."""


class DynamixelSdkError(DynamixelError):
    """A Dynamixel SDK transaction or device-status check failed."""


@runtime_checkable
class DynamixelIO(Protocol):
    """Hardware interface consumed by the teleoperation control core."""

    @property
    def torque_enabled(self) -> bool:
        ...

    def read_state(self) -> tuple[np.ndarray, np.ndarray]:
        ...

    def write_currents(self, currents_ma: Sequence[float]) -> None:
        ...

    def set_torque_enabled(self, enabled: bool) -> None:
        ...

    def close(self) -> None:
        ...


def resolve_serial_port(port: str) -> Path:
    """Expand a by-id basename and validate that the device is accessible."""
    if not isinstance(port, str) or not port.strip():
        raise DynamixelPortError("serial port must be a non-empty path")
    candidate = Path(port.strip())
    if not candidate.is_absolute() and candidate.parent == Path("."):
        candidate = SERIAL_BY_ID_DIRECTORY / candidate
    if not candidate.exists():
        raise DynamixelPortError(f"serial port does not exist: {candidate}")
    if candidate.is_dir():
        raise DynamixelPortError(f"serial port is a directory: {candidate}")
    if not os.access(candidate, os.R_OK | os.W_OK):
        raise DynamixelPortError(f"serial port is not readable and writable: {candidate}")
    return candidate


def read_ftdi_latency_timer(port: Path) -> int | None:
    """Read the Linux FTDI latency timer, or return ``None`` for non-FTDI ports."""
    try:
        tty_name = port.resolve(strict=True).name
    except OSError as exc:
        raise DynamixelPortError(f"failed to resolve serial port {port}: {exc}") from exc
    latency_path = SYS_TTY_DIRECTORY / tty_name / "device" / "latency_timer"
    if not latency_path.exists():
        return None
    try:
        return int(latency_path.read_text(encoding="ascii").strip())
    except (OSError, ValueError) as exc:
        raise DynamixelPortError(
            f"failed to read FTDI latency timer {latency_path}: {exc}"
        ) from exc


def _serial_port_candidate(port: str) -> Path:
    if not isinstance(port, str) or not port.strip():
        raise DynamixelPortError("serial port must be a non-empty path")
    candidate = Path(port.strip())
    if not candidate.is_absolute() and candidate.parent == Path("."):
        candidate = SERIAL_BY_ID_DIRECTORY / candidate
    return candidate


def _validate_ids(ids: Sequence[int]) -> tuple[int, ...]:
    try:
        values = tuple(ids)
    except TypeError as exc:
        raise TypeError("ids must be a sequence of integers") from exc
    if not values:
        raise ValueError("ids must not be empty")
    if any(isinstance(value, bool) or not isinstance(value, Integral) for value in values):
        raise TypeError("each Dynamixel ID must be an integer")
    result = tuple(int(value) for value in values)
    if any(value < 0 or value > 252 for value in result):
        raise ValueError("each Dynamixel ID must be in 0..252")
    if len(set(result)) != len(result):
        raise ValueError("Dynamixel IDs must be unique")
    return result



class DynamixelDriver(DynamixelIO):
    """Real ``dynamixel_sdk`` adapter with an explicit, safe lifecycle."""

    def __init__(
        self,
        ids: Sequence[int],
        servo_types: Sequence[str],
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        sync_read_attempts: int = DEFAULT_SYNC_READ_ATTEMPTS,
        *, read_present_current: bool = False,
    ) -> None:
        self._ids = _validate_ids(ids)
        if len(servo_types) != len(self._ids):
            raise ValueError("servo_types length must match ids length")
        self._servo_types = tuple(
            normalize_servo_type(servo_type) for servo_type in servo_types
        )
        if isinstance(baudrate, bool) or not isinstance(baudrate, Integral):
            raise TypeError("baudrate must be an integer")
        if baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if (
            isinstance(sync_read_attempts, bool)
            or not isinstance(sync_read_attempts, Integral)
        ):
            raise TypeError("sync_read_attempts must be an integer")
        if sync_read_attempts <= 0:
            raise ValueError("sync_read_attempts must be positive")
        if type(read_present_current) is not bool:
            raise TypeError("read_present_current must be a bool")

        self._configured_port = port
        self._port = _serial_port_candidate(port)
        self._baudrate = int(baudrate)
        self._sync_read_attempts = int(sync_read_attempts)
        self._read_present_current = read_present_current
        self._present_currents_ma: np.ndarray | None = None
        self._lock = RLock()
        self._port_open = False
        self._is_open = False
        self._read_only = False
        self._torque_enabled = False
        self._latency_timer: int | None = None
        self._device_info: tuple[DynamixelDeviceInfo, ...] = ()

        self._port_handler = PortHandler(str(self._port))
        self._packet_handler = PacketHandler(PROTOCOL_VERSION)
        self._sync_read = GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            ADDR_PRESENT_CURRENT if read_present_current else ADDR_PRESENT_VELOCITY,
            LEN_SYNC_STATE_WITH_CURRENT if read_present_current else LEN_SYNC_STATE,
        )
        self._sync_write_current = GroupSyncWrite(
            self._port_handler,
            self._packet_handler,
            ADDR_GOAL_CURRENT,
            LEN_GOAL_CURRENT,
        )

    @property
    def torque_enabled(self) -> bool:
        with self._lock:
            return self._torque_enabled

    @property
    def present_currents_ma(self) -> np.ndarray | None:
        """Latest measured eight-ID current sample, when enabled for this driver."""
        with self._lock:
            return (None if self._present_currents_ma is None
                    else self._present_currents_ma.copy())

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._is_open

    @property
    def read_only(self) -> bool:
        with self._lock:
            return self._read_only

    @property
    def device_info(self) -> tuple[DynamixelDeviceInfo, ...]:
        """Return device metadata captured by :meth:`open_read_only`."""
        with self._lock:
            return self._device_info

    @property
    def port(self) -> str:
        return str(self._port)

    @property
    def latency_timer(self) -> int | None:
        return self._latency_timer

    def open(self, *, enable_torque: bool = True) -> None:
        """Open and configure every expected servo in a safe order."""
        if not isinstance(enable_torque, bool):
            raise TypeError("enable_torque must be bool")
        with self._lock:
            if self._is_open:
                if self._read_only:
                    raise DynamixelError(
                        "Dynamixel driver is already open in read-only mode"
                    )
                return

            try:
                self._open_serial_locked()
                self._ping_devices_locked()
                self._register_sync_read_locked()

                self._set_torque_enabled_locked(False)
                self._set_operating_mode_locked(CURRENT_CONTROL_MODE)
                self._verify_operating_mode_locked(CURRENT_CONTROL_MODE)
                # Never enable current control with a Goal Current value left
                # behind by an earlier process on an already-powered bus.
                self._write_raw_currents_locked([0] * len(self._ids))
                if enable_torque:
                    self._set_torque_enabled_locked(True)
                self._read_only = False
                self._device_info = ()
                self._is_open = True
            except Exception:
                self._cleanup_failed_open_locked()
                raise

    def open_read_only(self) -> None:
        """Open the bus for phase-2.5 inspection without issuing any write.

        Every expected device must answer ping, report Return Delay Time 0, and
        already have torque disabled.  Failure cleanup only closes the serial
        port; it deliberately does not attempt a safety write because this
        lifecycle is used to prove read-only communication.
        """
        with self._lock:
            if self._is_open:
                if not self._read_only:
                    raise DynamixelError(
                        "Dynamixel driver is already open in writable mode"
                    )
                return

            try:
                self._open_serial_locked()
                model_numbers = self._ping_devices_locked()
                device_info = []
                for dxl_id, model_number in zip(self._ids, model_numbers):
                    return_delay = self._read_byte_locked(
                        dxl_id,
                        ADDR_RETURN_DELAY_TIME,
                        "read Return Delay Time",
                    )
                    torque_value = self._read_byte_locked(
                        dxl_id,
                        ADDR_TORQUE_ENABLE,
                        "read Torque Enable",
                    )
                    if return_delay != 0:
                        raise DynamixelSdkError(
                            "Return Delay Time is not 0 for device "
                            f"ID {dxl_id}: got {return_delay}"
                        )
                    if torque_value != TORQUE_DISABLE:
                        raise DynamixelSdkError(
                            f"torque is enabled for device ID {dxl_id}"
                        )
                    device_info.append(
                        DynamixelDeviceInfo(
                            device_id=dxl_id,
                            model_number=model_number,
                            return_delay_time=return_delay,
                            torque_enabled=False,
                        )
                    )

                self._register_sync_read_locked()
                self._device_info = tuple(device_info)
                self._read_only = True
                self._torque_enabled = False
                self._is_open = True
            except Exception:
                self._cleanup_failed_open_locked(allow_writes=False)
                raise

    def set_operating_mode(self, mode: int) -> None:
        """Set Operating Mode while all servos are torque-disabled."""
        with self._lock:
            self._require_port_open()
            self._require_writable()
            self._set_operating_mode_locked(mode)

    def verify_operating_mode(self, expected_mode: int) -> None:
        """Raise if any servo does not report *expected_mode*."""
        with self._lock:
            self._require_port_open()
            self._verify_operating_mode_locked(expected_mode)

    def set_torque_enabled(self, enabled: bool) -> None:
        """Enable or disable torque, verifying Current Control Mode first."""
        if not isinstance(enabled, bool):
            raise TypeError("enabled must be a bool")
        with self._lock:
            self._require_port_open()
            self._require_writable()
            if enabled:
                self._verify_operating_mode_locked(CURRENT_CONTROL_MODE)
            self._set_torque_enabled_locked(enabled)

    def read_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Synchronously read positions in rad and velocities in rad/s."""
        with self._lock:
            self._require_open()
            last_result = COMM_SUCCESS
            for _ in range(self._sync_read_attempts):
                last_result = self._sync_read.txRxPacket()
                if last_result == COMM_SUCCESS:
                    break
            else:
                detail = self._packet_handler.getTxRxResult(last_result)
                if self._read_only:
                    raise self._diagnose_sync_read_failure_locked(detail)
                raise DynamixelSdkError(
                    f"read_state sync read failed: {detail}"
                )

            positions = np.empty(len(self._ids), dtype=float)
            velocities = np.empty(len(self._ids), dtype=float)
            currents = np.empty(len(self._ids), dtype=float) if self._read_present_current else None
            for index, dxl_id in enumerate(self._ids):
                if currents is not None:
                    current_raw = self._read_sync_value_locked(
                        dxl_id,
                        ADDR_PRESENT_CURRENT,
                        LEN_PRESENT_CURRENT,
                        "read present current",
                    )
                    currents[index] = (
                        uint16_to_int16(current_raw)
                        * get_servo_spec(self._servo_types[index]).current_unit_ma
                    )
                velocity_raw = self._read_sync_value_locked(
                    dxl_id,
                    ADDR_PRESENT_VELOCITY,
                    LEN_PRESENT_VELOCITY,
                    "read present velocity",
                )
                position_raw = self._read_sync_value_locked(
                    dxl_id,
                    ADDR_PRESENT_POSITION,
                    LEN_PRESENT_POSITION,
                    "read present position",
                )
                velocities[index] = velocity_units_to_rad_s(
                    uint32_to_int32(velocity_raw)
                )
                positions[index] = position_ticks_to_radians(
                    uint32_to_int32(position_raw)
                )
            self._present_currents_ma = currents
            return positions, velocities

    def write_currents(self, currents_ma: Sequence[float]) -> None:
        """Synchronously write one finite, in-range current per servo."""
        raw_currents = self._validate_currents(currents_ma)
        with self._lock:
            self._require_open()
            self._require_writable()
            if not self._torque_enabled:
                raise DynamixelError("write_currents requires torque to be enabled")
            self._write_raw_currents_locked(raw_currents)

    def close(self) -> None:
        """Best-effort zero current and torque disable before closing the port."""
        with self._lock:
            if not self._port_open:
                self._is_open = False
                self._read_only = False
                self._torque_enabled = False
                return

            if self._read_only:
                self._port_handler.closePort()
                self._port_open = False
                self._is_open = False
                self._read_only = False
                self._torque_enabled = False
                return

            first_error: Exception | None = None
            if self._is_open:
                try:
                    self._write_raw_currents_locked([0] * len(self._ids))
                except Exception as exc:  # Continue to torque-disable and close.
                    first_error = exc
            try:
                self._set_torque_enabled_locked(False)
            except Exception as exc:  # The physical state is unknown on failure.
                if first_error is None:
                    first_error = exc
            finally:
                self._port_handler.closePort()
                self._port_open = False
                self._is_open = False
                self._read_only = False
                if first_error is None:
                    self._torque_enabled = False

            if first_error is not None:
                raise first_error

    def _open_serial_locked(self) -> None:
        self._port = resolve_serial_port(self._configured_port)
        self._latency_timer = read_ftdi_latency_timer(self._port)
        if self._latency_timer is not None and self._latency_timer != 1:
            raise DynamixelPortError(
                f"FTDI latency timer for {self._port} is "
                f"{self._latency_timer}, expected 1"
            )
        if not self._port_handler.openPort():
            raise DynamixelPortError(
                f"failed to open serial port {self._port}"
            )
        self._port_open = True
        if not self._port_handler.setBaudRate(self._baudrate):
            raise DynamixelPortError(
                f"failed to set baudrate {self._baudrate} on {self._port}"
            )

    def _ping_devices_locked(self) -> tuple[int, ...]:
        model_numbers = []
        for dxl_id in self._ids:
            ping_result = self._packet_handler.ping(self._port_handler, dxl_id)
            model_number, comm_result, device_error = ping_result
            self._check_sdk_result("ping", dxl_id, comm_result, device_error)
            model_numbers.append(model_number)
        return tuple(model_numbers)

    def _register_sync_read_locked(self) -> None:
        for dxl_id in self._ids:
            if not self._sync_read.addParam(dxl_id):
                raise DynamixelSdkError(
                    f"sync read addParam failed for device ID {dxl_id}"
                )

    def _set_operating_mode_locked(self, mode: int) -> None:
        if isinstance(mode, bool) or not isinstance(mode, Integral):
            raise TypeError("operating mode must be an integer")
        if self._torque_enabled:
            raise DynamixelError(
                "set_operating_mode requires torque to be disabled"
            )
        for dxl_id in self._ids:
            comm_result, device_error = self._packet_handler.write1ByteTxRx(
                self._port_handler,
                dxl_id,
                ADDR_OPERATING_MODE,
                int(mode),
            )
            self._check_sdk_result(
                "set operating mode", dxl_id, comm_result, device_error
            )

    def _verify_operating_mode_locked(self, expected_mode: int) -> None:
        if isinstance(expected_mode, bool) or not isinstance(expected_mode, Integral):
            raise TypeError("expected operating mode must be an integer")
        for dxl_id in self._ids:
            mode, comm_result, device_error = self._packet_handler.read1ByteTxRx(
                self._port_handler,
                dxl_id,
                ADDR_OPERATING_MODE,
            )
            self._check_sdk_result(
                "verify operating mode", dxl_id, comm_result, device_error
            )
            if mode != expected_mode:
                raise DynamixelSdkError(
                    f"verify operating mode failed for device ID {dxl_id}: "
                    f"expected {expected_mode}, got {mode}"
                )

    def _set_torque_enabled_locked(self, enabled: bool) -> None:
        value = TORQUE_ENABLE if enabled else TORQUE_DISABLE
        operation = "enable torque" if enabled else "disable torque"
        for dxl_id in self._ids:
            comm_result, device_error = self._packet_handler.write1ByteTxRx(
                self._port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                value,
            )
            self._check_sdk_result(
                operation, dxl_id, comm_result, device_error
            )
        self._torque_enabled = enabled

    def _read_byte_locked(
        self, dxl_id: int, address: int, operation: str
    ) -> int:
        value, comm_result, device_error = self._packet_handler.read1ByteTxRx(
            self._port_handler, dxl_id, address
        )
        self._check_sdk_result(operation, dxl_id, comm_result, device_error)
        return value

    def _read_sync_value_locked(
        self, dxl_id: int, address: int, length: int, operation: str
    ) -> int:
        if not self._sync_read.isAvailable(dxl_id, address, length):
            raise DynamixelSdkError(
                f"{operation} unavailable for device ID {dxl_id}"
            )
        return self._sync_read.getData(dxl_id, address, length)

    def _diagnose_sync_read_failure_locked(
        self, sync_detail: str
    ) -> DynamixelSdkError:
        failures = []
        for dxl_id in self._ids:
            _, comm_result, device_error = self._packet_handler.ping(
                self._port_handler, dxl_id
            )
            if comm_result == COMM_SUCCESS and device_error == 0:
                continue
            details = self._sdk_result_details(comm_result, device_error)
            failures.append(f"ping failed for device ID {dxl_id}: {details}")
        diagnosis = (
            "; ".join(failures)
            if failures
            else "all diagnostic pings replied"
        )
        return DynamixelSdkError(
            f"read_state sync read failed: {sync_detail}; {diagnosis}"
        )

    def _validate_currents(self, currents_ma: Sequence[float]) -> list[int]:
        if isinstance(currents_ma, (str, bytes)):
            raise TypeError("currents_ma must be a numeric sequence")
        try:
            values = list(currents_ma)
        except TypeError as exc:
            raise TypeError("currents_ma must be a numeric sequence") from exc
        if len(values) != len(self._ids):
            raise ValueError(
                f"currents_ma must contain {len(self._ids)} values, got {len(values)}"
            )
        return [
            current_ma_to_raw(current, servo_type)
            for current, servo_type in zip(values, self._servo_types)
        ]

    def _write_raw_currents_locked(self, raw_currents: Sequence[int]) -> None:
        try:
            for dxl_id, raw_current in zip(self._ids, raw_currents):
                if not self._sync_write_current.addParam(
                    dxl_id, encode_goal_current(raw_current)
                ):
                    raise DynamixelSdkError(
                        f"write_currents addParam failed for device ID {dxl_id}"
                    )
            comm_result = self._sync_write_current.txPacket()
            if comm_result != COMM_SUCCESS:
                detail = self._packet_handler.getTxRxResult(comm_result)
                raise DynamixelSdkError(
                    f"write_currents sync write failed: {detail}"
                )
        finally:
            self._sync_write_current.clearParam()

    def _check_sdk_result(
        self,
        operation: str,
        dxl_id: int,
        comm_result: int,
        device_error: int,
    ) -> None:
        if comm_result == COMM_SUCCESS and device_error == 0:
            return
        raise DynamixelSdkError(
            f"{operation} failed for device ID {dxl_id}: "
            f"{self._sdk_result_details(comm_result, device_error)}"
        )

    def _sdk_result_details(self, comm_result: int, device_error: int) -> str:
        details = []
        if comm_result != COMM_SUCCESS:
            details.append(self._packet_handler.getTxRxResult(comm_result))
        if device_error != 0:
            details.append(self._packet_handler.getRxPacketError(device_error))
        return "; ".join(details)

    def _require_port_open(self) -> None:
        if not self._port_open:
            raise DynamixelError("Dynamixel serial port is not open")

    def _require_open(self) -> None:
        if not self._is_open:
            raise DynamixelError("Dynamixel driver is not open")

    def _require_writable(self) -> None:
        if self._read_only:
            raise DynamixelError("operation is unavailable in read-only mode")

    def _cleanup_failed_open_locked(self, allow_writes: bool = True) -> None:
        if self._port_open:
            if allow_writes:
                try:
                    self._set_torque_enabled_locked(False)
                except Exception:
                    pass
            self._port_handler.closePort()
        self._port_open = False
        self._is_open = False
        self._read_only = False
        self._torque_enabled = False
