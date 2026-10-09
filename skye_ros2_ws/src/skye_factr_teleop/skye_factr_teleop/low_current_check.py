# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Phase 2.5.2 single-servo low-current acceptance utility."""

import argparse
from contextlib import contextmanager
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import sys
import time
from typing import Callable, Sequence

from .config import TeleopConfig, load_config
from .dynamixel_driver import (
    DEFAULT_BAUDRATE,
    MODEL_NUMBERS,
    DynamixelDriver,
    current_ma_to_raw,
    encode_goal_current,
    get_servo_spec,
    normalize_servo_type,
)


ABSOLUTE_SMOKE_CEILING_MA = 100.0
MODEL_LIMIT_FRACTION = 0.10
MAX_PULSE_SECONDS = 2.0
MAX_SETTLE_SECONDS = 2.0
DEFAULT_PULSE_SECONDS = 0.5
DEFAULT_SETTLE_SECONDS = 0.2
DEFAULT_MIN_DIRECTION_DELTA_RAD = 0.001


class TerminationRequested(RuntimeError):
    """Raised for a catchable process-termination signal."""

    def __init__(self, signum: int) -> None:
        self.signum = signum
        super().__init__(f"received signal {signal.Signals(signum).name}")


def smoke_current_ceiling_ma(servo_type: str) -> float:
    """Return the hard smoke-test ceiling for one supported model."""
    spec = get_servo_spec(servo_type)
    return min(
        ABSOLUTE_SMOKE_CEILING_MA,
        spec.current_limit_ma * MODEL_LIMIT_FRACTION,
    )


def run_low_current_check(
    *,
    config_path: Path,
    side: str,
    device_id: int,
    current_limit_ma: float,
    test_current_ma: float,
    pulse_seconds: float,
    settle_seconds: float,
    min_direction_delta_rad: float,
    output_path: Path,
    prepare_only: bool,
    driver_factory: Callable = DynamixelDriver,
    sleep: Callable[[float], None] = time.sleep,
) -> dict:
    """Prepare or execute one single-servo low-current smoke check.

    The writable path first performs a separate read-only identity and safety
    check. It then creates a one-ID production driver and sends positive, zero,
    negative, and zero current. ``DynamixelDriver.close`` provides the final
    zero-current and Torque Disable cleanup on every exit path.
    """
    config = load_config(config_path)
    model = _validate_arguments(
        config=config,
        side=side,
        device_id=device_id,
        current_limit_ma=current_limit_ma,
        test_current_ma=test_current_ma,
        pulse_seconds=pulse_seconds,
        settle_seconds=settle_seconds,
        min_direction_delta_rad=min_direction_delta_rad,
        output_path=output_path,
    )
    spec = get_servo_spec(model)
    expected_model_number = MODEL_NUMBERS[model]
    positive_raw = current_ma_to_raw(test_current_ma, model)
    negative_raw = current_ma_to_raw(-test_current_ma, model)
    report = {
        "schema": "skye_factr_teleop.phase_2_5_2.v1",
        "started_at": (
            datetime.now().astimezone().isoformat(timespec="seconds")
        ),
        "mode": "prepare-only" if prepare_only else "hardware",
        "side": side,
        "config": str(Path(config_path).resolve()),
        "configured_port": config.dynamixel.dynamixel_port,
        "resolved_port": None,
        "baudrate": config.dynamixel.baudrate,
        "latency_timer": None,
        "device": {
            "id": device_id,
            "configured_model": model,
            "expected_model_number": expected_model_number,
            "reported_model_number": None,
            "return_delay_time": None,
            "torque_enabled_before_test": None,
            "joint_sign": config.dynamixel.joint_signs[device_id - 1],
        },
        "limits": {
            "model_current_unit_ma": spec.current_unit_ma,
            "model_rated_limit_ma": spec.current_limit_ma,
            "smoke_ceiling_ma": smoke_current_ceiling_ma(model),
            "operator_limit_ma": float(current_limit_ma),
            "test_current_ma": float(test_current_ma),
            "pulse_seconds": float(pulse_seconds),
            "settle_seconds": float(settle_seconds),
            "min_direction_delta_rad": float(min_direction_delta_rad),
        },
        "initialization": {
            "expected_sequence": [
                "Torque Disable",
                "Current Control Mode",
                "Zero Goal Current",
                "Torque Enable",
            ],
            "result": "NOT_RUN" if prepare_only else "PENDING",
        },
        "commands": {
            "positive": {
                "current_ma": float(test_current_ma),
                "raw": positive_raw,
                "encoded": encode_goal_current(positive_raw),
                "effective_current_ma": positive_raw * spec.current_unit_ma,
            },
            "zero_after_positive": {
                "current_ma": 0.0,
                "raw": 0,
                "encoded": [0, 0],
            },
            "negative": {
                "current_ma": -float(test_current_ma),
                "raw": negative_raw,
                "encoded": encode_goal_current(negative_raw),
                "effective_current_ma": negative_raw * spec.current_unit_ma,
            },
            "zero_after_negative": {
                "current_ma": 0.0,
                "raw": 0,
                "encoded": [0, 0],
            },
        },
        "command_log": [],
        "direction": {"result": "NOT_RUN"},
        "shutdown": {"result": "NOT_RUN" if prepare_only else "PENDING"},
        "result": "PREPARED" if prepare_only else "FAIL",
        "interrupted": False,
        "error": None,
    }

    if prepare_only:
        _write_report(output_path, report)
        return report

    driver_kwargs = {
        "ids": (device_id,),
        "servo_types": (model,),
        "port": config.dynamixel.dynamixel_port,
        "baudrate": config.dynamixel.baudrate,
    }
    try:
        _run_read_only_preflight(
            driver_factory=driver_factory,
            driver_kwargs=driver_kwargs,
            expected_model_number=expected_model_number,
            report=report,
        )
    except (Exception, KeyboardInterrupt) as exc:
        report["interrupted"] = isinstance(
            exc, (KeyboardInterrupt, TerminationRequested)
        )
        report["error"] = _error_text(exc)
        report["shutdown"] = {
            "result": "NOT_REQUIRED",
            "detail": "writable driver was not opened",
        }
        _write_report(output_path, report)
        return report

    driver = None
    try:
        driver = driver_factory(**driver_kwargs)
        driver.open()
        report["initialization"]["result"] = "PASS"

        initial_position = float(driver.read_state()[0][0])
        _write_current(driver, report, "positive", test_current_ma)
        sleep(pulse_seconds)
        _write_current(driver, report, "zero_after_positive", 0.0)
        sleep(settle_seconds)
        positive_position = float(driver.read_state()[0][0])

        _write_current(driver, report, "negative", -test_current_ma)
        sleep(pulse_seconds)
        _write_current(driver, report, "zero_after_negative", 0.0)
        sleep(settle_seconds)
        negative_position = float(driver.read_state()[0][0])

        report["direction"] = _analyze_direction(
            initial_position=initial_position,
            positive_position=positive_position,
            negative_position=negative_position,
            joint_sign=config.dynamixel.joint_signs[device_id - 1],
            minimum_delta=min_direction_delta_rad,
        )
    except (Exception, KeyboardInterrupt) as exc:
        report["interrupted"] = isinstance(
            exc, (KeyboardInterrupt, TerminationRequested)
        )
        report["error"] = _error_text(exc)
    finally:
        if driver is not None:
            try:
                driver.close()
                report["shutdown"] = {
                    "result": "PASS",
                    "detail": (
                        "production close returned after its zero-current, "
                        "Torque Disable, and port-close cleanup path"
                    ),
                }
            except Exception as exc:
                report["shutdown"] = {
                    "result": "FAIL",
                    "detail": _error_text(exc),
                }
                if report["error"] is None:
                    report["error"] = (
                        f"safe shutdown failed: {_error_text(exc)}"
                    )

    if report["error"] is None and report["shutdown"]["result"] == "PASS":
        report["result"] = report["direction"]["result"]
    else:
        report["result"] = "FAIL"
    _write_report(output_path, report)
    return report


def _run_read_only_preflight(
    *,
    driver_factory: Callable,
    driver_kwargs: dict,
    expected_model_number: int,
    report: dict,
) -> None:
    driver = driver_factory(**driver_kwargs)
    close_error = None
    try:
        driver.open_read_only()
        info = tuple(driver.device_info)
        if len(info) != 1 or info[0].device_id != driver_kwargs["ids"][0]:
            raise RuntimeError("read-only preflight returned the wrong device")
        if info[0].model_number != expected_model_number:
            raise RuntimeError(
                f"device ID {info[0].device_id} reported model "
                f"{info[0].model_number}, expected {expected_model_number}"
            )
        report["resolved_port"] = driver.port
        report["latency_timer"] = driver.latency_timer
        if driver.latency_timer != 1:
            raise RuntimeError(
                f"FTDI latency timer is {driver.latency_timer!r}, expected 1"
            )
        report["device"].update(
            {
                "reported_model_number": info[0].model_number,
                "return_delay_time": info[0].return_delay_time,
                "torque_enabled_before_test": info[0].torque_enabled,
            }
        )
    finally:
        try:
            driver.close()
        except Exception as exc:
            close_error = exc
    if close_error is not None:
        raise RuntimeError(f"read-only preflight close failed: {close_error}")


def _write_current(driver, report: dict, name: str, current_ma: float) -> None:
    entry = {
        "name": name,
        "current_ma": float(current_ma),
        "result": "PENDING",
    }
    report["command_log"].append(entry)
    try:
        driver.write_currents([current_ma])
    except Exception:
        entry["result"] = "FAIL"
        raise
    entry["result"] = "PASS"


def _analyze_direction(
    *,
    initial_position: float,
    positive_position: float,
    negative_position: float,
    joint_sign: float,
    minimum_delta: float,
) -> dict:
    positive_delta = positive_position - initial_position
    negative_delta = negative_position - positive_position
    if (
        abs(positive_delta) < minimum_delta
        or abs(negative_delta) < minimum_delta
    ):
        result = "INCONCLUSIVE"
        detail = "one or both current pulses produced insufficient movement"
    elif positive_delta * negative_delta >= 0:
        result = "FAIL"
        detail = (
            "positive and negative current did not produce opposite motion"
        )
    else:
        result = "PASS"
        detail = "positive and negative current produced opposite motion"
    return {
        "result": result,
        "detail": detail,
        "initial_position_rad": initial_position,
        "positive_position_rad": positive_position,
        "negative_position_rad": negative_position,
        "positive_delta_rad": positive_delta,
        "negative_delta_rad": negative_delta,
        "signed_positive_delta_rad": positive_delta * joint_sign,
        "signed_negative_delta_rad": negative_delta * joint_sign,
    }


def _validate_arguments(
    *,
    config: TeleopConfig,
    side: str,
    device_id: int,
    current_limit_ma: float,
    test_current_ma: float,
    pulse_seconds: float,
    settle_seconds: float,
    min_direction_delta_rad: float,
    output_path: Path,
) -> str:
    if side not in {"left", "right"}:
        raise ValueError("side must be 'left' or 'right'")
    if f"_{side}" not in config.name:
        raise ValueError(
            f"config name {config.name!r} does not match selected "
            f"side {side!r}"
        )
    if isinstance(device_id, bool) or device_id not in range(1, 9):
        raise ValueError("device ID must be in 1..8")
    port = Path(config.dynamixel.dynamixel_port)
    if port.is_absolute() and port.parent != Path("/dev/serial/by-id"):
        raise ValueError("Dynamixel port must use /dev/serial/by-id")
    if not port.is_absolute() and port.parent != Path("."):
        raise ValueError(
            "Dynamixel port must be a by-id basename or absolute by-id path"
        )
    if config.dynamixel.baudrate != DEFAULT_BAUDRATE:
        raise ValueError(f"baudrate must be {DEFAULT_BAUDRATE}")
    model = normalize_servo_type(config.dynamixel.servo_types[device_id - 1])
    ceiling = smoke_current_ceiling_ma(model)
    current_limit_ma = _positive_finite(current_limit_ma, "current limit")
    test_current_ma = _positive_finite(test_current_ma, "test current")
    if current_limit_ma > ceiling:
        raise ValueError(
            f"current limit {current_limit_ma} mA exceeds {model} smoke "
            f"ceiling {ceiling} mA"
        )
    if test_current_ma > current_limit_ma:
        raise ValueError(
            f"test current {test_current_ma} mA exceeds operator limit "
            f"{current_limit_ma} mA"
        )
    effective_current_ma = abs(
        current_ma_to_raw(test_current_ma, model)
        * get_servo_spec(model).current_unit_ma
    )
    if effective_current_ma > current_limit_ma:
        raise ValueError(
            f"rounded test current {effective_current_ma} mA exceeds "
            f"operator limit {current_limit_ma} mA"
        )
    pulse_seconds = _positive_finite(pulse_seconds, "pulse duration")
    if pulse_seconds > MAX_PULSE_SECONDS:
        raise ValueError(
            f"pulse duration must not exceed {MAX_PULSE_SECONDS} seconds"
        )
    settle_seconds = _nonnegative_finite(settle_seconds, "settle duration")
    if settle_seconds > MAX_SETTLE_SECONDS:
        raise ValueError(
            f"settle duration must not exceed {MAX_SETTLE_SECONDS} seconds"
        )
    _positive_finite(min_direction_delta_rad, "minimum direction delta")
    if not str(output_path):
        raise ValueError("output path must not be empty")
    return model


def _positive_finite(value: float, name: str) -> float:
    value = _nonnegative_finite(value, name)
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


def _nonnegative_finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a real number")
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be finite and non-negative")
    return value


def _error_text(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"


def _write_report(output_path: Path, report: dict) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )


@contextmanager
def _termination_handlers():
    previous = {}

    def request_termination(signum, _frame):
        raise TerminationRequested(signum)

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous[signum] = signal.signal(signum, request_termination)
    try:
        yield
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Phase 2.5.2 single-servo low-current verification. Run "
            "--prepare-only before connecting hardware."
        )
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--id", type=int, required=True, dest="device_id")
    parser.add_argument("--current-limit-ma", type=float, required=True)
    parser.add_argument("--test-current-ma", type=float, required=True)
    parser.add_argument(
        "--pulse-seconds", type=float, default=DEFAULT_PULSE_SECONDS
    )
    parser.add_argument(
        "--settle-seconds", type=float, default=DEFAULT_SETTLE_SECONDS
    )
    parser.add_argument(
        "--min-direction-delta-rad",
        type=float,
        default=DEFAULT_MIN_DIRECTION_DELTA_RAD,
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument(
        "--confirm-single-servo",
        action="store_true",
        help="confirm that only the selected servo is connected or enabled",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if not args.prepare_only and not args.confirm_single_servo:
            raise ValueError(
                "hardware mode requires --confirm-single-servo after "
                "checking wiring"
            )
        kwargs = {
            "config_path": args.config,
            "side": args.side,
            "device_id": args.device_id,
            "current_limit_ma": args.current_limit_ma,
            "test_current_ma": args.test_current_ma,
            "pulse_seconds": args.pulse_seconds,
            "settle_seconds": args.settle_seconds,
            "min_direction_delta_rad": args.min_direction_delta_rad,
            "output_path": args.output,
            "prepare_only": args.prepare_only,
        }
        if args.prepare_only:
            report = run_low_current_check(**kwargs)
        else:
            with _termination_handlers():
                report = run_low_current_check(**kwargs)
    except Exception as exc:
        print(f"PRECHECK ERROR: {exc}", file=sys.stderr)
        return 2

    print(f"record: {args.output}")
    print(
        f"device: {report['side']} ID {report['device']['id']} "
        f"{report['device']['configured_model']}"
    )
    print(f"result: {report['result']}")
    if report["error"]:
        print(f"error: {report['error']}", file=sys.stderr)
    if report["interrupted"]:
        return 130
    return 0 if report["result"] in {"PREPARED", "PASS"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
