# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Phase 2.5.1 read-only Dynamixel bus acceptance utility."""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time
from typing import Callable, Sequence

import numpy as np

from .config import TeleopConfig, load_config
from .dynamixel_driver import (
    DEFAULT_BAUDRATE,
    MODEL_NUMBERS,
    DynamixelDriver,
    normalize_servo_type,
)


EXPECTED_IDS = tuple(range(1, 9))
DEFAULT_FRAMES = 100
DEFAULT_INTERVAL = 0.02
DEFAULT_MAX_ABS_VELOCITY = 20.0
DEFAULT_MAX_POSITION_STEP = math.pi


def analyze_samples(
    positions: Sequence[np.ndarray],
    velocities: Sequence[np.ndarray],
    joint_signs: Sequence[float],
    max_abs_velocity: float,
    max_position_step: float,
) -> dict:
    """Summarize samples and identify implausible values or discontinuities."""
    if not positions or len(positions) != len(velocities):
        raise ValueError(
            "positions and velocities must contain matching samples"
        )
    position_array = np.asarray(positions, dtype=float)
    velocity_array = np.asarray(velocities, dtype=float)
    signs = np.asarray(joint_signs, dtype=float)
    expected_shape = (len(positions), len(signs))
    if (
        position_array.shape != expected_shape
        or velocity_array.shape != expected_shape
    ):
        raise ValueError(f"each sample must contain {len(signs)} values")
    if not np.all(np.isfinite(position_array)):
        raise ValueError("position samples contain non-finite values")
    if not np.all(np.isfinite(velocity_array)):
        raise ValueError("velocity samples contain non-finite values")

    issues = []
    devices = []
    for index, dxl_id in enumerate(EXPECTED_IDS):
        device_positions = position_array[:, index]
        device_velocities = velocity_array[:, index]
        steps = np.abs(np.diff(device_positions))
        max_velocity = float(np.max(np.abs(device_velocities)))
        max_step = float(np.max(steps)) if len(steps) else 0.0
        if max_velocity > max_abs_velocity:
            issues.append(
                f"device ID {dxl_id} absolute velocity "
                f"{max_velocity:.6f} rad/s "
                f"exceeds {max_abs_velocity:.6f}"
            )
        if max_step > max_position_step:
            issues.append(
                f"device ID {dxl_id} position step {max_step:.6f} rad "
                f"exceeds {max_position_step:.6f}"
            )
        raw_delta = float(device_positions[-1] - device_positions[0])
        devices.append(
            {
                "id": dxl_id,
                "position_min_rad": float(np.min(device_positions)),
                "position_max_rad": float(np.max(device_positions)),
                "position_delta_rad": raw_delta,
                "signed_position_delta_rad": raw_delta * float(signs[index]),
                "max_abs_velocity_rad_s": max_velocity,
                "max_position_step_rad": max_step,
            }
        )
    return {"devices": devices, "issues": issues}


def run_readonly_check(
    *,
    config_path: Path,
    side: str,
    frames: int,
    interval: float,
    output_path: Path,
    max_abs_velocity: float = DEFAULT_MAX_ABS_VELOCITY,
    max_position_step: float = DEFAULT_MAX_POSITION_STEP,
    driver_factory: Callable = DynamixelDriver,
    sleep: Callable[[float], None] = time.sleep,
) -> dict:
    """Run one bus check and always write a machine-readable result record."""
    config = load_config(config_path)
    _validate_arguments(
        config,
        side,
        frames,
        interval,
        output_path,
        max_abs_velocity,
        max_position_step,
    )
    expected_models = [
        MODEL_NUMBERS[normalize_servo_type(model)]
        for model in config.dynamixel.servo_types
    ]
    report = {
        "schema": "skye_factr_teleop.phase_2_5_1.v1",
        "started_at": datetime.now()
        .astimezone()
        .isoformat(timespec="seconds"),
        "side": side,
        "config": str(Path(config_path).resolve()),
        "configured_port": config.dynamixel.dynamixel_port,
        "resolved_port": None,
        "ids": list(EXPECTED_IDS),
        "baudrate": config.arm_teleop.baudrate,
        "latency_timer": None,
        "devices": [],
        "samples": [],
        "analysis": None,
        "manual_checks": {
            "large_arm_motion_link_disconnected": "PENDING",
            "usb_side_mapping": "PENDING",
            "physical_id_order": "PENDING",
            "joint_direction": "PENDING",
            "continuous_angle": "PENDING",
            "single_device_disconnect": "PENDING",
        },
        "automated_result": "FAIL",
        "error": None,
    }

    driver = driver_factory(
        ids=EXPECTED_IDS,
        servo_types=config.dynamixel.servo_types,
        port=config.dynamixel.dynamixel_port,
        baudrate=config.arm_teleop.baudrate,
    )
    position_samples = []
    velocity_samples = []
    started = time.monotonic()
    close_error = None
    try:
        driver.open_read_only()
        report["resolved_port"] = driver.port
        report["latency_timer"] = driver.latency_timer
        if driver.latency_timer != 1:
            raise RuntimeError(
                f"FTDI latency timer is {driver.latency_timer!r}, expected 1"
            )

        for info, expected_model, configured_model in zip(
            driver.device_info,
            expected_models,
            config.dynamixel.servo_types,
        ):
            device_record = {
                "id": info.device_id,
                "configured_model": normalize_servo_type(configured_model),
                "model_number": info.model_number,
                "expected_model_number": expected_model,
                "return_delay_time": info.return_delay_time,
                "torque_enabled": info.torque_enabled,
            }
            report["devices"].append(device_record)
            if info.model_number != expected_model:
                raise RuntimeError(
                    f"device ID {info.device_id} reported model "
                    f"{info.model_number}, "
                    f"expected model {expected_model} "
                    f"({device_record['configured_model']})"
                )

        for sample_index in range(frames):
            positions, velocities = driver.read_state()
            positions = np.asarray(positions, dtype=float)
            velocities = np.asarray(velocities, dtype=float)
            position_samples.append(positions)
            velocity_samples.append(velocities)
            report["samples"].append(
                {
                    "index": sample_index,
                    "elapsed_s": time.monotonic() - started,
                    "positions_rad": positions.tolist(),
                    "velocities_rad_s": velocities.tolist(),
                }
            )
            if sample_index + 1 < frames and interval:
                sleep(interval)

        analysis = analyze_samples(
            position_samples,
            velocity_samples,
            config.dynamixel.joint_signs,
            max_abs_velocity,
            max_position_step,
        )
        report["analysis"] = analysis
        if analysis["issues"]:
            raise RuntimeError("; ".join(analysis["issues"]))
        report["automated_result"] = "PASS"
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        try:
            driver.close()
        except Exception as exc:
            close_error = str(exc)

    if close_error:
        report["automated_result"] = "FAIL"
        message = f"close failed: {close_error}"
        report["error"] = (
            f"{report['error']}; {message}" if report["error"] else message
        )
    report["finished_at"] = datetime.now().astimezone().isoformat(
        timespec="seconds"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report


def _validate_arguments(
    config: TeleopConfig,
    side: str,
    frames: int,
    interval: float,
    output_path: Path,
    max_abs_velocity: float,
    max_position_step: float,
) -> None:
    if side not in {"left", "right"}:
        raise ValueError("side must be 'left' or 'right'")
    expected_name = f"_{side}"
    if expected_name not in config.name:
        raise ValueError(
            f"config name {config.name!r} does not match selected "
            f"side {side!r}"
        )
    port = Path(config.dynamixel.dynamixel_port)
    if port.is_absolute() and port.parent != Path("/dev/serial/by-id"):
        raise ValueError("Dynamixel port must use /dev/serial/by-id")
    if not port.is_absolute() and port.parent != Path("."):
        raise ValueError(
            "Dynamixel port must be a by-id basename or absolute by-id path"
        )
    if config.arm_teleop.baudrate != DEFAULT_BAUDRATE:
        raise ValueError(
            f"baudrate must be {DEFAULT_BAUDRATE}, "
            f"got {config.arm_teleop.baudrate}"
        )
    if isinstance(frames, bool) or frames < 2:
        raise ValueError("frames must be at least 2")
    if interval < 0:
        raise ValueError("interval must be non-negative")
    if not math.isfinite(max_abs_velocity) or max_abs_velocity <= 0:
        raise ValueError("max_abs_velocity must be finite and positive")
    if not math.isfinite(max_position_step) or max_position_step <= 0:
        raise ValueError("max_position_step must be finite and positive")
    if not str(output_path):
        raise ValueError("output path must not be empty")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Phase 2.5.1 read-only Dynamixel verification. This command never "
            "enables torque or writes actuator registers."
        )
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--frames", type=int, default=DEFAULT_FRAMES)
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL)
    parser.add_argument(
        "--max-abs-velocity",
        type=float,
        default=DEFAULT_MAX_ABS_VELOCITY,
    )
    parser.add_argument(
        "--max-position-step",
        type=float,
        default=DEFAULT_MAX_POSITION_STEP,
    )
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = run_readonly_check(
            config_path=args.config,
            side=args.side,
            frames=args.frames,
            interval=args.interval,
            output_path=args.output,
            max_abs_velocity=args.max_abs_velocity,
            max_position_step=args.max_position_step,
        )
    except Exception as exc:
        print(f"PRECHECK ERROR: {exc}", file=sys.stderr)
        return 2

    print(f"record: {args.output}")
    print(f"port: {report['resolved_port'] or report['configured_port']}")
    print(f"automated result: {report['automated_result']}")
    if report["analysis"]:
        print("ID  signed_delta_rad  max_velocity_rad_s  max_step_rad")
        for device in report["analysis"]["devices"]:
            print(
                f"{device['id']:2d}  "
                f"{device['signed_position_delta_rad']:16.6f}  "
                f"{device['max_abs_velocity_rad_s']:18.6f}  "
                f"{device['max_position_step_rad']:12.6f}"
            )
    if report["error"]:
        print(f"error: {report['error']}", file=sys.stderr)
    return 0 if report["automated_result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
