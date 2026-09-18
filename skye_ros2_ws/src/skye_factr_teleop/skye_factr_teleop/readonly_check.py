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


def analyze_guided_attempt(
    positions: Sequence[np.ndarray],
    velocities: Sequence[np.ndarray],
    joint_signs: Sequence[float],
    target_id: int,
    baseline_frames: int,
    endpoint_frames: int,
    min_target_delta: float,
    max_other_delta: float,
    max_abs_velocity: float,
    max_position_step: float,
    max_hold_velocity: float = 0.05,
    max_hold_position_span: float = 0.01,
) -> dict:
    """Evaluate one guided positive-direction movement."""
    if target_id not in EXPECTED_IDS:
        raise ValueError(f"target_id must be one of {EXPECTED_IDS}")
    if baseline_frames < 1 or endpoint_frames < 1:
        raise ValueError("baseline_frames and endpoint_frames must be positive")
    if len(positions) < baseline_frames + endpoint_frames:
        raise ValueError("guided attempt does not contain both stable windows")
    if not math.isfinite(max_hold_velocity) or max_hold_velocity <= 0:
        raise ValueError("max_hold_velocity must be finite and positive")
    if (
        not math.isfinite(max_hold_position_span)
        or max_hold_position_span <= 0
    ):
        raise ValueError("max_hold_position_span must be finite and positive")

    sample_analysis = analyze_samples(
        positions,
        velocities,
        joint_signs,
        max_abs_velocity,
        max_position_step,
    )
    position_array = np.asarray(positions, dtype=float)
    velocity_array = np.asarray(velocities, dtype=float)
    signs = np.asarray(joint_signs, dtype=float)
    baseline = np.median(position_array[:baseline_frames], axis=0)
    endpoint_positions = position_array[-endpoint_frames:]
    endpoint_velocities = velocity_array[-endpoint_frames:]
    endpoint = np.median(endpoint_positions, axis=0)
    endpoint_position_spans = np.ptp(endpoint_positions, axis=0)
    endpoint_max_velocities = np.max(np.abs(endpoint_velocities), axis=0)
    raw_deltas = endpoint - baseline
    signed_deltas = raw_deltas * signs
    detected_id = int(np.argmax(np.abs(raw_deltas))) + 1

    devices = []
    for index, sampled_device in enumerate(sample_analysis["devices"]):
        device = dict(sampled_device)
        device["position_delta_rad"] = float(raw_deltas[index])
        device["signed_position_delta_rad"] = float(signed_deltas[index])
        device["endpoint_position_span_rad"] = float(
            endpoint_position_spans[index]
        )
        device["endpoint_max_abs_velocity_rad_s"] = float(
            endpoint_max_velocities[index]
        )
        devices.append(device)

    issues = list(sample_analysis["issues"])
    if detected_id != target_id:
        issues.append(
            f"expected device ID {target_id} to move most, detected ID "
            f"{detected_id}"
        )

    target_index = target_id - 1
    target_delta = float(signed_deltas[target_index])
    if target_delta <= min_target_delta:
        issues.append(
            f"device ID {target_id} signed movement {target_delta:.6f} rad "
            f"is not greater than {min_target_delta:.6f} rad"
        )

    for index, delta in enumerate(signed_deltas):
        dxl_id = index + 1
        if dxl_id == target_id:
            continue
        if abs(float(delta)) >= max_other_delta:
            issues.append(
                f"device ID {dxl_id} moved {float(delta):.6f} rad, "
                f"expected less than {max_other_delta:.6f} rad"
            )

    for index, (position_span, hold_velocity) in enumerate(
        zip(endpoint_position_spans, endpoint_max_velocities)
    ):
        dxl_id = index + 1
        if float(hold_velocity) > max_hold_velocity:
            issues.append(
                f"device ID {dxl_id} endpoint velocity "
                f"{float(hold_velocity):.6f} rad/s exceeds "
                f"{max_hold_velocity:.6f}"
            )
        if float(position_span) > max_hold_position_span:
            issues.append(
                f"device ID {dxl_id} endpoint position span "
                f"{float(position_span):.6f} rad exceeds "
                f"{max_hold_position_span:.6f}"
            )

    return {
        "target_id": target_id,
        "detected_id": detected_id,
        "result": "PASS" if not issues else "FAIL",
        "devices": devices,
        "issues": issues,
    }


def analyze_continuous_attempt(
    positions: Sequence[np.ndarray],
    velocities: Sequence[np.ndarray],
    joint_signs: Sequence[float],
    target_id: int,
    baseline_frames: int,
    settle_frames: int,
    endpoint_frames: int,
    raw_direction: int,
    boundary_rad: float,
    max_other_delta: float,
    max_abs_velocity: float,
    max_position_step: float,
    max_hold_velocity: float,
    max_hold_position_span: float,
    motion_reversal_tolerance: float,
) -> dict:
    """Evaluate a guided movement across one encoder revolution boundary."""
    if target_id not in EXPECTED_IDS[:-1]:
        raise ValueError("continuous target_id must be in 1..7")
    if raw_direction not in {-1, 1}:
        raise ValueError("raw_direction must be -1 or 1")
    if baseline_frames < 1 or settle_frames < 1 or endpoint_frames < 1:
        raise ValueError("continuous stable phase frame counts must be positive")
    movement_end = len(positions) - settle_frames - endpoint_frames
    if movement_end <= baseline_frames:
        raise ValueError("continuous attempt does not contain movement samples")
    if (
        not math.isfinite(motion_reversal_tolerance)
        or motion_reversal_tolerance < 0
    ):
        raise ValueError(
            "motion_reversal_tolerance must be finite and non-negative"
        )

    sample_analysis = analyze_samples(
        positions,
        velocities,
        joint_signs,
        max_abs_velocity,
        max_position_step,
    )
    position_array = np.asarray(positions, dtype=float)
    velocity_array = np.asarray(velocities, dtype=float)
    signs = np.asarray(joint_signs, dtype=float)
    baseline = np.median(position_array[:baseline_frames], axis=0)
    endpoint_positions = position_array[-endpoint_frames:]
    endpoint_velocities = velocity_array[-endpoint_frames:]
    endpoint = np.median(endpoint_positions, axis=0)
    raw_deltas = endpoint - baseline
    signed_deltas = raw_deltas * signs
    endpoint_position_spans = np.ptp(endpoint_positions, axis=0)
    endpoint_max_velocities = np.max(np.abs(endpoint_velocities), axis=0)
    detected_id = int(np.argmax(np.abs(raw_deltas))) + 1

    target_index = target_id - 1
    target_motion = position_array[
        baseline_frames - 1 : movement_end, target_index
    ]
    if raw_direction > 0:
        boundary_crossed = bool(
            baseline[target_index] < boundary_rad
            and np.max(target_motion) >= boundary_rad
        )
    else:
        boundary_crossed = bool(
            baseline[target_index] > boundary_rad
            and np.min(target_motion) <= boundary_rad
        )

    devices = []
    for index, sampled_device in enumerate(sample_analysis["devices"]):
        device = dict(sampled_device)
        device["position_delta_rad"] = float(raw_deltas[index])
        device["signed_position_delta_rad"] = float(signed_deltas[index])
        device["endpoint_position_span_rad"] = float(
            endpoint_position_spans[index]
        )
        device["endpoint_max_abs_velocity_rad_s"] = float(
            endpoint_max_velocities[index]
        )
        devices.append(device)

    issues = list(sample_analysis["issues"])
    if detected_id != target_id:
        issues.append(
            f"expected device ID {target_id} to move most, detected ID "
            f"{detected_id}"
        )
    if float(raw_deltas[target_index]) * raw_direction <= 0:
        issues.append(
            f"device ID {target_id} moved opposite the requested raw direction"
        )

    directed_steps = np.diff(target_motion) * raw_direction
    if len(directed_steps) and float(np.min(directed_steps)) < -float(
        motion_reversal_tolerance
    ):
        issues.append(
            f"device ID {target_id} reversed by more than "
            f"{motion_reversal_tolerance:.6f} rad during the crossing"
        )

    for index, delta in enumerate(signed_deltas):
        dxl_id = index + 1
        if dxl_id != target_id and abs(float(delta)) >= max_other_delta:
            issues.append(
                f"device ID {dxl_id} moved {float(delta):.6f} rad, "
                f"expected less than {max_other_delta:.6f} rad"
            )

    for index, (position_span, hold_velocity) in enumerate(
        zip(endpoint_position_spans, endpoint_max_velocities)
    ):
        dxl_id = index + 1
        if float(hold_velocity) > max_hold_velocity:
            issues.append(
                f"device ID {dxl_id} endpoint velocity "
                f"{float(hold_velocity):.6f} rad/s exceeds "
                f"{max_hold_velocity:.6f}"
            )
        if float(position_span) > max_hold_position_span:
            issues.append(
                f"device ID {dxl_id} endpoint position span "
                f"{float(position_span):.6f} rad exceeds "
                f"{max_hold_position_span:.6f}"
            )

    if issues:
        result = "FAIL"
    elif boundary_crossed:
        result = "PASS"
    else:
        result = "INCONCLUSIVE"
    return {
        "target_id": target_id,
        "detected_id": detected_id,
        "raw_direction": raw_direction,
        "boundary_rad": float(boundary_rad),
        "boundary_crossed": boundary_crossed,
        "result": result,
        "devices": devices,
        "issues": issues,
    }


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


def run_guided_check(
    *,
    config_path: Path,
    side: str,
    output_path: Path,
    baseline_frames: int = 50,
    movement_frames: int = 200,
    settle_frames: int = 50,
    endpoint_frames: int = 100,
    interval: float = DEFAULT_INTERVAL,
    min_target_delta: float = 0.05,
    max_other_delta: float = 0.03,
    max_abs_velocity: float = DEFAULT_MAX_ABS_VELOCITY,
    max_position_step: float = 0.2,
    max_hold_velocity: float = 0.05,
    max_hold_position_span: float = 0.01,
    driver_factory: Callable = DynamixelDriver,
    sleep: Callable[[float], None] = time.sleep,
    prompt: Callable[[str], str] = input,
    notify: Callable[[str], None] = print,
) -> dict:
    """Guide an operator through one positive movement for every device."""
    config = load_config(config_path)
    total_frames = (
        baseline_frames + movement_frames + settle_frames + endpoint_frames
    )
    _validate_arguments(
        config,
        side,
        total_frames,
        interval,
        output_path,
        max_abs_velocity,
        max_position_step,
    )
    if (
        baseline_frames < 1
        or movement_frames < 1
        or settle_frames < 1
        or endpoint_frames < 1
    ):
        raise ValueError("guided phase frame counts must be positive")
    if not math.isfinite(min_target_delta) or min_target_delta <= 0:
        raise ValueError("min_target_delta must be finite and positive")
    if not math.isfinite(max_other_delta) or max_other_delta <= 0:
        raise ValueError("max_other_delta must be finite and positive")
    if not math.isfinite(max_hold_velocity) or max_hold_velocity <= 0:
        raise ValueError("max_hold_velocity must be finite and positive")
    if (
        not math.isfinite(max_hold_position_span)
        or max_hold_position_span <= 0
    ):
        raise ValueError("max_hold_position_span must be finite and positive")

    expected_models = [
        MODEL_NUMBERS[normalize_servo_type(model)]
        for model in config.dynamixel.servo_types
    ]
    report = {
        "schema": "skye_factr_teleop.phase_2_5_1.guided.v1",
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
        "settings": {
            "baseline_frames": baseline_frames,
            "movement_frames": movement_frames,
            "settle_frames": settle_frames,
            "endpoint_frames": endpoint_frames,
            "interval": interval,
            "min_target_delta": min_target_delta,
            "max_other_delta": max_other_delta,
            "max_abs_velocity": max_abs_velocity,
            "max_position_step": max_position_step,
            "max_hold_velocity": max_hold_velocity,
            "max_hold_position_span": max_hold_position_span,
        },
        "attempts": [],
        "bus_result": "FAIL",
        "guided_result": "FAIL",
        "manual_checks": {
            "large_arm_motion_link_disconnected": "PENDING",
            "usb_side_mapping": "PENDING",
            "physical_id_order": "PENDING",
            "joint_direction": "PENDING",
            "continuous_angle": "PENDING",
            "single_device_disconnect": "PENDING",
        },
        "error": None,
    }

    driver = driver_factory(
        ids=EXPECTED_IDS,
        servo_types=config.dynamixel.servo_types,
        port=config.dynamixel.dynamixel_port,
        baudrate=config.arm_teleop.baudrate,
    )
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
                    f"{info.model_number}, expected model {expected_model} "
                    f"({device_record['configured_model']})"
                )
        report["bus_result"] = "PASS"

        for target_id in EXPECTED_IDS:
            target_name = (
                f"joint{target_id}_{side}"
                if target_id <= 7
                else "leader gripper"
            )
            direction = "URDF +" if target_id <= 7 else "open +"
            prompt(
                f"ID {target_id} ({target_name}): place it near mid-range, "
                f"prepare to move toward {direction}, then press Enter"
            )

            positions = []
            velocities = []
            samples = []
            attempt_started = time.monotonic()

            def collect(phase: str, count: int) -> None:
                for phase_index in range(count):
                    sample_positions, sample_velocities = driver.read_state()
                    sample_positions = np.asarray(sample_positions, dtype=float)
                    sample_velocities = np.asarray(sample_velocities, dtype=float)
                    positions.append(sample_positions)
                    velocities.append(sample_velocities)
                    samples.append(
                        {
                            "phase": phase,
                            "phase_index": phase_index,
                            "elapsed_s": time.monotonic() - attempt_started,
                            "positions_rad": sample_positions.tolist(),
                            "velocities_rad_s": sample_velocities.tolist(),
                        }
                    )
                    if phase_index + 1 < count and interval:
                        sleep(interval)

            notify(f"ID {target_id}: HOLD STILL (baseline)")
            collect("baseline", baseline_frames)
            notify(f"ID {target_id}: MOVE NOW toward {direction}")
            collect("movement", movement_frames)
            notify(f"ID {target_id}: STOP NOW")
            collect("settle", settle_frames)
            notify(f"ID {target_id}: HOLD STILL (endpoint)")
            collect("endpoint", endpoint_frames)

            analysis = analyze_guided_attempt(
                positions,
                velocities,
                config.dynamixel.joint_signs,
                target_id,
                baseline_frames,
                endpoint_frames,
                min_target_delta,
                max_other_delta,
                max_abs_velocity,
                max_position_step,
                max_hold_velocity,
                max_hold_position_span,
            )
            attempt = {
                "target_id": target_id,
                "target_name": target_name,
                "expected_direction": direction,
                "result": analysis["result"],
                "detected_id": analysis["detected_id"],
                "devices": analysis["devices"],
                "issues": analysis["issues"],
                "samples": samples,
            }
            report["attempts"].append(attempt)
            notify(
                f"ID {target_id}: {attempt['result']} "
                f"(detected ID {attempt['detected_id']})"
            )

        report["guided_result"] = (
            "PASS"
            if all(attempt["result"] == "PASS" for attempt in report["attempts"])
            else "FAIL"
        )
        report["manual_checks"]["physical_id_order"] = report[
            "guided_result"
        ]
        report["manual_checks"]["joint_direction"] = report["guided_result"]
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        try:
            driver.close()
        except Exception as exc:
            close_error = str(exc)

    if close_error:
        report["guided_result"] = "FAIL"
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


def _open_and_record_read_only_bus(driver, config: TeleopConfig, report: dict) -> None:
    """Open, validate, and record the common read-only bus metadata."""
    driver.open_read_only()
    report["resolved_port"] = driver.port
    report["latency_timer"] = driver.latency_timer
    if driver.latency_timer != 1:
        raise RuntimeError(
            f"FTDI latency timer is {driver.latency_timer!r}, expected 1"
        )

    expected_models = [
        MODEL_NUMBERS[normalize_servo_type(model)]
        for model in config.dynamixel.servo_types
    ]
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
                f"{info.model_number}, expected model {expected_model} "
                f"({device_record['configured_model']})"
            )
    report["bus_result"] = "PASS"


def run_guided_continuous_check(
    *,
    config_path: Path,
    side: str,
    target_ids: Sequence[int],
    output_path: Path,
    baseline_frames: int = 50,
    continuous_max_frames: int = 750,
    settle_frames: int = 50,
    endpoint_frames: int = 100,
    interval: float = DEFAULT_INTERVAL,
    max_other_delta: float = 0.03,
    max_abs_velocity: float = DEFAULT_MAX_ABS_VELOCITY,
    max_position_step: float = 0.2,
    max_hold_velocity: float = 0.05,
    max_hold_position_span: float = 0.01,
    motion_reversal_tolerance: float = 0.02,
    driver_factory: Callable = DynamixelDriver,
    sleep: Callable[[float], None] = time.sleep,
    prompt: Callable[[str], str] = input,
    notify: Callable[[str], None] = print,
) -> dict:
    """Guide safe joints across the nearest encoder revolution boundary."""
    config = load_config(config_path)
    target_ids = tuple(target_ids)
    total_frames = (
        baseline_frames
        + continuous_max_frames
        + settle_frames
        + endpoint_frames
    )
    _validate_arguments(
        config,
        side,
        total_frames,
        interval,
        output_path,
        max_abs_velocity,
        max_position_step,
    )
    if not target_ids:
        raise ValueError("continuous target_ids must not be empty")
    if len(set(target_ids)) != len(target_ids):
        raise ValueError("continuous target_ids must not contain duplicates")
    if any(target_id not in EXPECTED_IDS[:-1] for target_id in target_ids):
        raise ValueError("continuous target_ids must be in 1..7")
    if (
        baseline_frames < 1
        or continuous_max_frames < 1
        or settle_frames < 1
        or endpoint_frames < 1
    ):
        raise ValueError("continuous phase frame counts must be positive")
    if not math.isfinite(max_other_delta) or max_other_delta <= 0:
        raise ValueError("max_other_delta must be finite and positive")
    if not math.isfinite(max_hold_velocity) or max_hold_velocity <= 0:
        raise ValueError("max_hold_velocity must be finite and positive")
    if (
        not math.isfinite(max_hold_position_span)
        or max_hold_position_span <= 0
    ):
        raise ValueError("max_hold_position_span must be finite and positive")
    if (
        not math.isfinite(motion_reversal_tolerance)
        or motion_reversal_tolerance < 0
    ):
        raise ValueError(
            "motion_reversal_tolerance must be finite and non-negative"
        )

    report = {
        "schema": "skye_factr_teleop.phase_2_5_1.continuous.v1",
        "started_at": datetime.now()
        .astimezone()
        .isoformat(timespec="seconds"),
        "side": side,
        "config": str(Path(config_path).resolve()),
        "configured_port": config.dynamixel.dynamixel_port,
        "resolved_port": None,
        "ids": list(EXPECTED_IDS),
        "target_ids": list(target_ids),
        "baudrate": config.arm_teleop.baudrate,
        "latency_timer": None,
        "devices": [],
        "settings": {
            "baseline_frames": baseline_frames,
            "continuous_max_frames": continuous_max_frames,
            "settle_frames": settle_frames,
            "endpoint_frames": endpoint_frames,
            "interval": interval,
            "max_other_delta": max_other_delta,
            "max_abs_velocity": max_abs_velocity,
            "max_position_step": max_position_step,
            "max_hold_velocity": max_hold_velocity,
            "max_hold_position_span": max_hold_position_span,
            "motion_reversal_tolerance": motion_reversal_tolerance,
        },
        "attempts": [],
        "bus_result": "FAIL",
        "continuous_result": "FAIL",
        "manual_checks": {
            "large_arm_motion_link_disconnected": "PENDING",
            "usb_side_mapping": "PENDING",
            "physical_id_order": "PENDING",
            "joint_direction": "PENDING",
            "continuous_angle": "PENDING",
            "single_device_disconnect": "PENDING",
        },
        "error": None,
    }

    driver = driver_factory(
        ids=EXPECTED_IDS,
        servo_types=config.dynamixel.servo_types,
        port=config.dynamixel.dynamixel_port,
        baudrate=config.arm_teleop.baudrate,
    )
    close_error = None
    try:
        _open_and_record_read_only_bus(driver, config, report)
        for target_id in target_ids:
            target_name = f"joint{target_id}_{side}"
            prompt(
                f"ID {target_id} ({target_name}): place the joint at a safe "
                "starting position, hold still, then press Enter"
            )

            positions = []
            velocities = []
            samples = []
            attempt_started = time.monotonic()

            def collect(phase: str, count: int) -> None:
                for phase_index in range(count):
                    sample_positions, sample_velocities = driver.read_state()
                    sample_positions = np.asarray(sample_positions, dtype=float)
                    sample_velocities = np.asarray(sample_velocities, dtype=float)
                    positions.append(sample_positions)
                    velocities.append(sample_velocities)
                    samples.append(
                        {
                            "phase": phase,
                            "phase_index": phase_index,
                            "elapsed_s": time.monotonic() - attempt_started,
                            "positions_rad": sample_positions.tolist(),
                            "velocities_rad_s": sample_velocities.tolist(),
                        }
                    )
                    if phase_index + 1 < count and interval:
                        sleep(interval)

            notify(f"ID {target_id}: HOLD STILL (baseline)")
            collect("baseline", baseline_frames)
            target_index = target_id - 1
            start_position = float(
                np.median(
                    np.asarray(positions, dtype=float)[:, target_index]
                )
            )
            lower_boundary = math.floor(start_position / math.tau) * math.tau
            upper_boundary = lower_boundary + math.tau
            if start_position - lower_boundary <= upper_boundary - start_position:
                boundary_rad = lower_boundary
                raw_direction = -1
            else:
                boundary_rad = upper_boundary
                raw_direction = 1
            boundary_distance = abs(boundary_rad - start_position)
            urdf_direction = (
                "URDF +"
                if raw_direction
                * float(config.dynamixel.joint_signs[target_index])
                > 0
                else "URDF -"
            )
            response = prompt(
                f"ID {target_id}: current={start_position:.3f} rad, nearest "
                f"boundary={boundary_rad:.3f} rad, distance="
                f"{boundary_distance:.3f} rad, move toward {urdf_direction}. "
                "Press Enter to start, or type s:<reason> to skip"
            ).strip()
            if response.lower().startswith("s"):
                reason = (
                    response.split(":", 1)[1].strip()
                    if ":" in response
                    else ""
                )
                if not reason:
                    reason = prompt(
                        f"ID {target_id}: enter the mechanical or cable "
                        "limit reason"
                    ).strip()
                skip_result = "SKIPPED_UNSAFE" if reason else "INCONCLUSIVE"
                skip_issues = (
                    []
                    if reason
                    else ["unsafe skip requires a non-empty reason"]
                )
                report["attempts"].append(
                    {
                        "target_id": target_id,
                        "target_name": target_name,
                        "expected_direction": urdf_direction,
                        "raw_direction": raw_direction,
                        "start_position_rad": start_position,
                        "boundary_rad": boundary_rad,
                        "boundary_distance_rad": boundary_distance,
                        "boundary_crossed": False,
                        "termination": (
                            "operator_skip"
                            if reason
                            else "missing_skip_reason"
                        ),
                        "result": skip_result,
                        "skip_reason": reason or None,
                        "detected_id": None,
                        "devices": [],
                        "issues": skip_issues,
                        "samples": samples,
                    }
                )
                if reason:
                    notify(f"ID {target_id}: SKIPPED_UNSAFE ({reason})")
                else:
                    notify(f"ID {target_id}: INCONCLUSIVE (missing reason)")
                continue

            notify(f"ID {target_id}: MOVE NOW toward {urdf_direction}")
            previous_position = float(positions[-1][target_index])
            boundary_crossed = False
            termination = "timeout"
            for phase_index in range(continuous_max_frames):
                sample_positions, sample_velocities = driver.read_state()
                sample_positions = np.asarray(sample_positions, dtype=float)
                sample_velocities = np.asarray(sample_velocities, dtype=float)
                positions.append(sample_positions)
                velocities.append(sample_velocities)
                samples.append(
                    {
                        "phase": "movement",
                        "phase_index": phase_index,
                        "elapsed_s": time.monotonic() - attempt_started,
                        "positions_rad": sample_positions.tolist(),
                        "velocities_rad_s": sample_velocities.tolist(),
                    }
                )
                current_position = float(sample_positions[target_index])
                if abs(current_position - previous_position) > max_position_step:
                    termination = "position_discontinuity"
                    break
                if (
                    raw_direction > 0
                    and current_position >= boundary_rad
                ) or (
                    raw_direction < 0
                    and current_position <= boundary_rad
                ):
                    boundary_crossed = True
                    termination = "boundary_crossed"
                    break
                previous_position = current_position
                if phase_index + 1 < continuous_max_frames and interval:
                    sleep(interval)

            if boundary_crossed:
                notify(f"ID {target_id}: BOUNDARY CROSSED")
            elif termination == "position_discontinuity":
                notify(f"ID {target_id}: POSITION DISCONTINUITY DETECTED")
            else:
                notify(f"ID {target_id}: BOUNDARY NOT REACHED")
            notify(f"ID {target_id}: STOP NOW")
            collect("settle", settle_frames)
            notify(f"ID {target_id}: HOLD STILL (endpoint)")
            collect("endpoint", endpoint_frames)

            analysis = analyze_continuous_attempt(
                positions,
                velocities,
                config.dynamixel.joint_signs,
                target_id,
                baseline_frames,
                settle_frames,
                endpoint_frames,
                raw_direction,
                boundary_rad,
                max_other_delta,
                max_abs_velocity,
                max_position_step,
                max_hold_velocity,
                max_hold_position_span,
                motion_reversal_tolerance,
            )
            attempt = {
                "target_id": target_id,
                "target_name": target_name,
                "expected_direction": urdf_direction,
                "raw_direction": raw_direction,
                "start_position_rad": start_position,
                "boundary_rad": boundary_rad,
                "boundary_distance_rad": boundary_distance,
                "boundary_crossed": analysis["boundary_crossed"],
                "termination": termination,
                "result": analysis["result"],
                "skip_reason": None,
                "detected_id": analysis["detected_id"],
                "devices": analysis["devices"],
                "issues": analysis["issues"],
                "samples": samples,
            }
            report["attempts"].append(attempt)
            notify(f"ID {target_id}: {attempt['result']}")

        results = [attempt["result"] for attempt in report["attempts"]]
        if any(result == "FAIL" for result in results):
            report["continuous_result"] = "FAIL"
        elif any(result == "INCONCLUSIVE" for result in results):
            report["continuous_result"] = "INCONCLUSIVE"
        elif any(result == "SKIPPED_UNSAFE" for result in results):
            report["continuous_result"] = "PASS_WITH_SKIPS"
        else:
            report["continuous_result"] = "PASS"
        report["manual_checks"]["continuous_angle"] = report[
            "continuous_result"
        ]
    except Exception as exc:
        report["error"] = str(exc)
    finally:
        try:
            driver.close()
        except Exception as exc:
            close_error = str(exc)

    if close_error:
        report["continuous_result"] = "FAIL"
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
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--guided",
        action="store_true",
        help="interactively verify ID order and positive direction",
    )
    mode.add_argument(
        "--guided-continuous",
        action="store_true",
        help="interactively verify continuity across revolution boundaries",
    )
    parser.add_argument(
        "--ids",
        help="comma-separated target IDs for --guided-continuous",
    )
    parser.add_argument("--frames", type=int, default=DEFAULT_FRAMES)
    parser.add_argument("--baseline-frames", type=int, default=50)
    parser.add_argument("--movement-frames", type=int, default=200)
    parser.add_argument("--continuous-max-frames", type=int, default=750)
    parser.add_argument("--settle-frames", type=int, default=50)
    parser.add_argument("--endpoint-frames", type=int, default=100)
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL)
    parser.add_argument("--min-target-delta", type=float, default=0.05)
    parser.add_argument("--max-other-delta", type=float, default=0.03)
    parser.add_argument("--max-hold-velocity", type=float, default=0.05)
    parser.add_argument("--max-hold-position-span", type=float, default=0.01)
    parser.add_argument(
        "--motion-reversal-tolerance", type=float, default=0.02
    )
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
        if args.guided_continuous:
            if not args.ids:
                raise ValueError("--ids is required with --guided-continuous")
            try:
                target_ids = tuple(
                    int(value.strip()) for value in args.ids.split(",")
                )
            except ValueError as exc:
                raise ValueError(
                    "--ids must be a comma-separated list of integers"
                ) from exc
            report = run_guided_continuous_check(
                config_path=args.config,
                side=args.side,
                target_ids=target_ids,
                output_path=args.output,
                baseline_frames=args.baseline_frames,
                continuous_max_frames=args.continuous_max_frames,
                settle_frames=args.settle_frames,
                endpoint_frames=args.endpoint_frames,
                interval=args.interval,
                max_other_delta=args.max_other_delta,
                max_abs_velocity=args.max_abs_velocity,
                max_position_step=args.max_position_step,
                max_hold_velocity=args.max_hold_velocity,
                max_hold_position_span=args.max_hold_position_span,
                motion_reversal_tolerance=args.motion_reversal_tolerance,
            )
        elif args.guided:
            report = run_guided_check(
                config_path=args.config,
                side=args.side,
                output_path=args.output,
                baseline_frames=args.baseline_frames,
                movement_frames=args.movement_frames,
                settle_frames=args.settle_frames,
                endpoint_frames=args.endpoint_frames,
                interval=args.interval,
                min_target_delta=args.min_target_delta,
                max_other_delta=args.max_other_delta,
                max_abs_velocity=args.max_abs_velocity,
                max_position_step=args.max_position_step,
                max_hold_velocity=args.max_hold_velocity,
                max_hold_position_span=args.max_hold_position_span,
            )
        else:
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
    if args.guided_continuous:
        print(f"bus result: {report['bus_result']}")
        print(f"continuous result: {report['continuous_result']}")
        for attempt in report["attempts"]:
            if attempt["result"] == "SKIPPED_UNSAFE":
                print(
                    f"ID {attempt['target_id']}: SKIPPED_UNSAFE "
                    f"({attempt['skip_reason']})"
                )
            else:
                crossed = "yes" if attempt["boundary_crossed"] else "no"
                print(
                    f"ID {attempt['target_id']}: {attempt['result']} "
                    f"(crossed: {crossed})"
                )
                for issue in attempt.get("issues", ()):
                    print(f"  - {issue}")
        if report["error"]:
            print(f"error: {report['error']}", file=sys.stderr)
        return (
            0
            if report["continuous_result"] in {"PASS", "PASS_WITH_SKIPS"}
            else 1
        )

    if args.guided:
        print(f"bus result: {report['bus_result']}")
        print(f"guided result: {report['guided_result']}")
        for attempt in report["attempts"]:
            print(
                f"ID {attempt['target_id']}: {attempt['result']} "
                f"(detected ID {attempt['detected_id']})"
            )
            for issue in attempt.get("issues", ()):
                print(f"  - {issue}")
        if report["error"]:
            print(f"error: {report['error']}", file=sys.stderr)
        return 0 if report["guided_result"] == "PASS" else 1

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
