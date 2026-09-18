# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Offline tests for the phase-2.5 read-only acceptance tool."""

import json
from pathlib import Path

import numpy as np
import pytest

from skye_factr_teleop import readonly_check
from skye_factr_teleop.dynamixel_driver import DynamixelDeviceInfo


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LEFT_CONFIG = PACKAGE_ROOT / "config" / "grav_comp_m6_left.yaml"


class FakeReadOnlyDriver:
    instances = []

    def __init__(self, ids, servo_types, port, baudrate):
        self.ids = tuple(ids)
        self.servo_types = tuple(servo_types)
        self.port = f"/dev/serial/by-id/{port}"
        self.baudrate = baudrate
        self.latency_timer = 1
        self.device_info = tuple(
            DynamixelDeviceInfo(
                device_id=dxl_id,
                model_number=1020 if dxl_id <= 2 else 1220,
                return_delay_time=0,
                torque_enabled=False,
            )
            for dxl_id in ids
        )
        self.opened = False
        self.closed = False
        self.sample_index = 0
        self.instances.append(self)

    def open_read_only(self):
        self.opened = True

    def read_state(self):
        self.sample_index += 1
        positions = np.arange(8, dtype=float) + self.sample_index * 0.01
        velocities = np.full(8, 0.1, dtype=float)
        return positions, velocities

    def close(self):
        self.closed = True


def test_run_check_records_device_info_samples_and_signed_motion(tmp_path):
    FakeReadOnlyDriver.instances.clear()
    output = tmp_path / "left.json"

    report = readonly_check.run_readonly_check(
        config_path=LEFT_CONFIG,
        side="left",
        frames=3,
        interval=0.0,
        output_path=output,
        driver_factory=FakeReadOnlyDriver,
        sleep=lambda _: None,
    )

    assert report["automated_result"] == "PASS"
    assert report["ids"] == list(range(1, 9))
    assert report["baudrate"] == 4_000_000
    assert report["latency_timer"] == 1
    assert len(report["samples"]) == 3
    assert report["analysis"]["devices"][0][
        "signed_position_delta_rad"
    ] == pytest.approx(0.02)
    assert report["manual_checks"]["usb_side_mapping"] == "PENDING"
    assert json.loads(output.read_text(encoding="utf-8")) == report
    assert FakeReadOnlyDriver.instances[0].opened is True
    assert FakeReadOnlyDriver.instances[0].closed is True


def test_run_check_fails_when_ping_model_does_not_match_config(tmp_path):
    class WrongModelDriver(FakeReadOnlyDriver):
        def open_read_only(self):
            super().open_read_only()
            first = self.device_info[0]
            self.device_info = (
                DynamixelDeviceInfo(
                    first.device_id,
                    1220,
                    first.return_delay_time,
                    first.torque_enabled,
                ),
                *self.device_info[1:],
            )

    output = tmp_path / "wrong-model.json"

    report = readonly_check.run_readonly_check(
        config_path=LEFT_CONFIG,
        side="left",
        frames=2,
        interval=0.0,
        output_path=output,
        driver_factory=WrongModelDriver,
        sleep=lambda _: None,
    )

    assert report["automated_result"] == "FAIL"
    assert "device ID 1" in report["error"]
    assert "expected model" in report["error"]
    assert WrongModelDriver.instances[-1].closed is True


def test_analyze_samples_flags_velocity_and_position_discontinuity():
    positions = [np.zeros(8), np.array([2.0] + [0.0] * 7)]
    velocities = [np.zeros(8), np.array([6.0] + [0.0] * 7)]

    analysis = readonly_check.analyze_samples(
        positions,
        velocities,
        joint_signs=(1.0,) * 8,
        max_abs_velocity=5.0,
        max_position_step=1.0,
    )

    assert analysis["issues"] == [
        "device ID 1 absolute velocity 6.000000 rad/s exceeds 5.000000",
        "device ID 1 position step 2.000000 rad exceeds 1.000000",
    ]


def test_analyze_guided_attempt_accepts_unique_positive_target_motion():
    positions = [
        np.zeros(8),
        np.zeros(8),
        np.array([0.04] + [0.0] * 7),
        np.array([0.08] + [0.0] * 7),
        np.array([0.10] + [0.0] * 7),
        np.array([0.10] + [0.0] * 7),
    ]
    velocities = [np.zeros(8) for _ in positions]

    result = readonly_check.analyze_guided_attempt(
        positions,
        velocities,
        joint_signs=(1.0,) * 8,
        target_id=1,
        baseline_frames=2,
        endpoint_frames=2,
        min_target_delta=0.05,
        max_other_delta=0.03,
        max_abs_velocity=20.0,
        max_position_step=0.2,
    )

    assert result["result"] == "PASS"
    assert result["detected_id"] == 1
    assert result["devices"][0]["signed_position_delta_rad"] == pytest.approx(
        0.10
    )
    assert result["issues"] == []


def test_analyze_guided_attempt_rejects_moving_endpoint_window():
    positions = [
        np.zeros(8),
        np.zeros(8),
        np.array([0.04] + [0.0] * 7),
        np.array([0.08] + [0.0] * 7),
        np.array([0.09] + [0.0] * 7),
        np.array([0.10] + [0.0] * 7),
        np.array([0.11] + [0.0] * 7),
        np.array([0.12] + [0.0] * 7),
    ]
    velocities = [np.zeros(8) for _ in positions]
    for index in range(5, 8):
        velocities[index][0] = 0.08

    result = readonly_check.analyze_guided_attempt(
        positions,
        velocities,
        joint_signs=(1.0,) * 8,
        target_id=1,
        baseline_frames=2,
        endpoint_frames=3,
        min_target_delta=0.05,
        max_other_delta=0.03,
        max_abs_velocity=20.0,
        max_position_step=0.2,
        max_hold_velocity=0.05,
        max_hold_position_span=0.01,
    )

    assert result["result"] == "FAIL"
    assert any("endpoint velocity" in issue for issue in result["issues"])
    assert any("endpoint position span" in issue for issue in result["issues"])


def test_run_guided_check_records_one_positive_attempt_per_device(tmp_path):
    class GuidedDriver(FakeReadOnlyDriver):
        signs = np.array([1, 1, 1, 1, 1, -1, -1, 1], dtype=float)

        def read_state(self):
            attempt_index, phase = divmod(self.sample_index, 4)
            positions = np.zeros(8, dtype=float)
            if attempt_index:
                positions[:attempt_index] = 0.1 * self.signs[:attempt_index]
            if phase >= 1:
                positions[attempt_index] = 0.1 * self.signs[attempt_index]
            self.sample_index += 1
            return positions, np.zeros(8, dtype=float)

    output = tmp_path / "left-guided.json"
    prompts = []
    messages = []

    report = readonly_check.run_guided_check(
        config_path=LEFT_CONFIG,
        side="left",
        output_path=output,
        baseline_frames=1,
        movement_frames=1,
        settle_frames=1,
        endpoint_frames=1,
        interval=0.0,
        min_target_delta=0.05,
        max_other_delta=0.03,
        max_abs_velocity=20.0,
        max_position_step=0.2,
        driver_factory=GuidedDriver,
        sleep=lambda _: None,
        prompt=lambda message: prompts.append(message) or "",
        notify=messages.append,
    )

    assert report["guided_result"] == "PASS"
    assert [attempt["target_id"] for attempt in report["attempts"]] == list(
        range(1, 9)
    )
    assert all(attempt["result"] == "PASS" for attempt in report["attempts"])
    assert [
        sample["phase"] for sample in report["attempts"][0]["samples"]
    ] == ["baseline", "movement", "settle", "endpoint"]
    assert len(prompts) == 8
    assert json.loads(output.read_text(encoding="utf-8")) == report
    assert GuidedDriver.instances[-1].closed is True


def test_main_guided_mode_prints_compact_summary(monkeypatch, tmp_path, capsys):
    output = tmp_path / "guided.json"
    called = {}

    def fake_run_guided_check(**kwargs):
        called.update(kwargs)
        return {
            "configured_port": "left-port",
            "resolved_port": "/dev/ttyUSB0",
            "bus_result": "PASS",
            "guided_result": "PASS",
            "attempts": [
                {"target_id": dxl_id, "result": "PASS", "detected_id": dxl_id}
                for dxl_id in range(1, 9)
            ],
            "error": None,
        }

    monkeypatch.setattr(
        readonly_check, "run_guided_check", fake_run_guided_check
    )

    exit_code = readonly_check.main(
        [
            "--guided",
            "--side",
            "left",
            "--config",
            str(LEFT_CONFIG),
            "--max-position-step",
            "0.2",
            "--output",
            str(output),
        ]
    )

    assert exit_code == 0
    assert called["side"] == "left"
    assert called["baseline_frames"] == 50
    assert called["movement_frames"] == 200
    assert called["settle_frames"] == 50
    assert called["endpoint_frames"] == 100
    assert called["max_hold_velocity"] == 0.05
    assert called["max_hold_position_span"] == 0.01
    stdout = capsys.readouterr().out
    assert "bus result: PASS" in stdout
    assert "guided result: PASS" in stdout
    assert "ID 8: PASS (detected ID 8)" in stdout


def test_analyze_continuous_attempt_passes_real_boundary_crossing():
    tau = 2.0 * np.pi
    target_positions = [6.20, 6.20, 6.24, 6.28, 6.32, 6.33, 6.34, 6.34]
    positions = [
        np.array([target] + [0.0] * 7) for target in target_positions
    ]
    velocities = [np.zeros(8) for _ in positions]
    for index in range(2, 6):
        velocities[index][0] = 0.1

    result = readonly_check.analyze_continuous_attempt(
        positions,
        velocities,
        joint_signs=(1.0,) * 8,
        target_id=1,
        baseline_frames=2,
        settle_frames=1,
        endpoint_frames=2,
        raw_direction=1,
        boundary_rad=tau,
        max_other_delta=0.03,
        max_abs_velocity=20.0,
        max_position_step=0.2,
        max_hold_velocity=0.05,
        max_hold_position_span=0.01,
        motion_reversal_tolerance=0.02,
    )

    assert result["result"] == "PASS"
    assert result["boundary_crossed"] is True
    assert result["detected_id"] == 1
    assert result["issues"] == []


def test_run_guided_continuous_check_stops_after_detecting_boundary(tmp_path):
    class ContinuousDriver(FakeReadOnlyDriver):
        target_positions = iter([6.20, 6.24, 6.30, 6.31, 6.32, 6.32])

        def read_state(self):
            positions = np.zeros(8, dtype=float)
            positions[0] = next(self.target_positions)
            return positions, np.zeros(8, dtype=float)

    output = tmp_path / "left-continuous.json"
    responses = iter(["", ""])
    messages = []

    report = readonly_check.run_guided_continuous_check(
        config_path=LEFT_CONFIG,
        side="left",
        target_ids=(1,),
        output_path=output,
        baseline_frames=1,
        continuous_max_frames=10,
        settle_frames=1,
        endpoint_frames=2,
        interval=0.0,
        max_other_delta=0.03,
        max_abs_velocity=20.0,
        max_position_step=0.2,
        max_hold_velocity=0.05,
        max_hold_position_span=0.01,
        motion_reversal_tolerance=0.02,
        driver_factory=ContinuousDriver,
        sleep=lambda _: None,
        prompt=lambda message: next(responses),
        notify=messages.append,
    )

    assert report["continuous_result"] == "PASS"
    attempt = report["attempts"][0]
    assert attempt["result"] == "PASS"
    assert attempt["boundary_crossed"] is True
    assert attempt["boundary_rad"] == pytest.approx(2.0 * np.pi)
    assert attempt["expected_direction"] == "URDF +"
    assert [sample["phase"] for sample in attempt["samples"]] == [
        "baseline",
        "movement",
        "movement",
        "settle",
        "endpoint",
        "endpoint",
    ]
    assert any("BOUNDARY CROSSED" in message for message in messages)
    assert json.loads(output.read_text(encoding="utf-8")) == report
    assert ContinuousDriver.instances[-1].closed is True


def test_analyze_continuous_attempt_rejects_full_revolution_jump():
    target_positions = [6.20, 6.20, 6.25, 6.27, 0.01, 0.02, 0.02, 0.02]
    positions = [
        np.array([target] + [0.0] * 7) for target in target_positions
    ]
    velocities = [np.zeros(8) for _ in positions]

    result = readonly_check.analyze_continuous_attempt(
        positions,
        velocities,
        joint_signs=(1.0,) * 8,
        target_id=1,
        baseline_frames=2,
        settle_frames=1,
        endpoint_frames=2,
        raw_direction=1,
        boundary_rad=2.0 * np.pi,
        max_other_delta=0.03,
        max_abs_velocity=20.0,
        max_position_step=0.2,
        max_hold_velocity=0.05,
        max_hold_position_span=0.01,
        motion_reversal_tolerance=0.02,
    )

    assert result["result"] == "FAIL"
    assert result["boundary_crossed"] is False
    assert any("position step" in issue for issue in result["issues"])


def test_analyze_continuous_attempt_is_inconclusive_without_crossing():
    target_positions = [6.20, 6.20, 6.22, 6.25, 6.27, 6.27, 6.27, 6.27]
    positions = [
        np.array([target] + [0.0] * 7) for target in target_positions
    ]
    velocities = [np.zeros(8) for _ in positions]

    result = readonly_check.analyze_continuous_attempt(
        positions,
        velocities,
        joint_signs=(1.0,) * 8,
        target_id=1,
        baseline_frames=2,
        settle_frames=1,
        endpoint_frames=2,
        raw_direction=1,
        boundary_rad=2.0 * np.pi,
        max_other_delta=0.03,
        max_abs_velocity=20.0,
        max_position_step=0.2,
        max_hold_velocity=0.05,
        max_hold_position_span=0.01,
        motion_reversal_tolerance=0.02,
    )

    assert result["result"] == "INCONCLUSIVE"
    assert result["boundary_crossed"] is False
    assert result["issues"] == []


def test_main_guided_continuous_mode_parses_ids_and_prints_summary(
    monkeypatch, tmp_path, capsys
):
    output = tmp_path / "continuous.json"
    called = {}

    def fake_run_guided_continuous_check(**kwargs):
        called.update(kwargs)
        return {
            "configured_port": "left-port",
            "resolved_port": "/dev/ttyUSB0",
            "bus_result": "PASS",
            "continuous_result": "PASS_WITH_SKIPS",
            "attempts": [
                {"target_id": 1, "result": "PASS", "boundary_crossed": True},
                {
                    "target_id": 6,
                    "result": "SKIPPED_UNSAFE",
                    "boundary_crossed": False,
                    "skip_reason": "cable limit",
                },
            ],
            "error": None,
        }

    monkeypatch.setattr(
        readonly_check,
        "run_guided_continuous_check",
        fake_run_guided_continuous_check,
    )

    exit_code = readonly_check.main(
        [
            "--guided-continuous",
            "--ids",
            "1,6",
            "--side",
            "left",
            "--config",
            str(LEFT_CONFIG),
            "--max-position-step",
            "0.2",
            "--output",
            str(output),
        ]
    )

    assert exit_code == 0
    assert called["target_ids"] == (1, 6)
    assert called["continuous_max_frames"] == 750
    stdout = capsys.readouterr().out
    assert "continuous result: PASS_WITH_SKIPS" in stdout
    assert "ID 1: PASS (crossed: yes)" in stdout
    assert "ID 6: SKIPPED_UNSAFE (cable limit)" in stdout


def test_run_guided_continuous_check_records_unsafe_skip(tmp_path):
    class SkipDriver(FakeReadOnlyDriver):
        def read_state(self):
            return np.ones(8, dtype=float), np.zeros(8, dtype=float)

    output = tmp_path / "left-continuous-skip.json"
    responses = iter(["", "s:cable limit"])

    report = readonly_check.run_guided_continuous_check(
        config_path=LEFT_CONFIG,
        side="left",
        target_ids=(6,),
        output_path=output,
        baseline_frames=1,
        continuous_max_frames=10,
        settle_frames=1,
        endpoint_frames=1,
        interval=0.0,
        driver_factory=SkipDriver,
        sleep=lambda _: None,
        prompt=lambda message: next(responses),
        notify=lambda message: None,
    )

    assert report["continuous_result"] == "PASS_WITH_SKIPS"
    attempt = report["attempts"][0]
    assert attempt["result"] == "SKIPPED_UNSAFE"
    assert attempt["skip_reason"] == "cable limit"
    assert attempt["raw_direction"] == -1
    assert attempt["expected_direction"] == "URDF +"
    assert attempt["boundary_crossed"] is False
    assert json.loads(output.read_text(encoding="utf-8")) == report


def test_run_guided_continuous_check_requires_unsafe_skip_reason(tmp_path):
    class SkipDriver(FakeReadOnlyDriver):
        def read_state(self):
            return np.ones(8, dtype=float), np.zeros(8, dtype=float)

    responses = iter(["", "s", ""])
    report = readonly_check.run_guided_continuous_check(
        config_path=LEFT_CONFIG,
        side="left",
        target_ids=(6,),
        output_path=tmp_path / "missing-reason.json",
        baseline_frames=1,
        continuous_max_frames=10,
        settle_frames=1,
        endpoint_frames=1,
        interval=0.0,
        driver_factory=SkipDriver,
        sleep=lambda _: None,
        prompt=lambda message: next(responses),
        notify=lambda message: None,
    )

    assert report["continuous_result"] == "INCONCLUSIVE"
    attempt = report["attempts"][0]
    assert attempt["result"] == "INCONCLUSIVE"
    assert attempt["issues"] == ["unsafe skip requires a non-empty reason"]
