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
