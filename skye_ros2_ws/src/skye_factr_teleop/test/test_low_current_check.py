# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Hardware-independent tests for the phase-2.5.2 smoke utility."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from skye_factr_teleop import low_current_check


PACKAGE_ROOT = Path(__file__).parents[1]
LEFT_CONFIG = PACKAGE_ROOT / "config" / "grav_comp_m6_left.yaml"
RIGHT_CONFIG = PACKAGE_ROOT / "config" / "grav_comp_m6_right.yaml"


class FakeDriver:
    def __init__(self, *, mode, events, positions=None, write_error=None):
        self.mode = mode
        self.events = events
        self.positions = iter(positions or ())
        self.write_error = write_error
        self.port = "/dev/serial/by-id/fake"
        self.latency_timer = 1
        self.device_info = ()
        self.torque_enabled = False

    def open_read_only(self):
        self.events.append((self.mode, "open_read_only"))
        self.device_info = (
            SimpleNamespace(
                device_id=1,
                model_number=1020,
                return_delay_time=0,
                torque_enabled=False,
            ),
        )

    def open(self):
        self.events.append((self.mode, "open"))
        self.torque_enabled = True

    def read_state(self):
        position = next(self.positions)
        self.events.append((self.mode, "read", position))
        return np.asarray([position]), np.asarray([0.0])

    def write_currents(self, currents):
        value = float(currents[0])
        self.events.append((self.mode, "write", value))
        if self.write_error is not None and value > 0:
            raise self.write_error

    def close(self):
        self.events.append((self.mode, "close"))
        self.torque_enabled = False


def _factory(events, *, write_error=None):
    calls = 0

    def factory(**kwargs):
        nonlocal calls
        calls += 1
        events.append(("factory", calls, kwargs))
        if calls == 1:
            return FakeDriver(mode="preflight", events=events)
        return FakeDriver(
            mode="writable",
            events=events,
            positions=(0.0, 0.02, 0.0),
            write_error=write_error,
        )

    return factory


def test_prepare_only_calculates_xm_current_without_driver(tmp_path):
    output = tmp_path / "prepared.json"

    def forbidden_factory(**kwargs):
        raise AssertionError("prepare-only must not construct a driver")

    report = low_current_check.run_low_current_check(
        config_path=LEFT_CONFIG,
        side="left",
        device_id=1,
        current_limit_ma=50.0,
        test_current_ma=20.0,
        pulse_seconds=0.5,
        settle_seconds=0.1,
        min_direction_delta_rad=0.001,
        output_path=output,
        prepare_only=True,
        driver_factory=forbidden_factory,
    )

    assert report["result"] == "PREPARED"
    assert report["device"]["configured_model"] == "XM430-W350"
    assert report["limits"]["smoke_ceiling_ma"] == pytest.approx(100.0)
    assert report["commands"]["positive"]["raw"] == 7
    assert report["commands"]["negative"]["encoded"] == [249, 255]
    assert json.loads(output.read_text())["result"] == "PREPARED"


def test_xc_smoke_ceiling_rejects_excessive_operator_limit(tmp_path):
    with pytest.raises(ValueError, match="smoke ceiling"):
        low_current_check.run_low_current_check(
            config_path=RIGHT_CONFIG,
            side="right",
            device_id=3,
            current_limit_ma=100.0,
            test_current_ma=20.0,
            pulse_seconds=0.5,
            settle_seconds=0.1,
            min_direction_delta_rad=0.001,
            output_path=tmp_path / "unused.json",
            prepare_only=True,
        )


def test_operator_limit_applies_after_xm_register_rounding(tmp_path):
    with pytest.raises(ValueError, match="rounded test current"):
        low_current_check.run_low_current_check(
            config_path=LEFT_CONFIG,
            side="left",
            device_id=1,
            current_limit_ma=2.0,
            test_current_ma=2.0,
            pulse_seconds=0.5,
            settle_seconds=0.1,
            min_direction_delta_rad=0.001,
            output_path=tmp_path / "unused.json",
            prepare_only=True,
        )


def test_hardware_path_uses_one_id_and_positive_zero_negative_zero(tmp_path):
    events = []
    report = low_current_check.run_low_current_check(
        config_path=LEFT_CONFIG,
        side="left",
        device_id=1,
        current_limit_ma=50.0,
        test_current_ma=20.0,
        pulse_seconds=0.5,
        settle_seconds=0.1,
        min_direction_delta_rad=0.001,
        output_path=tmp_path / "result.json",
        prepare_only=False,
        driver_factory=_factory(events),
        sleep=lambda _: None,
    )

    writes = [
        event for event in events if event[:2] == ("writable", "write")
    ]
    assert writes == [
        ("writable", "write", 20.0),
        ("writable", "write", 0.0),
        ("writable", "write", -20.0),
        ("writable", "write", 0.0),
    ]
    factory_events = [event for event in events if event[0] == "factory"]
    assert all(event[2]["ids"] == (1,) for event in factory_events)
    assert report["direction"]["result"] == "PASS"
    assert report["shutdown"]["result"] == "PASS"
    assert report["result"] == "PASS"


def test_communication_failure_still_closes_and_records_failure(tmp_path):
    events = []
    report = low_current_check.run_low_current_check(
        config_path=LEFT_CONFIG,
        side="left",
        device_id=1,
        current_limit_ma=50.0,
        test_current_ma=20.0,
        pulse_seconds=0.5,
        settle_seconds=0.1,
        min_direction_delta_rad=0.001,
        output_path=tmp_path / "failed.json",
        prepare_only=False,
        driver_factory=_factory(
            events, write_error=RuntimeError("simulated communication failure")
        ),
        sleep=lambda _: None,
    )

    assert ("writable", "close") in events
    assert report["result"] == "FAIL"
    assert report["shutdown"]["result"] == "PASS"
    assert "communication failure" in report["error"]


def test_catchable_termination_still_closes_and_marks_interrupted(tmp_path):
    events = []
    report = low_current_check.run_low_current_check(
        config_path=LEFT_CONFIG,
        side="left",
        device_id=1,
        current_limit_ma=50.0,
        test_current_ma=20.0,
        pulse_seconds=0.5,
        settle_seconds=0.1,
        min_direction_delta_rad=0.001,
        output_path=tmp_path / "interrupted.json",
        prepare_only=False,
        driver_factory=_factory(
            events, write_error=low_current_check.TerminationRequested(15)
        ),
        sleep=lambda _: None,
    )

    assert ("writable", "close") in events
    assert report["interrupted"] is True
    assert report["shutdown"]["result"] == "PASS"
    assert report["result"] == "FAIL"
