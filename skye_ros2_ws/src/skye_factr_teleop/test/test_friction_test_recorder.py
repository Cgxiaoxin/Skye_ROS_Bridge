# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

from skye_factr_teleop.friction_test_recorder import FrictionTrial, finalize_friction_record


def _trial(enabled):
    trial = FrictionTrial(2, 20, enabled, 0.05)
    trial.state("IDLE", 0)
    trial.position([0] * 7, [0] * 7)
    trial.state("FRICTION_TEST", 1)
    trial.position([0] * 7, [0] * 7)
    trial.current([0, 5 if enabled else 0, 0, 0, 0, 0, 0, 0])
    trial.current([0, -5 if enabled else 0, 0, 0, 0, 0, 0, 0])
    trial.position([0] * 7, [0, 0.05 if enabled else 0, 0, 0, 0, 0, 0])
    trial.present_current([0] * 8)
    trial.state("IDLE", 2)
    trial.current([0] * 8)
    return trial


def _record(enabled):
    summary = _trial(enabled).summary(
        effect="less" if enabled else None,
        threshold_observed="yes" if enabled else None,
        motion="none", end_method="timeout", note="",
    )
    return {"side": "left", "joint_id": 2, "enabled": enabled,
            "config_path": "/config.yaml", "usb_by_id": "usb-left",
            "servo_model": "XM430-W350", "urdf": "m6left.urdf",
            "joint_offsets": [0.0] * 8, "joint_signs": [1] * 8,
            "configured_current_limit_ma": [50.0] * 8,
            "gravity_mode": "position_only",
            "gravity_gain": 0.2, "friction_gain": 0.15,
            "enable_speed": 0.05, "gain_scale": 0.1,
            "current_limit_ma": 20, "ramp_seconds": 2,
            "max_duration_seconds": 5, **summary}


def _readback():
    return {"side": "left", "automated_result": "PASS",
            "devices": [{"id": i, "torque_enabled": False} for i in range(1, 9)]}


def test_disabled_then_enabled_pair_can_pass_after_readback():
    baseline = finalize_friction_record(
        _record(False), _readback(), process_exited_normally=True,
        readback_path="baseline_readback.json",
    )
    assert baseline["assessment"] == "PASS"
    enabled = finalize_friction_record(
        _record(True), _readback(), process_exited_normally=True,
        readback_path="enabled_readback.json", baseline=baseline,
        baseline_path="baseline.json",
    )
    assert enabled["assessment"] == "PASS"
    assert enabled["baseline_valid"]


def test_missing_or_mismatched_baseline_is_incomplete():
    record = _record(True)
    missing = finalize_friction_record(
        record, _readback(), process_exited_normally=True, readback_path="readback.json")
    assert missing["assessment"] == "INCOMPLETE"
    baseline = _record(False)
    baseline["assessment"] = "PASS"
    baseline["gravity_gain"] = 0.5
    mismatched = finalize_friction_record(
        record, _readback(), process_exited_normally=True,
        readback_path="readback.json", baseline=baseline)
    assert mismatched["assessment"] == "INCOMPLETE"


def test_current_or_motion_violation_fails():
    trial = _trial(True)
    trial.phase = "testing"
    trial.current([1, 25, 0, 0, 0, 0, 0, 0])
    trial.phase = "finished"
    result = trial.summary(effect="less", threshold_observed="yes",
                           motion="none", end_method="timeout", note="")
    assert result["assessment"] == "FAIL"
    sudden = _trial(True).summary(effect="less", threshold_observed="yes",
                                  motion="sudden", end_method="timeout", note="")
    assert sudden["assessment"] == "FAIL"


def test_enabled_run_without_recorded_threshold_crossing_is_not_passable():
    trial = _trial(True)
    trial.above_threshold_samples = 0
    result = trial.summary(effect="less", threshold_observed="yes",
                           motion="none", end_method="timeout", note="")
    assert result["assessment"] == "INCOMPLETE"


def test_observed_nonzero_output_above_threshold_fails():
    result = _trial(True).summary(effect="less", threshold_observed="no",
                                  motion="none", end_method="timeout", note="")
    assert result["assessment"] == "FAIL"
