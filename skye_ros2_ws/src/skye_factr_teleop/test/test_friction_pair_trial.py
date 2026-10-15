# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

from skye_factr_teleop.friction_pair_trial import FrictionPairTrial
from skye_factr_teleop.friction_test_recorder import finalize_friction_record


def _trial(*, on_start=0.4):
    trial = FrictionPairTrial(2, 20, 0.05)
    trial.state("IDLE", 0)
    trial.position([0] * 7, [0] * 7)
    trial.state("FRICTION_TEST", 1)
    trial.phase_event("off", 1)
    trial.position([0] * 7, [0] * 7)
    trial.current([0] * 8)
    trial.present_current([0] * 8)
    trial.position([0, 0.4, 0, 0, 0, 0, 0], [0] * 7)
    trial.phase_event("on", 2)
    trial.current([0, 5, 0, 0, 0, 0, 0, 0])
    trial.current([0, -5, 0, 0, 0, 0, 0, 0])
    trial.present_current([0] * 8)
    trial.position([0, on_start, 0, 0, 0, 0, 0], [0, 0.05, 0, 0, 0, 0, 0])
    trial.state("IDLE", 3)
    trial.phase_event("idle", 3)
    trial.current([0] * 8)
    return trial


def _summary(trial):
    return trial.summary(effect="less", threshold_observed="yes",
                         motion="none", end_method="timeout", note="")


def test_pair_collects_off_and_on_in_one_record():
    result = _summary(_trial())
    assert result["assessment"] == "PENDING_SHUTDOWN_CHECK"
    assert result["baseline"]["positive_peak_command_ma"] == 0
    assert result["active"]["positive_peak_command_ma"] == 5
    assert result["active"]["negative_peak_command_ma"] == -5
    assert result["baseline_within_pose"]
    assert result["baseline"]["start_position_rad"][1] == 0
    assert result["switch_off_position_rad"][1] == 0.4
    assert result["switch_pose_max_difference_rad"] == 0
    readback = {"side": "left", "automated_result": "PASS",
                "devices": [{"id": i, "torque_enabled": False} for i in range(1, 9)]}
    record = {"paired": True, "enabled": True, "side": "left", **result}
    final = finalize_friction_record(record, readback, process_exited_normally=True,
                                     readback_path="readback.json")
    assert final["assessment"] == "PASS"


def test_pair_rejects_nonzero_baseline_and_missing_on_phase():
    trial = _trial()
    trial.off.positive_peak_ma = 1
    assert _summary(trial)["assessment"] == "FAIL"
    trial = _trial()
    trial.phases = [item for item in trial.phases if item["phase"] != "on"]
    assert _summary(trial)["assessment"] == "INCOMPLETE"


def test_pair_requires_pose_to_stay_near_first_off_to_on_switch():
    result = _summary(_trial(on_start=0.6))
    assert result["assessment"] == "INCOMPLETE"
    assert not result["baseline_within_pose"]
    assert abs(result["switch_pose_max_difference_rad"] - 0.2) < 1e-9


def test_phase_boundary_current_is_kept_in_trace_but_not_phase_assessment():
    trial = FrictionPairTrial(2, 20, 0.05)
    trial.state("IDLE", 0)
    trial.state("FRICTION_TEST", 1)
    trial.phase_event("off", 1)
    trial.current([0, 4, 0, 0, 0, 0, 0, 0], 1.05)
    assert trial.off.command_samples == 0
    trial.current([0] * 8, 1.11)
    assert trial.off.command_samples == 1
