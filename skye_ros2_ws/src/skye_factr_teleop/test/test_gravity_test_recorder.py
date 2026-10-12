"""Hardware-independent checks for the phase-7.2 topic summary."""

from skye_factr_teleop.gravity_test_recorder import GravityTrial


def finished_trial(*, with_present_current=True):
    trial = GravityTrial(joint_id=2, current_limit_ma=20.0)
    trial.state("IDLE", 0.0)
    trial.position([0.0] * 7, [0.0] * 7)
    trial.state("GRAVITY_TEST", 1.0)
    trial.current([0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    if with_present_current:
        trial.present_current([0.0, 4.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        trial.present_current([0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    trial.position([0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.1] + [0.0] * 5)
    trial.state("IDLE", 11.0)
    trial.current([0.0] * 8)
    return trial


def test_recorder_preserves_evidence_and_does_not_claim_final_pass():
    result = finished_trial().summary("less", "none", "timeout")
    assert result["assessment"] == "PENDING_SHUTDOWN_CHECK"
    assert result["target_peak_command_ma"] == 5.0
    assert result["other_ids_peak_command_ma"] == 0.0
    assert result["present_current_samples"] == 2
    assert result["target_present_current_min_ma"] == -2.0
    assert result["target_present_current_max_ma"] == 4.0
    assert result["other_ids_peak_abs_present_current_ma"] == 1.0
    assert result["start_position_rad"] == [0.0] * 7
    assert result["end_position_rad"][1] == 0.1
    assert result["final_zero_command_observed"]


def test_recorder_needs_clear_assistance_and_detects_non_target_current():
    trial = finished_trial()
    assert trial.summary("unclear", "none", "timeout")["assessment"] == "INCONCLUSIVE"
    trial.other_peak_ma = 1.0
    result = trial.summary("less", "none", "timeout")
    assert result["assessment"] == "FAIL"
    assert "non-target ID had nonzero commanded current" in result["issues"]


def test_operator_reports_abnormal_motion_as_fail():
    trial = finished_trial()
    assert trial.summary("less", "sudden", "stop")["assessment"] == "FAIL"
    assert trial.summary("more", "none", "stop")["assessment"] == "FAIL"


def test_missing_present_current_is_reported_as_incomplete():
    trial = finished_trial(with_present_current=False)
    result = trial.summary("less", "none", "timeout")
    assert result["assessment"] == "INCOMPLETE"
    assert "no gravity-test present current samples" in result["issues"]
