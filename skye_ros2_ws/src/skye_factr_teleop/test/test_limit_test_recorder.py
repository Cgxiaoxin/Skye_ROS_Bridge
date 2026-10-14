from skye_factr_teleop.limit_test_recorder import LimitTrial, finalize_limit_record


def test_limit_trial_requires_inward_current_and_shutdown_evidence():
    trial = LimitTrial(2, "lower", 20)
    trial.state("IDLE", 0)
    trial.state("LIMIT_TEST", 1)
    trial.position(-1.95, -0.01)
    trial.current([0, -10, 0, 0, 0, 0, 0, 0], physical_sign=-1)
    trial.present_current([0, -2, 0, 0, 0, 0, 0, 0])
    trial.state("IDLE", 2)
    trial.current([0] * 8, physical_sign=-1)
    result = trial.summary("inward", "stop")
    assert result["assessment"] == "PENDING_SHUTDOWN_CHECK"
    assert result["correct_sign_samples"] == 1


def test_limit_trial_flags_outward_current():
    trial = LimitTrial(8, "upper", 20)
    trial.state("IDLE", 0)
    trial.state("LIMIT_TEST", 1)
    trial.position(0.6, 0)
    trial.current([0] * 7 + [10], physical_sign=1)
    trial.present_current([0] * 8)
    trial.state("IDLE", 2)
    trial.current([0] * 8, physical_sign=1)
    assert trial.summary("unclear", "timeout")["assessment"] == "FAIL"


def test_finalize_requires_normal_exit_and_all_eight_disabled():
    record = {"assessment": "PENDING_SHUTDOWN_CHECK", "side": "left"}
    readback = {"automated_result": "PASS", "side": "left", "devices": [
        {"id": i, "torque_enabled": False} for i in range(1, 9)
    ]}
    passed = finalize_limit_record(record, readback, process_exited_normally=True,
                                   readback_path="readback.json")
    assert passed["assessment"] == "PASS"
    assert passed["torque_disable_ids_disabled"]
    assert record["assessment"] == "PENDING_SHUTDOWN_CHECK"
    assert finalize_limit_record(record, readback, process_exited_normally=False,
                                 readback_path="readback.json")["assessment"] == "INCOMPLETE"
    readback["side"] = "right"
    assert finalize_limit_record(record, readback, process_exited_normally=True,
                                 readback_path="readback.json")["assessment"] == "INCOMPLETE"
    readback["side"] = "left"
    readback["devices"][7]["torque_enabled"] = True
    assert finalize_limit_record(record, readback, process_exited_normally=True,
                                 readback_path="readback.json")["assessment"] == "FAIL"
