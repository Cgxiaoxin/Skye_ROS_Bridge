# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from skye_factr_teleop.config import load_config
from skye_factr_teleop.sync_trial import TrialSummary, fresh_idle, preview_move


@pytest.fixture(params=["left", "right"])
def config(request):
    return load_config(Path(__file__).parents[1] / "config" /
                       f"grav_comp_m6_{request.param}.yaml", require_runtime=True)


def test_real_ruckig_preview_includes_gripper_and_small_joint_move(config):
    plan = preview_move(config, [0] * 8, [0] * 8, joint_id=6, delta=-0.05)
    np.testing.assert_allclose(plan["target_position_rad"], [0, 0, 0, 0, 0, -0.05, 0, 0.4])
    assert plan["trajectory_duration_s"] == pytest.approx(4.7)


@pytest.mark.parametrize("kwargs,match", [
    ({"target": [0, 0, 0, 0, 0, 0, 0.75]}, "target is outside"),
    ({"target": [1.5, 0, 0, 0, 0, 0, 0]}, "timeout"),
    ({"target": [float("nan")] * 7}, "finite"),
    ({"joint_id": 8, "delta": 0.05}, "joint-id"),
    ({"joint_id": 1, "delta": 0.05, "target": [0] * 7}, "choose"),
])
def test_preview_rejects_bad_or_overlong_targets(config, kwargs, match):
    with pytest.raises(ValueError, match=match):
        preview_move(config, [0] * 8, [0] * 8, **kwargs)


def test_preview_rejects_moving_or_out_of_bounds_start(config):
    with pytest.raises(ValueError, match="moving"):
        preview_move(config, [0] * 8, [0.06] * 8, joint_id=1, delta=0.05)
    with pytest.raises(ValueError, match="current pose"):
        preview_move(config, [3] + [0] * 7, [0] * 8, joint_id=1, delta=-0.05)


def sample(mode, seq, **updates):
    return {"mode": mode, "side": "left", "sequence": seq, "stamp_s": 100.0,
            "monotonic_s": seq * 0.002, "cycle_dt_s": 0.002,
            "torque_enabled_cached": mode != "IDLE", "wrote_current": mode != "IDLE",
            "position_rad": [0.0] * 8, "target_position_rad": [0.02] * 8,
            "velocity_rad_s": [0.01] * 8, "current_before_clip_ma": [60.0] * 8,
            "current_after_clip_ma": [50.0] * 8, "present_current_ma": [48.0] * 8,
            **updates}


def test_summary_counts_gaps_clipping_and_software_stop_without_claiming_hardware_pass():
    trial = TrialSummary()
    trial.observe(sample("IDLE", 1))
    trial.observe(sample("TELEOP_SYNCING", 2))
    trial.observe(sample("HOLD_SETTLING", 4))
    trial.observe(sample("SYNCED", 5))
    trial.observe(sample("IDLE", 6, current_before_clip_ma=[0] * 8,
                         current_after_clip_ma=[0] * 8, target_position_rad=None))
    result = trial.result()
    json.dumps(result, allow_nan=False)
    assert result["assessment"] == "RECORDED"
    assert result["missing_samples"] == 1
    assert result["synced_observed"]
    assert not result["hardware_torque_disable_verified"]
    phase = result["phases"]["TELEOP_SYNCING"]
    assert phase["max_abs_error_rad"] == [0.02] * 8
    assert phase["clipped_samples_per_axis"] == [1] * 8
    assert result["transitions"][-1]["current_change_ma"] == [-50] * 8


def test_incomplete_record_and_sequence_restart_are_visible():
    trial = TrialSummary()
    trial.observe(sample("TELEOP_SYNCING", 10))
    trial.observe(sample("TELEOP_SYNCING", 1))
    assert trial.result()["assessment"] == "INCOMPLETE"
    assert trial.result()["sequence_restarts"] == 1


def test_fresh_idle_checks_ros_time_receive_time_side_and_mode(config):
    packet = sample("IDLE", 1)
    kwargs = dict(side="left", ros_now=100.01, received_at=20.0,
                  monotonic_now=20.01, runtime=config.runtime)
    fresh_idle(packet, **kwargs)
    for key, value in (("ros_now", 100.3), ("monotonic_now", 20.3), ("side", "right")):
        with pytest.raises(ValueError):
            fresh_idle(packet, **{**kwargs, key: value})
    moving = deepcopy(packet)
    moving["mode"] = "SYNCED"
    with pytest.raises(ValueError, match="IDLE"):
        fresh_idle(moving, **kwargs)
