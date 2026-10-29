# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from skye_factr_teleop.config import load_config
from skye_factr_teleop.sync_trial import TrialSummary, fresh_idle, preview_move
from skye_factr_teleop import sync_trial


@pytest.fixture(params=["left", "right"])
def config(request):
    config = load_config(Path(__file__).parents[1] / "config" /
                         f"grav_comp_m6_{request.param}.yaml", require_runtime=True)
    # Keep the duration cases independent of the packaged rest-pose tuning.
    barrier = config.controller.limit_barrier_control
    return replace(config, controller=replace(
        config.controller, limit_barrier_control=replace(
            barrier, gripper=replace(barrier.gripper, gripper_setpoint=0.4))))


@pytest.mark.parametrize("velocity,acceleration,jerk,duration", [
    (0.1, 0.2, 1.0, 4.7),
    (0.8, 1.0, 5.0, 1.4806248474865698),
])
def test_real_ruckig_preview_includes_gripper_and_small_joint_move(
        config, velocity, acceleration, jerk, duration):
    sync = config.controller.sync_hold_control
    ruckig = replace(sync.ruckig, max_velocity=(velocity,) * 8,
                     max_acceleration=(acceleration,) * 8, max_jerk=(jerk,) * 8)
    config = replace(config, controller=replace(
        config.controller, sync_hold_control=replace(sync, ruckig=ruckig)))
    plan = preview_move(config, [0] * 8, [0] * 8, joint_id=6, delta=-0.05)
    np.testing.assert_allclose(plan["target_position_rad"], [0, 0, 0, 0, 0, -0.05, 0, 0.4])
    assert plan["trajectory_duration_s"] == pytest.approx(duration)


def with_trajectory_timeout(config, seconds):
    sync = config.controller.sync_hold_control
    return replace(config, controller=replace(config.controller, sync_hold_control=replace(
        sync, ruckig=replace(sync.ruckig, trajectory_timeout=seconds))))


def test_multijoint_relative_move_uses_sampled_nonzero_pose(config):
    position = [-1.46, 1.70, 0.14, 0.005, 0.10, 0.11, 0.17, 0.4]
    deltas = [0, 0, 0, 0.05, 0, -0.05, 0]
    plan = preview_move(config, position, [0] * 8, deltas=deltas)
    expected = np.append(np.asarray(position[:7]) + deltas, 0.4)
    np.testing.assert_allclose(plan["target_position_rad"], expected)
    assert position == [-1.46, 1.70, 0.14, 0.005, 0.10, 0.11, 0.17, 0.4]
    assert deltas == [0, 0, 0, 0.05, 0, -0.05, 0]
    assert plan["trajectory_duration_s"] < 2
    # A new invocation bases the same increments on its newly sampled pose.
    updated = position.copy()
    updated[0] += 0.2
    again = preview_move(config, updated, [0] * 8, deltas=deltas)
    assert again["target_position_rad"][0] == updated[0]


@pytest.mark.parametrize("kwargs,match", [
    ({"deltas": [0] * 6}, "seven finite"),
    ({"deltas": [float("nan")] * 7}, "seven finite"),
    ({"deltas": [0] * 7, "target": [0] * 7}, "choose"),
    ({"deltas": [0] * 7, "joint_id": 4}, "choose"),
    ({"deltas": [0] * 7, "delta": 0.05}, "choose"),
    ({"deltas": [0, 0, 0, 0, 0, 0, 0.75]}, "target is outside"),
    ({"deltas": [1.5, 0, 0, 0, 0, 0, 0]}, "timeout"),
])
def test_multijoint_relative_move_validates_input_and_trajectory(config, kwargs, match):
    if match == "timeout":
        config = with_trajectory_timeout(config, 1.0)
    with pytest.raises(ValueError, match=match):
        preview_move(config, [0] * 8, [0] * 8, **kwargs)


def test_multijoint_relative_cli_offline_preview(config, capsys):
    side = "left" if "left" in config.controller.joint_gravity_comp.dynamics.leader_urdf else "right"
    path = Path(__file__).parents[1] / "config" / f"grav_comp_m6_{side}.yaml"
    sync_trial.main(["move", "--side", side, "--config", str(path),
                     "--start", "-1.46", "1.70", "0.14", "0.005", "0.10", "0.11", "0.17", "0.4",
                     "--deltas", "0", "0", "0", "0.05", "0", "-0.05", "0"])
    plan = json.loads(capsys.readouterr().out)
    setpoint = load_config(path).controller.limit_barrier_control.gripper.gripper_setpoint
    np.testing.assert_allclose(plan["target_position_rad"],
                               [-1.46, 1.70, 0.14, 0.055, 0.10, 0.06, 0.17, setpoint])


@pytest.mark.parametrize("kwargs,match", [
    ({"target": [0, 0, 0, 0, 0, 0, 0.75]}, "target is outside"),
    ({"target": [1.5, 0, 0, 0, 0, 0, 0]}, "timeout"),
    ({"target": [float("nan")] * 7}, "finite"),
    ({"joint_id": 8, "delta": 0.05}, "joint-id"),
    ({"joint_id": 1, "delta": 0.05, "target": [0] * 7}, "choose"),
])
def test_preview_rejects_bad_or_overlong_targets(config, kwargs, match):
    if match == "timeout":
        config = with_trajectory_timeout(config, 1.0)
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


def run_sync_monitor(monkeypatch, config, modes, *, stale=False):
    """Feed diagnostic cycles to the real wait loop with a deterministic clock."""
    clock = [100.0]
    initial = sample("IDLE", 1)
    latest = [initial, clock[0]]
    pending = iter(modes)

    def spin_once(node, timeout_sec):
        clock[0] += 0.05
        try:
            mode = next(pending)
        except StopIteration:
            mode = latest[0]["mode"]
        if not stale:
            updates = mode if isinstance(mode, dict) else {"mode": mode}
            latest[:] = [sample(updates["mode"], latest[0]["sequence"] + 1,
                                stamp_s=clock[0], **{k: v for k, v in updates.items() if k != "mode"}),
                         clock[0]]

    monkeypatch.setattr(sync_trial.time, "monotonic", lambda: clock[0])
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(spin_once=spin_once))
    node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(clock[0] * 1e9))))
    sync_trial._wait_for_sync(node, latest, packet=initial, target=[0.02] * 8,
                             config=config, wait_seconds=0.5)


def test_execute_monitor_waits_through_settling_until_synced(monkeypatch, config, capsys):
    run_sync_monitor(monkeypatch, config,
                     ["IDLE", "TELEOP_SYNCING", "TELEOP_SYNCING", "HOLD_SETTLING", "SYNCED"])
    output = capsys.readouterr().out
    assert output.count("Observed mode: TELEOP_SYNCING") == 1
    assert "Observed mode: HOLD_SETTLING" in output
    assert "Synchronization complete: SYNCED" in output
    assert "继续保持目标姿态" in output


def test_execute_monitor_accepts_synced_when_intermediate_samples_are_missed(monkeypatch, config, capsys):
    run_sync_monitor(monkeypatch, config, ["SYNCED"])
    assert "Synchronization complete: SYNCED" in capsys.readouterr().out


@pytest.mark.parametrize("mode", ["IDLE", "TELEOP"])
def test_execute_monitor_reports_abort_before_synced(monkeypatch, config, capsys, mode):
    with pytest.raises(ValueError, match=f"entered {mode} before SYNCED"):
        run_sync_monitor(monkeypatch, config, ["TELEOP_SYNCING", "HOLD_SETTLING", mode])
    assert "Synchronization complete" not in capsys.readouterr().out


def test_execute_monitor_reports_lost_diagnostics(monkeypatch, config):
    with pytest.raises(TimeoutError, match="diagnostics are stale"):
        run_sync_monitor(monkeypatch, config, [], stale=True)


def test_execute_monitor_reports_missing_acknowledgement(monkeypatch, config):
    with pytest.raises(TimeoutError, match="acknowledgement not observed"):
        run_sync_monitor(monkeypatch, config, ["IDLE"])


def test_execute_monitor_bounds_completion_wait(monkeypatch, config):
    with pytest.raises(TimeoutError, match="completion not observed; last mode: HOLD_SETTLING"):
        run_sync_monitor(monkeypatch, config, ["TELEOP_SYNCING", "HOLD_SETTLING"])


def test_sync_conditions_identifies_position_and_velocity_blockers(config):
    state = sample("HOLD_SETTLING", 3, position_rad=[0.0] * 8,
                   velocity_rad_s=[0.0, 0.08] + [0.0] * 6,
                   sync_status={"settled_for_s": 0.0})
    text = sync_trial._sync_conditions(state, [0.1] + [0.0] * 6 + [0.4],
                                       config.controller.sync_hold_control.hold)
    tolerance = config.controller.sync_hold_control.hold.sync_position_tolerance[0]
    assert f"J1: |position error|=0.100000 rad > {tolerance:.6f} rad" in text
    assert "J2: |velocity|=0.080000 rad/s > 0.050000 rad/s" in text
    assert "gripper(ID8): |position error|=0.400000 rad > 0.050000 rad" in text
    assert "J3:" not in text
    assert "Continuous settling: 0.000 / 0.250 s" in text


def test_sync_conditions_instantaneous_pass_still_requires_continuous_settling(config):
    hold = config.controller.sync_hold_control.hold
    state = sample("HOLD_SETTLING", 3, velocity_rad_s=list(hold.sync_velocity_tolerance),
                   sync_status={"settled_for_s": 0.12})
    text = sync_trial._sync_conditions(state, list(hold.sync_position_tolerance), hold)
    assert "All eight axes meet instantaneous tolerances" in text
    assert "Continuous settling: 0.120 / 0.250 s" in text


@pytest.mark.parametrize("reason", ["sync_timeout", "settle_timeout"])
def test_execute_monitor_reports_controller_timeout_and_measured_blockers(
        monkeypatch, config, reason):
    sync = config.controller.sync_hold_control
    expected = (f"Ruckig reference trajectory did not finish within {sync.ruckig.trajectory_timeout:.3f} s"
                if reason == "sync_timeout" else
                f"within {sync.hold.settle_timeout:.3f} s of HOLD_SETTLING")
    tolerance = sync.hold.sync_position_tolerance[0]
    error = tolerance + 0.01
    final = {"mode": "IDLE", "sync_status": {"transition_reason": reason},
             "position_rad": [0.02 + error] + [0.0] * 7, "velocity_rad_s": [0.0] * 8}
    with pytest.raises(ValueError) as exc:
        run_sync_monitor(monkeypatch, config, ["TELEOP_SYNCING", "HOLD_SETTLING", final])
    assert expected in str(exc.value)
    assert f"J1: |position error|={error:.6f} rad > {tolerance:.6f} rad" in str(exc.value)


def test_execute_monitor_prints_blockers_while_settling(monkeypatch, config, capsys):
    run_sync_monitor(monkeypatch, config, ["TELEOP_SYNCING",
        {"mode": "HOLD_SETTLING", "velocity_rad_s": [0.1] + [0.0] * 7}, "SYNCED"])
    text = capsys.readouterr().out
    assert "Waiting for HOLD_SETTLING: Ruckig reference trajectory is not finished" in text
    assert "Waiting for SYNCED:" in text
    assert "J1: |velocity|=0.100000 rad/s > 0.050000 rad/s" in text


def test_failure_keeps_active_currents_and_torques_after_idle_zeroes_them(monkeypatch, config):
    active = dict(mode="HOLD_SETTLING", present_current_ma=[39.0] * 8,
                  current_before_clip_ma=[70.0] * 8, current_after_clip_ma=[50.0] * 8,
                  current_limit_ma=[50.0] * 8, gravity_torque_nm=[0.04] * 8,
                  friction_torque_nm=[-0.005] * 8, joint_limit_torque_nm=[0.0] * 8,
                  tracking_torque_nm=[0.03] * 8)
    idle = dict(mode="IDLE", current_before_clip_ma=[0.0] * 8,
                current_after_clip_ma=[0.0] * 8, present_current_ma=[0.0] * 8,
                sync_status={"transition_reason": "settle_timeout"})
    with pytest.raises(ValueError) as exc:
        run_sync_monitor(monkeypatch, config, [active, idle])
    text = str(exc.value)
    assert "Last active sample: mode=HOLD_SETTLING" in text
    assert "present=+39.00 mA, command=+70.00 -> +50.00 mA, limit=+50.00 mA" in text
    assert "CLIP samples=1/1" in text
    assert "gravity=+0.04000, friction=-0.00500, barrier=+0.00000, tracking=+0.03000" in text


def test_failure_counts_clipping_even_when_last_active_frame_is_not_clipped(monkeypatch, config):
    unclipped = dict(mode="HOLD_SETTLING", current_before_clip_ma=[30.0] * 8,
                     current_after_clip_ma=[30.0] * 8)
    with pytest.raises(ValueError) as exc:
        run_sync_monitor(monkeypatch, config, ["TELEOP_SYNCING", unclipped, "IDLE"])
    text = str(exc.value)
    assert "command=+30.00 -> +30.00 mA" in text
    assert "CLIP samples=1/2" in text


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


def test_record_status_shows_only_unmet_conditions_and_waiting_time(config):
    display = sync_trial.RecordStatus(config)
    display.observe(sample("HOLD_SETTLING", 1, target_position_rad=[0.1] + [0.0] * 7,
                           velocity_rad_s=[0.0, 0.08] + [0.0] * 6,
                           sync_status={"mode_elapsed_s": 12.4, "settled_for_s": 0.0}), received_at=100.0)
    text = display.render(now=100.01, ros_now=100.01)
    assert "Waiting for SYNCED" in text and "12.400 / 25.000 s" in text
    tolerance = config.controller.sync_hold_control.hold.sync_position_tolerance[0]
    assert f"J1: |position error|=0.100000 rad > {tolerance:.6f} rad" in text
    assert "J2: |velocity|=0.080000 rad/s > 0.050000 rad/s" in text
    assert "Continuous settling: 0.000 / 0.250 s" in text
    for absent in ("J3:", "J7:", "gripper(ID8):", "CLIP", "actual/limit"):
        assert absent not in text


def test_record_status_limits_refresh_and_reports_synced_once(config):
    display = sync_trial.RecordStatus(config)
    display.observe(sample("TELEOP_SYNCING", 1, sync_status={"mode_elapsed_s": 0.5}), received_at=100.0)
    text = display.render(now=100.0, ros_now=100.0)
    assert "Waiting for HOLD_SETTLING" in text and "0.500 / 30.000 s" in text
    for seq, now in ((2, 100.1), (3, 100.99)):
        display.observe(sample("TELEOP_SYNCING", seq, stamp_s=now), received_at=now)
        assert display.render(now=now, ros_now=now) is None
    assert display.render(now=101.0, ros_now=101.0) is not None
    display.observe(sample("SYNCED", 4, stamp_s=101.01), received_at=101.01)
    assert "Synchronization complete: SYNCED" in display.render(now=101.01, ros_now=101.01)
    display.observe(sample("SYNCED", 5, stamp_s=102.1), received_at=102.1)
    assert display.render(now=102.1, ros_now=102.1) is None


def test_record_status_reports_missing_stale_and_resumed_diagnostics(config):
    display = sync_trial.RecordStatus(config)
    assert "WAITING" in display.render(now=100.0, ros_now=100.0)
    display.observe(sample("HOLD_SETTLING", 1), received_at=100.0)
    assert "Waiting for SYNCED" in display.render(now=100.01, ros_now=100.01)
    stale = display.render(now=100.3, ros_now=100.3)
    assert "STALE" in stale and "0.300 s" in stale and "position error" not in stale
    display.observe(sample("HOLD_SETTLING", 2, stamp_s=100.31), received_at=100.31)
    assert "Waiting for SYNCED" in display.render(now=100.31, ros_now=100.31)


@pytest.mark.parametrize("stamp", [99.0, 101.0])
def test_record_status_checks_message_timestamp_as_well_as_receipt(config, stamp):
    display = sync_trial.RecordStatus(config)
    display.observe(sample("HOLD_SETTLING", 1, stamp_s=stamp), received_at=100.0)
    assert "STALE" in display.render(now=100.0, ros_now=100.0)


def test_record_status_timeout_keeps_goal_after_idle_clears_target(config):
    display = sync_trial.RecordStatus(config)
    display.observe(sample("HOLD_SETTLING", 1, target_position_rad=[0.1] + [0.0] * 7), received_at=100.0)
    display.render(now=100.0, ros_now=100.0)
    display.observe(sample("IDLE", 2, stamp_s=100.1, target_position_rad=None,
                           velocity_rad_s=[0.0] * 8,
                           sync_status={"transition_reason": "settle_timeout"}), received_at=100.1)
    text = display.render(now=100.1, ros_now=100.1)
    assert "settle_timeout" in text and "25.000 s" in text
    assert "J1: |position error|=0.100000 rad" in text and "J3:" not in text


def test_record_cli_removes_progress_option(tmp_path, capsys):
    with pytest.raises(SystemExit) as caught:
        sync_trial.main(["record", "--side", "left", "--config", "not-loaded.yaml",
                         "--operator", "test", "--output", str(tmp_path / "record.json"),
                         "--progress-hz", "2"])
    assert caught.value.code == 2
    assert "unrecognized arguments: --progress-hz 2" in capsys.readouterr().err
