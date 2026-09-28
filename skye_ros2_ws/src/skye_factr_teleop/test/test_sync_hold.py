# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Task 4.3 tests using explicit time, fake trajectories and fake hardware."""

from dataclasses import replace

import numpy as np
import pytest

from conftest import FakeDynamixelIO
from test_control_core import CONFIG, FakeDynamics
from skye_factr_teleop.config import load_config
from skye_factr_teleop.control_core import (
    ControlCore, ControlTerms, RuckigTrajectory, TrajectorySample,
)
from skye_factr_teleop.state_machine import ControlMode


class FakeTrajectory:
    def __init__(self):
        self.calls = 0
        self.finished = False
        self.position = np.full(8, 0.04)
        self.velocity = np.full(8, 0.02)
        self.error = None

    def start(self, position, velocity, target):
        self.start_position = position.copy()
        self.start_velocity = velocity.copy()
        self.target = target.copy()

    def update(self):
        self.calls += 1
        if self.error is not None:
            raise self.error
        return TrajectorySample(self.position.copy(), self.velocity.copy(), self.finished)


def make_core(*, gravity=True):
    config = load_config(CONFIG)
    hardware = FakeDynamixelIO(np.zeros(8), np.zeros(8), torque_enabled=True)
    dynamics = FakeDynamics()
    trajectories = []

    def factory():
        trajectory = FakeTrajectory()
        trajectories.append(trajectory)
        return trajectory

    core = ControlCore(
        config, hardware, dynamics, np.zeros(8), 50.0,
        ControlTerms(False, gravity, False, False), trajectory_factory=factory,
    )
    return core, hardware, dynamics, trajectories


def begin(core, *, now=1.0, target=0.1):
    return core.handle(
        "switch_sync", now=now,
        follower_position=np.full(7, target), gripper_setpoint=target,
    )


def test_sync_captures_measured_eight_axis_state_and_copies_target():
    core, hardware, _, trajectories = make_core()
    raw_position = np.arange(8) * 0.03
    raw_velocity = np.arange(8) * 0.01
    core.joint_offsets[:] = 0.01
    hardware.inject_state(raw_position, raw_velocity)
    target = np.full(7, 0.1)
    core.handle("switch_sync", now=1.0, follower_position=target)
    trajectory = trajectories[0]
    np.testing.assert_allclose(trajectory.start_position, (raw_position - 0.01) * core.joint_signs)
    np.testing.assert_allclose(trajectory.start_velocity, raw_velocity * core.joint_signs)
    target[:] = 99
    np.testing.assert_allclose(trajectory.target[:7], 0.1)
    assert trajectory.target[-1] == core.config.gripper_teleop.limit_setpoint
    assert trajectory.calls == 0
    assert hardware.current_history_ma == []


def test_one_update_per_cycle_pd_phase_gravity_and_current_clipping():
    core, hardware, dynamics, trajectories = make_core()
    velocity = np.full(8, 0.01)
    hardware.inject_state(np.zeros(8), velocity * core.joint_signs)
    begin(core)
    cfg = core.config.controller.hold_ruckig_control
    result = core.step(now=1.002)
    assert trajectories[0].calls == 1
    assert len(hardware.current_history_ma) == 1
    expected_pd = np.array(cfg.ruckig_kp) * 0.04 + np.array(cfg.ruckig_kd) * 0.01
    expected_gravity = dynamics.gravity_value * cfg.ruckig_gravity_comp_gain
    np.testing.assert_allclose(result.tracking_torque, expected_pd)
    np.testing.assert_allclose(result.gravity_torque[:7], expected_gravity)
    np.testing.assert_allclose(result.logical_torque[:7], expected_pd[:7] + expected_gravity)
    np.testing.assert_allclose(result.logical_torque[-1], expected_pd[-1])
    assert result.physical_current_ma[5] < 0
    assert np.max(np.abs(result.physical_current_ma)) <= 50
    assert result.mode == ControlMode.TELEOP_SYNCING
    core.step(now=1.004)
    assert trajectories[0].calls == 2


def test_completion_enters_hold_with_distinct_gravity_and_pid_gains():
    core, hardware, dynamics, trajectories = make_core()
    ctrl = core.config.controller
    hold = replace(ctrl.hold_ruckig_control, hold_gravity_comp_gain=(0.2,) * 7)
    core.config = replace(core.config, controller=replace(ctrl, hold_ruckig_control=hold))
    begin(core)
    trajectories[0].finished = True
    result = core.step(now=1.002)
    assert core.mode == ControlMode.SYNCED
    np.testing.assert_allclose(result.target_position, 0.1)
    np.testing.assert_array_equal(result.target_velocity, np.zeros(8))
    np.testing.assert_array_equal(core.hold_integral, np.zeros(8))
    np.testing.assert_allclose(result.gravity_torque[:7], dynamics.gravity_value * 0.2)
    np.testing.assert_allclose(result.tracking_torque, np.array(hold.hold_kp) * 0.1)

    hardware.inject_state(np.zeros(8), np.full(8, 0.02) * core.joint_signs)
    result = core.step(now=1.012)
    np.testing.assert_allclose(core.hold_integral, 0.001)
    expected = np.array(hold.hold_kp) * 0.1 - np.array(hold.hold_kd) * 0.02
    expected += np.array(hold.hold_ki) * 0.001
    np.testing.assert_allclose(result.tracking_torque, expected)
    assert trajectories[0].calls == 1  # Hold never advances the old trajectory.


def test_hold_integral_clamps_both_signs_and_resets_outside_threshold():
    core, hardware, _, trajectories = make_core(gravity=False)
    begin(core, target=0.0)
    trajectories[0].finished = True
    core.step(now=1.002)
    position = np.array([-0.1, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1])
    hardware.inject_state(position * core.joint_signs, np.zeros(8))
    result = core.step(now=3.002)
    expected = -np.sign(position) * np.array(core.config.controller.hold_ruckig_control.hold_i_max)
    np.testing.assert_allclose(core.hold_integral, expected)
    # Snapshots returned to the caller cannot mutate the controller's target/integral.
    core.hold_integral[:] = 100
    result.target_position[:] = 100
    hardware.inject_state(np.full(8, 0.2) * core.joint_signs, np.zeros(8))
    result = core.step(now=3.004)
    np.testing.assert_array_equal(core.hold_integral, np.zeros(8))
    np.testing.assert_array_equal(result.target_position, np.zeros(8))


@pytest.mark.parametrize("target", [[0] * 6, [0] * 8, [float("nan")] * 7])
def test_invalid_follower_target_stops_without_starting_trajectory(target):
    core, hardware, _, trajectories = make_core()
    with pytest.raises(ValueError, match="follower_position"):
        core.handle("switch_sync", now=1.0, follower_position=target)
    assert core.mode == ControlMode.IDLE
    assert trajectories == []
    assert not hardware.torque_enabled
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))


@pytest.mark.parametrize("grip", [float("nan"), float("inf")])
def test_invalid_gripper_target_stops(grip):
    core, hardware, _, _ = make_core()
    with pytest.raises(ValueError, match="gripper_setpoint"):
        core.handle("switch_sync", now=1.0, follower_position=np.zeros(7), gripper_setpoint=grip)
    assert core.mode == ControlMode.IDLE
    assert not hardware.torque_enabled


def test_deadline_wins_over_completion_and_repeated_sync_does_not_extend_it():
    core, hardware, _, trajectories = make_core()
    begin(core)
    trajectories[0].finished = True
    assert begin(core, now=2.0, target=0.2).outcome == "ignored"
    result = core.step(now=11.0)
    assert result.mode == ControlMode.IDLE
    assert core.last_transition.reason == "sync_timeout"
    assert trajectories[0].calls == 0
    assert result.target_position is None
    assert not hardware.torque_enabled
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))
    assert core.handle("switch_teleop", now=12.0).outcome == "rejected"
    hardware.set_torque_enabled(True)
    begin(core, now=13.0, target=0.2)
    assert len(trajectories) == 2
    assert core.step(now=13.002).mode == ControlMode.TELEOP_SYNCING
    np.testing.assert_allclose(trajectories[1].target, 0.2)


@pytest.mark.parametrize("now", [0.0, float("nan"), float("inf")])
def test_stop_works_with_invalid_time_and_drops_old_motion(now):
    core, hardware, _, trajectories = make_core()
    begin(core)
    core.step(now=1.002)
    assert core.handle("switch_stop", now=now).outcome == "accepted"
    assert core.mode == ControlMode.IDLE
    assert not hardware.torque_enabled
    np.testing.assert_array_equal(core.hold_integral, np.zeros(8))
    hardware.set_torque_enabled(True)
    result = core.step(now=2.0)
    assert result.target_position is None
    np.testing.assert_array_equal(result.tracking_torque, np.zeros(8))
    assert trajectories[0].calls == 1


@pytest.mark.parametrize("event", ["switch_stop", "switch_teleop"])
def test_leaving_hold_clears_integral_target_and_trajectory(event):
    core, hardware, _, trajectories = make_core(gravity=False)
    begin(core)
    trajectories[0].finished = True
    core.step(now=1.002)
    core.step(now=1.102)
    assert np.any(core.hold_integral)
    result = core.handle(event, now=1.2)
    assert result.outcome == "accepted"
    np.testing.assert_array_equal(core.hold_integral, np.zeros(8))
    result = core.step(now=1.202)
    assert result.target_position is None
    assert result.target_velocity is None
    np.testing.assert_array_equal(result.tracking_torque, np.zeros(8))
    assert trajectories[0].calls == 1
    if event == "switch_teleop":
        assert result.mode == ControlMode.TELEOP
        assert hardware.torque_enabled
        assert core.handle(event, now=1.204).outcome == "rejected"


@pytest.mark.parametrize("fault", ["trajectory", "sample", "gravity_on_completion", "write"])
def test_control_failure_returns_idle_and_disables_torque(fault):
    core, hardware, dynamics, trajectories = make_core()
    begin(core)
    core.step(now=1.002)
    if fault == "trajectory":
        trajectories[0].error = RuntimeError("trajectory failed")
    elif fault == "sample":
        trajectories[0].position[0] = np.nan
    elif fault == "gravity_on_completion":
        trajectories[0].finished = True
        dynamics.gravity_value[0] = np.nan
    else:
        hardware.fail_next_write(RuntimeError("write failed"))
    with pytest.raises((RuntimeError, ValueError)):
        core.step(now=1.004)
    assert core.mode == ControlMode.IDLE
    assert not hardware.torque_enabled
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))
    np.testing.assert_array_equal(core.hold_integral, np.zeros(8))


def test_start_failure_rolls_back_and_external_completion_is_not_accepted():
    core, hardware, _, _ = make_core()
    core._trajectory_factory = lambda: (_ for _ in ()).throw(RuntimeError("setup failed"))
    with pytest.raises(RuntimeError, match="setup failed"):
        begin(core)
    assert core.mode == ControlMode.IDLE
    assert not hardware.torque_enabled
    with pytest.raises(ValueError, match="internal"):
        core.handle("sync_complete", now=2.0)
    assert core.mode == ControlMode.IDLE


def test_torque_loss_discards_trajectory_without_updating_it():
    core, hardware, _, trajectories = make_core()
    begin(core)
    hardware.set_torque_enabled(False)
    result = core.step(now=1.002)
    assert result.mode == ControlMode.IDLE
    assert not result.wrote_current
    assert trajectories[0].calls == 0
    assert hardware.current_history_ma == []


def test_missing_time_during_motion_stops():
    core, hardware, _, _ = make_core()
    begin(core)
    with pytest.raises(ValueError, match="now is required"):
        core.step()
    assert core.mode == ControlMode.IDLE
    assert not hardware.torque_enabled


@pytest.mark.parametrize("event", ["switch_sync", "switch_teleop"])
def test_starting_motion_requires_enabled_torque(event):
    core, hardware, _, trajectories = make_core()
    if event == "switch_teleop":
        begin(core)
        trajectories[0].finished = True
        core.step(now=1.002)
    hardware.set_torque_enabled(False)
    with pytest.raises(RuntimeError, match="enabled torque"):
        core.handle(event, now=2.0, follower_position=np.zeros(7))
    assert core.mode == ControlMode.IDLE
    assert not hardware.torque_enabled


def test_sync_requires_an_explicit_trajectory_factory():
    core, hardware, _, _ = make_core()
    core._trajectory_factory = None
    with pytest.raises(RuntimeError, match="trajectory_factory"):
        begin(core)
    assert core.mode == ControlMode.IDLE
    assert not hardware.torque_enabled


def test_stop_disables_torque_even_if_zero_write_fails():
    core, hardware, _, _ = make_core()
    begin(core)
    hardware.fail_next_write(RuntimeError("zero write failed"))
    with pytest.raises(RuntimeError, match="zero write failed"):
        core.stop()
    assert core.mode == ControlMode.IDLE
    assert not hardware.torque_enabled
    assert core.step(now=2.0).target_position is None


def test_real_ruckig_reaches_target_within_explicit_limits():
    pytest.importorskip("ruckig")
    trajectory = RuckigTrajectory(
        frequency=500, max_velocity=[0.5] * 8,
        max_acceleration=[1.0] * 8, max_jerk=[4.0] * 8,
    )
    target = np.linspace(0.02, 0.1, 8)
    trajectory.start(np.zeros(8), np.zeros(8), target)
    previous_velocity = np.zeros(8)
    previous_acceleration = np.zeros(8)
    for _ in range(2000):
        sample = trajectory.update()
        acceleration = (sample.velocity - previous_velocity) / 0.002
        jerk = (acceleration - previous_acceleration) / 0.002
        assert np.max(np.abs(sample.velocity)) <= 0.5 + 1e-9
        assert np.max(np.abs(acceleration)) <= 1.0 + 1e-9
        assert np.max(np.abs(jerk)) <= 4.0 + 1e-7
        previous_velocity, previous_acceleration = sample.velocity, acceleration
        if sample.finished:
            break
    else:
        pytest.fail("Ruckig did not finish")
    np.testing.assert_allclose(sample.position, target, atol=1e-9)
    np.testing.assert_allclose(sample.velocity, 0.0, atol=1e-9)


@pytest.mark.parametrize("limits", [[1] * 7, [0] * 8, [float("nan")] * 8])
def test_ruckig_rejects_invalid_limits(limits):
    with pytest.raises(ValueError):
        RuckigTrajectory(
            frequency=500, max_velocity=limits,
            max_acceleration=[1] * 8, max_jerk=[4] * 8,
        )
