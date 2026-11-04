# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Task 4.4: real core/adapter/state machine with fake clock, Ruckig and bus."""

from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from conftest import FakeDynamixelIO
from test_control_core import FakeDynamics
from skye_factr_teleop.config import load_config
from skye_factr_teleop.control_core import ControlCore, ControlTerms
from skye_factr_teleop.state_machine import ControlMode
from skye_factr_teleop.trajectory import RuckigTrajectory


@dataclass
class FakeClock:
    now: float = 100.0

    def advance(self, dt=0.002):
        self.now += dt
        return self.now


class FakeResult(IntEnum):
    Working = 0
    Finished = 1
    Error = -1


class FakeInput:
    def __init__(self, dofs):
        assert dofs == 8


class FakeOutput:
    def __init__(self, dofs):
        assert dofs == 8
        self.pass_calls = 0

    def pass_to_input(self, inp):
        self.pass_calls += 1
        inp.current_position = list(self.new_position)
        inp.current_velocity = list(self.new_velocity)
        inp.current_acceleration = list(self.new_acceleration)


class FakeRuckig:
    def __init__(self, dofs, dt):
        assert dofs == 8
        self.dt = dt
        self.calls = 0
        self.result = FakeResult.Working
        self.inputs = []

    def reset(self):
        self.calls = 0

    def validate_input(self, inp, check_current, check_target):
        assert check_current and check_target
        self.start_position = np.array(inp.current_position)
        self.start_velocity = np.array(inp.current_velocity)
        self.start_acceleration = np.array(inp.current_acceleration)
        self.target = np.array(inp.target_position)
        np.testing.assert_array_equal(inp.target_velocity, np.zeros(8))
        np.testing.assert_array_equal(inp.target_acceleration, np.zeros(8))
        return True

    def update(self, inp, out):
        self.calls += 1
        self.inputs.append(np.array(inp.current_position))
        position = np.array(inp.current_position)
        target = np.array(inp.target_position)
        out.new_position = (target if self.result == FakeResult.Finished
                            else (position + target) / 2).tolist()
        out.new_velocity = [0.0] * 8
        out.new_acceleration = [0.0] * 8
        return self.result


class CountingHardware(FakeDynamixelIO):
    def __init__(self):
        super().__init__(np.zeros(8), np.zeros(8), torque_enabled=True)
        self.read_calls = 0

    def read_state(self):
        self.read_calls += 1
        return super().read_state()


@pytest.fixture(params=["left", "right"])
def rig(request, monkeypatch):
    backends = []

    def create_backend(dofs, dt):
        backend = FakeRuckig(dofs, dt)
        backends.append(backend)
        return backend

    # Exercise the production adapter's lazy import/update/pass_to_input path.
    # No installed Ruckig library or actual servo device is used by this suite.
    monkeypatch.setitem(sys.modules, "ruckig", SimpleNamespace(
        Ruckig=create_backend, InputParameter=FakeInput,
        OutputParameter=FakeOutput, Result=FakeResult,
    ))
    config = load_config(Path(__file__).parents[1] / "config" /
                         f"grav_comp_m6_{request.param}.yaml")
    clock = FakeClock()
    hardware = CountingHardware()

    def trajectory_factory():
        return RuckigTrajectory(
            frequency=config.controller.frequency, max_velocity=[0.5] * 8,
            max_acceleration=[1.0] * 8, max_jerk=[4.0] * 8,
        )

    core = ControlCore(
        config, hardware, FakeDynamics(), np.full(8, 0.01), 50,
        ControlTerms(False, True, False, False),
        trajectory_factory=trajectory_factory, now=clock.now,
    )
    return SimpleNamespace(core=core, hardware=hardware, clock=clock, backends=backends)


def start(rig, target=0.1):
    return rig.core.handle(
        "switch_sync", now=rig.clock.now,
        follower_position=np.full(7, target), gripper_setpoint=target,
    )


def tick(rig, dt=0.002):
    return rig.core.step(now=rig.clock.advance(dt))


def test_complete_lifecycle_and_repeated_requests(rig):
    core, hardware, clock = rig.core, rig.hardware, rig.clock
    q = np.linspace(0.01, 0.04, 8)
    dq = np.linspace(0.0, 0.02, 8)
    hardware.inject_state(q * core.joint_signs + core.joint_offsets, dq * core.joint_signs)
    assert core.handle("switch_teleop", now=clock.now).outcome == "rejected"
    started = start(rig)
    backend = rig.backends[0]
    assert started.state == ControlMode.TELEOP_SYNCING
    assert hardware.read_calls == 1
    np.testing.assert_allclose(backend.start_position, q)
    np.testing.assert_allclose(backend.start_velocity, dq)
    np.testing.assert_array_equal(backend.start_acceleration, np.zeros(8))
    assert backend.dt == 1 / core.config.controller.frequency

    first = tick(rig)
    second = tick(rig)
    assert first.mode == second.mode == ControlMode.TELEOP_SYNCING
    assert backend.calls == 2
    # The second Ruckig input must be the first reference, not measured q.
    np.testing.assert_allclose(backend.inputs[1], first.target_position)
    assert not np.allclose(first.target_position, second.target_position)

    reads, writes = hardware.read_calls, len(hardware.current_history_ma)
    repeated = core.handle("switch_sync", now=clock.advance(), follower_position=[np.nan])
    assert repeated.outcome == "ignored"
    assert repeated.entered_at == started.entered_at
    assert core.handle("switch_teleop", now=clock.now).outcome == "rejected"
    assert hardware.read_calls == reads
    assert len(hardware.current_history_ma) == writes
    assert len(rig.backends) == 1

    backend.result = FakeResult.Finished
    completed = tick(rig)
    assert completed.mode == ControlMode.HOLD_SETTLING
    np.testing.assert_allclose(completed.target_position, 0.1)
    np.testing.assert_array_equal(core.hold_integral, np.zeros(8))
    hardware.inject_state(np.full(8, 0.09) * core.joint_signs + core.joint_offsets,
                          np.zeros(8))
    tick(rig, 0.01)
    assert core.mode == ControlMode.HOLD_SETTLING
    tick(rig, core.config.controller.sync_hold_control.hold.settle_duration)
    assert core.mode == ControlMode.SYNCED
    integral = core.hold_integral
    assert np.all(integral > 0)
    assert start(rig, target=0.2).outcome == "ignored"
    np.testing.assert_array_equal(core.hold_integral, integral)
    np.testing.assert_allclose(tick(rig).target_position, 0.1)

    reads, writes = hardware.read_calls, len(hardware.current_history_ma)
    assert core.handle("switch_teleop", now=clock.advance()).outcome == "accepted"
    assert core.mode == ControlMode.TELEOP
    np.testing.assert_array_equal(core.hold_integral, np.zeros(8))
    assert core.handle("switch_teleop", now=clock.now).outcome == "rejected"
    assert start(rig).outcome == "rejected"
    assert hardware.read_calls == reads
    assert len(hardware.current_history_ma) == writes
    calls = backend.calls
    backend.result = FakeResult.Error  # Using the old trajectory would now fail.
    for _ in range(3):
        result = tick(rig)
        assert result.mode == ControlMode.TELEOP
        assert result.target_position is result.target_velocity is None
        np.testing.assert_array_equal(result.tracking_torque[:7], np.zeros(7))
        gripper = core.config.controller.limit_barrier_control.gripper
        assert result.tracking_torque[-1] == pytest.approx(
            gripper.kp * (gripper.gripper_setpoint - result.state.gripper_position)
            - gripper.kd * result.state.gripper_velocity)
        assert result.wrote_current  # Normal compensation remains available.
    assert backend.calls == calls
    core.handle("switch_stop", now=clock.advance())
    assert core.mode == ControlMode.IDLE
    assert not hardware.torque_enabled
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))


@pytest.mark.parametrize("offset", [-0.001, 0.0, 0.001], ids=["before", "at", "after"])
def test_completion_at_timeout_boundary(rig, offset):
    entered = start(rig).entered_at
    backend = rig.backends[0]
    backend.result = FakeResult.Finished
    rig.clock.now = entered + rig.core.config.controller.sync_hold_control.ruckig.trajectory_timeout + offset
    result = rig.core.step(now=rig.clock.now)
    if offset < 0:
        assert result.mode == ControlMode.HOLD_SETTLING
        assert backend.calls == 1
        assert rig.hardware.torque_enabled
    else:
        assert result.mode == ControlMode.IDLE
        assert backend.calls == 0
        assert rig.core.last_transition.reason == "sync_timeout"
        assert result.target_position is result.target_velocity is None
        assert not rig.hardware.torque_enabled
        np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
        assert rig.core.handle("switch_teleop", now=rig.clock.now).outcome == "rejected"


@pytest.mark.parametrize("exit_path", ["sync_stop", "hold_stop", "timeout"])
def test_restart_captures_new_start_and_cannot_reuse_old_motion(rig, exit_path):
    start(rig)
    tick(rig)
    old = rig.backends[0]
    if exit_path == "hold_stop":
        old.result = FakeResult.Finished
        tick(rig)
        tick(rig, 0.02)
        assert np.any(rig.core.hold_integral)
    if exit_path == "timeout":
        tick(rig, rig.core.config.controller.sync_hold_control.ruckig.trajectory_timeout)
    else:
        reads = rig.hardware.read_calls
        rig.core.handle("switch_stop", now=rig.clock.advance())
        assert rig.hardware.read_calls == reads
    assert rig.core.mode == ControlMode.IDLE
    assert not rig.hardware.torque_enabled
    np.testing.assert_array_equal(rig.core.hold_integral, np.zeros(8))
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
    assert rig.core.handle("switch_teleop", now=rig.clock.now).outcome == "rejected"
    calls = old.calls
    old.result = FakeResult.Error
    q = np.linspace(0.02, 0.05, 8)
    dq = np.linspace(-0.02, 0.02, 8)
    rig.hardware.inject_state(q * rig.core.joint_signs + rig.core.joint_offsets,
                              dq * rig.core.joint_signs)
    rig.hardware.set_torque_enabled(True)
    rig.clock.advance()
    start(rig, target=0.2)
    assert len(rig.backends) == 2
    new = rig.backends[1]
    np.testing.assert_allclose(new.start_position, q)
    np.testing.assert_allclose(new.start_velocity, dq)
    np.testing.assert_allclose(new.target, 0.2)
    np.testing.assert_array_equal(rig.core.hold_integral, np.zeros(8))
    assert tick(rig).mode == ControlMode.TELEOP_SYNCING
    new.result = FakeResult.Finished
    result = tick(rig)
    assert result.mode == ControlMode.HOLD_SETTLING
    np.testing.assert_allclose(result.target_position, 0.2)
    np.testing.assert_array_equal(rig.core.hold_integral, np.zeros(8))
    assert old.calls == calls


@pytest.mark.parametrize("bad_target", [
    np.zeros((7, 1)), np.full(7, np.inf), np.full(7, -np.inf),
], ids=["matrix", "positive_inf", "negative_inf"])
def test_rejected_target_does_not_leave_motion_before_retry(rig, bad_target):
    with pytest.raises(ValueError, match="follower_position"):
        rig.core.handle("switch_sync", now=rig.clock.now, follower_position=bad_target)
    assert rig.core.mode == ControlMode.IDLE
    assert not rig.backends
    assert not rig.hardware.torque_enabled
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
    rig.hardware.set_torque_enabled(True)
    rig.clock.advance()
    start(rig)
    assert tick(rig).mode == ControlMode.TELEOP_SYNCING


def test_ruckig_error_result_stops_instead_of_becoming_synced(rig):
    start(rig)
    tick(rig)
    rig.backends[0].result = FakeResult.Error
    with pytest.raises(RuntimeError, match="Ruckig update failed"):
        tick(rig)
    assert rig.core.mode == ControlMode.IDLE
    assert not rig.hardware.torque_enabled
    np.testing.assert_array_equal(rig.hardware.last_currents_ma, np.zeros(8))
    np.testing.assert_array_equal(rig.core.hold_integral, np.zeros(8))
    assert rig.core.handle("switch_teleop", now=rig.clock.now).outcome == "rejected"
