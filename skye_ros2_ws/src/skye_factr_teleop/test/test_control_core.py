# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Stage-3 control tests: no ROS, serial port, or Pinocchio required."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from conftest import FakeDynamixelIO
from skye_factr_teleop.config import load_config
from skye_factr_teleop.control_core import (
    ControlCore, ControlTerms, GravityTestSettings, LeaderState, gravity_compensation,
    joint_limit_barrier, LimitTestSettings, FrictionTestSettings, nullspace_regulation,
    static_friction_compensation, transform_leader_state,
)
from skye_factr_teleop.state_machine import ControlMode


CONFIG = Path(__file__).parents[1] / "config" / "grav_comp_m6_left.yaml"


class FakeDynamics:
    def __init__(self):
        self.gravity_value = np.arange(1.0, 8.0)
        self.jacobian_value = np.eye(6, 7)

    def gravity(self, q, dq):
        return self.gravity_value.copy()

    def jacobian(self, q):
        return self.jacobian_value.copy()

    def forward_kinematics(self, q):
        return np.eye(4)


def _core(*, terms=None, torque_enabled=True, limit=100.0, gravity_mode="position_velocity"):
    config = load_config(CONFIG)
    config = replace(config, controller=replace(
        config.controller,
        joint_gravity_comp=replace(config.controller.joint_gravity_comp, mode=gravity_mode),
    ))
    hardware = FakeDynamixelIO(np.zeros(8), np.zeros(8), torque_enabled=torque_enabled)
    dynamics = FakeDynamics()
    core = ControlCore(config, hardware, dynamics, np.zeros(8), limit, terms)
    return core, hardware, dynamics


def test_signed_state_mapping():
    raw = np.array([np.pi / 2 + 0.1, -np.pi / 2 - 0.1, 0, 0, 0, 0, 0, 0.8])
    signs = np.array([1, -1, 1, 1, 1, -1, -1, -1])
    original = raw.copy()
    offsets = np.array([np.pi / 2, -np.pi / 2, 0, 0, 0, 0, 0, 0.8])
    state = transform_leader_state(raw, np.ones(8), offsets, signs)
    np.testing.assert_allclose(state.arm_position[:2], [0.1, 0.1])
    np.testing.assert_allclose(state.arm_velocity[:2], [1, -1])
    assert state.gripper_position == 0
    assert state.gripper_velocity == -1
    np.testing.assert_array_equal(raw, original)


def test_joint_limit_barrier_including_gripper_and_margin():
    state = LeaderState(np.array([1.95, -1.95, 0, 0, 0, 0, 0]), np.ones(7), 1.2, 0.5)
    result = joint_limit_barrier(
        state, [-2] * 7, [2] * 7, 2, 0.5, -1, 1,
        margin=0.1, gripper_kp=3, gripper_kd=0.25,
    )
    np.testing.assert_allclose(result[:3], [-0.6, 0.1, 0])
    assert result[-1] == pytest.approx(-0.725)


def test_limit_barrier_damping_never_reverses_recovery():
    state = LeaderState(np.array([-1.95, 1.95, 0, 0, 0, 0, 0]),
                        np.array([10.0, -10.0, 0, 0, 0, 0, 0]), -1.1, 10.0)
    torque = joint_limit_barrier(
        state, [-2] * 7, [2] * 7, 2, 0.5, -1, 1,
        margin=0.1, gripper_kp=3, gripper_kd=0.25,
    )
    assert torque[0] > 0 and torque[1] < 0 and torque[7] > 0


def test_gravity_and_stiction_fixed_outputs_and_no_input_mutation():
    dynamics = FakeDynamics()
    q = np.zeros(7)
    gravity = gravity_compensation(q, np.zeros(7), dynamics, [0.5] * 7)
    np.testing.assert_allclose(gravity, np.arange(1, 8) * 0.5)
    flags = np.zeros(7, dtype=bool)
    torque, next_flags = static_friction_compensation(
        [0, 0.1, 0, 0, 0, 0, 0], gravity, [0.05] * 7,
        [0.2] * 7, flags,
    )
    np.testing.assert_allclose(torque[:3], [-0.1, 0, -0.3])
    assert next_flags.tolist()[:3] == [True, False, True]
    assert not flags.any()
    np.testing.assert_array_equal(q, np.zeros(7))


@pytest.mark.parametrize("mode, expected_speed", [("position_velocity", 0.2), ("position_only", 0.0)])
def test_gravity_passes_velocity_and_protects_inputs_from_dynamics(mode, expected_speed):
    dynamics = FakeDynamics()
    q = np.arange(7, dtype=float)
    dq = np.full(7, 0.2)

    def inverse_dynamics(position, velocity):
        np.testing.assert_array_equal(position, np.arange(7))
        np.testing.assert_array_equal(velocity, np.full(7, expected_speed))
        torque = position + 2 * velocity
        position[:] = 99
        velocity[:] = 99
        return torque

    dynamics.gravity = inverse_dynamics
    result = gravity_compensation(q, dq, dynamics, [0.5] * 7, mode=mode)
    np.testing.assert_allclose(result, np.arange(7) * 0.5 + expected_speed)
    np.testing.assert_array_equal(q, np.arange(7))
    np.testing.assert_array_equal(dq, np.full(7, 0.2))


@pytest.mark.parametrize("gravity_enabled", [True, False])
@pytest.mark.parametrize("mode", ["position_velocity", "position_only"])
def test_cycle_shares_velocity_dependent_compensation_with_stiction(gravity_enabled, mode):
    core, hardware, dynamics = _core(
        terms=ControlTerms(False, gravity_enabled, True, False),
        gravity_mode=mode,
    )
    raw_velocity = np.full(8, 0.02)
    raw_velocity[1] = 0.1  # Above the friction threshold, even in position_only mode.
    hardware.inject_state(np.zeros(8), raw_velocity)
    expected_velocity = raw_velocity[:7] * core.joint_signs[:7]
    model_velocity = expected_velocity if mode == "position_velocity" else np.zeros(7)
    calls = []

    def inverse_dynamics(q, dq):
        calls.append((q.copy(), dq.copy()))
        # A known velocity-dependent output exposes dropped or zeroed dq.
        return np.arange(1.0, 8.0) + 10 * dq

    dynamics.gravity = inverse_dynamics
    first = core.step()
    second = core.step()
    expected = (
        np.arange(1.0, 8.0) + 10 * model_velocity
    ) * core.config.controller.joint_gravity_comp.gain
    expected_friction = np.abs(expected) * core.config.controller.static_friction_comp.gain
    expected_friction[1] = 0
    assert len(calls) == 2  # One dynamics calculation per cycle for both terms.
    for q, dq in calls:
        np.testing.assert_array_equal(q, np.zeros(7))
        np.testing.assert_allclose(dq, model_velocity)
    np.testing.assert_allclose(first.gravity_torque[:7], expected if gravity_enabled else 0)
    np.testing.assert_allclose(first.friction_torque[:7], expected_friction)
    np.testing.assert_allclose(second.friction_torque[:7], -expected_friction)
    np.testing.assert_allclose(first.state.arm_velocity, expected_velocity)
    assert first.friction_torque[-1] == 0


@pytest.mark.parametrize("gravity_enabled, friction_enabled", [
    (False, False), (False, True), (True, False), (True, True),
])
def test_default_control_terms_are_independent(gravity_enabled, friction_enabled):
    config = load_config(CONFIG)
    config = replace(config, controller=replace(
        config.controller,
        joint_gravity_comp=replace(config.controller.joint_gravity_comp, enable=gravity_enabled),
        static_friction_comp=replace(config.controller.static_friction_comp, enable=friction_enabled),
    ))
    hardware = FakeDynamixelIO(np.zeros(8), np.zeros(8), torque_enabled=True)
    core = ControlCore(config, hardware, FakeDynamics(), np.zeros(8), 100.0)
    assert core.terms.gravity is gravity_enabled
    assert core.terms.static_friction is friction_enabled

    result = core.step()
    expected_gravity = np.arange(1, 8) * config.controller.joint_gravity_comp.gain
    expected_friction = np.abs(expected_gravity) * config.controller.static_friction_comp.gain
    np.testing.assert_allclose(result.gravity_torque[:7], expected_gravity if gravity_enabled else 0)
    np.testing.assert_allclose(result.friction_torque[:7], expected_friction if friction_enabled else 0)


def test_position_only_preserves_joint_limit_and_nullspace_damping():
    core, hardware, dynamics = _core(
        terms=ControlTerms(True, True, False, True), gravity_mode="position_only",
    )
    position = np.array([3.2, 0, 0, 0, 0, 0, 0.1, 0])
    velocity = np.full(8, 0.2)
    hardware.inject_state(position * core.joint_signs, velocity * core.joint_signs)

    def gravity(q, dq):
        np.testing.assert_allclose(q, position[:7])
        np.testing.assert_array_equal(dq, np.zeros(7))
        return np.zeros(7)

    dynamics.gravity = gravity
    result = core.step()
    ctrl = core.config.controller
    barrier = ctrl.limit_barrier_control
    upper = barrier.joint.limit_max[0] - barrier.joint.safety_margin
    expected_limit = -barrier.joint.kp * (position[0] - upper) - barrier.joint.kd * velocity[0]
    n = ctrl.null_space_regulation
    expected_nullspace = -n.kp[6] * (position[6] - n.null_space_joint_target[6]) - n.kd[6] * velocity[6]
    assert result.joint_limit_torque[0] == pytest.approx(expected_limit)
    assert result.nullspace_torque[6] == pytest.approx(expected_nullspace)
    np.testing.assert_allclose(result.state.arm_velocity, velocity[:7])


def test_invalid_gravity_mode_fails_before_dynamics():
    dynamics = FakeDynamics()
    dynamics.gravity = lambda q, dq: pytest.fail("unexpected dynamics call")
    with pytest.raises(ValueError, match="gravity mode"):
        gravity_compensation(np.zeros(7), np.zeros(7), dynamics, np.ones(7), mode="unknown")


def test_nullspace_projection_only_acts_on_seventh_joint():
    dynamics = FakeDynamics()
    result = nullspace_regulation(
        np.ones(7), np.zeros(7), dynamics, np.zeros(7),
        [2] * 7, [0] * 7,
    )
    np.testing.assert_allclose(result, [0, 0, 0, 0, 0, 0, -2])


def test_one_cycle_per_call_and_model_current_limit():
    core, hardware, _ = _core(terms=ControlTerms(False, True, False, False), limit=20)
    for _ in range(3):
        result = core.step()
        assert result.wrote_current
        assert np.max(np.abs(result.physical_current_ma)) <= 20
    assert len(hardware.current_history_ma) == 3
    np.testing.assert_allclose(result.gravity_torque[:7], np.arange(1, 8) * core.config.controller.joint_gravity_comp.gain)


def test_gravity_test_isolates_one_joint_ramps_and_times_out():
    core, hardware, _ = _core(limit=50)
    core.gravity_test = GravityTestSettings(6, 0.5, 20.0, 2.0, 5.0)
    started = core.handle("switch_gravity_test", now=0.0)
    assert started.state == ControlMode.GRAVITY_TEST

    first = core.step(now=1.0)
    expected = 6 * core.config.controller.joint_gravity_comp.gain[5] * 0.5 * 0.5
    np.testing.assert_allclose(first.gravity_torque[5], expected)
    np.testing.assert_array_equal(np.delete(first.logical_torque, 5), np.zeros(7))
    assert first.physical_current_ma[5] < 0
    assert np.max(np.abs(first.physical_current_ma)) <= 20
    assert not np.any(first.joint_limit_torque)
    assert not np.any(first.friction_torque)
    assert not np.any(first.nullspace_torque)
    assert not np.any(first.tracking_torque)

    full = core.step(now=2.0)
    np.testing.assert_allclose(full.gravity_torque[5], expected * 2)
    expired = core.step(now=5.0)
    assert expired.mode == ControlMode.IDLE
    assert core.last_transition.reason == "gravity_test_timeout"
    assert not expired.wrote_current
    assert not hardware.torque_enabled
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))


@pytest.mark.parametrize("joint_id,position,direction", [
    (1, -2.95, 1), (1, 2.95, -1), (8, -0.1, 1), (8, 0.6, -1),
])
def test_limit_test_isolated_ramped_clipped_and_stopped(joint_id, position, direction):
    core, hardware, dynamics = _core(limit=50)
    core.limit_test = LimitTestSettings(joint_id, 0.5, 20, 2, 5)
    raw = np.zeros(8)
    raw[joint_id - 1] = position * core.joint_signs[joint_id - 1]
    hardware.inject_state(raw, np.zeros(8))
    dynamics.gravity = lambda q, dq: (_ for _ in ()).throw(AssertionError("dynamics called"))
    core.handle("switch_limit_test", now=0)
    first = core.step(now=1)
    assert first.mode == ControlMode.LIMIT_TEST
    assert first.joint_limit_torque[joint_id - 1] * direction > 0
    np.testing.assert_array_equal(np.delete(first.logical_torque, joint_id - 1), np.zeros(7))
    assert not np.any(first.gravity_torque)
    assert not np.any(first.friction_torque)
    assert not np.any(first.tracking_torque)
    assert abs(first.physical_current_ma[joint_id - 1]) <= 10
    full = core.step(now=2)
    assert abs(full.joint_limit_torque[joint_id - 1]) == pytest.approx(
        2 * abs(first.joint_limit_torque[joint_id - 1]))
    assert abs(full.physical_current_ma[joint_id - 1]) <= 20
    core.step(now=5)
    assert core.mode == ControlMode.IDLE
    assert core.last_transition.reason == "limit_test_timeout"
    assert not hardware.torque_enabled
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))


def test_limit_test_inside_safe_interval_has_zero_output():
    core, hardware, _ = _core(limit=50)
    core.limit_test = LimitTestSettings(1, 0.5, 20, 2, 5)
    core.handle("switch_limit_test", now=0)
    result = core.step(now=2)
    np.testing.assert_array_equal(result.logical_torque, np.zeros(8))
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))


def test_friction_test_isolated_alternating_speed_gated_and_stopped():
    core, hardware, _ = _core(limit=50)
    core.friction_test = FrictionTestSettings(6, 0.5, 20, 2, 5)
    core.handle("switch_friction_test", now=0)
    first = core.step(now=1)
    reference = 6 * core.config.controller.joint_gravity_comp.gain[5]
    amplitude = reference * core.config.controller.static_friction_comp.gain[5]
    assert first.friction_torque[5] == pytest.approx(amplitude * 0.5 * 0.5)
    np.testing.assert_array_equal(np.delete(first.logical_torque, 5), np.zeros(7))
    assert not np.any(first.gravity_torque)
    assert not np.any(first.joint_limit_torque)
    assert not np.any(first.nullspace_torque)
    assert not np.any(first.tracking_torque)
    assert abs(first.physical_current_ma[5]) <= 10
    second = core.step(now=2)
    assert second.friction_torque[5] == pytest.approx(-amplitude * 0.5)
    assert abs(second.physical_current_ma[5]) <= 20

    velocity = np.zeros(8)
    velocity[5] = core.config.controller.static_friction_comp.enable_speed[5]
    hardware.inject_state(np.zeros(8), velocity * core.joint_signs)
    gated = core.step(now=3)
    np.testing.assert_array_equal(gated.logical_torque, np.zeros(8))
    expired = core.step(now=5)
    assert expired.mode == ControlMode.IDLE
    assert core.last_transition.reason == "friction_test_timeout"
    assert not hardware.torque_enabled
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))


def test_friction_test_disabled_control_has_zero_output():
    core, hardware, dynamics = _core(limit=50)
    core.friction_test = FrictionTestSettings(1, 0.5, 20, 2, 5, enabled=False)
    dynamics.gravity = lambda q, dq: pytest.fail("disabled run called dynamics")
    core.handle("switch_friction_test", now=0)
    result = core.step(now=2)
    np.testing.assert_array_equal(result.logical_torque, np.zeros(8))
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))


def test_friction_test_switches_off_on_without_extending_timeout():
    core, hardware, _ = _core(limit=50)
    core.friction_test = FrictionTestSettings(1, 0.5, 20, 2, 5, enabled=False)
    core.handle("switch_friction_test", now=0)
    assert not core.friction_test_enabled
    assert not np.any(core.step(now=1).logical_torque)
    assert core.set_friction_test_enabled(True, now=2)
    assert core.friction_test_enabled
    assert not np.any(hardware.last_currents_ma)  # An explicit zero precedes the new phase.
    first = core.step(now=3)
    assert first.friction_torque[0] > 0
    assert abs(first.physical_current_ma[0]) <= 10
    assert core.set_friction_test_enabled(False, now=3.1)
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))
    assert not np.any(core.step(now=3.2).logical_torque)
    assert core.set_friction_test_enabled(True, now=3.3)
    assert core.step(now=3.4).friction_torque[0] > 0  # Sign and ramp reset.
    assert core.step(now=5).mode == ControlMode.IDLE
    assert not hardware.torque_enabled
    assert core.last_transition.reason == "friction_test_timeout"


def test_friction_switch_requires_active_test():
    core, hardware, _ = _core(limit=50)
    core.friction_test = FrictionTestSettings(1, 0.5, 20, 2, 5, enabled=False)
    with pytest.raises(ValueError, match="FRICTION_TEST"):
        core.set_friction_test_enabled(True, now=1)
    assert not hardware.torque_enabled


def test_friction_test_allows_300_seconds_without_extending_on_switch():
    core, hardware, _ = _core(limit=50)
    core.friction_test = FrictionTestSettings(1, 0.5, 20, 2, 300, enabled=False)
    core.handle("switch_friction_test", now=0)
    assert core.step(now=299).mode == ControlMode.FRICTION_TEST
    assert core.set_friction_test_enabled(True, now=299)
    assert core.step(now=299.5).mode == ControlMode.FRICTION_TEST
    assert core.step(now=300).mode == ControlMode.IDLE
    assert not hardware.torque_enabled
    assert core.last_transition.reason == "friction_test_timeout"


@pytest.mark.parametrize("values", [
    (0, 0.5, 20, 2, 5), (8, 0.5, 20, 2, 5),
    (1, 1.1, 20, 2, 5), (1, 0.5, 51, 2, 5),
    (1, 0.5, 20, 0.5, 5), (1, 0.5, 20, 2, 301),
    (1, 0.5, 20, 2, 5, 1),
])
def test_friction_test_settings_reject_unsafe_values(values):
    with pytest.raises(ValueError, match="friction test"):
        FrictionTestSettings(*values)


@pytest.mark.parametrize("values", [
    (0, 0.5, 20, 2, 5), (9, 0.5, 20, 2, 5),
    (1, 1.1, 20, 2, 5), (1, 0.5, 51, 2, 5),
    (1, 0.5, 20, 0.5, 5), (1, 0.5, 20, 2, 31),
])
def test_limit_test_settings_reject_unsafe_values(values):
    with pytest.raises(ValueError, match="limit test"):
        LimitTestSettings(*values)


@pytest.mark.parametrize("values", [
    (0, 0.5, 20.0, 2.0, 5.0),
    (1, 1.1, 20.0, 2.0, 5.0),
    (1, 0.5, 20.0, 5.0, 5.0),
    (1, 0.5, 51.0, 2.0, 5.0),
    (1, 0.5, 20.0, 0.5, 5.0),
    (1, 0.5, 20.0, 2.0, 31.0),
])
def test_gravity_test_settings_reject_unsafe_values(values):
    with pytest.raises(ValueError, match="gravity test"):
        GravityTestSettings(*values)


def test_physical_direction_mapping_and_model_rating_clip():
    core, hardware, dynamics = _core(terms=ControlTerms(False, True, False, False), limit=5000)
    dynamics.gravity_value[:] = 100
    result = core.step()
    assert result.physical_current_ma[0] > 0
    assert result.physical_current_ma[5] < 0
    for current, model in zip(result.physical_current_ma, core.config.dynamixel.servo_types):
        from skye_factr_teleop.dynamixel_driver import get_servo_spec
        assert abs(current) <= get_servo_spec(model).current_limit_ma
    np.testing.assert_array_equal(hardware.last_currents_ma, result.physical_current_ma)


def test_per_servo_current_limits_clip_independently():
    limits = np.array([10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0])
    core, _, dynamics = _core(terms=ControlTerms(False, True, False, False), limit=limits)
    dynamics.gravity_value[:] = 100
    result = core.step()
    np.testing.assert_allclose(np.abs(result.physical_current_ma[:7]), limits[:7])


def test_each_term_can_be_switched_off_independently():
    core, hardware, _ = _core(terms=ControlTerms(False, False, False, False))
    result = core.step()
    np.testing.assert_array_equal(result.logical_torque, np.zeros(8))
    core.terms = ControlTerms(False, False, True, False)
    result = core.step()
    assert np.any(result.friction_torque[:7])
    np.testing.assert_array_equal(result.gravity_torque, np.zeros(8))
    core.terms = ControlTerms(False, False, False, True)
    hardware.inject_state(np.ones(8), np.zeros(8))
    result = core.step()
    assert result.nullspace_torque[6] != 0
    core.terms = ControlTerms(True, False, False, False)
    hardware.inject_state(np.full(8, 4.0), np.zeros(8))
    result = core.step()
    assert np.any(result.joint_limit_torque)


def test_torque_disabled_does_not_write_or_call_dynamics():
    core, hardware, dynamics = _core(torque_enabled=False)
    dynamics.gravity = lambda q, dq: (_ for _ in ()).throw(AssertionError("called dynamics"))
    result = core.step()
    assert not result.wrote_current
    assert hardware.current_history_ma == []


@pytest.mark.parametrize("bad_state", [float("nan"), float("inf")])
@pytest.mark.parametrize("field", ["_positions", "_velocities"])
def test_invalid_state_zeros_previous_output_and_disables_torque(bad_state, field):
    core, hardware, _ = _core(terms=ControlTerms(False, True, False, False))
    core.step()
    assert np.any(hardware.last_currents_ma)
    getattr(hardware, field)[0] = bad_state  # inject invalid hardware read
    with pytest.raises(ValueError, match="finite"):
        core.step()
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))
    assert not hardware.torque_enabled


def test_invalid_dynamics_zeros_previous_output_and_disables_torque():
    core, hardware, dynamics = _core(terms=ControlTerms(False, True, False, False))
    core.step()
    dynamics.gravity_value[0] = np.nan
    with pytest.raises(ValueError, match="gravity"):
        core.step()
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))
    assert not hardware.torque_enabled


def test_read_failure_zeros_previous_output_and_disables_torque():
    core, hardware, _ = _core(terms=ControlTerms(False, True, False, False))
    core.step()
    hardware.fail_next_read(TimeoutError("sync read timeout"))
    with pytest.raises(TimeoutError, match="sync read timeout"):
        core.step()
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))
    assert not hardware.torque_enabled


def test_write_failure_disables_torque_after_zero_retry():
    core, hardware, _ = _core(terms=ControlTerms(False, True, False, False))
    hardware.fail_next_write(TimeoutError("sync write timeout"))
    with pytest.raises(TimeoutError, match="sync write timeout"):
        core.step()
    np.testing.assert_array_equal(hardware.last_currents_ma, np.zeros(8))
    assert not hardware.torque_enabled


def test_invalid_constructor_inputs_do_not_touch_hardware():
    config = load_config(CONFIG)
    hardware = FakeDynamixelIO(np.zeros(8), np.zeros(8), torque_enabled=True)
    with pytest.raises(ValueError, match="joint_offsets"):
        ControlCore(config, hardware, FakeDynamics(), [0] * 7, 50)
    with pytest.raises(ValueError, match="current_limit_ma"):
        ControlCore(config, hardware, FakeDynamics(), [0] * 8, -1)
    assert hardware.current_history_ma == []
