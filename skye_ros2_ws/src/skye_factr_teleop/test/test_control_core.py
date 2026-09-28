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
    ControlCore, ControlTerms, LeaderState, gravity_compensation,
    joint_limit_barrier, nullspace_regulation, select_joint_offsets,
    static_friction_compensation, transform_leader_state,
)


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
        gravity_comp=replace(config.controller.gravity_comp, mode=gravity_mode),
    ))
    hardware = FakeDynamixelIO(np.zeros(8), np.zeros(8), torque_enabled=torque_enabled)
    dynamics = FakeDynamics()
    core = ControlCore(config, hardware, dynamics, np.zeros(8), limit, terms)
    return core, hardware, dynamics


def test_quarter_turn_calibration_and_signed_state_mapping():
    raw = np.array([np.pi / 2 + 0.1, -np.pi / 2 - 0.1, 0, 0, 0, 0, 0, 0.8])
    signs = np.array([1, -1, 1, 1, 1, -1, -1, -1])
    original = raw.copy()
    offsets = select_joint_offsets(raw, signs, np.zeros(7))
    np.testing.assert_allclose(offsets[:2], [np.pi / 2, -np.pi / 2])
    assert offsets[-1] == 0.8
    state = transform_leader_state(raw, np.ones(8), offsets, signs)
    np.testing.assert_allclose(state.arm_position[:2], [0.1, 0.1])
    np.testing.assert_allclose(state.arm_velocity[:2], [1, -1])
    assert state.gripper_position == 0
    assert state.gripper_velocity == -1
    np.testing.assert_array_equal(raw, original)


def test_joint_limit_barrier_including_gripper_and_margin():
    state = LeaderState(np.array([1.95, -1.95, 0, 0, 0, 0, 0]), np.ones(7), 1.2, 0.5)
    result = joint_limit_barrier(state, [-2] * 7, [2] * 7, 2, 0.5, -1, 1, margin=0.1)
    np.testing.assert_allclose(result[:3], [-0.6, -0.4, 0])
    assert result[-1] == pytest.approx(-0.65)


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
    ) * core.config.controller.gravity_comp.gain
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
    arm, ctrl = core.config.arm_teleop, core.config.controller
    upper = arm.arm_joint_limits_max[0] - arm.arm_joint_limits_safety_margin
    expected_limit = -ctrl.joint_limit_barrier.kp * (position[0] - upper) - ctrl.joint_limit_barrier.kd * velocity[0]
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
    np.testing.assert_allclose(result.gravity_torque[:7], np.arange(1, 8) * core.config.controller.gravity_comp.gain)


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
