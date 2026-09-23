# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Stage-3 control tests: no ROS, serial port, or Pinocchio required."""

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

    def gravity(self, q):
        return self.gravity_value.copy()

    def jacobian(self, q):
        return self.jacobian_value.copy()

    def forward_kinematics(self, q):
        return np.eye(4)


def _core(*, terms=None, torque_enabled=True, limit=100.0):
    config = load_config(CONFIG)
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
    gravity = gravity_compensation(q, dynamics, [0.5] * 7)
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
    dynamics.gravity = lambda q: (_ for _ in ()).throw(AssertionError("called dynamics"))
    result = core.step()
    assert not result.wrote_current
    assert hardware.current_history_ma == []


@pytest.mark.parametrize("bad_state", [float("nan"), float("inf")])
def test_invalid_state_zeros_previous_output_and_disables_torque(bad_state):
    core, hardware, _ = _core(terms=ControlTerms(False, True, False, False))
    core.step()
    assert np.any(hardware.last_currents_ma)
    hardware._positions[0] = bad_state  # inject invalid hardware read
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
