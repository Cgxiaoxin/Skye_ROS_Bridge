# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Numerical checks of the runtime model's gravity and local Jacobian."""

from pathlib import Path

import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
from skye_factr_teleop.dynamics import PinocchioDynamics


@pytest.mark.parametrize("side", ["left", "right"])
def test_model_gravity_and_jacobian_match_finite_differences(side):
    urdf = Path(__file__).parents[2] / "skye_factr_description" / "urdf" / f"m6{side}_isomorphism.urdf"
    dynamics = PinocchioDynamics(urdf)
    q = np.linspace(-0.2, 0.3, 7)
    dq = np.linspace(0.1, 0.4, 7)
    epsilon = 1e-6
    gravity = dynamics.gravity(q, np.zeros(7))
    numerical = []
    for axis in np.eye(7):
        plus = pin.computePotentialEnergy(dynamics.model, dynamics.data, q + epsilon * axis)
        minus = pin.computePotentialEnergy(dynamics.model, dynamics.data, q - epsilon * axis)
        numerical.append((plus - minus) / (2 * epsilon))
    np.testing.assert_allclose(gravity, numerical, atol=1e-7)
    assert np.linalg.norm(dynamics.gravity(q, dq) - gravity) > 1e-8
    transform = dynamics.forward_kinematics(q)
    moved = dynamics.forward_kinematics(q + epsilon * dq)
    numerical_velocity = pin.log6(transform.inverse() * moved).vector / epsilon
    np.testing.assert_allclose(dynamics.jacobian(q) @ dq, numerical_velocity, atol=1e-6)
