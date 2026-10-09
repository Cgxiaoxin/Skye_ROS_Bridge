# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Pinocchio runtime adapter for the seven-joint leader model."""

import numpy as np


class PinocchioDynamics:
    def __init__(self, urdf_path):
        import pinocchio as pin

        self._pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        if self.model.nq != 7 or self.model.nv != 7 or self.model.njoints != 8:
            raise ValueError("leader URDF must have seven scalar joints")
        self.data = self.model.createData()

    def gravity(self, q, dq):
        return np.array(self._pin.rnea(self.model, self.data, q, dq, np.zeros(7)), copy=True)

    def jacobian(self, q):
        return np.array(self._pin.computeJointJacobian(self.model, self.data, q, 7), copy=True)

    def forward_kinematics(self, q):
        """Return the end-effector pose; currently used only by the dynamics test."""
        self._pin.forwardKinematics(self.model, self.data, q)
        return self.data.oMi[7].copy()
