# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Ruckig runtime adapter for eight-axis leader trajectories."""

import numpy as np

from .config import MOTOR_COUNT
from .control_core import TrajectorySample, _scalar, _vector


class RuckigTrajectory:
    """Eight-axis Ruckig adapter, imported only when explicitly constructed.

    Limits are mandatory, in rad/s, rad/s² and rad/s³ in logical leader order.
    No acceleration measurement is available, so initial acceleration is zero.
    Target velocity/acceleration are zero. Each update advances one fixed cycle.
    """

    def __init__(
        self, *, frequency: float, max_velocity: object,
        max_acceleration: object, max_jerk: object,
    ) -> None:
        frequency = _scalar(frequency, "frequency")
        if frequency <= 0:
            raise ValueError("frequency must be positive")
        limits = [
            _vector(value, MOTOR_COUNT, name)
            for value, name in (
                (max_velocity, "max_velocity"),
                (max_acceleration, "max_acceleration"), (max_jerk, "max_jerk"),
            )
        ]
        if any(np.any(value <= 0) for value in limits):
            raise ValueError("trajectory limits must be positive")
        from ruckig import InputParameter, OutputParameter, Result, Ruckig

        self._result_type = Result
        self._otg = Ruckig(MOTOR_COUNT, 1.0 / frequency)
        self._input = InputParameter(MOTOR_COUNT)
        self._output = OutputParameter(MOTOR_COUNT)
        self._input.max_velocity = limits[0].tolist()
        self._input.max_acceleration = limits[1].tolist()
        self._input.max_jerk = limits[2].tolist()
        self._started = False

    def start(self, position: np.ndarray, velocity: np.ndarray, target: np.ndarray) -> None:
        self._started = False
        self._otg.reset()
        self._input.current_position = _vector(position, MOTOR_COUNT, "position").tolist()
        self._input.current_velocity = _vector(velocity, MOTOR_COUNT, "velocity").tolist()
        self._input.current_acceleration = [0.0] * MOTOR_COUNT
        self._input.target_position = _vector(target, MOTOR_COUNT, "target").tolist()
        self._input.target_velocity = [0.0] * MOTOR_COUNT
        self._input.target_acceleration = [0.0] * MOTOR_COUNT
        self._otg.validate_input(self._input, True, True)
        self._started = True

    def update(self) -> TrajectorySample:
        if not self._started:
            raise RuntimeError("trajectory has not been started")
        result = self._otg.update(self._input, self._output)
        if result not in (self._result_type.Working, self._result_type.Finished):
            raise RuntimeError(f"Ruckig update failed: {result}")
        sample = TrajectorySample(
            _vector(self._output.new_position, MOTOR_COUNT, "trajectory position"),
            _vector(self._output.new_velocity, MOTOR_COUNT, "trajectory velocity"),
            result == self._result_type.Finished,
        )
        self._output.pass_to_input(self._input)
        return sample
