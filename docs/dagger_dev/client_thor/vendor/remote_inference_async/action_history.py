"""Executed policy-action history sent with each observation."""

from __future__ import annotations

import numpy as np

from remote_inference_async.protocol import ACTION_DIM
from remote_inference_async.units import robot_action_to_policy


class ExecutedActionHistory:
    """Keep a left-padded window of commands actually published to the robot."""

    def __init__(self, horizon: int) -> None:
        if horizon <= 0:
            raise ValueError("history horizon must be positive")
        self.horizon = horizon
        self.reset()

    def reset(self) -> None:
        self._actions = np.zeros((self.horizon, ACTION_DIM), dtype=np.float32)
        self._is_pad = np.ones(self.horizon, dtype=np.bool_)

    def append_robot_command(self, command: np.ndarray) -> None:
        policy_action = robot_action_to_policy(command)
        self._actions[:-1] = self._actions[1:]
        self._actions[-1] = policy_action
        self._is_pad[:-1] = self._is_pad[1:]
        self._is_pad[-1] = False

    def snapshot(self) -> tuple[np.ndarray, np.ndarray]:
        return self._actions.copy(), self._is_pad.copy()
