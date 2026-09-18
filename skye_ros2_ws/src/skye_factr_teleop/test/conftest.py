# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Reusable pytest fixtures for hardware-independent control tests."""

from collections import deque
from collections.abc import Sequence

import numpy as np
import pytest

from skye_factr_teleop.dynamixel_driver import DynamixelError


class FakeDynamixelIO:
    """In-memory DynamixelIO implementation used only by tests.

    State can be injected between control cycles. Read and write exceptions are
    queued as one-shot failures so tests can model a transient timeout followed
    by recovery as well as a persistent failure by queuing multiple exceptions.
    """

    def __init__(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        *,
        torque_enabled: bool = False,
    ) -> None:
        self._positions = np.empty(0, dtype=float)
        self._velocities = np.empty(0, dtype=float)
        self._torque_enabled = bool(torque_enabled)
        self._closed = False
        self._read_failures: deque[Exception] = deque()
        self._write_failures: deque[Exception] = deque()
        self.last_currents_ma: np.ndarray | None = None
        self.current_history_ma: list[np.ndarray] = []
        self.inject_state(positions, velocities)

    @property
    def torque_enabled(self) -> bool:
        return self._torque_enabled

    @property
    def closed(self) -> bool:
        return self._closed

    def inject_state(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
    ) -> None:
        """Replace the position and velocity returned by the next read."""
        position_array = _finite_vector(positions, "positions")
        velocity_array = _finite_vector(velocities, "velocities")
        if position_array.shape != velocity_array.shape:
            raise ValueError("positions and velocities must have equal lengths")
        self._positions = position_array
        self._velocities = velocity_array

    def fail_next_read(self, error: Exception) -> None:
        """Queue a one-shot read exception such as TimeoutError."""
        self._read_failures.append(_exception(error))

    def fail_next_write(self, error: Exception) -> None:
        """Queue a one-shot communication exception for the next write."""
        self._write_failures.append(_exception(error))

    def read_state(self) -> tuple[np.ndarray, np.ndarray]:
        self._require_open()
        if self._read_failures:
            raise self._read_failures.popleft()
        return self._positions.copy(), self._velocities.copy()

    def write_currents(self, currents_ma: Sequence[float]) -> None:
        self._require_open()
        if self._write_failures:
            raise self._write_failures.popleft()
        if not self._torque_enabled:
            raise DynamixelError("write_currents requires torque to be enabled")
        currents = _finite_vector(currents_ma, "currents_ma")
        if currents.shape != self._positions.shape:
            raise ValueError(
                f"currents_ma must contain {self._positions.size} values, "
                f"got {currents.size}"
            )
        self.last_currents_ma = currents
        self.current_history_ma.append(currents.copy())

    def set_torque_enabled(self, enabled: bool) -> None:
        self._require_open()
        if not isinstance(enabled, bool):
            raise TypeError("enabled must be a bool")
        self._torque_enabled = enabled

    def close(self) -> None:
        self._torque_enabled = False
        self._closed = True

    def _require_open(self) -> None:
        if self._closed:
            raise DynamixelError("fake Dynamixel IO is closed")


def _finite_vector(values: Sequence[float], name: str) -> np.ndarray:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{name} must be a numeric sequence")
    try:
        result = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{name} must be a numeric sequence") from exc
    if result.ndim != 1:
        raise ValueError(f"{name} must be one-dimensional")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result.copy()


def _exception(error: Exception) -> Exception:
    if not isinstance(error, Exception):
        raise TypeError("failure must be an Exception instance")
    return error


@pytest.fixture
def fake_dynamixel_io() -> FakeDynamixelIO:
    """Return an eight-motor fake matching one leader arm plus gripper."""
    return FakeDynamixelIO(np.zeros(8), np.zeros(8))
