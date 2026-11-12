"""Unit tests for RosGripperFeedback helpers (no ROS required)."""

from __future__ import annotations

import numpy as np
import pytest

from remote_inference_async.dagger_ros_facade import RosGripperFeedback


def test_validate_startup_range_accepts_open_and_partial():
    grip = RosGripperFeedback()
    grip._validate_startup_range(np.asarray([0.02, 0.4], dtype=np.float32))


def test_validate_startup_range_allows_fully_open_with_warning(caplog):
    grip = RosGripperFeedback()
    grip._validate_startup_range(np.zeros(2, dtype=np.float32))
    assert any("fully open" in r.message for r in caplog.records)


def test_validate_startup_range_rejects_out_of_unit_interval():
    grip = RosGripperFeedback()
    with pytest.raises(RuntimeError, match="outside \\[0,1\\]"):
        grip._validate_startup_range(np.asarray([0.1, 1.5], dtype=np.float32))


def test_on_state_clips_and_updates_age():
    grip = RosGripperFeedback()

    class _Msg:
        def __init__(self, value: float) -> None:
            self.position = [value]

    grip._on_state(0, _Msg(1.2))
    grip._on_state(1, _Msg(-0.1))
    assert float(grip._values[0]) == pytest.approx(1.0)
    assert float(grip._values[1]) == pytest.approx(0.0)
    assert grip._stamp_mono[0] is not None
    assert grip._stamp_mono[1] is not None
