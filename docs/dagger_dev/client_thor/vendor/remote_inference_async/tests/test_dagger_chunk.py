"""Unit tests for PolicyActionChunk packing (no ROS import required)."""

from __future__ import annotations

import numpy as np
import pytest

from remote_inference_async.dagger_chunk import (
    ARBITER_CHUNK_SIZE,
    policy_chunk_to_arbiter_fields,
    robot_command_deg_to_hold_chunk_rad,
    robot_command_deg_to_stream_chunk_rad,
)


def test_policy_chunk_to_arbiter_fields_layout():
    t = 16
    chunk = np.zeros((t, 16), dtype=np.float32)
    for i in range(t):
        chunk[i, 0] = 0.1 * i  # left J1
        chunk[i, 7] = -0.2 * i  # right J1
        chunk[i, 14] = 0.3  # left grip
        chunk[i, 15] = 0.4
    fields = policy_chunk_to_arbiter_fields(chunk, dt=1.0 / 30.0)
    assert fields["chunk_size"] == 16
    assert fields["dt"] == pytest.approx(1.0 / 30.0)
    assert len(fields["left_joints"]) == 16 * 7
    assert len(fields["right_joints"]) == 16 * 7
    assert fields["left_joints"][0] == pytest.approx(0.0)
    assert fields["left_joints"][7] == pytest.approx(0.1)  # step1 left J1
    assert fields["right_joints"][0] == pytest.approx(0.0)
    assert fields["right_joints"][7] == pytest.approx(-0.2)
    assert fields["left_gripper"] == pytest.approx([0.3] * 16)
    assert fields["right_gripper"] == pytest.approx([0.4] * 16)


def test_truncates_long_chunks_to_arbiter_size():
    chunk = np.arange(50 * 16, dtype=np.float32).reshape(50, 16) * 0.001
    fields = policy_chunk_to_arbiter_fields(chunk, dt=0.05)
    assert fields["chunk_size"] == ARBITER_CHUNK_SIZE
    assert len(fields["left_gripper"]) == ARBITER_CHUNK_SIZE
    # First step left J1 preserved
    assert fields["left_joints"][0] == pytest.approx(float(chunk[0, 0]))


def test_pads_short_chunks_by_repeating_last_pose():
    chunk = np.zeros((4, 16), dtype=np.float32)
    chunk[-1, 0] = 1.25
    fields = policy_chunk_to_arbiter_fields(chunk, dt=0.05)
    assert fields["chunk_size"] == 16
    # steps 3..15 are copies of last pose → left J1 of step 15
    assert fields["left_joints"][15 * 7] == pytest.approx(1.25)


def test_rejects_bad_shapes():
    with pytest.raises(ValueError):
        policy_chunk_to_arbiter_fields(np.zeros((16, 14)), dt=0.05)
    with pytest.raises(ValueError):
        policy_chunk_to_arbiter_fields(np.zeros((0, 16)), dt=0.05)
    with pytest.raises(ValueError):
        policy_chunk_to_arbiter_fields(np.zeros((16, 16)), dt=0.0)


def _bimanual_deg(left, right, lg=0.0, rg=0.1):
    return np.asarray(list(left) + list(right) + [lg, rg], dtype=np.float64)


def test_hold_chunk_repeats_target():
    command = _bimanual_deg(
        [-155.0, -35.0, 136.0, -126.0, 85.0, -38.0, 39.0],
        [-149.0, -6.0, 160.0, -118.0, -26.0, -48.0, -17.0],
    )
    hold = robot_command_deg_to_hold_chunk_rad(command)
    assert hold.shape == (16, 16)
    assert np.allclose(hold, hold[:1])
    assert hold[0, 0] == pytest.approx(np.deg2rad(-155.0))


def test_stream_chunk_interpolates_feedback_to_target():
    start = _bimanual_deg([0.0] * 7, [0.0] * 7, 0.0, 0.0)
    target = _bimanual_deg([3.0] + [0.0] * 6, [-1.5] + [0.0] * 6, 0.2, 0.4)
    stream = robot_command_deg_to_stream_chunk_rad(start, target)
    assert stream.shape == (16, 16)
    assert stream[0, 0] == pytest.approx(0.0)
    assert stream[-1, 0] == pytest.approx(np.deg2rad(3.0))
    assert stream[-1, 7] == pytest.approx(np.deg2rad(-1.5))
    assert stream[0, 14] == pytest.approx(0.0)
    assert stream[-1, 15] == pytest.approx(0.4)
    # linspace endpoint-inclusive: index k uses alpha=k/(steps-1).
    assert stream[8, 0] == pytest.approx(np.deg2rad(3.0 * 8.0 / 15.0))
    # Monotonic toward target.
    assert np.all(np.diff(stream[:, 0]) > 0.0)
