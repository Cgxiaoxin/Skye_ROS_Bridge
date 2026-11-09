"""Unit tests for PolicyActionChunk packing (no ROS import required)."""

from __future__ import annotations

import numpy as np
import pytest

from remote_inference_async.dagger_chunk import (
    ARBITER_CHUNK_SIZE,
    policy_chunk_to_arbiter_fields,
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
