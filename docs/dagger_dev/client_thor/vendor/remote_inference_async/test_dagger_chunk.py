"""Unit tests for PolicyActionChunk packing (no ROS import required)."""

from __future__ import annotations

import numpy as np
import pytest

from remote_inference_async.dagger_chunk import (
    ARBITER_CHUNK_SIZE,
    policy_chunk_to_arbiter_fields,
    setpoint_dt,
    setpoint_ramp_to_arbiter_fields,
)
from remote_inference_async.dagger_setpoint import ErrMaxGuard



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


def test_setpoint_dt_spans_one_action_period():
    dt = setpoint_dt(30.0)
    assert dt == pytest.approx(1.0 / (30.0 * 16))
    assert dt * ARBITER_CHUNK_SIZE == pytest.approx(1.0 / 30.0)


def test_setpoint_ramp_linear_in_joint_and_gripper():
    start = np.zeros(16, dtype=np.float64)
    end = np.zeros(16, dtype=np.float64)
    end[0] = 0.16  # left J1
    end[14] = 0.8  # left gripper
    dt = setpoint_dt(30.0)
    fields = setpoint_ramp_to_arbiter_fields(start, end, dt=dt)
    assert fields["chunk_size"] == 16
    assert fields["dt"] == pytest.approx(dt)
    assert fields["left_joints"][0] == pytest.approx(0.0)
    assert fields["left_joints"][15 * 7] == pytest.approx(0.16)
    assert fields["left_joints"][8 * 7] == pytest.approx(0.16 * 8 / 15)
    assert fields["left_gripper"][0] == pytest.approx(0.0)
    assert fields["left_gripper"][-1] == pytest.approx(0.8)


def test_err_max_guard_holds_when_command_leads_feedback():
    guard = ErrMaxGuard(err_max_rad=0.15, sustained_s=0.5)
    last_cmd = np.zeros(16, dtype=np.float64)
    feedback = np.zeros(16, dtype=np.float64)
    # 10 deg ≈ 0.1745 rad > 0.15
    last_cmd[0] = 10.0
    decision = guard.evaluate(last_cmd, feedback, now=1000.0)
    assert decision.hold is True
    assert decision.err_rad == pytest.approx(np.deg2rad(10.0))
    assert decision.trip_count == 1
    sustained = guard.evaluate(last_cmd, feedback, now=1000.6)
    assert sustained.hold is True
    assert sustained.sustained is True
