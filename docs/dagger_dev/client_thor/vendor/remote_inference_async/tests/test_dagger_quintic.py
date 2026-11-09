"""Unit tests for SDK-free quintic home helpers."""

from __future__ import annotations

import numpy as np
import pytest

from remote_inference_async.dagger_chunk import robot_command_deg_to_hold_chunk_rad
from remote_inference_async.dagger_quintic import (
    max_command_lead_deg,
    plan_quintic_trajectory_steps,
    quintic_smoothstep,
    sample_quintic_commands,
)


def test_quintic_endpoints():
    assert quintic_smoothstep(0.0) == pytest.approx(0.0)
    assert quintic_smoothstep(1.0) == pytest.approx(1.0)
    mid = quintic_smoothstep(0.5)
    assert 0.4 < mid < 0.6


def test_plan_steps_respects_peak_slope():
    starts = {0: np.zeros(7), 1: np.zeros(7)}
    targets = {0: np.array([10.0] + [0.0] * 6), 1: np.zeros(7)}
    steps, dist = plan_quintic_trajectory_steps(starts, targets, step_deg=0.5)
    assert dist == pytest.approx(10.0)
    # peak slope 1.875 → ceil(1.875*10/0.5)=38
    assert steps == 38


def test_sample_monotonic_and_step_bound():
    starts = {0: np.zeros(7), 1: np.zeros(7)}
    targets = {0: np.full(7, 5.0), 1: np.full(7, -5.0)}
    steps, _ = plan_quintic_trajectory_steps(starts, targets, step_deg=0.5)
    prev = sample_quintic_commands(
        starts, targets, trajectory_index=0, trajectory_steps=steps
    )
    for idx in range(1, steps + 1):
        cur = sample_quintic_commands(
            starts, targets, trajectory_index=idx, trajectory_steps=steps
        )
        for obj in (0, 1):
            delta = float(np.max(np.abs(cur[obj] - prev[obj])))
            assert delta <= 0.5 + 1e-6
        prev = cur
    assert np.allclose(prev[0], targets[0])
    assert np.allclose(prev[1], targets[1])


def test_max_command_lead():
    commands = {0: np.full(7, 3.0), 1: np.full(7, -1.0)}
    feedback = {0: np.zeros(7), 1: np.zeros(7)}
    lead, arm = max_command_lead_deg(commands, feedback)
    assert lead == pytest.approx(3.0)
    assert arm == 0


def test_robot_command_hold_is_radian():
    command = np.array(
        [-155.0, -35.0, 136.0, -126.0, 85.0, -38.0, 39.0]
        + [-149.0, -46.0, 168.0, -122.0, -78.0, -11.0, -28.0]
        + [0.1, 0.2],
        dtype=np.float64,
    )
    hold = robot_command_deg_to_hold_chunk_rad(command)
    assert hold.shape == (16, 16)
    assert float(np.max(np.abs(hold[:, :14]))) < 6.5
    assert hold[0, 0] == pytest.approx(np.deg2rad(-155.0))
    assert hold[-1, 14] == pytest.approx(0.1)
