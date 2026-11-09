"""SDK-free quintic joint planning for HITL DAgger slow-home.

Mirrors the synchronized quintic planner used by
``robot_control.move_arms_to_joint_poses`` (peak slope 1.875, lead gating),
but returns poses only — no Gento writes.
"""

from __future__ import annotations

import math
from typing import Mapping

import numpy as np

ARM_DOF = 7


def quintic_smoothstep(progress: float) -> float:
    """Quintic blend with zero velocity/acceleration at endpoints."""
    p = float(progress)
    if p <= 0.0:
        return 0.0
    if p >= 1.0:
        return 1.0
    return p**3 * (10.0 + p * (-15.0 + 6.0 * p))


def plan_quintic_trajectory_steps(
    starts: Mapping[int, np.ndarray],
    targets: Mapping[int, np.ndarray],
    *,
    step_deg: float,
) -> tuple[int, float]:
    """Return (trajectory_steps, max_distance_deg) for peak-step <= step_deg."""
    if step_deg <= 0.0:
        raise ValueError(f"step_deg must be positive, got {step_deg}")
    if not starts or not targets or set(starts) != set(targets):
        raise ValueError("starts/targets must be non-empty with matching keys")
    max_distance_deg = 0.0
    for obj, target in targets.items():
        start = np.asarray(starts[obj], dtype=np.float64).reshape(ARM_DOF)
        goal = np.asarray(target, dtype=np.float64).reshape(ARM_DOF)
        max_distance_deg = max(
            max_distance_deg, float(np.max(np.abs(goal - start)))
        )
    steps = max(1, int(math.ceil(1.875 * max_distance_deg / step_deg)))
    return steps, max_distance_deg


def sample_quintic_commands(
    starts: Mapping[int, np.ndarray],
    targets: Mapping[int, np.ndarray],
    *,
    trajectory_index: int,
    trajectory_steps: int,
) -> dict[int, np.ndarray]:
    """Sample absolute degree joint commands at one quintic index."""
    if trajectory_steps <= 0:
        raise ValueError("trajectory_steps must be positive")
    progress = float(trajectory_index) / float(trajectory_steps)
    blend = quintic_smoothstep(progress)
    out: dict[int, np.ndarray] = {}
    for obj, target in targets.items():
        start = np.asarray(starts[obj], dtype=np.float64).reshape(ARM_DOF)
        goal = np.asarray(target, dtype=np.float64).reshape(ARM_DOF)
        out[int(obj)] = start + blend * (goal - start)
    return out


def max_command_lead_deg(
    commands: Mapping[int, np.ndarray],
    feedback: Mapping[int, np.ndarray],
) -> tuple[float, int]:
    """Return (max |command-feedback| deg, arm_id with the largest lead)."""
    max_lead = 0.0
    blocked_arm = -1
    for obj, command in commands.items():
        current = np.asarray(feedback[obj], dtype=np.float64).reshape(ARM_DOF)
        cmd = np.asarray(command, dtype=np.float64).reshape(ARM_DOF)
        lead = float(np.max(np.abs(cmd - current)))
        if lead > max_lead:
            max_lead = lead
            blocked_arm = int(obj)
    return max_lead, blocked_arm
