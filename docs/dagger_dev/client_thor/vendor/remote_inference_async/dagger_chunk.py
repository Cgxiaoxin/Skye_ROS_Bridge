"""Pack UniJEPA policy actions into HITL DAgger /skye/policy_action fields.

Policy chunk layout (radian absolute): [T, 16] =
  left_arm[7] + right_arm[7] + left_gripper + right_gripper.

control_arbiter ChunkPlayer currently requires chunk_size == 16.
"""

from __future__ import annotations

from typing import Any

import numpy as np

ARBITER_CHUNK_SIZE = 16
ACTION_DIM = 16
ARM_DOF = 7
ARM_JOINT_DIM = 14


def setpoint_dt(model_action_hz: float) -> float:
    """Micro-ramp step dt so 16 steps span exactly one model action period."""
    hz = float(model_action_hz)
    if not np.isfinite(hz) or hz <= 0.0:
        raise ValueError(f"model_action_hz must be finite and > 0, got {model_action_hz}")
    return 1.0 / (hz * ARBITER_CHUNK_SIZE)


def _actions_to_fields(actions: np.ndarray, dt: float) -> dict[str, Any]:
    left_steps = actions[:, 0:ARM_DOF]
    right_steps = actions[:, ARM_DOF : 2 * ARM_DOF]
    left_grip = actions[:, 14].tolist()
    right_grip = actions[:, 15].tolist()
    return {
        "chunk_size": ARBITER_CHUNK_SIZE,
        "dt": float(dt),
        "left_joints": left_steps.reshape(-1).tolist(),
        "right_joints": right_steps.reshape(-1).tolist(),
        "left_gripper": left_grip,
        "right_gripper": right_grip,
    }


def policy_chunk_to_arbiter_fields(
    policy_actions: np.ndarray,
    *,
    dt: float,
) -> dict[str, Any]:
    """Convert absolute-radian [T,16] policy actions to PolicyActionChunk fields."""
    actions = np.asarray(policy_actions, dtype=np.float64)
    if actions.ndim != 2 or actions.shape[1] != ACTION_DIM or actions.shape[0] == 0:
        raise ValueError(
            f"policy actions must be non-empty [T,{ACTION_DIM}], got {actions.shape}"
        )
    if not np.isfinite(actions).all():
        raise ValueError("policy actions contain NaN or infinity")
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError(f"dt must be finite and > 0, got {dt}")

    if actions.shape[0] > ARBITER_CHUNK_SIZE:
        actions = actions[:ARBITER_CHUNK_SIZE]
    elif actions.shape[0] < ARBITER_CHUNK_SIZE:
        # Pad by holding the last pose so ChunkPlayer (fixed 16) accepts the msg.
        pad = np.repeat(actions[-1:], ARBITER_CHUNK_SIZE - actions.shape[0], axis=0)
        actions = np.concatenate([actions, pad], axis=0)

    return _actions_to_fields(actions, dt)


def setpoint_ramp_to_arbiter_fields(
    start_action: np.ndarray,
    end_action: np.ndarray,
    *,
    dt: float,
) -> dict[str, Any]:
    """Build a 16-step linear ramp in absolute radian action space.

    ``start_action`` / ``end_action`` are policy-unit vectors [16]:
    arms in radian, grippers normalized.
    """
    start = np.asarray(start_action, dtype=np.float64).reshape(-1)
    end = np.asarray(end_action, dtype=np.float64).reshape(-1)
    if start.shape != (ACTION_DIM,) or end.shape != (ACTION_DIM,):
        raise ValueError(
            f"start/end must be shape ({ACTION_DIM},), got {start.shape} / {end.shape}"
        )
    if not np.isfinite(start).all() or not np.isfinite(end).all():
        raise ValueError("start/end actions contain NaN or infinity")
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError(f"dt must be finite and > 0, got {dt}")

    alphas = np.linspace(0.0, 1.0, ARBITER_CHUNK_SIZE, dtype=np.float64)
    actions = start[None, :] + alphas[:, None] * (end - start)[None, :]
    return _actions_to_fields(actions, dt)
