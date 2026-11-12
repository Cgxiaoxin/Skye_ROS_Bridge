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
