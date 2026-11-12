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
# Typical Skye joint limits are well under ~π..±4 rad. Values above this almost
# always mean someone passed degree payloads into the radian PolicyActionChunk.
MAX_ABS_JOINT_RAD = 6.5


def assert_policy_chunk_radians(
    policy_actions: np.ndarray,
    *,
    context: str,
    max_abs_rad: float = MAX_ABS_JOINT_RAD,
) -> None:
    """Fail closed if arm joints look like degrees (would slam the robot)."""
    actions = np.asarray(policy_actions, dtype=np.float64)
    if actions.ndim != 2 or actions.shape[1] != ACTION_DIM or actions.shape[0] == 0:
        raise ValueError(
            f"{context}: policy actions must be non-empty [T,{ACTION_DIM}], "
            f"got {actions.shape}"
        )
    if not np.isfinite(actions).all():
        raise ValueError(f"{context}: policy actions contain NaN or infinity")
    joints = actions[:, : 2 * ARM_DOF]
    peak = float(np.max(np.abs(joints)))
    if peak > max_abs_rad:
        sample_deg = np.rad2deg(joints[0])
        raise ValueError(
            f"{context}: arm joints look like DEGREES mislabeled as radians "
            f"(max|q|={peak:.2f} > {max_abs_rad}). Refusing publish. "
            f"first_row_as_if_rad_to_deg={np.round(sample_deg, 1).tolist()}"
        )


def _command_deg_to_rad_row(command_deg_16d: np.ndarray) -> np.ndarray:
    command = np.asarray(command_deg_16d, dtype=np.float64).reshape(ACTION_DIM)
    if not np.isfinite(command).all():
        raise ValueError("robot command must be finite")
    return np.concatenate(
        [
            np.deg2rad(command[0:ARM_DOF]),
            np.deg2rad(command[ARM_DOF : 2 * ARM_DOF]),
            command[14:16],
        ]
    )


def robot_command_deg_to_hold_chunk_rad(
    command_deg_16d: np.ndarray,
    *,
    steps: int = ARBITER_CHUNK_SIZE,
) -> np.ndarray:
    """Convert one robot-space command (deg arms + gripper) to abs-rad hold chunk.

    Layout matches async client state/action vectors:
    [L7_deg, R7_deg, left_gripper, right_gripper].

    Used by slow-home (tiny quintic steps). Policy streaming should prefer
    robot_command_deg_to_stream_chunk_rad so step0 stays near feedback.
    """
    if steps <= 0:
        raise ValueError(f"steps must be positive, got {steps}")
    row = _command_deg_to_rad_row(command_deg_16d)
    out = np.repeat(row[None, :], steps, axis=0)
    assert_policy_chunk_radians(out, context="robot_command_deg_to_hold_chunk_rad")
    return out


def robot_command_deg_to_stream_chunk_rad(
    start_deg_16d: np.ndarray,
    target_deg_16d: np.ndarray,
    *,
    steps: int = ARBITER_CHUNK_SIZE,
) -> np.ndarray:
    """Linear abs-rad chunk from feedback/start → slew-limited target.

    step0 matches start (keeps arbiter chunk_start_max_jump happy). Later steps
    walk to target so ChunkPlayer actually moves within one republish period
    when dt ≈ 1/(action_hz * steps).

    Layout: [L7_deg, R7_deg, left_gripper, right_gripper] in / out as rad arms.
    """
    if steps <= 0:
        raise ValueError(f"steps must be positive, got {steps}")
    start = _command_deg_to_rad_row(start_deg_16d)
    target = _command_deg_to_rad_row(target_deg_16d)
    if steps == 1:
        out = target[None, :].copy()
    else:
        alphas = np.linspace(0.0, 1.0, steps, dtype=np.float64)[:, None]
        out = start[None, :] + (target[None, :] - start[None, :]) * alphas
    assert_policy_chunk_radians(out, context="robot_command_deg_to_stream_chunk_rad")
    return out


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
    assert_policy_chunk_radians(actions, context="policy_chunk_to_arbiter_fields")

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
