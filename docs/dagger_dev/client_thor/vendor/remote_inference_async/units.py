"""Unit conversions at the robot/policy transport boundary.

The Gento runtime and all robot-side safety limits use degrees. The trained
policies use radians for the fourteen arm joints. Grippers are normalized to
``[0, 1]`` on both sides and must never be angle-converted.
"""

from __future__ import annotations

import numpy as np


ARM_JOINT_DIM = 14
VECTOR_DIM = 16
POLICY_ARM_UNIT = "radian"
ROBOT_ARM_UNIT = "degree"
STATE_GRIPPER_UNIT = "calibrated_0_open_1_closed"
ACTION_GRIPPER_UNIT = "command_0_open_0.9_max_grasp"


def robot_state_to_policy(robot_state: np.ndarray) -> np.ndarray:
    """Convert one robot state from arm degrees to policy radians."""
    state = np.asarray(robot_state, dtype=np.float32)
    if state.shape != (VECTOR_DIM,) or not np.isfinite(state).all():
        raise ValueError(f"robot state must be finite [{VECTOR_DIM}], got {state.shape}")
    converted = state.copy()
    converted[:ARM_JOINT_DIM] = np.deg2rad(converted[:ARM_JOINT_DIM])
    return converted


def robot_action_to_policy(robot_action: np.ndarray) -> np.ndarray:
    """Convert one executed robot command to the policy's action units."""
    action = np.asarray(robot_action, dtype=np.float32)
    if action.shape != (VECTOR_DIM,) or not np.isfinite(action).all():
        raise ValueError(f"robot action must be finite [{VECTOR_DIM}], got {action.shape}")
    converted = action.copy()
    converted[:ARM_JOINT_DIM] = np.deg2rad(converted[:ARM_JOINT_DIM])
    return converted


def policy_actions_to_robot(policy_actions: np.ndarray) -> np.ndarray:
    """Convert a policy action chunk from arm radians to robot degrees."""
    actions = np.asarray(policy_actions, dtype=np.float32)
    if actions.ndim != 2 or actions.shape[1] != VECTOR_DIM or len(actions) == 0:
        raise ValueError(
            f"policy actions must be a non-empty [T,{VECTOR_DIM}] array, got {actions.shape}"
        )
    if not np.isfinite(actions).all():
        raise ValueError("policy actions contain NaN or infinity")
    converted = actions.copy()
    converted[:, :ARM_JOINT_DIM] = np.rad2deg(converted[:, :ARM_JOINT_DIM])
    return converted
