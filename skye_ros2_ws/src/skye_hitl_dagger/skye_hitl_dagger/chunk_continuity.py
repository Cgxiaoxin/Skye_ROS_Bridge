"""Policy chunk continuity helpers (no ROS deps)."""

from __future__ import annotations

from typing import Optional

DOF = 7


def max_step0_jump_rad(
        left_joints, right_joints, left_pose, right_pose,
        dof: int = DOF) -> float:
    """Max |step0 - pose| across both arms (follower / absolute rad)."""
    if (len(left_joints) < dof or len(right_joints) < dof
            or len(left_pose) < dof or len(right_pose) < dof):
        return float("inf")
    err = 0.0
    for i in range(dof):
        err = max(err, abs(float(left_joints[i]) - float(left_pose[i])))
        err = max(err, abs(float(right_joints[i]) - float(right_pose[i])))
    return err


def rebase_arm_joints_to_pose(
        joints, pose, steps: int, dof: int = DOF) -> list:
    """Shift a flat chunk so step0 matches pose; preserve step-to-step deltas."""
    n = steps * dof
    if len(joints) != n or len(pose) != dof:
        raise ValueError("joint/pose length mismatch for rebase")
    delta = [float(pose[i]) - float(joints[i]) for i in range(dof)]
    out = [0.0] * n
    for step in range(steps):
        base = step * dof
        for i in range(dof):
            out[base + i] = float(joints[base + i]) + delta[i]
    return out


def chunk_is_fresh(
        stamp_s: float, return_time: Optional[float],
        receive_time: Optional[float] = None,
        return_wall_time: Optional[float] = None,
        now_wall_time: Optional[float] = None,
        fallback_after_s: float = 2.0) -> bool:
    """Accept post-return chunks by stamp, then fall back to receive time."""
    if return_time is None:
        return True
    if stamp_s > 0.0 and stamp_s >= return_time:
        return True
    if stamp_s == 0.0 and receive_time is not None:
        return (return_wall_time is None
                or receive_time >= return_wall_time)
    return (return_wall_time is not None and now_wall_time is not None
            and now_wall_time - return_wall_time >= fallback_after_s)
