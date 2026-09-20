"""Convert follower-space absolute poses for /gento/*_joint_control_abs.

Policy chunks and holds are follower-space (match /gento/joint_states).
Driver abs applies apply_joint_mapping(leader, order, signs, offsets), same as
follower_align — so we publish the inverse (leader-space) absolute command.
"""

from __future__ import annotations

from typing import List, Optional, Sequence

DOF = 7


def follower_pose_to_leader_abs(
    follower: Sequence[float],
    signs: Sequence[float],
    joint_order: Sequence[int],
    offsets: Optional[Sequence[float]] = None,
) -> List[float]:
    """Inverse of DriverCore::apply_joint_mapping for one arm pose."""
    if len(follower) != DOF:
        raise ValueError(f"expected {DOF} follower joints, got {len(follower)}")
    if len(signs) != DOF or len(joint_order) != DOF:
        raise ValueError("signs and joint_order must have length 7")
    off = list(offsets) if offsets is not None else [0.0] * DOF
    if len(off) != DOF:
        raise ValueError("offsets must have length 7")

    leader = [0.0] * DOF
    for out in range(DOF):
        src = int(joint_order[out])
        if src < 0 or src >= DOF:
            raise ValueError(f"joint_order[{out}]={src} out of range")
        sign = float(signs[out])
        if sign == 0.0:
            raise ValueError(f"signs[{out}] must be non-zero")
        leader[src] = (float(follower[out]) - float(off[out])) / sign
    return leader


def apply_joint_mapping(
    leader: Sequence[float],
    signs: Sequence[float],
    joint_order: Sequence[int],
    offsets: Optional[Sequence[float]] = None,
) -> List[float]:
    """Forward map (test helper); mirrors DriverCore::apply_joint_mapping."""
    off = list(offsets) if offsets is not None else [0.0] * DOF
    mapped = [0.0] * DOF
    for out in range(DOF):
        src = int(joint_order[out])
        mapped[out] = float(leader[src]) * float(signs[out]) + float(off[out])
    return mapped
