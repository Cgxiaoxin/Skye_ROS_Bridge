"""Synthesize driver-relative joint_control from follower-space policy targets.

Mirrors skye_robot_driver DriverCore::apply_relative_joint_mapping so policy
rollout uses the same incremental path as FACTR teleop (unwrap at +-pi).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence

DOF = 7
PI = math.pi
DEFAULT_ORDER = tuple(range(DOF))
DEFAULT_SIGNS = (1.0,) * DOF


def unwrap_frame_delta(leader_now: float, leader_prev: float) -> float:
    """Match DriverCore frame-to-frame unwrap (per-frame |delta| < pi)."""
    delta = leader_now - leader_prev
    if delta > PI:
        delta -= 2.0 * PI
    elif delta < -PI:
        delta += 2.0 * PI
    return delta


def follower_target_to_leader_continuous(
        target: Sequence[float],
        gento_ref: Sequence[float],
        leader_cont_ref: Sequence[float],
        signs: Sequence[float],
        joint_order: Sequence[int],
) -> List[float]:
    """Inverse of relative teleop mapping for one follower-space target pose."""
    required = [0.0] * DOF
    for out in range(DOF):
        src = joint_order[out]
        delta_follower = target[out] - gento_ref[out]
        required[src] = leader_cont_ref[src] + signs[out] * delta_follower
    return required


@dataclass
class PolicyRelativeSession:
    """Track virtual leader state for one arm's policy relative commands."""

    signs: Sequence[float] = field(default_factory=lambda: DEFAULT_SIGNS)
    joint_order: Sequence[int] = field(default_factory=lambda: DEFAULT_ORDER)
    gento_ref: List[float] = field(default_factory=lambda: [0.0] * DOF)
    leader_prev: List[float] = field(default_factory=lambda: [0.0] * DOF)
    leader_continuous: List[float] = field(default_factory=lambda: [0.0] * DOF)
    leader_cont_ref: List[float] = field(default_factory=lambda: [0.0] * DOF)
    active: bool = False

    def begin(self, follower_feedback: Sequence[float]) -> None:
        """Seed refs like driver relative teleop session entry."""
        pose = [float(v) for v in follower_feedback]
        if len(pose) != DOF:
            raise ValueError(f"expected {DOF} follower joints, got {len(pose)}")
        self.gento_ref = list(pose)
        self.leader_prev = list(pose)
        self.leader_continuous = list(pose)
        self.leader_cont_ref = list(pose)
        self.active = True

    def invalidate(self) -> None:
        self.active = False

    def follower_target_to_leader(self, target: Sequence[float]) -> List[float]:
        """Return raw leader joint_control positions for the desired follower pose."""
        if len(target) != DOF:
            raise ValueError(f"expected {DOF} target joints, got {len(target)}")
        required = follower_target_to_leader_continuous(
            target, self.gento_ref, self.leader_cont_ref, self.signs,
            self.joint_order)
        leader_now = [0.0] * DOF
        for joint in range(DOF):
            delta = required[joint] - self.leader_continuous[joint]
            leader_now[joint] = self.leader_prev[joint] + delta
        return leader_now

    def commit_published(self, leader_now: Sequence[float]) -> None:
        """Advance unwrap state after publishing leader_now to the driver."""
        for joint in range(DOF):
            self.leader_continuous[joint] += unwrap_frame_delta(
                leader_now[joint], self.leader_prev[joint])
        self.leader_prev = [float(v) for v in leader_now]
