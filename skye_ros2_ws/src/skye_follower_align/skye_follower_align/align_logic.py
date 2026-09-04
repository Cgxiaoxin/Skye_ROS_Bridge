from __future__ import annotations

from enum import Enum, auto
import time
from typing import Optional, Sequence

DOF = 7


class AlignPhase(Enum):
    IDLE = auto()
    ALIGNING = auto()
    ALIGNED = auto()
    TIMEOUT_WARN = auto()


def map_leader_to_follower(leader: Sequence[float], signs: Sequence[float]) -> list[float]:
    if len(leader) != DOF or len(signs) != DOF:
        raise ValueError("expected 7 joints")
    return [float(signs[i]) * float(leader[i]) for i in range(DOF)]


def leader_positions_for_abs_command(leader: Sequence[float]) -> list[float]:
    """Raw leader joints for *_joint_control_abs; driver applies signs."""
    if len(leader) != DOF:
        raise ValueError("expected 7 joints")
    return [float(v) for v in leader]


def combine_phase(left: AlignPhase, right: AlignPhase) -> AlignPhase:
    phases = {left, right}
    if AlignPhase.TIMEOUT_WARN in phases:
        return AlignPhase.TIMEOUT_WARN
    if AlignPhase.ALIGNING in phases:
        return AlignPhase.ALIGNING
    if left == AlignPhase.ALIGNED and right == AlignPhase.ALIGNED:
        return AlignPhase.ALIGNED
    return AlignPhase.IDLE


def max_abs_err(cmd: Sequence[float], measured: Sequence[float]) -> float:
    return max(abs(float(cmd[i]) - float(measured[i])) for i in range(DOF))


class AlignSession:
    def __init__(
        self,
        threshold_rad: float = 0.05,
        hold_frames: int = 5,
        timeout_s: float = 10.0,
    ):
        self.threshold_rad = threshold_rad
        self.hold_frames = hold_frames
        self.timeout_s = timeout_s
        self.phase = AlignPhase.IDLE
        self._ok_frames = 0
        self._t0 = 0.0

    def start(self, now: Optional[float] = None) -> bool:
        if self.phase == AlignPhase.ALIGNING:
            return False
        self.phase = AlignPhase.ALIGNING
        self._ok_frames = 0
        self._t0 = time.monotonic() if now is None else now
        return True

    def cancel(self) -> None:
        self.phase = AlignPhase.IDLE
        self._ok_frames = 0

    def on_tick(
        self,
        leader: Sequence[float],
        big: Sequence[float],
        signs: Sequence[float],
        now: Optional[float] = None,
    ) -> AlignPhase:
        if self.phase != AlignPhase.ALIGNING:
            return self.phase
        t = time.monotonic() if now is None else now
        cmd = map_leader_to_follower(leader, signs)
        if max_abs_err(cmd, big) < self.threshold_rad:
            self._ok_frames += 1
            if self._ok_frames >= self.hold_frames:
                self.phase = AlignPhase.ALIGNED
                return self.phase
        else:
            self._ok_frames = 0
        if (t - self._t0) >= self.timeout_s:
            self.phase = AlignPhase.TIMEOUT_WARN
        return self.phase
