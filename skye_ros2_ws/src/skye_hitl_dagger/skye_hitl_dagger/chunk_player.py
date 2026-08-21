from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Optional

DOF = 7
DEFAULT_STEPS = 16


@dataclass
class _Chunk:
    t0: float
    dt: float
    steps: int
    left: List[float]
    right: List[float]
    left_gripper: List[float]
    right_gripper: List[float]


class ChunkPlayer:
    def __init__(self) -> None:
        self._chunk: Optional[_Chunk] = None

    def load(self, chunk_size, dt, t0, left_joints, right_joints,
             left_gripper, right_gripper) -> bool:
        if (chunk_size != DEFAULT_STEPS or dt <= 0.0
                or not math.isfinite(dt) or not math.isfinite(t0)):
            return False
        n = chunk_size * DOF
        if (len(left_joints) != n or len(right_joints) != n
                or len(left_gripper) != chunk_size
                or len(right_gripper) != chunk_size):
            return False
        self._chunk = _Chunk(
            t0=t0, dt=dt, steps=chunk_size,
            left=list(left_joints), right=list(right_joints),
            left_gripper=list(left_gripper),
            right_gripper=list(right_gripper),
        )
        return True

    def sample(self, t_now: float):
        if self._chunk is None:
            return None
        c = self._chunk
        elapsed = t_now - c.t0
        if elapsed <= 0.0:
            idx, holding = 0, False
        else:
            end_t = c.steps * c.dt
            eps = max(1e-12, abs(end_t) * 1e-9)
            holding = elapsed + eps >= end_t
            if holding:
                idx = c.steps - 1
            else:
                idx = int(elapsed / c.dt)
                if idx >= c.steps:
                    idx = c.steps - 1
                    holding = True
        base = idx * DOF
        return {
            "left": c.left[base:base+DOF],
            "right": c.right[base:base+DOF],
            "left_gripper": c.left_gripper[idx],
            "right_gripper": c.right_gripper[idx],
            "holding_tail": holding,
        }
