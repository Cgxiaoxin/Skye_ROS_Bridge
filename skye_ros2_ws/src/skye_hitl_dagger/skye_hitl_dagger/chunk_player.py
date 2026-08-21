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
    step_t0s: List[float]
    end_t: float
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
        step_t0s = [t0 + i * dt for i in range(chunk_size)]
        end_t = t0 + chunk_size * dt
        self._chunk = _Chunk(
            t0=t0, dt=dt, steps=chunk_size,
            step_t0s=step_t0s, end_t=end_t,
            left=list(left_joints), right=list(right_joints),
            left_gripper=list(left_gripper),
            right_gripper=list(right_gripper),
        )
        return True

    def sample(self, t_now: float):
        if self._chunk is None:
            return None
        c = self._chunk
        if t_now <= c.step_t0s[0]:
            idx, holding = 0, False
        else:
            eps = max(1e-12, abs(c.dt) * 1e-9, abs(c.end_t) * 1e-9)
            holding = t_now + eps >= c.end_t
            idx = 0
            for i in range(1, c.steps):
                if t_now >= c.step_t0s[i]:
                    idx = i
                else:
                    break
            if holding:
                idx = c.steps - 1
        base = idx * DOF
        return {
            "left": c.left[base:base+DOF],
            "right": c.right[base:base+DOF],
            "left_gripper": c.left_gripper[idx],
            "right_gripper": c.right_gripper[idx],
            "holding_tail": holding,
        }
