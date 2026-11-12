"""Setpoint-mode helpers: ERR_MAX guard and micro-ramp publish planning."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from remote_inference_async.dagger_chunk import ARM_JOINT_DIM, setpoint_dt
from remote_inference_async.units import robot_action_to_policy

LOGGER = logging.getLogger("tianji_dagger_setpoint")


@dataclass
class ErrMaxDecision:
    hold: bool
    err_rad: float
    sustained: bool
    trip_count: int
    sustained_warn_count: int


class ErrMaxGuard:
    """Freeze command advance when |last_cmd - fb| on arms exceeds ERR_MAX."""

    def __init__(
        self,
        *,
        err_max_rad: float = 0.15,
        sustained_s: float = 0.5,
    ) -> None:
        if not np.isfinite(err_max_rad) or err_max_rad <= 0.0:
            raise ValueError("err_max_rad must be finite and > 0")
        if not np.isfinite(sustained_s) or sustained_s <= 0.0:
            raise ValueError("sustained_s must be finite and > 0")
        self.err_max_rad = float(err_max_rad)
        self.sustained_s = float(sustained_s)
        self._exceed_since: Optional[float] = None
        self.trip_count = 0
        self.sustained_warn_count = 0

    def evaluate(
        self,
        last_cmd_robot: np.ndarray,
        feedback_robot: np.ndarray,
        *,
        now: Optional[float] = None,
    ) -> ErrMaxDecision:
        last_cmd = np.asarray(last_cmd_robot, dtype=np.float64)
        feedback = np.asarray(feedback_robot, dtype=np.float64)
        if last_cmd.shape[-1] < ARM_JOINT_DIM or feedback.shape[-1] < ARM_JOINT_DIM:
            raise ValueError("last_cmd/feedback must include 14 arm joints")
        # Client arm units are degrees; ERR_MAX is configured in radians.
        err_rad = float(
            np.max(
                np.abs(
                    np.deg2rad(last_cmd[:ARM_JOINT_DIM] - feedback[:ARM_JOINT_DIM])
                )
            )
        )
        now_mono = time.monotonic() if now is None else float(now)
        if err_rad > self.err_max_rad:
            if self._exceed_since is None:
                self._exceed_since = now_mono
                self.trip_count += 1
                LOGGER.warning(
                    "ERR_MAX trip#%d err=%.4f rad > %.4f; holding last_cmd",
                    self.trip_count,
                    err_rad,
                    self.err_max_rad,
                )
            sustained = (now_mono - self._exceed_since) >= self.sustained_s
            if sustained:
                self.sustained_warn_count += 1
                LOGGER.error(
                    "ERR_MAX sustained >%.2fs (count=%d) err=%.4f rad; "
                    "holding last_cmd — operator intervention needed",
                    self.sustained_s,
                    self.sustained_warn_count,
                    err_rad,
                )
            return ErrMaxDecision(
                hold=True,
                err_rad=err_rad,
                sustained=sustained,
                trip_count=self.trip_count,
                sustained_warn_count=self.sustained_warn_count,
            )

        self._exceed_since = None
        return ErrMaxDecision(
            hold=False,
            err_rad=err_rad,
            sustained=False,
            trip_count=self.trip_count,
            sustained_warn_count=self.sustained_warn_count,
        )


def robot_commands_to_setpoint_policy(
    start_robot: np.ndarray,
    end_robot: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert degree/gripper robot vectors to policy-unit start/end."""
    return (
        robot_action_to_policy(start_robot),
        robot_action_to_policy(end_robot),
    )


def micro_ramp_dt(model_action_hz: float) -> float:
    return setpoint_dt(model_action_hz)
