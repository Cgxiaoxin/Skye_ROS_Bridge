# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Pure teleoperation transitions; callers execute actions and supply time.

The control adapter emits trajectory and settling events, prioritizing timeout
at each deadline. This module neither runs trajectories nor reads a clock.
"""

from dataclasses import dataclass
from enum import Enum, IntEnum
from math import isfinite
from typing import Literal


class ControlMode(IntEnum):
    IDLE = 0
    TELEOP_SYNCING = 1
    SYNCED = 2
    TELEOP = 3
    HOLD_SETTLING = 4
    TELEPOSE = 5  # Reserved; no transition into this mode in the base machine.
    GRAVITY_TEST = 6
    LIMIT_TEST = 7


class Event(str, Enum):
    SWITCH_SYNC = "switch_sync"
    SWITCH_TELEOP = "switch_teleop"
    SWITCH_STOP = "switch_stop"
    SWITCH_GRAVITY_TEST = "switch_gravity_test"
    GRAVITY_TEST_TIMEOUT = "gravity_test_timeout"
    SWITCH_LIMIT_TEST = "switch_limit_test"
    LIMIT_TEST_TIMEOUT = "limit_test_timeout"
    SYNC_COMPLETE = "sync_complete"
    SYNC_TIMEOUT = "sync_timeout"
    TRAJECTORY_COMPLETE = "trajectory_complete"
    SETTLE_TIMEOUT = "settle_timeout"


class Action(str, Enum):
    """Ordered instructions for the control adapter, not hardware operations.

    STOP_MOTION stops automatic leader motion, hold, and teleop publication.
    CLEAR_SYNC clears the trajectory, hold integral, and previous target.
    START_SYNC captures a fresh synchronization start and target.
    START_HOLD holds the completed synchronization target.
    START_TELEOP enables leader-to-follower command publication.
    """

    STOP_MOTION = "stop_motion"
    CLEAR_SYNC = "clear_sync"
    START_SYNC = "start_sync"
    START_HOLD = "start_hold"
    START_TELEOP = "start_teleop"
    START_GRAVITY_TEST = "start_gravity_test"
    START_LIMIT_TEST = "start_limit_test"


@dataclass(frozen=True)
class TransitionResult:
    previous_state: ControlMode
    state: ControlMode
    outcome: Literal["accepted", "rejected", "ignored"]
    reason: str | None
    actions: tuple[Action, ...]
    entered_at: float


def _finite_time(now: float) -> float:
    value = float(now)
    if not isfinite(value):
        raise ValueError("now must be finite")
    return value


class StateMachine:
    """Single-caller state machine using explicit, nondecreasing timestamps.

    State entry time changes only on a state change. Rejected/ignored events
    return no actions. The most recent rejection reason persists across later
    accepted/ignored events; each result also carries its own reason.
    """

    def __init__(self, *, now: float):
        self._state = ControlMode.IDLE
        self._entered_at = _finite_time(now)
        self._last_now = self._entered_at
        self._last_rejection_reason: str | None = None

    @property
    def state(self) -> ControlMode:
        return self._state

    @property
    def entered_at(self) -> float:
        return self._entered_at

    @property
    def last_rejection_reason(self) -> str | None:
        return self._last_rejection_reason

    def handle(self, event: Event | str, *, now: float) -> TransitionResult:
        now = _finite_time(now)
        if now < self._last_now:
            raise ValueError("now must be nondecreasing")
        self._last_now = now
        try:
            event = Event(event)
        except ValueError:
            return self._result("rejected", reason="unsupported_event", now=now)

        if event == Event.SWITCH_STOP:
            return self._result(
                "accepted", state=ControlMode.IDLE,
                actions=(Action.STOP_MOTION, Action.CLEAR_SYNC), now=now,
            )

        if event == Event.SWITCH_GRAVITY_TEST:
            if self.state == ControlMode.IDLE:
                return self._result(
                    "accepted", state=ControlMode.GRAVITY_TEST,
                    actions=(Action.CLEAR_SYNC, Action.START_GRAVITY_TEST), now=now,
                )
            if self.state == ControlMode.GRAVITY_TEST:
                return self._result("ignored", reason="already_testing", now=now)
            return self._result("rejected", reason="stop_required", now=now)

        if event == Event.GRAVITY_TEST_TIMEOUT and self.state == ControlMode.GRAVITY_TEST:
            return self._result(
                "accepted", state=ControlMode.IDLE, reason="gravity_test_timeout",
                actions=(Action.STOP_MOTION, Action.CLEAR_SYNC), now=now,
            )

        if event == Event.SWITCH_LIMIT_TEST:
            if self.state == ControlMode.IDLE:
                return self._result(
                    "accepted", state=ControlMode.LIMIT_TEST,
                    actions=(Action.CLEAR_SYNC, Action.START_LIMIT_TEST), now=now,
                )
            if self.state == ControlMode.LIMIT_TEST:
                return self._result("ignored", reason="already_testing", now=now)
            return self._result("rejected", reason="stop_required", now=now)

        if event == Event.LIMIT_TEST_TIMEOUT and self.state == ControlMode.LIMIT_TEST:
            return self._result(
                "accepted", state=ControlMode.IDLE, reason="limit_test_timeout",
                actions=(Action.STOP_MOTION, Action.CLEAR_SYNC), now=now,
            )

        if event == Event.SWITCH_SYNC:
            if self.state == ControlMode.IDLE:
                return self._result(
                    "accepted", state=ControlMode.TELEOP_SYNCING,
                    actions=(Action.CLEAR_SYNC, Action.START_SYNC), now=now,
                )
            if self.state in (ControlMode.TELEOP_SYNCING, ControlMode.HOLD_SETTLING, ControlMode.SYNCED):
                return self._result("ignored", reason="already_syncing_or_synced", now=now)
            return self._result("rejected", reason="stop_required", now=now)

        if event == Event.SWITCH_TELEOP:
            if self.state != ControlMode.SYNCED:
                return self._result("rejected", reason="not_synced", now=now)
            return self._result(
                "accepted", state=ControlMode.TELEOP,
                actions=(Action.STOP_MOTION, Action.CLEAR_SYNC, Action.START_TELEOP),
                now=now,
            )

        if self.state == ControlMode.TELEOP_SYNCING:
            if event == Event.SYNC_TIMEOUT:
                return self._result(
                    "accepted", state=ControlMode.IDLE, reason="sync_timeout",
                    actions=(Action.STOP_MOTION, Action.CLEAR_SYNC), now=now,
                )
            if event == Event.TRAJECTORY_COMPLETE:
                return self._result(
                    "accepted", state=ControlMode.HOLD_SETTLING,
                    actions=(Action.STOP_MOTION, Action.START_HOLD), now=now,
                )
        elif self.state == ControlMode.HOLD_SETTLING:
            if event == Event.SETTLE_TIMEOUT:
                return self._result(
                    "accepted", state=ControlMode.IDLE, reason="settle_timeout",
                    actions=(Action.STOP_MOTION, Action.CLEAR_SYNC), now=now,
                )
            if event == Event.SYNC_COMPLETE:
                return self._result("accepted", state=ControlMode.SYNCED, now=now)
        return self._result("ignored", reason="not_applicable", now=now)

    def _result(
        self, outcome: Literal["accepted", "rejected", "ignored"], *,
        now: float, state: ControlMode | None = None,
        reason: str | None = None, actions: tuple[Action, ...] = (),
    ) -> TransitionResult:
        previous_state = self.state
        if state is not None and state != self.state:
            self._state = state
            self._entered_at = now
        if outcome == "rejected":
            self._last_rejection_reason = reason
        return TransitionResult(
            previous_state, self.state, outcome, reason, actions, self.entered_at,
        )
