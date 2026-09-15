"""Pure session state machine for the operator UI (no ROS)."""

from __future__ import annotations

from enum import Enum, auto

_VALID_PROFILES = frozenset({"thor", "orin"})


class SessionState(Enum):
    IDLE = auto()
    PRECHECK = auto()
    STARTING = auto()
    READY = auto()
    RUNNING = auto()
    FAILED = auto()
    STOPPING = auto()
    DEGRADED = auto()


class UiMode(Enum):
    teleop_record = auto()
    dagger = auto()


class SessionLogic:
    """Session lifecycle state machine with illegal-transition guards."""

    def __init__(self) -> None:
        self._state = SessionState.IDLE
        self._profile: str | None = None
        self._mode: UiMode | None = None

    def state(self) -> SessionState:
        return self._state

    def profile(self) -> str | None:
        return self._profile

    def mode(self) -> UiMode | None:
        return self._mode

    def can_change_profile_or_mode(self) -> bool:
        return self._state == SessionState.IDLE

    def begin_start(self, profile: str, mode: UiMode) -> bool:
        if self._state != SessionState.IDLE:
            return False
        if profile not in _VALID_PROFILES:
            return False
        self._profile = profile
        self._mode = mode
        self._state = SessionState.PRECHECK
        return True

    def precheck_ok(self) -> bool:
        if self._state != SessionState.PRECHECK:
            return False
        self._state = SessionState.STARTING
        return True

    def precheck_fail(self) -> bool:
        if self._state != SessionState.PRECHECK:
            return False
        self._state = SessionState.FAILED
        return True

    def enter_starting(self) -> bool:
        if self._state != SessionState.PRECHECK:
            return False
        self._state = SessionState.STARTING
        return True

    def mark_ready(self) -> bool:
        if self._state != SessionState.STARTING:
            return False
        self._state = SessionState.READY
        return True

    def mark_failed(self) -> bool:
        if self._state not in (SessionState.PRECHECK, SessionState.STARTING):
            return False
        self._state = SessionState.FAILED
        return True

    def resume_starting(self) -> bool:
        if self._state != SessionState.FAILED:
            return False
        self._state = SessionState.STARTING
        return True

    def mark_running(self) -> bool:
        if self._state != SessionState.READY:
            return False
        self._state = SessionState.RUNNING
        return True

    def mark_degraded(self) -> bool:
        if self._state not in (SessionState.READY, SessionState.RUNNING):
            return False
        self._state = SessionState.DEGRADED
        return True

    def begin_stop(self) -> bool:
        if self._state in (SessionState.IDLE, SessionState.STOPPING):
            return False
        self._state = SessionState.STOPPING
        return True

    def mark_idle(self) -> bool:
        if self._state != SessionState.STOPPING:
            return False
        self._state = SessionState.IDLE
        self._profile = None
        self._mode = None
        return True
