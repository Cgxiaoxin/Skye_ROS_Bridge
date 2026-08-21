"""FACTR handover handshake tracking, independent of ROS."""

from __future__ import annotations

from enum import Enum, auto
from typing import Optional

TELEOP_STATE = "TELEOP"
ALIGNED_TOKEN = "SYNCED"


def normalize_state(state: Optional[str]) -> Optional[str]:
    if not isinstance(state, str):
        return None
    stripped = state.strip().upper()
    return stripped or None


def is_teleop_state(state: Optional[str]) -> bool:
    return normalize_state(state) == TELEOP_STATE


def is_aligned_state(state: Optional[str]) -> bool:
    """FACTR reports SYNCED once the leader arm matches the follower pose.

    TELEOP_SYNCING must not qualify: alignment is still in progress there.
    """
    normalized = normalize_state(state)
    if normalized is None or normalized == TELEOP_STATE:
        return False
    return ALIGNED_TOKEN in normalized


class SyncPhase(Enum):
    IDLE = auto()
    WAIT_ALIGNED = auto()
    WAIT_TELEOP = auto()


class TeleopHandshake:
    """Two-phase handover: switch_sync -> SYNCED -> switch_teleop -> TELEOP.

    switch_teleop is published as soon as FACTR reports alignment; waiting for
    TELEOP before requesting it would deadlock because FACTR only enters TELEOP
    in response to switch_teleop.
    """

    def __init__(self) -> None:
        self._phase = SyncPhase.IDLE
        self._state: Optional[str] = None

    def phase(self) -> SyncPhase:
        return self._phase

    def state(self) -> Optional[str]:
        return self._state

    def reset(self) -> None:
        self._phase = SyncPhase.IDLE
        self._state = None

    def start_sync(self) -> None:
        """Enter phase 1 right after switch_sync is published."""
        self._phase = SyncPhase.WAIT_ALIGNED

    def start_teleop(self) -> None:
        """Enter phase 2 right after switch_teleop is published.

        The cached state is dropped so a latched TELEOP from an earlier session
        cannot satisfy phase 2 without a fresh publication.
        """
        self._phase = SyncPhase.WAIT_TELEOP
        self._state = None

    def on_state(self, state: Optional[str]) -> None:
        normalized = normalize_state(state)
        if normalized is not None:
            self._state = normalized

    def aligned_ready(self) -> bool:
        return (self._phase is SyncPhase.WAIT_ALIGNED
                and is_aligned_state(self._state))

    def teleop_ready(self) -> bool:
        return (self._phase is SyncPhase.WAIT_TELEOP
                and is_teleop_state(self._state))

    def pending_command(self) -> Optional[str]:
        """Mode command to (re)publish while the current phase is unconfirmed."""
        if self._phase is SyncPhase.WAIT_ALIGNED:
            return "switch_sync"
        if self._phase is SyncPhase.WAIT_TELEOP:
            return "switch_teleop"
        return None
