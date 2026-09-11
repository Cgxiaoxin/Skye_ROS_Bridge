"""FACTR handover handshake tracking, independent of ROS."""

from __future__ import annotations

import time
from enum import Enum, auto
from typing import Callable, Optional

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

    Latched pre-takeover SYNCED is ignored: start_sync clears state and requires
    seeing a non-aligned state (e.g. TELEOP_SYNCING) before a fresh SYNCED can
    satisfy phase 1. Optional min_sync_hold_s keeps SYNCED stable before ready.

    switch_teleop is published once aligned_ready(); waiting for TELEOP before
    requesting it would deadlock because FACTR only enters TELEOP in response
    to switch_teleop.
    """

    def __init__(
            self,
            min_sync_hold_s: float = 0.0,
            clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._min_sync_hold_s = max(0.0, float(min_sync_hold_s))
        self._clock = clock
        self._phase = SyncPhase.IDLE
        self._state: Optional[str] = None
        self._seen_unaligned = False
        self._aligned_since: Optional[float] = None

    def phase(self) -> SyncPhase:
        return self._phase

    def state(self) -> Optional[str]:
        return self._state

    def reset(self) -> None:
        self._phase = SyncPhase.IDLE
        self._state = None
        self._seen_unaligned = False
        self._aligned_since = None

    def start_sync(self) -> None:
        """Enter phase 1 right after switch_sync is published.

        Clears cached state so a latched SYNCED from an earlier session cannot
        satisfy alignment without a fresh post-sync publication path.
        """
        self._phase = SyncPhase.WAIT_ALIGNED
        self._state = None
        self._seen_unaligned = False
        self._aligned_since = None

    def start_teleop(self) -> None:
        """Enter phase 2 right after switch_teleop is published.

        The cached state is dropped so a latched TELEOP from an earlier session
        cannot satisfy phase 2 without a fresh publication.
        """
        self._phase = SyncPhase.WAIT_TELEOP
        self._state = None

    def on_state(self, state: Optional[str]) -> None:
        normalized = normalize_state(state)
        if normalized is None:
            return
        self._state = normalized
        if self._phase is not SyncPhase.WAIT_ALIGNED:
            return
        if is_aligned_state(normalized):
            if self._seen_unaligned and self._aligned_since is None:
                self._aligned_since = self._clock()
            return
        self._seen_unaligned = True
        self._aligned_since = None

    def aligned_ready(self) -> bool:
        if (self._phase is not SyncPhase.WAIT_ALIGNED
                or not self._seen_unaligned
                or not is_aligned_state(self._state)
                or self._aligned_since is None):
            return False
        return (self._clock() - self._aligned_since) >= self._min_sync_hold_s

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
