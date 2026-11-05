"""Sync republish policy and per-arm failure classification."""

from __future__ import annotations

import re
from enum import Enum, auto
from typing import Optional, Sequence


_DETERMINISTIC_PATTERNS = (
    re.compile(r"outside configured safe bounds", re.I),
    re.compile(r"leaves configured safe bounds", re.I),
    re.compile(r"reposition before trial", re.I),
    re.compile(r"target is outside", re.I),
    re.compile(r"sync_preflight", re.I),
)


class SideSyncOutcome(Enum):
    UNKNOWN = auto()
    SYNCING = auto()
    SYNCED = auto()
    FAILED_RETRYABLE = auto()
    FAILED_DETERMINISTIC = auto()


def is_deterministic_sync_failure(text: Optional[str]) -> bool:
    if not text:
        return False
    return any(pattern.search(text) for pattern in _DETERMINISTIC_PATTERNS)


def classify_sync_failure(
        outcome: Optional[str], text: Optional[str]) -> SideSyncOutcome:
    normalized = (outcome or "").strip().lower()
    if normalized == "synced":
        return SideSyncOutcome.SYNCED
    if normalized == "active":
        return SideSyncOutcome.SYNCING
    if normalized == "failed":
        if is_deterministic_sync_failure(text):
            return SideSyncOutcome.FAILED_DETERMINISTIC
        return SideSyncOutcome.FAILED_RETRYABLE
    if normalized == "stopped":
        return SideSyncOutcome.FAILED_RETRYABLE
    return SideSyncOutcome.UNKNOWN


class SyncRetryPolicy:
    """Cap and back off switch_sync republishes; stop once aligned or terminal."""

    def __init__(
            self,
            backoff_s: Sequence[float] = (5.0, 10.0, 20.0),
            max_attempts: int = 3,
    ) -> None:
        delays = [max(0.0, float(v)) for v in backoff_s]
        if not delays:
            delays = [5.0]
        self._backoff_s = tuple(delays)
        self._max_attempts = max(1, int(max_attempts))
        self._attempts = 0
        self._last_publish_at: Optional[float] = None

    @property
    def attempts(self) -> int:
        return self._attempts

    def reset(self) -> None:
        self._attempts = 0
        self._last_publish_at = None

    def exhausted(self) -> bool:
        return self._attempts >= self._max_attempts

    def note_sync_published(self, now: float) -> None:
        self._attempts += 1
        self._last_publish_at = float(now)

    def next_backoff_s(self) -> float:
        if self._attempts <= 0:
            return self._backoff_s[0]
        index = min(self._attempts - 1, len(self._backoff_s) - 1)
        return self._backoff_s[index]

    def should_republish_sync(
            self,
            now: float,
            *,
            aligned_ready: bool,
            dual_terminal: bool,
    ) -> bool:
        if aligned_ready or dual_terminal or self.exhausted():
            return False
        if self._last_publish_at is None:
            return True
        return (float(now) - self._last_publish_at) >= self.next_backoff_s()


class DualArmSyncTracker:
    """Independent left/right sync outcomes from /leader_arm/*/sync_progress."""

    SIDES = ("left", "right")

    def __init__(self) -> None:
        self._outcomes = {
            side: SideSyncOutcome.UNKNOWN for side in self.SIDES}
        self._reasons = {side: "" for side in self.SIDES}

    def reset(self) -> None:
        for side in self.SIDES:
            self._outcomes[side] = SideSyncOutcome.UNKNOWN
            self._reasons[side] = ""

    def on_progress(
            self, side: str, outcome: Optional[str], text: Optional[str],
    ) -> SideSyncOutcome:
        if side not in self._outcomes:
            return SideSyncOutcome.UNKNOWN
        classified = classify_sync_failure(outcome, text)
        # Do not downgrade a SYNCED arm because the sibling rejected a shared
        # switch_sync republish.
        current = self._outcomes[side]
        if (current is SideSyncOutcome.SYNCED
                and classified is not SideSyncOutcome.SYNCED):
            return current
        self._outcomes[side] = classified
        if text:
            self._reasons[side] = text
        return classified

    def outcome(self, side: str) -> SideSyncOutcome:
        return self._outcomes.get(side, SideSyncOutcome.UNKNOWN)

    def reason(self, side: str) -> str:
        return self._reasons.get(side, "")

    def any_synced(self) -> bool:
        return any(
            self._outcomes[side] is SideSyncOutcome.SYNCED
            for side in self.SIDES)

    def all_deterministic_failed(self) -> bool:
        return all(
            self._outcomes[side] is SideSyncOutcome.FAILED_DETERMINISTIC
            for side in self.SIDES)

    def dual_terminal(self) -> bool:
        """True when further shared switch_sync republish is useless."""
        if self.all_deterministic_failed():
            return True
        if not self.any_synced():
            return False
        # One arm SYNCED: stop once the sibling is also terminal (synced or
        # deterministic fail). Keep retrying while sibling is unknown/syncing
        # or only retryable-failed.
        for side in self.SIDES:
            status = self._outcomes[side]
            if status in (
                    SideSyncOutcome.UNKNOWN,
                    SideSyncOutcome.SYNCING,
                    SideSyncOutcome.FAILED_RETRYABLE):
                return False
        return True

    def operator_hints(self) -> list[str]:
        hints = []
        for side in self.SIDES:
            if self._outcomes[side] is SideSyncOutcome.FAILED_DETERMINISTIC:
                reason = self._reasons[side] or "deterministic sync reject"
                hints.append(f"{side}: {reason}")
        return hints
