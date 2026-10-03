_LOCK_STATES = frozenset({"TELEOP_SYNCING", "SYNCED", "TELEOP"})


class LeaderArmGate:
    def __init__(self) -> None:
        self._left = True
        self._right = True
        self._locked = False

    def snapshot(self) -> dict:
        return {
            "left_enabled": self._left,
            "right_enabled": self._right,
            "locked": self._locked,
        }

    def reset(self) -> None:
        self._left = True
        self._right = True
        self._locked = False

    def set_enabled(self, side: str, enabled: bool) -> bool:
        if self._locked:
            return False
        if side == "left":
            self._left = bool(enabled)
            return True
        if side == "right":
            self._right = bool(enabled)
            return True
        raise ValueError(side)

    def note_sync_dispatched(self) -> None:
        self._locked = True

    def note_teleop_state(self, state: str | None) -> None:
        if state in _LOCK_STATES:
            self._locked = True

    def any_enabled(self) -> bool:
        return self._left or self._right

    def align_payload(self) -> str | None:
        if self._left and self._right:
            return "align_follower"
        if self._left:
            return "align_follower_left"
        if self._right:
            return "align_follower_right"
        return None
