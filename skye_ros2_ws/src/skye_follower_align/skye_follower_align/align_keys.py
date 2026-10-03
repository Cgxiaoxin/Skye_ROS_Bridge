"""Keyboard key → align action mapping (no ROS deps, unit-testable)."""

from __future__ import annotations

from typing import Optional

KEY_TO_ACTION = {
    "s": "align_follower",
    "x": "align_cancel",
    "q": "quit",
}


def map_key(key: str) -> Optional[str]:
    """Map a single key or line to an align action."""
    normalized = key.strip().lower()
    if not normalized:
        return None
    return KEY_TO_ACTION.get(normalized)


def parse_align_sides(data: str) -> Optional[tuple[bool, bool]]:
    """Map /mode/align_follower payload to (left_active, right_active)."""
    if data == "align_follower":
        return True, True
    if data == "align_follower_left":
        return True, False
    if data == "align_follower_right":
        return False, True
    return None
