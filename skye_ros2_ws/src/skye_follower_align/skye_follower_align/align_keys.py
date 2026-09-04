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
