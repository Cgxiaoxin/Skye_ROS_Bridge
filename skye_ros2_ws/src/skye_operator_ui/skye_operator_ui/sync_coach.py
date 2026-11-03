# skye_operator_ui/sync_coach.py
from __future__ import annotations

from typing import Any

FRESH_S_DEFAULT = 2.0


def _phase_active(teleop_state: str | None, hitl_mode: str | None) -> bool:
    if teleop_state == "TELEOP_SYNCING":
        return True
    if hitl_mode == "HANDOVER_SYNC":
        return True
    return False


def _decorate_side(
    raw: dict[str, Any] | None,
    *,
    side: str,
    enabled: bool,
    now_s: float,
    fresh_s: float,
) -> dict[str, Any]:
    if not enabled:
        return {
            "side": side,
            "disabled": True,
            "stale": False,
            "mode": None,
            "within": None,
            "total": None,
            "blockers": [],
            "settling": None,
            "outcome": None,
            "stamp_s": None,
            "raw_text": "",
            "parse_ok": True,
        }
    if raw is None:
        return {
            "side": side,
            "disabled": False,
            "stale": True,
            "mode": None,
            "within": None,
            "total": None,
            "blockers": [],
            "settling": None,
            "outcome": None,
            "stamp_s": None,
            "raw_text": "",
            "parse_ok": True,
        }
    stamp = raw.get("stamp_s")
    stale = True
    if isinstance(stamp, (int, float)):
        stale = (now_s - float(stamp)) > fresh_s
    out = dict(raw)
    out["side"] = side
    out["disabled"] = False
    out["stale"] = stale
    return out


def build_sync_coach(
    *,
    left: dict[str, Any] | None,
    right: dict[str, Any] | None,
    teleop_state: str | None,
    hitl_mode: str | None,
    leader_arms: dict[str, Any] | None,
    now_s: float,
    fresh_s: float = FRESH_S_DEFAULT,
) -> dict[str, Any]:
    arms = leader_arms or {}
    left_en = arms.get("left_enabled", True) is not False
    right_en = arms.get("right_enabled", True) is not False
    left_view = _decorate_side(
        left, side="left", enabled=left_en, now_s=now_s, fresh_s=fresh_s
    )
    right_view = _decorate_side(
        right, side="right", enabled=right_en, now_s=now_s, fresh_s=fresh_s
    )
    visible = _phase_active(teleop_state, hitl_mode)
    return {"visible": visible, "left": left_view, "right": right_view}
