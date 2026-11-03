# skye_operator_ui/sync_progress_parse.py
from __future__ import annotations

import re
from typing import Any

_WITHIN_RE = re.compile(r"(\d+)\s*/\s*(\d+)\s*轴当前达标")
_MODE_RE = re.compile(r"\b(TELEOP_SYNCING|HOLD_SETTLING|IDLE|SYNCED|TELEOP)\b")
_POS_RE = re.compile(
    r"(J[1-7]|gripper(?:\(ID8\))?):\s*\|position error\|=([0-9.]+)\s*rad\s*>\s*([0-9.]+)\s*rad"
)
_VEL_RE = re.compile(
    r"(J[1-7]|gripper(?:\(ID8\))?):\s*\|velocity\|=([0-9.]+)\s*rad/s\s*>\s*([0-9.]+)\s*rad/s"
)
_SETTLE_RE = re.compile(r"Continuous settling:\s*([0-9.]+)\s*/\s*([0-9.]+)\s*s")


def _norm_axis(name: str) -> str:
    if name.startswith("gripper"):
        return "gripper"
    return name


def parse_sync_progress_text(text: str) -> dict[str, Any]:
    text = text or ""
    within = total = None
    m = _WITHIN_RE.search(text)
    if m:
        within, total = int(m.group(1)), int(m.group(2))
    mode = None
    mm = _MODE_RE.search(text)
    if mm:
        mode = mm.group(1)
    blockers: list[dict[str, Any]] = []
    for m in _POS_RE.finditer(text):
        blockers.append(
            {
                "axis": _norm_axis(m.group(1)),
                "kind": "pos",
                "value": float(m.group(2)),
                "limit": float(m.group(3)),
            }
        )
    for m in _VEL_RE.finditer(text):
        blockers.append(
            {
                "axis": _norm_axis(m.group(1)),
                "kind": "vel",
                "value": float(m.group(2)),
                "limit": float(m.group(3)),
            }
        )
    settling = None
    sm = _SETTLE_RE.search(text)
    if sm:
        settling = {"current_s": float(sm.group(1)), "need_s": float(sm.group(2))}
    parse_ok = within is not None or bool(blockers) or settling is not None or mode is not None
    return {
        "mode": mode,
        "within": within,
        "total": total,
        "blockers": blockers,
        "settling": settling,
        "parse_ok": parse_ok,
    }


def parse_sync_progress_message(payload: dict[str, Any]) -> dict[str, Any]:
    text = str(payload.get("text") or "")
    parsed = parse_sync_progress_text(text)
    parsed["side"] = payload.get("side")
    parsed["outcome"] = payload.get("outcome")
    parsed["stamp_s"] = payload.get("stamp_s")
    parsed["raw_text"] = text
    return parsed
