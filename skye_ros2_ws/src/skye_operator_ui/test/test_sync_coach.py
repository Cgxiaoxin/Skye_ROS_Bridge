# test/test_sync_coach.py
from skye_operator_ui.sync_coach import build_sync_coach

NOW = 1000.0


def _side(stamp=999.5, outcome="active", blockers=None, disabled=False):
    return {
        "side": "left",
        "mode": "HOLD_SETTLING",
        "within": 6,
        "total": 8,
        "blockers": blockers or [],
        "settling": {"current_s": 0.0, "need_s": 0.25},
        "outcome": outcome,
        "stamp_s": stamp,
        "raw_text": "x",
        "parse_ok": True,
    }


def test_hidden_when_synced_even_if_progress_fresh():
    coach = build_sync_coach(
        left=_side(),
        right=_side(),
        teleop_state="SYNCED",
        hitl_mode=None,
        leader_arms={"left_enabled": True, "right_enabled": True},
        now_s=NOW,
    )
    assert coach["visible"] is False


def test_visible_during_teleop_syncing():
    coach = build_sync_coach(
        left=_side(),
        right=None,
        teleop_state="TELEOP_SYNCING",
        hitl_mode=None,
        leader_arms={"left_enabled": True, "right_enabled": True},
        now_s=NOW,
    )
    assert coach["visible"] is True
    assert coach["left"]["stale"] is False


def test_visible_during_handover_sync():
    coach = build_sync_coach(
        left=_side(),
        right=_side(),
        teleop_state="SYNCED",
        hitl_mode="HANDOVER_SYNC",
        leader_arms={"left_enabled": True, "right_enabled": True},
        now_s=NOW,
    )
    assert coach["visible"] is True


def test_disabled_side_when_gate_off():
    coach = build_sync_coach(
        left=_side(),
        right=None,
        teleop_state="TELEOP_SYNCING",
        hitl_mode=None,
        leader_arms={"left_enabled": True, "right_enabled": False},
        now_s=NOW,
    )
    assert coach["right"]["disabled"] is True
    assert coach["left"]["disabled"] is False


def test_stale_flag_when_stamp_old():
    coach = build_sync_coach(
        left=_side(stamp=NOW - 5.0),
        right=None,
        teleop_state="TELEOP_SYNCING",
        hitl_mode=None,
        leader_arms={"left_enabled": True, "right_enabled": True},
        now_s=NOW,
    )
    assert coach["left"]["stale"] is True
