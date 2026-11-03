# test/test_sync_progress_parse.py
from skye_operator_ui.sync_progress_parse import (
    parse_sync_progress_text,
    parse_sync_progress_message,
)

SAMPLE_RIGHT = """\
[right] HOLD_SETTLING：6/8 轴当前达标（相对最终同步目标）
[right] Waiting for SYNCED: HOLD_SETTLING (1.000 / 25.000 s)
[right] J1: |position error|=0.215827 rad > 0.160000 rad (actual=0.100000, target=0.315827)
[right] J2: |velocity|=0.191847 rad/s > 0.100000 rad/s
[right] Continuous settling: 0.000 / 0.250 s
"""

SAMPLE_OK = """\
[left] HOLD_SETTLING：8/8 轴当前达标（相对最终同步目标）
[left] Waiting for SYNCED: HOLD_SETTLING (0.000 / 25.000 s)
[left] All eight axes meet instantaneous tolerances in this sample; they must remain within tolerance continuously for 0.250 s.
[left] Continuous settling: 0.120 / 0.250 s
"""


def test_parse_blockers_pos_and_vel():
    parsed = parse_sync_progress_text(SAMPLE_RIGHT)
    assert parsed["parse_ok"] is True
    assert parsed["mode"] == "HOLD_SETTLING"
    assert parsed["within"] == 6
    assert parsed["total"] == 8
    assert parsed["blockers"] == [
        {"axis": "J1", "kind": "pos", "value": 0.215827, "limit": 0.16},
        {"axis": "J2", "kind": "vel", "value": 0.191847, "limit": 0.1},
    ]
    assert parsed["settling"] == {"current_s": 0.0, "need_s": 0.25}


def test_parse_all_within_no_blockers():
    parsed = parse_sync_progress_text(SAMPLE_OK)
    assert parsed["within"] == 8
    assert parsed["blockers"] == []
    assert parsed["settling"] == {"current_s": 0.12, "need_s": 0.25}


def test_parse_message_merges_envelope():
    parsed = parse_sync_progress_message(
        {"side": "right", "text": SAMPLE_RIGHT, "outcome": "active", "stamp_s": 10.5}
    )
    assert parsed["side"] == "right"
    assert parsed["outcome"] == "active"
    assert parsed["stamp_s"] == 10.5
    assert parsed["parse_ok"] is True


def test_parse_garbage_sets_parse_ok_false():
    parsed = parse_sync_progress_text("not a sync progress blob")
    assert parsed["parse_ok"] is False
    assert parsed["blockers"] == []
