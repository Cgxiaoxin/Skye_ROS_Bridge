from skye_operator_ui.hints import next_hint
from skye_operator_ui.session_state import SessionLogic, SessionState, UiMode


def test_idle_prompt_select_profile():
    s = SessionLogic()
    hint = next_hint(
        session=s,
        teleop_state=None,
        hitl_mode=None,
        align_status=None,
    )
    assert "机台" in hint or "profile" in hint.lower()


def test_precheck_waiting():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    hint = next_hint(
        session=s,
        teleop_state=None,
        hitl_mode=None,
        align_status=None,
    )
    assert "预检" in hint


def test_starting_waiting():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    s.precheck_ok()
    hint = next_hint(
        session=s,
        teleop_state=None,
        hitl_mode=None,
        align_status=None,
    )
    assert "启动" in hint


def test_failed_suggest_retry_or_stop():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    s.precheck_fail()
    hint = next_hint(
        session=s,
        teleop_state=None,
        hitl_mode=None,
        align_status=None,
    )
    assert "失败" in hint or "重试" in hint


def test_teleop_synced_suggests_align():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    s.precheck_ok()
    s.mark_ready()
    hint = next_hint(
        session=s,
        teleop_state="SYNCED",
        hitl_mode=None,
        align_status="IDLE",
    )
    assert "对齐" in hint


def test_dagger_autonomous_suggests_takeover():
    s = SessionLogic()
    s.begin_start("thor", UiMode.dagger)
    s.precheck_ok()
    s.mark_ready()
    hint = next_hint(
        session=s,
        teleop_state=None,
        hitl_mode="AUTONOMOUS",
        align_status=None,
    )
    assert "接管" in hint


def test_dagger_human_suggests_return():
    s = SessionLogic()
    s.begin_start("thor", UiMode.dagger)
    s.precheck_ok()
    s.mark_ready()
    hint = next_hint(
        session=s,
        teleop_state=None,
        hitl_mode="HUMAN",
        align_status=None,
    )
    assert "交还" in hint


def test_stopping_hint():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    s.precheck_ok()
    s.mark_ready()
    s.begin_stop()
    hint = next_hint(
        session=s,
        teleop_state=None,
        hitl_mode=None,
        align_status=None,
    )
    assert s.state() == SessionState.STOPPING
    assert "结束" in hint or "停止" in hint
