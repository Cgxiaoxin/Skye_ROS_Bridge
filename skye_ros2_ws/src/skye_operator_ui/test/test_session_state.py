from skye_operator_ui.session_state import SessionLogic, SessionState, UiMode


def test_start_only_from_idle():
    s = SessionLogic()
    assert s.begin_start("thor", UiMode.teleop_record)
    assert s.state() == SessionState.PRECHECK
    assert not s.begin_start("orin", UiMode.dagger)


def test_happy_path_to_ready_and_stop():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    assert s.precheck_ok()
    assert s.state() == SessionState.STARTING
    assert s.mark_ready()
    assert s.state() == SessionState.READY
    assert s.begin_stop()
    assert s.state() == SessionState.STOPPING
    assert s.mark_idle()
    assert s.state() == SessionState.IDLE


def test_profile_locked_until_idle():
    s = SessionLogic()
    s.begin_start("thor", UiMode.dagger)
    assert not s.can_change_profile_or_mode()
    s.precheck_fail()
    s.begin_stop()
    s.mark_idle()
    assert s.can_change_profile_or_mode()


def test_degraded_from_ready():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    s.precheck_ok()
    s.mark_ready()
    assert s.mark_degraded()
    assert s.state() == SessionState.DEGRADED


def test_illegal_transitions_from_idle_leave_state_unchanged():
    s = SessionLogic()
    assert s.state() == SessionState.IDLE
    assert s.profile() is None
    assert s.mode() is None

    assert not s.mark_ready()
    assert s.state() == SessionState.IDLE
    assert s.profile() is None
    assert s.mode() is None

    assert not s.begin_stop()
    assert s.state() == SessionState.IDLE
    assert s.profile() is None
    assert s.mode() is None


def test_begin_start_invalid_profile_leaves_idle():
    s = SessionLogic()
    assert not s.begin_start("invalid", UiMode.teleop_record)
    assert s.state() == SessionState.IDLE
    assert s.profile() is None
    assert s.mode() is None
