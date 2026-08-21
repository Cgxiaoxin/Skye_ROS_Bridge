from skye_hitl_dagger.control_mode import ControlArbiterLogic, ControlModeState


def test_starts_autonomous():
    logic = ControlArbiterLogic()
    assert logic.mode() == ControlModeState.AUTONOMOUS
    assert logic.active_source() == "policy"


def test_takeover_enters_handover_then_human():
    logic = ControlArbiterLogic()
    assert logic.request_takeover() is True
    assert logic.mode() == ControlModeState.HANDOVER_SYNC
    assert logic.active_source() == "hold"
    assert logic.sync_completed() is True
    assert logic.mode() == ControlModeState.HUMAN
    assert logic.active_source() == "teleop"


def test_return_only_from_human():
    logic = ControlArbiterLogic()
    assert logic.request_return() is False
    logic.request_takeover()
    logic.sync_completed()
    assert logic.request_return() is True
    assert logic.mode() == ControlModeState.AUTONOMOUS


def test_return_aborts_handover_sync():
    logic = ControlArbiterLogic()
    logic.request_takeover()
    assert logic.request_return() is True
    assert logic.mode() == ControlModeState.AUTONOMOUS


def test_takeover_ignored_when_not_autonomous():
    logic = ControlArbiterLogic()
    logic.request_takeover()
    assert logic.request_takeover() is False
