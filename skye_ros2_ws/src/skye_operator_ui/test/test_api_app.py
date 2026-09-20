import pytest
from fakes import make_fake_stack
from fastapi.testclient import TestClient

from skye_operator_ui.api_app import PendingTracker, create_app
from skye_operator_ui.session_state import SessionState, UiMode


@pytest.fixture(name="fake_stack")
def fake_stack_fixture():
    """Local copy of the conftest fixture so --noconftest runs still work."""
    return make_fake_stack()


def _ready_session(supervisor, mode=UiMode.teleop_record):
    supervisor.logic.begin_start("thor", mode)
    supervisor.logic.precheck_ok()
    supervisor.logic.mark_ready()


def test_command_rejects_unknown(fake_stack):
    app = create_app(*fake_stack)
    client = TestClient(app)
    response = client.post("/api/command", json={"op": "joint_control"})
    assert response.status_code == 400


def test_session_start_body(fake_stack):
    supervisor, bridge, builder = fake_stack
    app = create_app(supervisor, bridge, builder)
    client = TestClient(app)
    response = client.post(
        "/api/session/start",
        json={"profile": "thor", "mode": "teleop_record"},
    )
    assert response.status_code in (200, 202)
    assert response.json()["ok"] is True
    assert supervisor.logic.state() == SessionState.STARTING
    assert bridge._mode == UiMode.teleop_record


def test_snapshot_endpoint(fake_stack):
    app = create_app(*fake_stack)
    client = TestClient(app)
    response = client.get("/api/snapshot")
    assert response.status_code == 200
    body = response.json()
    assert "session" in body
    assert "next_hint" in body


def test_command_rejects_before_ready(fake_stack):
    supervisor, bridge, builder = fake_stack
    app = create_app(supervisor, bridge, builder)
    client = TestClient(app)
    response = client.post("/api/command", json={"op": "switch_sync"})
    assert response.status_code == 400
    assert "尚未就绪" in response.json()["reason"]


def test_emergency_stop_allowed_in_idle_when_bridge_available(fake_stack):
    supervisor, bridge, builder = fake_stack
    app = create_app(supervisor, bridge, builder)
    client = TestClient(app)
    assert supervisor.logic.state() == SessionState.IDLE
    response = client.post("/api/command", json={"op": "emergency_stop"})
    assert response.status_code == 200
    assert response.json()["ok"] is True
    assert bridge.dispatched == ["emergency_stop"]


def test_session_stop(fake_stack):
    supervisor, bridge, builder = fake_stack
    app = create_app(supervisor, bridge, builder)
    client = TestClient(app)
    client.post(
        "/api/session/start",
        json={"profile": "thor", "mode": "teleop_record"},
    )
    response = client.post("/api/session/stop")
    assert response.status_code == 200
    assert supervisor.logic.state() == SessionState.IDLE


def test_logs_endpoint(fake_stack):
    app = create_app(*fake_stack)
    client = TestClient(app)
    response = client.get("/api/logs/driver")
    assert response.status_code == 200
    assert response.json()["lines"] == ["log:driver"]


def test_pending_op_set_then_cleared_by_state(fake_stack):
    supervisor, bridge, builder = fake_stack
    _ready_session(supervisor)
    app = create_app(supervisor, bridge, builder)
    client = TestClient(app)

    bridge.state["teleop_state"] = "SYNCED"
    bridge.state["align_status"] = "IDLE"
    assert client.post("/api/command", json={"op": "align_start"}).status_code == 200
    assert client.get("/api/snapshot").json()["pending_op"] == "align_start"

    bridge.state["align_status"] = "ALIGNING"
    assert client.get("/api/snapshot").json()["pending_op"] is None


def test_pending_op_cleared_on_timeout(fake_stack):
    supervisor, bridge, builder = fake_stack
    _ready_session(supervisor)
    app = create_app(supervisor, bridge, builder, pending_timeout_s=0.05)
    client = TestClient(app)

    assert client.post("/api/command", json={"op": "align_start"}).status_code == 200
    assert client.get("/api/snapshot").json()["pending_op"] == "align_start"

    import time

    time.sleep(0.06)
    assert client.get("/api/snapshot").json()["pending_op"] is None


def test_pending_op_not_set_for_unconfirmable_op(fake_stack):
    supervisor, bridge, builder = fake_stack
    app = create_app(supervisor, bridge, builder)
    client = TestClient(app)

    assert client.post("/api/command", json={"op": "emergency_stop"}).status_code == 200
    assert client.get("/api/snapshot").json()["pending_op"] is None


def test_pending_op_not_set_when_dispatch_fails(fake_stack):
    supervisor, bridge, builder = fake_stack
    _ready_session(supervisor)
    bridge.dispatch_result = (False, "服务不可用")
    app = create_app(supervisor, bridge, builder)
    client = TestClient(app)

    assert client.post("/api/command", json={"op": "align_start"}).status_code == 400
    assert client.get("/api/snapshot").json()["pending_op"] is None


def test_session_stop_clears_pending(fake_stack):
    supervisor, bridge, builder = fake_stack
    _ready_session(supervisor)
    app = create_app(supervisor, bridge, builder)
    client = TestClient(app)

    client.post("/api/command", json={"op": "align_start"})
    assert client.post("/api/session/stop").status_code == 200
    assert client.get("/api/snapshot").json()["pending_op"] is None


def test_tick_loop_survives_supervisor_exception(fake_stack):
    supervisor, bridge, builder = fake_stack

    def boom() -> None:
        raise RuntimeError("tick exploded")

    supervisor.tick = boom
    app = create_app(supervisor, bridge, builder)
    with TestClient(app) as client:
        assert client.get("/api/snapshot").status_code == 200


@pytest.mark.parametrize(
    ("op", "mailbox", "expected"),
    [
        ("recorder_start", {"recording_active": True}, None),
        ("recorder_start", {"recording_active": False}, "recorder_start"),
        ("takeover", {"hitl_mode": "HANDOVER_SYNC"}, None),
        ("takeover", {"hitl_mode": "AUTONOMOUS"}, "takeover"),
        ("enter_teleop", {"hitl_mode": "HUMAN"}, None),
        ("enter_teleop", {"hitl_mode": "HANDOVER_SYNC"}, "enter_teleop"),
    ],
)
def test_pending_tracker_confirmers(op, mailbox, expected):
    tracker = PendingTracker(timeout_s=60.0)
    tracker.set(op)
    assert tracker.resolve(mailbox) == expected
