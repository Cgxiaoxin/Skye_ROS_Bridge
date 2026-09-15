from fastapi.testclient import TestClient

from skye_operator_ui.api_app import create_app
from skye_operator_ui.session_state import SessionState, UiMode


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
