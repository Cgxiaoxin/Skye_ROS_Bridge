from skye_operator_ui.session_state import SessionState, UiMode
from skye_operator_ui.supervisor import SessionSupervisor


def test_supervisor_mock_playbook(tmp_path, monkeypatch):
    cfg = {
        "step_timeout_s": 5.0,
        "log_ring_size": 100,
        "cleanup_stale_commands": [],
        "precheck": {"skip_ping": True},
        "playbook_override": {
            "teleop_record": [
                {
                    "id": "a",
                    "argv": ["/bin/sleep", "60"],
                    "health_key": "a",
                    "timeout_s": 3.0,
                    "optional": False,
                },
            ]
        },
    }
    healthy = {"a": False}

    def health(key):
        return healthy.get(key, False)

    sup = SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=cfg,
        health_fn=health,
        precheck_fn=lambda: (True, "ok"),
    )
    ok, _ = sup.start("thor", UiMode.teleop_record)
    assert ok
    # still starting
    for _ in range(5):
        sup.tick()
    healthy["a"] = True
    for _ in range(20):
        sup.tick()
        if sup.logic.state() == SessionState.READY:
            break
    assert sup.logic.state() == SessionState.READY
    ok, _ = sup.stop()
    assert ok
    assert sup.logic.state() == SessionState.IDLE
