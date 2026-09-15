"""Tests for session safe teardown helpers."""

from pathlib import Path
from unittest.mock import MagicMock, patch

from skye_operator_ui.session_state import SessionState, UiMode
from skye_operator_ui.supervisor import SessionSupervisor
from skye_operator_ui.teardown import run_safe_teardown


def test_safe_teardown_rms_container_then_disables(tmp_path):
    cfg = {
        "teardown": {
            "marvin_container_name": "skye_marvin_m6",
            "disable_leader": True,
            "disable_via_docker": True,
            "marvin_image": "test-image:tag",
            "docker_rm_timeout_s": 5,
            "disable_timeout_s": 5,
        }
    }
    disable_script = Path(tmp_path) / "scripts" / "disable_leader_dynamixel.py"
    disable_script.parent.mkdir(parents=True)
    disable_script.write_text("# stub\n")
    (Path(tmp_path) / "marvin_ws").mkdir()

    with patch("skye_operator_ui.teardown.subprocess.run") as run:
        run.return_value = MagicMock(returncode=0, stdout="", stderr="")
        notes = run_safe_teardown(str(tmp_path), cfg)

    assert run.call_count == 2
    docker_rm = run.call_args_list[0].args[0]
    docker_disable = run.call_args_list[1].args[0]
    assert docker_rm[:4] == ["docker", "rm", "-f", "skye_marvin_m6"]
    assert docker_disable[0:3] == ["docker", "run", "--rm"]
    assert "test-image:tag" in docker_disable
    assert "/scripts/disable_leader_dynamixel.py" in docker_disable
    assert notes == []


def test_safe_teardown_falls_back_to_host_python(tmp_path):
    cfg = {
        "teardown": {
            "marvin_container_name": "skye_marvin_m6",
            "disable_leader": True,
            "disable_via_docker": True,
            "marvin_image": "test-image:tag",
        }
    }
    disable_script = Path(tmp_path) / "scripts" / "disable_leader_dynamixel.py"
    disable_script.parent.mkdir(parents=True)
    disable_script.write_text("# stub\n")
    (Path(tmp_path) / "marvin_ws").mkdir()

    rm_ok = MagicMock(returncode=0, stdout="", stderr="")
    docker_fail = MagicMock(returncode=1, stdout="", stderr="image missing")
    host_ok = MagicMock(returncode=0, stdout="", stderr="")

    with patch("skye_operator_ui.teardown.subprocess.run") as run:
        run.side_effect = [rm_ok, docker_fail, host_ok]
        notes = run_safe_teardown(str(tmp_path), cfg)

    assert run.call_count == 3
    host_argv = run.call_args_list[2].args[0]
    assert str(disable_script) in host_argv
    assert notes == []


def test_safe_teardown_skips_disable_when_configured_off(tmp_path):
    cfg = {
        "teardown": {
            "marvin_container_name": "skye_marvin_m6",
            "disable_leader": False,
        }
    }
    with patch("skye_operator_ui.teardown.subprocess.run") as run:
        run.return_value = MagicMock(returncode=0)
        run_safe_teardown(str(tmp_path), cfg)

    assert run.call_count == 1
    assert run.call_args.args[0][:3] == ["docker", "rm", "-f"]


def test_safe_teardown_continues_when_docker_rm_fails(tmp_path):
    cfg = {"teardown": {"disable_leader": False, "marvin_container_name": "x"}}
    with patch("skye_operator_ui.teardown.subprocess.run") as run:
        run.side_effect = FileNotFoundError("docker missing")
        notes = run_safe_teardown(str(tmp_path), cfg)
    assert notes
    assert "docker" in notes[0].lower() or "失败" in notes[0]


def _mock_cfg():
    return {
        "step_timeout_s": 5.0,
        "log_ring_size": 100,
        "cleanup_stale_commands": [],
        "precheck": {"skip_ping": True},
        "teardown": {
            "marvin_container_name": "skye_marvin_m6",
            "disable_leader": True,
        },
        "playbook_override": {
            "teleop_record": [
                {
                    "id": "driver",
                    "argv": ["/bin/sleep", "60"],
                    "health_key": "driver",
                    "timeout_s": 3.0,
                    "optional": False,
                },
            ]
        },
    }


def test_supervisor_stop_invokes_safe_teardown(tmp_path, monkeypatch):
    calls = []

    def fake_teardown(repo_root, cfg):
        calls.append((repo_root, cfg.get("teardown", {})))
        return []

    monkeypatch.setattr(
        "skye_operator_ui.supervisor.run_safe_teardown", fake_teardown
    )

    healthy = {"driver": True}
    sup = SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=_mock_cfg(),
        health_fn=lambda k: healthy.get(k, False),
        precheck_fn=lambda: (True, "ok"),
    )
    assert sup.start("thor", UiMode.teleop_record)[0]
    for _ in range(30):
        healthy["driver"] = True
        sup.tick()
        if sup.logic.state() == SessionState.READY:
            break
    assert sup.logic.state() == SessionState.READY
    ok, _ = sup.stop()
    assert ok
    assert len(calls) == 1
    assert calls[0][0] == str(tmp_path)
