"""Exercise USB identity and cross-pairing without opening serial ports."""

import importlib.util
from pathlib import Path
import runpy
import subprocess
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("teleop_entrypoint", SCRIPTS / "teleop.py")
teleop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(teleop)


@pytest.fixture
def usb(monkeypatch, tmp_path):
    import yaml

    by_id = tmp_path / "by-id"
    by_id.mkdir()
    monkeypatch.setattr(teleop, "BY_ID", by_id)
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    def connect(device, side):
        directory = teleop.PACKAGE / ("config" if device == "a" else "config/orin")
        config = directory / f"grav_comp_m6_{side}.yaml"
        serial = yaml.safe_load(config.read_text())["dynamixel"]["dynamixel_port"]
        (by_id / serial).symlink_to("/dev/null")
        return config
    return connect


@pytest.mark.parametrize("robot", ["thor", "orin"])
@pytest.mark.parametrize("device", ["a", "b"])
def test_cross_pairing_and_environment(usb, monkeypatch, robot, device):
    configs = {side: usb(device, side) for side in ("left", "right")}
    captured = []
    monkeypatch.setattr(teleop, "check_ports", lambda *a, **kw: None)
    monkeypatch.setattr(teleop, "ros_environment", lambda: {})
    monkeypatch.setattr(teleop.os, "chdir", lambda *a: None)
    monkeypatch.setattr(Path, "is_file", lambda path: True)
    monkeypatch.setattr(teleop.subprocess, "run", lambda cmd, **kw:
                        subprocess.CompletedProcess(cmd, 1 if cmd[0] == "pgrep" else 0))
    monkeypatch.setattr(teleop.os, "execvpe", lambda exe, cmd, env: captured.append((cmd, env)))
    teleop.main(["start", "--robot", robot])
    command, env = captured[0]
    assert f"robot_profile:={robot}" in command
    assert env["ROS_DOMAIN_ID"] == ("20" if robot == "thor" else "21")
    for side in configs:
        assert f"{side}_config:={configs[side]}" in command


def test_missing_or_ambiguous_usb_never_selects_arbitrarily(usb):
    with pytest.raises(RuntimeError, match="left: 检测到 0"):
        teleop.select_leaders("dual")
    usb("a", "left")
    usb("b", "left")
    with pytest.raises(RuntimeError, match="left: 检测到 2"):
        teleop.select_leaders("left")
    assert teleop.select_leaders("left", "b")["left"][0] == "b"
    with pytest.raises(RuntimeError, match="right: 检测到 0"):
        teleop.select_leaders("dual", "a")


def test_individual_sides_keep_their_own_calibration(usb):
    left = usb("a", "left")
    right = usb("b", "right")
    selected = teleop.select_leaders("dual")
    assert selected["left"][1] == left
    assert selected["right"][1] == right
    teleop.validate_configs(selected)


def test_dry_run_does_not_change_latency_or_load_ros(usb, monkeypatch, capsys):
    usb("b", "right")
    def unexpected(*args, **kwargs):
        pytest.fail("dry run attempted hardware/environment operation")
    monkeypatch.setattr(teleop, "check_ports", unexpected)
    monkeypatch.setattr(teleop, "ros_environment", unexpected)
    assert teleop.main(["start", "--robot", "thor", "--side", "right", "--dry-run"]) == 0
    output = capsys.readouterr().out
    assert "robot_profile:=thor" in output
    assert "config/orin/grav_comp_m6_right.yaml" in output


def test_robot_configuration_has_no_silent_thor_fallback(monkeypatch, tmp_path):
    monkeypatch.delenv("ROBOT_PROFILE", raising=False)
    monkeypatch.setenv("MARVIN_WS", str(tmp_path))
    with pytest.raises(RuntimeError, match="configure"):
        teleop.robot_profile()
    (tmp_path / ".skye").mkdir()
    (tmp_path / ".skye/robot_profile").write_text("orin\n")
    assert teleop.robot_profile() == "orin"
    assert teleop.robot_profile("thor") == "thor"


def test_build_preserves_driver_install_mode(monkeypatch):
    calls = []
    monkeypatch.setattr(teleop, "ros_environment", lambda **kw: {})
    monkeypatch.setattr(teleop.subprocess, "run", lambda cmd, **kw: calls.append(cmd))
    assert teleop.main(["build"]) == 0
    assert "--symlink-install" in calls[0]
    assert "skye_factr_teleop" in calls[0]
    assert "--symlink-install" not in calls[1]
    assert "skye_robot_driver" in calls[1]
    assert "-DAMENT_CMAKE_SYMLINK_INSTALL=OFF" in calls[1]


def test_port_checks_and_latency_setting(usb, monkeypatch, tmp_path):
    usb("a", "left")
    leaders = teleop.select_leaders("left")
    sys_tty = tmp_path / "sys"
    latency = sys_tty / "null/latency_timer"
    latency.parent.mkdir(parents=True)
    latency.write_text("16\n")
    monkeypatch.setattr(teleop, "SYS_TTY", sys_tty)
    monkeypatch.setattr(teleop.shutil, "which", lambda name: "/usr/bin/fuser")
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        if command[0] == "tee":
            latency.write_text(kwargs["input"])
        return subprocess.CompletedProcess(command, 1 if command[0] == "fuser" else 0)
    monkeypatch.setattr(teleop.subprocess, "run", run)
    with pytest.raises(RuntimeError, match="fix-latency"):
        teleop.check_ports(leaders)
    assert latency.read_text() == "16\n"
    teleop.check_ports(leaders, fix_latency=True)
    assert latency.read_text() == "1\n"
    assert len([cmd for cmd in calls if cmd[0] == "tee"]) == 1
    teleop.check_ports(leaders, fix_latency=True)
    assert len([cmd for cmd in calls if cmd[0] == "tee"]) == 1


def test_occupied_bus_prevents_latency_changes(usb, monkeypatch):
    usb("a", "left")
    monkeypatch.setattr(teleop.shutil, "which", lambda name: "/usr/bin/fuser")
    monkeypatch.setattr(teleop.subprocess, "run", lambda cmd, **kw:
                        subprocess.CompletedProcess(cmd, 0, b"1234", b""))
    with pytest.raises(RuntimeError, match="串口占用"):
        teleop.check_ports(teleop.select_leaders("left"), fix_latency=True)


def test_environment_exports_detected_hardware_without_loading_ros(usb, capsys):
    left = usb("b", "left")
    right = usb("b", "right")
    assert teleop.main(["env", "--robot", "thor", "--domain", "73"]) == 0
    script = capsys.readouterr().out
    assert f"LEFT_CONFIG={left}" in script
    assert f"RIGHT_CONFIG={right}" in script
    assert "export ROBOT_PROFILE=thor" in script
    assert "export ROS_DOMAIN_ID=73" in script
    assert f"cd {teleop.WS}" in script
    assert "sudo" not in script


def test_system_launch_passes_explicit_crossed_configs(usb, monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    pytest.importorskip("launch_ros")
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription, RegisterEventHandler

    sys.path.insert(0, str(teleop.PACKAGE))
    namespace = runpy.run_path(str(SCRIPTS / "teleop_system.launch.py"))
    monkeypatch.setitem(namespace["setup"].__globals__, "get_package_share_directory",
                        lambda package: str(teleop.WS / "src" / package))
    context = LaunchContext()
    configs = {f"{side}_config": str(usb("b", side)) for side in ("left", "right")}
    context.launch_configurations.update(robot_profile="thor", side="dual",
                                         driver_config="/test/driver.yaml", **configs)
    actions = namespace["setup"](context)
    assert isinstance(actions[0], RegisterEventHandler)
    assert all(isinstance(action, IncludeLaunchDescription) for action in actions[1:])
    driver, leader = (dict(action.launch_arguments) for action in actions[1:])
    assert driver["robot_profile"] == "thor"
    assert leader == {"robot_profile": "thor", "sync_diagnostics": "true", **configs}
    context.launch_configurations["right_config"] = "/missing/right.yaml"
    with pytest.raises((ValueError, OSError)):
        namespace["setup"](context)
