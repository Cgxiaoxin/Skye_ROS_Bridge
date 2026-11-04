# skye_ros2_ws/src/skye_operator_ui/test/test_disable_leader_side.py
import importlib.util
import os
from pathlib import Path

def _load():
    root = Path(__file__).resolve().parents[4]
    path = root / "scripts" / "disable_leader_dynamixel.py"
    spec = importlib.util.spec_from_file_location("disable_leader_dynamixel", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

def test_sides_for_arg():
    mod = _load()
    assert mod.sides_for_arg("left") == ("left",)
    assert mod.sides_for_arg("right") == ("right",)
    assert mod.sides_for_arg("both") == ("left", "right")


def test_ports_from_profile_yaml(monkeypatch):
    mod = _load()
    root = Path(__file__).resolve().parents[4]
    monkeypatch.delenv("ROBOT_LEADER_DYNAMIXEL_PORT_LEFT", raising=False)
    monkeypatch.delenv("ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT", raising=False)
    monkeypatch.setenv("ROBOT_PROFILE", "orin")

    ports = mod._ports_from_host_yaml(root, "orin")
    assert "FTB8HNOT" in ports["left"]
    assert "FTAO51EA" in ports["right"]

    mod._fill_ports_from_profile_if_missing(root)
    assert "FTB8HNOT" in os.environ["ROBOT_LEADER_DYNAMIXEL_PORT_LEFT"]
    assert "FTAO51EA" in os.environ["ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT"]
