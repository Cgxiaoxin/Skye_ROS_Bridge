"""Static checks for leader_arm_gate.sh (no hardware)."""

from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "leader_arm_gate.sh"


def test_gate_script_soft_mutes_without_pkill_or_nohup():
    text = SCRIPT.read_text()
    assert "pkill" not in text
    assert "nohup" not in text
    assert "disable_leader_dynamixel.py" not in text
    assert "start_teleop_m6_" not in text
    assert "/mode/${SIDE}/leader_gate" in text
    assert "ros2 topic pub" in text
    assert "_source_ros" in text


def test_gate_script_is_executable():
    assert SCRIPT.is_file()
    assert SCRIPT.stat().st_mode & 0o111
