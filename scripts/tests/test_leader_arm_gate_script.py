"""Static checks for leader_arm_gate.sh (no hardware)."""

from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "leader_arm_gate.sh"


def test_off_branch_exports_robot_profile_before_disable():
    text = SCRIPT.read_text()
    off_idx = text.index('if [[ "$ACTION" == "off" ]]; then')
    off_block = text[off_idx : text.index("fi", off_idx) + 2]
    assert 'export ROBOT_PROFILE="$PROFILE"' in off_block
    export_pos = off_block.index('export ROBOT_PROFILE="$PROFILE"')
    disable_pos = off_block.index("disable_leader_dynamixel.py")
    assert export_pos < disable_pos
