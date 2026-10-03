# skye_ros2_ws/src/skye_operator_ui/test/test_disable_leader_side.py
from pathlib import Path
import importlib.util

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
