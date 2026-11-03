# Task 6 Report: Host leader arm gate + YAML ports

## Status
**DONE**

## Changes
- `scripts/leader_arm_gate.sh`: host `pkill skye_factr_teleop_{side}` + disable script; `on` sources `skye_ros2_ws` + `setup_factr_teleop_env.sh`, launches `start_teleop_m6_{side}.launch.py`.
- `scripts/disable_leader_dynamixel.py`: `_ports_from_host_yaml` / `_fill_ports_from_profile_if_missing` when `ROBOT_PROFILE` is thor|orin and env ports unset.
- `ros_bridge.py`: `set_session_profile`, `run_leader_arm_gate` exports `ROBOT_PROFILE`; removed Marvin container error translation.
- `api_app.py`: `set_session_profile(body.profile)` on session start.

## Tests
- Added `test_ports_from_profile_yaml`, `test_run_leader_arm_gate_sets_robot_profile`, updated docker-centric ros_bridge tests.
- `21 passed` in `test_disable_leader_side.py` + `test_ros_bridge.py`.

## Concerns
- Host `on` requires built `skye_ros2_ws/install` and `dynamixel_sdk` on host for `off`.
- `run_leader_arm_gate` still accepts unused `container`/`image` kwargs for test injection compatibility.

## Commit
See git log for `feat(teleop): host leader arm gate without Marvin docker`.

## Fix (Task 6 review — Important)
- **Issue:** Manual `leader_arm_gate.sh … off` did not export `ROBOT_PROFILE`; `disable_leader_dynamixel.py` could not load YAML ports unless env was set externally (Operator UI sets it via `run_leader_arm_gate`).
- **Change:** `export ROBOT_PROFILE="$PROFILE"` on the `off` branch before `disable_leader_dynamixel.py` (`PROFILE` from `${ROBOT_PROFILE:-thor}`).
- **Test:** `scripts/tests/test_leader_arm_gate_script.py` — asserts export precedes disable in the `off` block.
- **Pytest:** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=skye_ros2_ws/src/skye_operator_ui python3 -m pytest skye_ros2_ws/src/skye_operator_ui/test/test_disable_leader_side.py skye_ros2_ws/src/skye_operator_ui/test/test_ros_bridge.py scripts/tests/test_leader_arm_gate_script.py -q` → **22 passed**.
- **Commit:** `fix(teleop): export ROBOT_PROFILE on leader_arm_gate off`
