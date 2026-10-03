# Task 5 Report: Wire LeaderArmGate into bridge, API, snapshot, node

## Status
Implemented and verified.

## Changes
- `ros_bridge.py`
  - Added `run_leader_arm_gate(repo_root, side, action, container=..., timeout_s=15.0)` around `scripts/leader_arm_gate.sh`.
  - `RosBridge` now owns/injects a `LeaderArmGate`, container config, repo root, and script runner.
  - `dispatch("leader_*")` runs the script and flips gate state only after script success.
  - `align_start` derives `align_payload` from `gate.align_payload()` and rejects before publish when both arms are off.
  - Successful `switch_sync` string publish locks the gate via `note_sync_dispatched()`.
  - `/teleop/state` updates feed `gate.note_teleop_state(...)`.
  - `mailbox()` includes `leader_arms`.
- `snapshot.py`
  - Snapshot now exposes `leader_arms`.
  - Snapshot now exposes `features.leader_arm_gate`, defaulting to `true`.
- `api_app.py`
  - `/api/command` passes `leader_arms` from mailbox and `leader_arm_gate_enabled` from supervisor cfg into `command_allowed`.
- `operator_ui_node.py`
  - Constructs one `LeaderArmGate` for the process and injects it into `RosBridge`.
  - Resets the gate on `on_before_stop` and process shutdown.
- `config/default.yaml`
  - Added `features.leader_arm_gate: true`.

## Tests Added
- `test_ros_bridge.py`
  - Script runner invokes `leader_arm_gate.sh` with `MARVIN_CONTAINER_NAME`.
  - Script failure returns an error and does not flip gate state.
  - Script success flips gate state.
  - `align_start` publishes side-specific payload and rejects when no arms are enabled.
  - `switch_sync` and teleop state lock the gate.
- `test_snapshot.py`
  - Snapshot includes `leader_arms`.
  - Snapshot includes `features.leader_arm_gate` and defaults it to enabled.
- `test_api_app.py`
  - API command authorization receives leader arm snapshot.
  - Feature flag disables leader arm toggle commands.

## Verification
```bash
cd skye_ros2_ws && PYTHONPATH=src/skye_operator_ui PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest src/skye_operator_ui/test/test_snapshot.py \
  src/skye_operator_ui/test/test_api_app.py \
  src/skye_operator_ui/test/test_ros_bridge.py -v
# 41 passed, 1 warning

cd skye_ros2_ws && PYTHONPATH=src/skye_operator_ui PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest src/skye_operator_ui/test/ -v
# 100 passed, 1 skipped, 1 warning
```

IDE lints: no errors found for edited Python files/tests.

## Commit
Commit message:
```bash
git commit -m "feat(operator_ui): wire LeaderArmGate into backend"
```

## Notes / Concerns
- Task 2's `leader_arm_gate.sh on` still exits after 0.5s without verifying the ROS node is alive. Task 5 keeps that contract; `RosBridge` only treats a zero script exit as success.
- Frontend UI and follower align node changes were intentionally left out for Tasks 7 and 6.

---

## Review Fix (Important)

**Status:** Complete

### Change
- `run_leader_arm_gate`: catch `subprocess.TimeoutExpired` and `FileNotFoundError`; return `(False, msg)` so `dispatch` does not flip `LeaderArmGate` on timeout or missing script.

### Tests added (`test_ros_bridge.py`)
- `test_run_leader_arm_gate_timeout_returns_false`
- `test_run_leader_arm_gate_missing_script_returns_false`

### Test run
```bash
cd skye_ros2_ws && PYTHONPATH=src/skye_operator_ui PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest src/skye_operator_ui/test/ -k 'leader or gate or script' -v
# 22 passed, 1 warning
```

### Commit
```
fix(operator_ui): handle leader_arm_gate script timeout
```
