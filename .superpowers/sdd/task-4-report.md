# Task 4 Report: Snapshot exposes sync_coach

## Status
Complete.

## Changes
- `snapshot.py`: `SnapshotBuilder.build` calls `build_sync_coach` with mailbox `sync_progress_left/right`, `teleop_state`, `hitl_mode`, `leader_arms`, and `now_s=time.time()`; adds top-level `sync_coach` to the snapshot dict.
- `test_snapshot.py`: `test_snapshot_includes_sync_coach_when_syncing`; SYNCED default path asserts `sync_coach.visible is False`; partial `RosBridge` mock sets `_sync_progress` for mailbox test compatibility with Task 3 bridge fields.

## Tests
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=skye_ros2_ws/src/skye_operator_ui:$PYTHONPATH \
  python -m pytest skye_ros2_ws/src/skye_operator_ui/test/test_snapshot.py \
  skye_ros2_ws/src/skye_operator_ui/test/test_api_app.py -v
```
Result: **29 passed**.

## Commit
`feat(operator_ui): expose sync_coach on WS snapshot`

## Notes
- `fakes.py` unchanged: missing sync progress keys are handled via `mailbox.get` and default leader arms in snapshot output.
- `sync_coach.visible` follows phase gate in `build_sync_coach` (e.g. hidden when `teleop_state` is `SYNCED`).
