# Task 5 Report: RosBridge + SnapshotBuilder

## Status
**Complete** — TDD cycle finished; 38 passed, 1 skipped.

## Deliverables
| File | Action |
|------|--------|
| `skye_operator_ui/ros_bridge.py` | Created |
| `skye_operator_ui/snapshot.py` | Created |
| `test/test_snapshot.py` | Created |

## TDD
1. Wrote `test_snapshot_contains_hint_and_session` — failed `ModuleNotFoundError`.
2. Implemented `SnapshotBuilder` + `RosBridge` — tests pass.
3. Added `test_dispatch_plan_mode_and_recorder_routing` for pure dispatch mapping.

## APIs
- **RosBridge**: subscribes `/gento/joint_states`, `/gento/robot_state`, grippers, `/teleop/state`, `/align/status`, `/skye/control_mode`; `mailbox()`, `health(key)`, `dispatch(op)`, `set_session_mode(mode)`; no joint_control publishers.
- **dispatch_plan** (pure): mode String topics, recorder services by `UiMode`, intervention + e-stop Trigger.
- **SnapshotBuilder.build**: spec §5.4 fields + `next_hint` via `hints.next_hint` + amber/red `banner`.

## Commit
```
feat(operator_ui): add RosBridge and snapshot builder
```

## Notes
- Brief `FakeSup.logic = logic` in class body → `FakeSup.logic = logic` after class (Python scoping).
- `health`: driver/align/arbiter freshness; marvin/recorder/policy probes for playbook keys.

## Out of Scope (Task 6+)
- `ApiApp`, `operator_ui_node`, WebSocket, FastAPI.

## Follow-up: cache `/gento/robot_state`
- **Status**: Complete — `RosBridge._robot_state_callback` stores latest `Int16MultiArray.data`; `mailbox()` exposes `robot_state` (copy or `None`).
- **Test**: `test_robot_state_cached_in_mailbox` in `test/test_snapshot.py` (`object.__new__` + mock msg, no rclpy).
- **Commit**: `fix(operator_ui): cache robot_state in RosBridge mailbox`
