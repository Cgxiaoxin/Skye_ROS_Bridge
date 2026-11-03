# Task 8 Report: Frontend steps/panel + Operator UI docs

## Status
**Complete**

## Changes
- `web/src/lib/steps.js`: `TELEOP_STEPS` uses playbook step `teleop` + runtime `teleop_run`; `PLAYBOOK_IDS.teleop_record` → `['teleop','recorder']`; removed marvin/align steps and align status logic.
- `web/src/lib/commands.js`: `switch_teleop` gated on `SYNCED` only; removed align ops/handlers from teleop mirror.
- `web/src/components/TeleopPanel.svelte`: removed align buttons and align status row.
- `docs/Operator_UI使用说明.md`: host `teleop.sh` path, flow without align, FTDI offline note, DAgger vs teleop teardown/docker notes.
- `api_app.py`: **no change** — `switch_teleop` pending confirmer already requires `teleop_state == TELEOP` (not ALIGNED).

## Build
```bash
cd skye_ros2_ws/src/skye_operator_ui/web && npm ci && npm run build
```
(dist updated if build succeeded)

## Tests
- Backend `test_commands.py` already covers SYNCED-only `switch_teleop` (Task 7).
- No dedicated frontend unit tests for steps/commands in repo.

## Concerns
- Snapshot may still expose `align` fields; UI simply ignores them in teleop mode.
- DAgger playbook/docs still reference Docker marvin; teleop_record path is host-only.

## Commit
See git log for Task 8 commit message per brief.
