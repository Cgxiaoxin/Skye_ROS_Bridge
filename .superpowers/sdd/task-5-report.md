# Task 5 Report: Teardown + hold_current on stop

## Status
Complete.

## Changes
- `teardown.py`: `docker_rm` defaults false; skip `docker rm -f` when disabled. `disable_via_docker` defaults false (host `disable_leader_dynamixel.py` first; docker fallback unchanged when true).
- `default.yaml`: host-era defaults; removed default `docker rm` from `cleanup_stale_commands`; `skye_factr_teleop` pkill pattern.
- `operator_ui_node.py`: `on_before_stop` best-effort `hold_current` after `switch_stop`.
- `test_teardown.py`: new skip-docker and host-disable tests; existing docker tests opt in with `docker_rm: true`.

## Tests
```bash
cd skye_ros2_ws/src/skye_operator_ui
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. pytest test/test_teardown.py test/test_supervisor.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. pytest test/ -q
```
111 passed, 1 skipped (full suite).

## Concerns
- Dagger / Marvin-docker deployments must set `teardown.docker_rm: true` and/or `disable_via_docker: true` via overlay if still needed.
- No unit test for `on_before_stop` `hold_current` dispatch (covered indirectly via supervisor stop path).

## Commit
`feat(operator-ui): host-first teardown without docker rm`
