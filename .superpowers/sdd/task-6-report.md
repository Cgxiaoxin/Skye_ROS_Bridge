# Task 6 Report — Follower align hardware validation (offline template)

**Status:** Complete (offline template only); **no hardware run in this session**

## Commits

- `3f5d362` — `docs: add follower-align HW validation checklist`

## Summary

- Created `docs/superpowers/plans/2026-09-04-follower-align-hw-checklist.md` — unchecked operator checklist covering Thor/Orin `1→s→2`, slow motion + ratio 10, `ALIGNED`/restore, Orin right wrist signs, forced `TIMEOUT_WARN` → still `2`, `x` cancel/hold, and topic-only path (`enable_keyboard:=false`).
- Noted **pending** live `/gento/set_motion_rates` smoke from Task 2 (driver connected).
- `docs/superpowers/plans/2026-09-04-follower-align-after-sync.md` already tracked on `main`; not re-added.

## Operator next steps

Run checklist on Thor then Orin; fill recorded fields; check sign-off boxes when HW passes.

**Report path:** `.superpowers/sdd/task-6-report.md`

## Final-review fixes

**Status:** All Critical + High findings addressed; M1–M3 included.

### Findings

| ID | Fix |
|----|-----|
| C1 | `start_follower_align.sh`: `set +u`/`set -u` around ROS setup, `ROBOT_PROFILE` thor\|orin validation, FastDDS XML check |
| C2 | `ReentrantCallbackGroup` for service clients + `MultiThreadedExecutor` (comment documents deadlock) |
| C3 | Publish raw leader on `*_joint_control_abs`; signs only in `on_tick`; spec §5.4 note |
| H1 | `request_stop()` + skip `join` when quit from reader thread |
| H2 | Stale `/gento/joint_states`: skip abs publish but keep `on_tick` so timeout fires |
| H3 | `_shutdown_cleanup()` on KeyboardInterrupt / destroy restores rates |
| M1 | Failed align-rate set still attempts restore to 30% |
| M2 | Launch `OpaqueFunction` raises if `robot_profile` ∉ {thor, orin} |
| M3 | `/align/status` QoS `TRANSIENT_LOCAL` |

### Verification

```text
$ cd skye_ros2_ws && PYTHONPATH=src/skye_follower_align python3 -m pytest src/skye_follower_align/test/test_align_logic.py -v
8 passed in 0.01s

$ ./scripts/build.sh skye_follower_align
Summary: 1 package finished [0.53s]

$ ROBOT_PROFILE=thor timeout 3 ./scripts/start_follower_align.sh
== follower_align profile=thor ROS_DOMAIN_ID=21 ==
[INFO] [follower_align_node-1]: process started
[follower_align_node-1] align status: IDLE
(past setup.bash — no AMENT_TRACE unbound)
```

### Commits

- `28958eb` — `fix(align): unblock service calls, signs, start script, shutdown safety`
