# Task 4 Report: Document effort semantics + live acceptance checklist

**Status:** Complete (docs + commit); live HW acceptance deferred to operator.

## Step 1 — `docs/ros_interfaces.md`

- Updated three `joint_states` topic rows: 14-DoF and left/right 7-DoF now document `effort` = SDK `ExternalTorEst` (Nm, axis external torque) and FACTR `torque_feedback` usage.
- Added blockquote under Topic table: `ExternalTorEst` vs `SensorTor`, `enable_follower_gravity_comp: False` requirement, probe script path.

## Step 2 — Commit

```
docs: document joint_states effort as SDK ExternalTorEst

Clarify FACTR torque-feedback contract and yaml gravity flag.
```

Commit: `d215964` — docs: document joint_states effort as SDK ExternalTorEst

## Step 3 — Live acceptance (operator; deferred)

Not run in this session — robot/stack may be busy. Operator checklist when ready:

1. Rebuild driver; restart with `ROBOT_PROFILE=orin ./scripts/start_skye_for_factr.sh` (Docker yaml refresh).
2. `ros2 topic echo /gento/{left,right}_joint_states --once` and `/gento/joint_states --once` (ROS_DOMAIN_ID=21, FastRTPS profile per repo).
3. Verify: `effort` length 7 per side / 14 combined; idle `|effort|` ~±2; `[0:7]`/`[7:14]` match side topics.
4. Docker TELEOP (`2`): gentle push on big-arm tip → leader contact feel (`enable_follower_gravity_comp: False` in grav_comp yaml).

No `docs/log.md` entry (optional per brief).

## Spec coverage

| Requirement | Done |
|-------------|------|
| `ros_interfaces.md` effort semantics | Yes |
| Live echo + TELEOP feel | Deferred — operator |
