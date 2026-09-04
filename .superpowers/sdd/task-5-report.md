# Task 5 Report — Docs + operator checklist

**Status:** Complete

## Commits

- `docs: document 1 → s → 2 follower align flow` (docs only)

## Summary

- **`docs/Thor_Orin_遥操启动.md`**: Thor/Orin startup lines now `1 → s → 2`; added shared「对齐（FACTR sync 之后）」section with `start_follower_align.sh`, topic pub, `/align/status` echo; listed script in related files.
- **`docs/ros_interfaces.md`**: Documented `/mode/align_follower`, `/mode/align_cancel`, `/align/status`, `/gento/set_motion_rates`, and follower-align topic graph.
- **`docs/superpowers/specs/2026-09-04-follower-align-after-sync-design.md`**: Status → 已实现（软件）；待 Thor/Orin 实机 HW 验收.
- **`docs/小臂大臂启动步骤.md`**: Link paragraph under 双机切换; keyboard table adds host `s` row.

## Concerns

- None for docs scope. HW validation (ALIGNED threshold, Orin right wrist, TIMEOUT_WARN → teleop) remains operator checklist on real robots.

## APIs verified against code

- `skye_follower_align` nodes + `scripts/start_follower_align.sh`
- `skye_robot_driver/srv/SetMotionRates` remapped to `/gento/set_motion_rates`
