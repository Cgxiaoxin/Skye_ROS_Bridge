# Task 6 Report: Hardware validation checklist

**Status:** Complete (offline); hardware steps pending operator

**Commit:** `docs: add robot profile hardware checklist`

## Offline checks

- Profiles present: `config/profiles/{thor,orin}.yaml`, `marvin_ws/configs/{thor,orin}/grav_comp_m6_*.yaml`
- `ros2 launch … --show-args` OK for `robot_profile:=thor|orin`
- Brief launch `connect_on_startup:=false` loads thor overlay (no SDK connect)
- `sync_marvin_overlay.sh` OK for both `ROBOT_PROFILE=thor|orin`

## Deliverables

- `docs/superpowers/plans/2026-09-03-robot-profile-hw-checklist.md` — Steps 1–9; offline items `[x]`, HW `[ ]`
- Doc polish: `docs/小臂大臂启动步骤.md` — 双机切换 demoted; `ROBOT_PROFILE` in daily §1/§2

## Operator notes

- Thor right leader `joint_signs` match Orin — verify on Thor hardware (Step 4)
- Rollback: `git checkout d591505`

**Report path:** `.superpowers/sdd/task-6-report.md`
