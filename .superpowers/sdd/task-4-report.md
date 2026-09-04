# Task 4 Report: Leader grav_comp profiles + sync script

**Status:** Complete

**Commit:** `3738ab8` — `feat(marvin): per-profile grav_comp configs for thor/orin`

## Changes

- Seeded `marvin_ws/configs/thor/` from Thor install; `marvin_ws/configs/orin/` from `robotiq_teleop` (right) + install (left).
- Removed flat `marvin_ws/configs/grav_comp_m6_right.yaml` (moved to `orin/`).
- `_grav_comp_config` in both teleop launch overlays now resolves `configs/<ROBOT_PROFILE>/`.
- `sync_marvin_overlay.sh` copies `grav_comp_m6_*.yaml` from active profile dir into install.
- `run_marvin_m6_impedance.sh` exports `ROBOT_PROFILE` (default thor) before sync and passes it into Docker.
- `bind_leader_arms.py` patches profile yaml first, then install copies.

## Dry-run sync

```
ROBOT_PROFILE=thor  → config[thor]: grav_comp_m6_left.yaml, grav_comp_m6_right.yaml
ROBOT_PROFILE=orin  → config[orin]: grav_comp_m6_left.yaml, grav_comp_m6_right.yaml
```

Restored `ROBOT_PROFILE=thor` on this machine.

## Verification

- Orin right `joint_signs: [1, -1, 1, -1, 1, -1, -1, -1]` ✓
- Thor configs carry header: `# profile: thor — verify joint_signs on Thor hardware before relying on teleop`

## Concerns

- Thor right leader yaml currently matches Orin signs (install may have been Orin-polluted). Hardware validation in Task 6 required before production teleop on Thor.
- Orin left seeded from current install (same as Thor left); confirm on Orin hardware if signs differ.
