# Impedance Trajectory Follow Test — Design

**Date:** 2026-09-21  
**Status:** Approved  
**Location:** `pd_test/imp_traj_test.py`

## Problem

Under joint impedance, raising command rate (e.g. faster policy inference) can make the arm lag: commanded waypoints are not reached in the expected time, so absolute pose drifts. We need a small standalone SDK script to quantify this with taught waypoints.

## Approach

Single Python script using `GentoRobot` (Python binding of Gento L1 SDK):

1. **`--collect`**: enter `DragJoint`, record sparse joint waypoints (deg) on Enter → `waypoints.txt`.
2. **`--play`**: linear joint-space interpolate → `ImpJoint` at high vel/acc → stream `SetJointPosCmd` at fixed `--dt` (do not wait for arrival) → log lag metrics.

## CLI

| Flag | Default | Meaning |
|------|---------|---------|
| `--collect` / `--play` | required one | mode |
| `--arm` | `left` | `left`=ARM0, `right`=ARM1 |
| `--profile` | `orin` | `orin`→`6.6.7.190`, `thor`→`6.6.7.191` |
| `--ip` | from profile | override IP |
| `--waypoints` | `pd_test/waypoints.txt` | sparse deg file |
| `--log` | `pd_test/play_log.csv` | play metrics |
| `--vel` / `--acc` | `100` | speed ratios (near limit) |
| `--dt` | `0.02` | command period (s) |
| `--seg-time` | `1.0` | expected duration per taught segment (s) |
| `--tol` | `1.0` | arrival ∞-norm threshold (deg) |

`GENTO_SDK_ROOT` points at a tree containing `PYTHON_SDK/GentoRobot.py` + `libGentoSDKPY.so` (not vendored under `third_party/`).

## Metrics (play)

Per command step CSV: `i,t_cmd,err_max_deg,fb_vel_max,arrived,cmd0..6,fb0..6`  
Summary: late count (`arrived==0`), max/mean `err_max_deg`, max/mean `fb_vel_max`.

## Safety

- Exclusive link: stop ROS `skye_robot_driver` first.
- Enter → emergency stop; always IDLE + unlink in `finally`.

## Non-goals

ROS integration, Cartesian impedance, MoveJ planner upload, dual-arm sync.
