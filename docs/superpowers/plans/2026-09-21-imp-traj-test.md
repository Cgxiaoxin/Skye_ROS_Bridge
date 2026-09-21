# Imp Traj Test Implementation Plan

> **For agentic workers:** Small single-file task; implement directly (no subagent loop required).

**Goal:** Ship `pd_test/imp_traj_test.py` for drag-teach waypoints + impedance stream replay with lag logs.

**Architecture:** One argparse script; collect writes `waypoints.txt`; play interpolates and streams ImpJoint cmds.

**Tech Stack:** Python3, GentoRobot (`GENTO_SDK_ROOT`).

## Task 1: Implement script

- [ ] Create `pd_test/imp_traj_test.py` per design (collect + play + CSV log)
- [ ] Smoke: `--help` works; import path resolves when `GENTO_SDK_ROOT` set

## Task 2: Docs only (this plan + design already written)

No further files.
