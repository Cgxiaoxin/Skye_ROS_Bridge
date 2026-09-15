# Operator UI Safe Teardown Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans (or implement directly in-session with TDD).

**Goal:** Session stop always removes `skye_marvin_m6` and (by default) disables leader Dynamixel.

**Architecture:** `teardown.safe_teardown()` after process-group terminate; fixed Docker name in marvin wrapper; disable script shared with docs.

**Tech Stack:** Python 3.10, subprocess, pytest, bash, dynamixel_sdk (optional at runtime).

---

### Task 1: Disable script + Docker name

**Files:**
- Create: `scripts/disable_leader_dynamixel.py`
- Modify: `scripts/run_marvin_m6_impedance.sh`

**Steps:** script loads `marvin_ws/.skye/leader_arms.env` or env; writes Torque Enable=0; marvin script uses `--name skye_marvin_m6` and pre-rm.

### Task 2: `teardown.py` + supervisor hook (TDD)

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/teardown.py`
- Modify: `supervisor.py`, `operator_ui_node.py`, `config/default.yaml`
- Test: `test/test_teardown.py`, extend `test_supervisor.py`

**Steps:** failing tests for docker rm + disable call order / skip when disabled → implement → green.

### Task 3: Docs

**Files:** `docs/Operator_UI使用说明.md`, link from design if needed.
