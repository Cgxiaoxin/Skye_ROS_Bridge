# HITL Abs Path + Split Takeover Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox syntax.

**Goal:** Stop synthesizing virtual leader angles for policy; publish follower absolute targets on `*_joint_control_abs`. Split DAgger takeover into sync vs enter-teleop, and reseed relative teleop before HUMAN.

**Architecture:** Arbiter AUTONOMOUS/HANDOVER_SYNC hold → abs pubs (follower pose inverted through joint signs so driver `apply_joint_mapping` recovers joint_states space). Takeover only `switch_sync` and stays in `HANDOVER_SYNC`. New `enter_teleop` calls `hold_current`, then `switch_teleop`, then HUMAN. UI mirrors with two buttons.

**Tech Stack:** ROS2 Humble, skye_hitl_dagger, skye_operator_ui (FastAPI + Svelte)

## Global Constraints

- Policy chunk joints remain follower-space absolute rad (match `/gento/joint_states`)
- Abs topic expects leader-space (same as follower_align); arbiter must inverse-sign before publish
- HUMAN still uses relative `/gento/*_joint_control` via FACTR
- Do not auto `switch_teleop` after sync

---

## Task 1: Follower→abs helper + arbiter publishes abs

**Files:** `policy_abs.py` (new), `control_arbiter_node.py`, tests

- [ ] Add `follower_pose_to_leader_abs(pose, signs, order, offsets=None)`
- [ ] Arbiter: abs pubs; AUTONOMOUS/HANDOVER hold use abs; remove PolicyRelativeSession from hot path
- [ ] HUMAN still relative pubs
- [ ] Tests for inverse-sign roundtrip
- [ ] Commit: `fix(hitl): publish policy/hold on joint_control_abs`

## Task 2: Split takeover state machine

**Files:** `control_mode.py`, `control_arbiter_node.py`, `teleop_sync.py` (if needed), `hitl_keyboard_node.py`, tests

- [ ] `takeover` → HANDOVER_SYNC only (sync, no auto teleop)
- [ ] `enter_teleop` → hold_current + switch_teleop → wait TELEOP → HUMAN
- [ ] Keyboard: `q`=takeover, `e`=enter_teleop, `w`=return
- [ ] Commit: `feat(hitl): split takeover into sync and enter_teleop`

## Task 3: Operator UI

**Files:** commands.py/js, ros_bridge, api_app pending, DaggerPanel, hints, tests, docs

- [ ] Ops: `enter_teleop`; pending: takeover→HANDOVER_SYNC, enter_teleop→HUMAN
- [ ] Buttons: 同步 / 进入遥操 / 交还
- [ ] Commit: `feat(operator-ui): split DAgger sync and teleop buttons`
- [ ] Doc touch: Hint_Dagger + Operator_UI briefly
