# DAgger P0 Setpoint Pipeline Implementation Plan

> **For agentic workers:** Execute task-by-task; each task ends with its own commit. P1 deferred until P0 verified.

**Goal:** Fix DAgger AUTONOMOUS motion lag/undershoot with minimal, revertible changes: instrument → setpoint publish → driver abs limit tier → real gripper feedback.

**Architecture:** Keep arbiter as sole `/gento/*` writer. Change Thor client so `--dagger-publish-mode setpoint` publishes a 16-step micro-ramp from `last_cmd`→slewed command each 30 Hz tick; raise arbiter rebase threshold via config only; split driver `max_delta` by topic; replace `NullGripper` with ROS gripper state.

**Tech Stack:** Python client (`docs/dagger_dev/client_thor/vendor/remote_inference_async`), ROS2 `skye_hitl_dagger`, C++ `skye_robot_driver`.

## Global Constraints

- Do not let client write `/gento/*`; do not open RTC; do not change pi05 model/port/history configs.
- Do not change teleop (relative) feel except abs-only limit tier.
- Each subtask = one commit; old params remain readable.
- Arm units in client = degrees; ERR_MAX configured in rad; convert at compare.
- Gripper fields unchanged until P0-c.

---

### Task 0: Instrumentation (commit0)

**Files:**
- Modify: `docs/dagger_dev/client_thor/vendor/remote_inference_async/dagger_publisher.py`
- Modify: `docs/dagger_dev/client_thor/vendor/remote_inference_async/async_robot_client.py` (dagger publish log only)
- Modify: `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/control_arbiter_node.py`
- Docs: `docs/dagger_dev/INVESTIGATION.md` (already updated; include in this commit)

**Produces:** Counters/logs usable in raw mode baseline.

- [ ] **Step 1:** In `DaggerPolicyPublisher`, track `last_publish_mono`, log interval_ms, stamp_sec, dt, chunk_size on each publish (info, throttle OK).
- [ ] **Step 2:** In arbiter `_policy_callback` / joint timer: count `rebase_count`, log jump each rebase; accumulate `hold_tail_s` while `holding_tail`; periodic summary log (e.g. 2s).
- [ ] **Step 3:** Unit smoke: existing `test_chunk_player` / `test_dagger_chunk` still pass.
- [ ] **Step 4:** Commit `chore(dagger): add raw-mode baseline publish/rebase/hold-tail logs`

---

### Task 1: Scheme A setpoint publish (commit1)

**Files:**
- Create/Modify: `dagger_chunk.py` — `setpoint_ramp_to_arbiter_fields(start16, end16, dt)`
- Modify: `dagger_publisher.py` — publish helpers
- Modify: `async_robot_client.py` — `--dagger-publish-mode`, `--dagger-err-max-rad`, ERR_MAX hold, move publish to post-slew for setpoint; raw keeps on-receive publish
- Modify: `run_robot_dagger.sh` / `run_client_dagger.sh` — env `DAGGER_PUBLISH_MODE`
- Modify: `skye_hitl_dagger/config/profiles/{thor,orin}.yaml` — `chunk_start_max_jump_rad: 0.3`
- Test: `test_dagger_chunk.py` — ramp layout + dt

**Produces:** `setpoint` default continuous micro-chunks; raw unchanged path.

- [ ] **Step 1:** Failing tests for 16-step linear ramp fields and dt=`1/(hz*16)`.
- [ ] **Step 2:** Implement ramp builder (arms+gripper dims from start/end vectors in **rad** for publish).
- [ ] **Step 3:** Wire CLI + ERR_MAX (deg2rad compare on arms only); on exceed: publish hold at last_cmd, warn; >0.5s escalate.
- [ ] **Step 4:** setpoint: after slew, convert command to policy rad, build ramp from previous published rad cmd → new; publish; update last published. Do **not** publish raw chunk on receive in setpoint mode.
- [ ] **Step 5:** raw: keep publish-on-receive; no ramp.
- [ ] **Step 6:** Set profile `chunk_start_max_jump_rad: 0.3`.
- [ ] **Step 7:** Commit `feat(dagger): setpoint micro-ramp publish with ERR_MAX and rebase threshold`

---

### Task 2: P0-b driver limit tiers (commit2)

**Files:**
- Modify: `skye_robot_driver/config/skye_robot.yaml`
- Modify: `driver_node.hpp` / `driver_node.cpp` — `max_delta_abs_per_cycle`; abs path uses it; relative keeps `max_delta_per_cycle`
- Test: extend or add driver test if present

**Produces:** abs默认 0.02；relative 不变 0.01；旧参数兼容。

- [ ] **Step 1:** Declare `max_delta_abs_per_cycle` defaulting to `max_delta_per_cycle` when unset; yaml set abs=0.02.
- [ ] **Step 2:** `handle_absolute_command` uses abs limit; relative unchanged.
- [ ] **Step 3:** Confirm impedance ratios already 100 — no code change; note in commit body.
- [ ] **Step 4:** Commit `feat(driver): split max_delta for abs vs relative teleop`

---

### Task 3: P0-c RosGripperFeedback (commit3)

**Files:**
- Modify: `dagger_ros_facade.py` — `RosGripperFeedback` (+ fix `NullGripper.diagnostics` shape for safety)
- Modify: `async_robot_client.py` — use RosGripper in dagger; age guards; startup range check
- Test: new unit tests with fake JointState ages / ranges

**Produces:** Real gripper obs; no KeyError; stale >200ms skips infer.

- [ ] **Step 1:** `RosGripperFeedback.state()` → `(2,) float32 [0,1]`; diagnostics with `left/right.contact_latched` (False until latch ported).
- [ ] **Step 2:** Age warn >50ms; >200ms hold last cmd / skip new infer that cycle.
- [ ] **Step 3:** Startup print values; if outside [0,1] or constant-zero after samples → error exit (no pi05 stats → relaxed rule).
- [ ] **Step 4:** Commit `feat(dagger): RosGripperFeedback from /left|right_gripper/state`

---

## Verification (operator)

1. commit0 + `--dagger-publish-mode raw` + pi05: capture rebase/hold-tail/interval logs as baseline.
2. commit1 setpoint: same task; expect rebase≈0, less hold-tail, smoother cmd.
3. commit2: abs clip rare; teleop feel unchanged.
4. commit3: gripper state non-constant in obs logs.
