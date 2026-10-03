# Leader Arm Gate UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let Operator UI teleop_record sessions keep dual-arm startup, then optionally disable left/right leader FACTR+DXL before sync so sync/align/teleop run on the remaining arm(s).

**Architecture:** Pure in-process `LeaderArmGate` state feeds snapshot + command gates. `leader_*_off/on` ops run `scripts/leader_arm_gate.sh` via `docker exec` into the existing `skye_marvin_m6` container. Global `/mode/switch_sync` stays unchanged; `align_start` payload becomes side-aware. Feature flag `features.leader_arm_gate` can hide/disable without code delete.

**Tech Stack:** Python 3.10, pytest, FastAPI, Svelte, bash, ROS 2 Humble, docker exec, dynamixel_sdk (inside Marvin container).

**Spec:** `docs/superpowers/specs/2026-10-03-leader-arm-gate-ui-design.md`

## Global Constraints

- Session start playbook stays dual-arm; do not change default `MARVIN_LAUNCH_CMD`.
- Thor and Orin share the same UI/scripts; ports come from `marvin_ws/.skye/leader_arms.env`.
- After `TELEOP_SYNCING` / `SYNCED` / `TELEOP`, or a successful `switch_sync`, leader gate buttons lock.
- Both arms off → `switch_sync` disabled.
- Failed gate script must not flip `enabled` flags.
- Default path (no gate clicks) must match current latency/behavior.
- Hold-arm warning before disable (UI confirm).

## File map

| File | Responsibility |
|------|----------------|
| `scripts/disable_leader_dynamixel.py` | Add `--side left\|right\|both` |
| `scripts/leader_arm_gate.sh` | `off`/`on` one side inside running Marvin container |
| `skye_operator_ui/.../leader_arms.py` | Pure gate state (`enabled`, `locked`, reset, apply) |
| `skye_operator_ui/.../commands.py` | Whitelist + `command_allowed` for gate/sync/align |
| `skye_operator_ui/.../ros_bridge.py` | `dispatch_plan` align payload; script dispatch; lock on sync |
| `skye_operator_ui/.../snapshot.py` | Expose `leader_arms` + `features` |
| `skye_operator_ui/.../api_app.py` | Pass gate state into `command_allowed` |
| `skye_operator_ui/.../operator_ui_node.py` | Construct gate; reset on session stop |
| `skye_operator_ui/config/default.yaml` | `features.leader_arm_gate: true` |
| `skye_follower_align/.../align_logic.py` | `combine_phase` / helpers for active sides |
| `skye_follower_align/.../follower_align_node.py` | Parse `align_follower_{left\|right}` |
| `web/.../TeleopPanel.svelte` + `commands.js` | Buttons + gating + confirm |
| Docs | Operator UI + Thor/Orin note |

---

### Task 1: `disable_leader_dynamixel.py --side`

**Files:**
- Modify: `scripts/disable_leader_dynamixel.py`
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_disable_leader_side.py` (import/parse via subprocess `--help` or extract argparse into testable function)

**Interfaces:**
- Produces: CLI `python3 scripts/disable_leader_dynamixel.py [--side left|right|both]`; default `both` (teardown unchanged)

- [ ] **Step 1: Write failing test for side selection**

```python
# skye_ros2_ws/src/skye_operator_ui/test/test_disable_leader_side.py
from pathlib import Path
import importlib.util

def _load():
    root = Path(__file__).resolve().parents[4]
    path = root / "scripts" / "disable_leader_dynamixel.py"
    spec = importlib.util.spec_from_file_location("disable_leader_dynamixel", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

def test_sides_for_arg():
    mod = _load()
    assert mod.sides_for_arg("left") == ("left",)
    assert mod.sides_for_arg("right") == ("right",)
    assert mod.sides_for_arg("both") == ("left", "right")
```

- [ ] **Step 2: Run test — expect FAIL (no `sides_for_arg`)**

```bash
cd skye_ros2_ws && python3 -m pytest src/skye_operator_ui/test/test_disable_leader_side.py -v
```

Expected: `AttributeError` or import failure for `sides_for_arg`.

- [ ] **Step 3: Implement `--side` + `sides_for_arg`**

In `scripts/disable_leader_dynamixel.py`:

```python
def sides_for_arg(side: str) -> tuple[str, ...]:
    key = side.strip().lower()
    if key == "left":
        return ("left",)
    if key == "right":
        return ("right",)
    if key == "both":
        return ("left", "right")
    raise ValueError(f"invalid side: {side}")

# main(): argparse --side default both
# loop only selected sides with env keys:
#   left -> ROBOT_LEADER_DYNAMIXEL_PORT_LEFT
#   right -> ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT
```

- [ ] **Step 4: Re-run test — PASS**

- [ ] **Step 5: Commit**

```bash
git add scripts/disable_leader_dynamixel.py skye_ros2_ws/src/skye_operator_ui/test/test_disable_leader_side.py
git commit -m "feat(scripts): add --side to disable_leader_dynamixel"
```

---

### Task 2: `scripts/leader_arm_gate.sh`

**Files:**
- Create: `scripts/leader_arm_gate.sh` (executable)
- Test: manual dry structure via `bash -n`; optional pytest invoking `bash -n`

**Interfaces:**
- Consumes: running container (default `skye_marvin_m6`), `ROBOT_PROFILE` inside container, `leader_arms.env`
- Produces: CLI `./scripts/leader_arm_gate.sh <left|right> <off|on>` exit 0 on success

- [ ] **Step 1: Write script**

```bash
#!/usr/bin/env bash
# Usage: leader_arm_gate.sh <left|right> <off|on>
# Runs inside existing Marvin container via docker exec (no new container).
set -euo pipefail
SIDE="${1:?left|right}"
ACTION="${2:?off|on}"
CONTAINER="${MARVIN_CONTAINER_NAME:-skye_marvin_m6}"
case "$SIDE" in left|right) ;; *) echo "bad side"; exit 2 ;; esac
case "$ACTION" in off|on) ;; *) echo "bad action"; exit 2 ;; esac

docker exec "$CONTAINER" bash -lc '
set -euo pipefail
source /marvin_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/marvin_ws/fastrtps_no_shm.xml
export ROS_LOCALHOST_ONLY=0
SIDE='"$SIDE"'
ACTION='"$ACTION"'
NODE="factr_teleop_${SIDE}"
CFG="/marvin_ws/configs/${ROBOT_PROFILE:-thor}/grav_comp_m6_${SIDE}.yaml"
if [[ "$ACTION" == "off" ]]; then
  pkill -f "$NODE" || true
  sleep 0.2
  python3 /marvin_ws/../scripts/disable_leader_dynamixel.py --side "$SIDE" \
    || python3 /scripts/disable_leader_dynamixel.py --side "$SIDE"
  exit 0
fi
# on: start FACTR if missing
if ros2 node list 2>/dev/null | grep -qx "/${NODE}"; then
  exit 0
fi
nohup ros2 run factr_teleop factr_teleop_robot_driver.py --ros-args \
  -r __node:='"$NODE"' \
  -p config_file:="${CFG}" \
  -p debug_state_print:=true -p print_joint_states:=true \
  -p print_period:=0.5 -p cb_print_period:=1.0 \
  -r /joint_control:=/gento/'"${SIDE}"'_joint_control \
  -r /joint_state:=/gento/'"${SIDE}"'_joint_states \
  -r /joint_move:=/'"${SIDE}"'_joint_move \
  -r /leader_arm/current_state:=/'"${SIDE}"'_leader_arm/current_state \
  -r /leader_arm/target_joint_state:=/'"${SIDE}"'_leader_arm/target_joint_state \
  -r /gripper/ctrl:=/'"${SIDE}"'_teleop_gripper/ctrl \
  -r /gripper/state:=/'"${SIDE}"'_gripper/state \
  >/tmp/'"${NODE}"'.log 2>&1 &
sleep 0.5
'
```

**Path note:** Marvin mounts repo `marvin_ws` at `/marvin_ws`. Host script lives at repo `scripts/`. Prefer mounting or copying: implementer must verify bind mounts in `run_marvin_m6_impedance.sh`. If host `scripts/` is not mounted, call disable via:

```bash
docker exec -e MARVIN_WS=/marvin_ws "$CONTAINER" \
  python3 -c '...' 
```

or mount `-v "$REPO/scripts:/scripts:ro"` if already present. **Check `run_marvin_m6_impedance.sh` mounts**; if `scripts` is not mounted, add `-v "${REPO_ROOT}/scripts:/scripts:ro"` in a minimal follow-up change in this task (only if missing).

Recommended off path when `/scripts` is mounted:

```bash
python3 /scripts/disable_leader_dynamixel.py --side "$SIDE"
```

- [ ] **Step 2: `bash -n scripts/leader_arm_gate.sh` — no syntax error**

- [ ] **Step 3: Commit**

```bash
git add scripts/leader_arm_gate.sh scripts/run_marvin_m6_impedance.sh  # if mount added
git commit -m "feat(scripts): add leader_arm_gate.sh for per-side FACTR off/on"
```

---

### Task 3: Pure `LeaderArmGate` state

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/leader_arms.py`
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_leader_arms.py`

**Interfaces:**
- Produces:

```python
class LeaderArmGate:
    def __init__(self) -> None: ...
    def snapshot(self) -> dict:  # left_enabled, right_enabled, locked
    def reset(self) -> None: ...
    def set_enabled(self, side: str, enabled: bool) -> bool:  # False if locked
    def note_sync_dispatched(self) -> None: ...
    def note_teleop_state(self, state: str | None) -> None: ...
    def any_enabled(self) -> bool: ...
    def align_payload(self) -> str | None:  # None if none enabled
```

- [ ] **Step 1: Failing tests**

```python
from skye_operator_ui.leader_arms import LeaderArmGate

def test_default_both_enabled_unlocked():
    g = LeaderArmGate()
    assert g.snapshot() == {
        "left_enabled": True, "right_enabled": True, "locked": False
    }

def test_disable_left_before_lock():
    g = LeaderArmGate()
    assert g.set_enabled("left", False)
    assert g.snapshot()["left_enabled"] is False
    assert g.align_payload() == "align_follower_right"

def test_lock_on_sync_blocks_toggle():
    g = LeaderArmGate()
    g.note_sync_dispatched()
    assert g.snapshot()["locked"] is True
    assert g.set_enabled("left", False) is False

def test_lock_on_teleop_state():
    g = LeaderArmGate()
    g.note_teleop_state("TELEOP_SYNCING")
    assert g.snapshot()["locked"] is True

def test_both_off_align_none():
    g = LeaderArmGate()
    g.set_enabled("left", False)
    g.set_enabled("right", False)
    assert g.any_enabled() is False
    assert g.align_payload() is None

def test_reset_clears_lock():
    g = LeaderArmGate()
    g.set_enabled("left", False)
    g.note_sync_dispatched()
    g.reset()
    assert g.snapshot() == {
        "left_enabled": True, "right_enabled": True, "locked": False
    }
```

- [ ] **Step 2: Run — FAIL**

```bash
cd skye_ros2_ws && python3 -m pytest src/skye_operator_ui/test/test_leader_arms.py -v
```

- [ ] **Step 3: Implement `leader_arms.py`**

```python
_LOCK_STATES = frozenset({"TELEOP_SYNCING", "SYNCED", "TELEOP"})

class LeaderArmGate:
    def __init__(self) -> None:
        self._left = True
        self._right = True
        self._locked = False

    def snapshot(self) -> dict:
        return {
            "left_enabled": self._left,
            "right_enabled": self._right,
            "locked": self._locked,
        }

    def reset(self) -> None:
        self._left = True
        self._right = True
        self._locked = False

    def set_enabled(self, side: str, enabled: bool) -> bool:
        if self._locked:
            return False
        if side == "left":
            self._left = bool(enabled)
            return True
        if side == "right":
            self._right = bool(enabled)
            return True
        raise ValueError(side)

    def note_sync_dispatched(self) -> None:
        self._locked = True

    def note_teleop_state(self, state: str | None) -> None:
        if state in _LOCK_STATES:
            self._locked = True

    def any_enabled(self) -> bool:
        return self._left or self._right

    def align_payload(self) -> str | None:
        if self._left and self._right:
            return "align_follower"
        if self._left:
            return "align_follower_left"
        if self._right:
            return "align_follower_right"
        return None
```

- [ ] **Step 4: Tests PASS → Commit**

```bash
git add skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/leader_arms.py \
  skye_ros2_ws/src/skye_operator_ui/test/test_leader_arms.py
git commit -m "feat(operator_ui): add LeaderArmGate pure state"
```

---

### Task 4: Command whitelist + `command_allowed` + `dispatch_plan`

**Files:**
- Modify: `skye_operator_ui/commands.py`
- Modify: `skye_operator_ui/ros_bridge.py` (`dispatch_plan`, `dispatch`)
- Test: `test/test_commands.py`, `test/test_snapshot.py` (dispatch_plan cases)

**Interfaces:**
- Consumes: `LeaderArmGate.snapshot()`, `features.leader_arm_gate`
- Produces: ops `leader_left_off|leader_left_on|leader_right_off|leader_right_on`
- `dispatch_plan(op, ui_mode, *, align_payload=None) -> tuple[str,str,str]|None`
  - `align_start` uses `align_payload` or default `align_follower`
  - leader ops → `("script", "left"|"right", "off"|"on")`

- [ ] **Step 1: Extend failing tests in `test_commands.py`**

```python
def test_switch_sync_blocked_when_both_leaders_off():
    session = SessionLogic()
    session.begin_start("thor", UiMode.teleop_record)
    session.precheck_ok(); session.mark_ready()
    allowed, reason = command_allowed(
        op="switch_sync", session=session, teleop_state="IDLE",
        hitl_mode=None, align_status=None,
        leader_arms={"left_enabled": False, "right_enabled": False, "locked": False},
        leader_arm_gate_enabled=True,
    )
    assert not allowed
    assert "两侧" in reason or "关闭" in reason

def test_leader_off_blocked_when_locked():
    session = SessionLogic()
    session.begin_start("thor", UiMode.teleop_record)
    session.precheck_ok(); session.mark_ready()
    allowed, _ = command_allowed(
        op="leader_left_off", session=session, teleop_state="SYNCED",
        hitl_mode=None, align_status=None,
        leader_arms={"left_enabled": True, "right_enabled": True, "locked": True},
        leader_arm_gate_enabled=True,
    )
    assert not allowed

def test_dispatch_plan_align_right():
    from skye_operator_ui.ros_bridge import dispatch_plan
    plan = dispatch_plan(
        "align_start", UiMode.teleop_record, align_payload="align_follower_right"
    )
    assert plan == ("string", "/mode/align_follower", "align_follower_right")

def test_dispatch_plan_leader_script():
    from skye_operator_ui.ros_bridge import dispatch_plan
    assert dispatch_plan("leader_left_off", UiMode.teleop_record) == (
        "script", "left", "off"
    )
```

- [ ] **Step 2: Run — FAIL**

- [ ] **Step 3: Implement gating + dispatch_plan**

`commands.py` additions:

```python
_LEADER_GATE_OPS = frozenset({
    "leader_left_off", "leader_left_on",
    "leader_right_off", "leader_right_on",
})
# add to ALLOWED_OPS and _TELEOP_OPS

def command_allowed(..., leader_arms=None, leader_arm_gate_enabled=True):
    ...
    if op in _LEADER_GATE_OPS:
        if not leader_arm_gate_enabled:
            return False, "单臂开关未启用"
        if mode != UiMode.teleop_record:
            return False, "当前为 DAgger 模式，无法开关小臂"
        arms = leader_arms or {}
        if arms.get("locked"):
            return False, "已进入同步/遥操，无法开关小臂"
        return True, ""
    if op == "switch_sync":
        arms = leader_arms or {}
        if leader_arm_gate_enabled and not (
            arms.get("left_enabled", True) or arms.get("right_enabled", True)
        ):
            return False, "两侧小臂已关闭，无法同步"
        # existing teleop_state checks...
```

`ros_bridge.py`:

```python
_LEADER_SCRIPT_OPS = {
    "leader_left_off": ("left", "off"),
    "leader_left_on": ("left", "on"),
    "leader_right_off": ("right", "off"),
    "leader_right_on": ("right", "on"),
}

def dispatch_plan(op, ui_mode, *, align_payload: str | None = None):
    if op == "align_start":
        return ("string", "/mode/align_follower", align_payload or "align_follower")
    # remove align_start from static _STRING_OPS
    if op in _LEADER_SCRIPT_OPS:
        side, action = _LEADER_SCRIPT_OPS[op]
        return ("script", side, action)
    ...
```

- [ ] **Step 4: Tests PASS → Commit**

```bash
git commit -m "feat(operator_ui): gate sync/align and leader arm ops"
```

---

### Task 5: Wire gate into bridge, API, snapshot, node

**Files:**
- Modify: `ros_bridge.py` (hold `LeaderArmGate`, run script, lock on sync, feed teleop_state)
- Modify: `snapshot.py` — add `leader_arms`, `features.leader_arm_gate`
- Modify: `api_app.py` — pass arms into `command_allowed`
- Modify: `operator_ui_node.py` — create gate; `on_before_stop` → `gate.reset()`
- Modify: `config/default.yaml` — `features.leader_arm_gate: true`
- Test: `test/test_api_app.py`, `test/test_snapshot.py`

**Interfaces:**
- Consumes: `repo_root`, `cfg["features"]`, `cfg["teardown"]["marvin_container_name"]`
- Script runner:

```python
def run_leader_arm_gate(repo_root: str, side: str, action: str, *,
                        container: str, timeout_s: float = 15.0) -> tuple[bool, str]:
    script = Path(repo_root) / "scripts" / "leader_arm_gate.sh"
    env = os.environ.copy()
    env["MARVIN_CONTAINER_NAME"] = container
    r = subprocess.run(
        [str(script), side, action], capture_output=True, text=True,
        timeout=timeout_s, env=env, check=False,
    )
    if r.returncode != 0:
        return False, (r.stderr or r.stdout or "leader_arm_gate failed").strip()
    return True, ""
```

`dispatch` for script kind:

```python
ok, msg = run_leader_arm_gate(...)
if not ok:
    return False, msg
enabled = action == "on"
if not self._leader_gate.set_enabled(side, enabled):
    return False, "已锁定，无法开关小臂"
return True, ""
```

On successful `switch_sync` string publish: `self._leader_gate.note_sync_dispatched()`.

In `_teleop_state_callback`: `self._leader_gate.note_teleop_state(msg.data)`.

`mailbox()` includes `"leader_arms": self._leader_gate.snapshot()`.

Snapshot:

```python
"leader_arms": mailbox.get("leader_arms") or {
    "left_enabled": True, "right_enabled": True, "locked": False
},
"features": {"leader_arm_gate": bool(features.get("leader_arm_gate", True))},
```

- [ ] **Step 1: Failing API/snapshot tests for `leader_arms` field and script mock**

- [ ] **Step 2: Implement wiring**

- [ ] **Step 3: `colcon test --packages-select skye_operator_ui` / pytest green**

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(operator_ui): wire LeaderArmGate into API and snapshot"
```

---

### Task 6: Single-side `follower_align`

**Files:**
- Modify: `skye_follower_align/align_logic.py` — e.g. `combine_active_phases(phases: list[AlignPhase])`
- Modify: `skye_follower_align/follower_align_node.py`
- Test: `skye_follower_align/test/test_align_logic.py` (+ node unit if present)

**Interfaces:**
- Consumes: String `data` in `{align_follower, align_follower_left, align_follower_right}`
- Produces: unchanged `/align/status` strings

- [ ] **Step 1: Failing test**

```python
def test_combine_active_phases_single_aligned():
    from skye_follower_align.align_logic import AlignPhase, combine_active_phases
    assert combine_active_phases([AlignPhase.ALIGNED]) == AlignPhase.ALIGNED

def test_combine_active_phases_ignores_idle_inactive():
    # only active sessions passed in
    assert combine_active_phases(
        [AlignPhase.ALIGNED, AlignPhase.ALIGNING]
    ) == AlignPhase.ALIGNING
```

- [ ] **Step 2: Implement node parsing**

```python
def _parse_align_sides(data: str) -> tuple[bool, bool] | None:
    if data == "align_follower":
        return True, True
    if data == "align_follower_left":
        return True, False
    if data == "align_follower_right":
        return False, True
    return None
```

On start: only start sessions for True sides; freshness only for those sides; publish abs only for those sides; combine only active session phases.

- [ ] **Step 3: Tests PASS → Commit**

```bash
git commit -m "feat(align): support align_follower_left/right single-arm align"
```

---

### Task 7: Frontend TeleopPanel + commands.js

**Files:**
- Modify: `skye_operator_ui/web/src/components/TeleopPanel.svelte`
- Modify: `skye_operator_ui/web/src/lib/commands.js`
- Rebuild: `cd skye_operator_ui/web && npm ci && npm run build` (then colcon install picks `web/dist`)

**Interfaces:**
- Consumes: `snapshot.leader_arms`, `snapshot.features.leader_arm_gate`
- Produces: toggle calls `leader_{side}_off|on` after `confirm()`

- [ ] **Step 1: Mirror gates in `commands.js`**

```javascript
if (op.startsWith('leader_')) {
  if (snapshot.features?.leader_arm_gate === false) {
    return { allowed: false, reason: '单臂开关未启用' };
  }
  if (snapshot.leader_arms?.locked) {
    return { allowed: false, reason: '已进入同步/遥操，无法开关小臂' };
  }
  // operational + teleop_record checks same as TELEOP_OPS
}
if (op === 'switch_sync') {
  const arms = snapshot.leader_arms;
  if (arms && !arms.left_enabled && !arms.right_enabled) {
    return { allowed: false, reason: '两侧小臂已关闭，无法同步' };
  }
  // existing...
}
```

- [ ] **Step 2: UI under 同步**

```svelte
{#if snapshot?.features?.leader_arm_gate !== false}
  <div class="leader-gate-row">
    <button type="button" class="btn" class:btn-amber={!leftEnabled}
      disabled={!gateLeft.allowed || !!snapshot?.leader_arms?.locked || busyOp}
      title={gateLeft.reason || '关左臂'}
      on:click={() => toggleSide('left')}>关左臂</button>
    <button ...>关右臂</button>
  </div>
  <p class="mono">小臂：左{leftEnabled ? '开' : '关'} · 右{rightEnabled ? '开' : '关'}</p>
{/if}
```

`toggleSide(side)`:

```javascript
const enabled = snapshot.leader_arms?.[`${side}_enabled`] !== false;
if (!enabled) {
  await postCommand(`leader_${side}_on`);
  return;
}
if (!confirm('请托住该侧小臂，去使能后会下落。确认关闭？')) return;
await postCommand(`leader_${side}_off`);
```

- [ ] **Step 3: Build web assets**

```bash
cd skye_ros2_ws/src/skye_operator_ui/web && npm ci && npm run build
cd skye_ros2_ws && ./scripts/build.sh   # or colcon build --packages-select skye_operator_ui
```

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(operator_ui): add 关左臂/关右臂 toggles under sync"
```

---

### Task 8: Docs + Operator UI smoke note

**Files:**
- Modify: `docs/Operator_UI使用说明.md`
- Modify: `docs/Thor_Orin_遥操启动.md` (one short note)
- Optional: tick list under 真机状态 for single-arm path

- [ ] **Step 1: Document buttons, hold warning, lock-after-sync, `features.leader_arm_gate`**

- [ ] **Step 2: Commit**

```bash
git commit -m "docs: document leader arm gate UI for Thor/Orin"
```

---

## Spec coverage checklist

| Spec section | Task |
|--------------|------|
| §2 flow + lock after sync | 3, 4, 5, 7 |
| §4 UI buttons + confirm | 7 |
| §5 API ops + snapshot + feature flag | 4, 5 |
| §6 gate script + `--side` | 1, 2 |
| §7 single-side align | 6 |
| §8 efficiency (exec existing container) | 2, 5 |
| §9 restore defaults | 5 (`features.leader_arm_gate`) |
| §10 tests | 1, 3, 4, 5, 6 |
| §11 docs | 8 |

## Self-review notes

- No TBD left in tasks; script mount path called out for implementer verification in Task 2.
- `align_start` removed from static `_STRING_OPS` and handled in `dispatch_plan` with payload — keep all call sites updated.
- `command_allowed` signature gains kwargs with defaults so existing tests keep working if they omit `leader_arms`.
