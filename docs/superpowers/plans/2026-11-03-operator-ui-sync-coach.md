# Operator UI Sync Coach Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** During sync (teleop_record + DAgger), show a structured left/right “同步校正” panel of out-of-tolerance joints from existing `/leader_arm/*/sync_progress`, then hide it when sync ends—without spawning `dual_trial`.

**Architecture:** Pure parse + coach builder modules; `RosBridge` subscribes to latched `sync_progress` String topics and caches per side; `SnapshotBuilder` computes `sync_coach` with a hard phase gate (so TRANSIENT_LOCAL latch does not keep the panel after SYNCED); Svelte `SyncCoach` renders under sync controls in TeleopPanel and DaggerPanel.

**Tech Stack:** Python 3 / pytest (`skye_operator_ui`), ROS 2 rclpy String + QoS RELIABLE+TRANSIENT_LOCAL, Svelte 4 frontend in `web/`, existing WS snapshot.

**Spec:** `docs/superpowers/specs/2026-11-03-operator-ui-sync-coach-design.md`

## Global Constraints

- Data source only: `/leader_arm/left/sync_progress` and `/leader_arm/right/sync_progress` (no `dual_trial` process).
- Do not change `skye_factr_teleop` driver publish schema this round.
- Freshness threshold for progress stamps: **2.0 seconds** (`stamp_s` wall clock in JSON).
- **Hard phase gate:** `visible` requires syncing phase (`teleop_state == TELEOP_SYNCING` OR `hitl_mode == HANDOVER_SYNC`); never keep panel visible solely because of a latched last progress message after SYNCED/TELEOP/IDLE.
- UI density: structured cards; list **blockers only**; reuse Operator UI CSS variables.
- Scope: teleop_record + DAgger; single-arm gate shows disabled side as「已关闭」.
- TDD for Python modules; rebuild frontend (`npm run build`) before claiming UI done.
- Prefer small commits per task when the user has asked for commits; otherwise leave working tree ready to commit.

## File map

| File | Responsibility |
|---|---|
| Create `skye_operator_ui/sync_progress_parse.py` | Parse driver `text` → side fields |
| Create `skye_operator_ui/sync_coach.py` | `build_sync_coach(...)` visibility + merge |
| Modify `skye_operator_ui/ros_bridge.py` | Subscribe, cache, clear, mailbox raw sides |
| Modify `skye_operator_ui/snapshot.py` | Attach `sync_coach` via builder |
| Create `test/test_sync_progress_parse.py` | Parser unit tests |
| Create `test/test_sync_coach.py` | Visibility / disabled / stale |
| Modify `test/test_snapshot.py`, `test/fakes.py` | Snapshot shape / defaults |
| Create `web/src/components/SyncCoach.svelte` | Presentational panel |
| Modify `TeleopPanel.svelte`, `DaggerPanel.svelte`, `styles.css` | Mount + styles |
| Modify `docs/Operator_UI使用说明.md` | Operator note |

---

### Task 1: Parse sync_progress text

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/sync_progress_parse.py`
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_sync_progress_parse.py`

**Interfaces:**
- Consumes: driver multi-line `text` (and optional envelope `side`/`outcome`/`stamp_s`)
- Produces: `parse_sync_progress_text(text: str) -> dict` and `parse_sync_progress_message(payload: dict) -> dict` with keys: `mode`, `within`, `total`, `blockers` (list of `{axis, kind, value, limit}`), `settling` (`{current_s, need_s}` or None), `parse_ok`, plus envelope fields on the message helper

- [ ] **Step 1: Write the failing tests**

```python
# test/test_sync_progress_parse.py
from skye_operator_ui.sync_progress_parse import (
    parse_sync_progress_text,
    parse_sync_progress_message,
)

SAMPLE_RIGHT = """\
[right] HOLD_SETTLING：6/8 轴当前达标（相对最终同步目标）
[right] Waiting for SYNCED: HOLD_SETTLING (1.000 / 25.000 s)
[right] J1: |position error|=0.215827 rad > 0.160000 rad (actual=0.100000, target=0.315827)
[right] J2: |velocity|=0.191847 rad/s > 0.100000 rad/s
[right] Continuous settling: 0.000 / 0.250 s
"""

SAMPLE_OK = """\
[left] HOLD_SETTLING：8/8 轴当前达标（相对最终同步目标）
[left] Waiting for SYNCED: HOLD_SETTLING (0.000 / 25.000 s)
[left] All eight axes meet instantaneous tolerances in this sample; they must remain within tolerance continuously for 0.250 s.
[left] Continuous settling: 0.120 / 0.250 s
"""


def test_parse_blockers_pos_and_vel():
    parsed = parse_sync_progress_text(SAMPLE_RIGHT)
    assert parsed["parse_ok"] is True
    assert parsed["mode"] == "HOLD_SETTLING"
    assert parsed["within"] == 6
    assert parsed["total"] == 8
    assert parsed["blockers"] == [
        {"axis": "J1", "kind": "pos", "value": 0.215827, "limit": 0.16},
        {"axis": "J2", "kind": "vel", "value": 0.191847, "limit": 0.1},
    ]
    assert parsed["settling"] == {"current_s": 0.0, "need_s": 0.25}


def test_parse_all_within_no_blockers():
    parsed = parse_sync_progress_text(SAMPLE_OK)
    assert parsed["within"] == 8
    assert parsed["blockers"] == []
    assert parsed["settling"] == {"current_s": 0.12, "need_s": 0.25}


def test_parse_message_merges_envelope():
    parsed = parse_sync_progress_message(
        {"side": "right", "text": SAMPLE_RIGHT, "outcome": "active", "stamp_s": 10.5}
    )
    assert parsed["side"] == "right"
    assert parsed["outcome"] == "active"
    assert parsed["stamp_s"] == 10.5
    assert parsed["parse_ok"] is True


def test_parse_garbage_sets_parse_ok_false():
    parsed = parse_sync_progress_text("not a sync progress blob")
    assert parsed["parse_ok"] is False
    assert parsed["blockers"] == []
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
cd /data/coding/tianji/Skye_ROS_Bridge
python -m pytest skye_ros2_ws/src/skye_operator_ui/test/test_sync_progress_parse.py -v
```

Expected: FAIL (module not found / import error).

- [ ] **Step 3: Implement parser**

```python
# skye_operator_ui/sync_progress_parse.py
from __future__ import annotations

import re
from typing import Any

_WITHIN_RE = re.compile(r"(\d+)\s*/\s*(\d+)\s*轴当前达标")
_MODE_RE = re.compile(r"\b(TELEOP_SYNCING|HOLD_SETTLING|IDLE|SYNCED|TELEOP)\b")
_POS_RE = re.compile(
    r"(J[1-7]|gripper(?:\(ID8\))?):\s*\|position error\|=([0-9.]+)\s*rad\s*>\s*([0-9.]+)\s*rad"
)
_VEL_RE = re.compile(
    r"(J[1-7]|gripper(?:\(ID8\))?):\s*\|velocity\|=([0-9.]+)\s*rad/s\s*>\s*([0-9.]+)\s*rad/s"
)
_SETTLE_RE = re.compile(r"Continuous settling:\s*([0-9.]+)\s*/\s*([0-9.]+)\s*s")


def _norm_axis(name: str) -> str:
    if name.startswith("gripper"):
        return "gripper"
    return name


def parse_sync_progress_text(text: str) -> dict[str, Any]:
    text = text or ""
    within = total = None
    m = _WITHIN_RE.search(text)
    if m:
        within, total = int(m.group(1)), int(m.group(2))
    mode = None
    mm = _MODE_RE.search(text)
    if mm:
        mode = mm.group(1)
    blockers: list[dict[str, Any]] = []
    for m in _POS_RE.finditer(text):
        blockers.append(
            {
                "axis": _norm_axis(m.group(1)),
                "kind": "pos",
                "value": float(m.group(2)),
                "limit": float(m.group(3)),
            }
        )
    for m in _VEL_RE.finditer(text):
        blockers.append(
            {
                "axis": _norm_axis(m.group(1)),
                "kind": "vel",
                "value": float(m.group(2)),
                "limit": float(m.group(3)),
            }
        )
    settling = None
    sm = _SETTLE_RE.search(text)
    if sm:
        settling = {"current_s": float(sm.group(1)), "need_s": float(sm.group(2))}
    parse_ok = within is not None or bool(blockers) or settling is not None or mode is not None
    return {
        "mode": mode,
        "within": within,
        "total": total,
        "blockers": blockers,
        "settling": settling,
        "parse_ok": parse_ok,
    }


def parse_sync_progress_message(payload: dict[str, Any]) -> dict[str, Any]:
    text = str(payload.get("text") or "")
    parsed = parse_sync_progress_text(text)
    parsed["side"] = payload.get("side")
    parsed["outcome"] = payload.get("outcome")
    parsed["stamp_s"] = payload.get("stamp_s")
    parsed["raw_text"] = text
    return parsed
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python -m pytest skye_ros2_ws/src/skye_operator_ui/test/test_sync_progress_parse.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit** (only if user asked for commits)

```bash
git add skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/sync_progress_parse.py \
  skye_ros2_ws/src/skye_operator_ui/test/test_sync_progress_parse.py
git commit -m "$(cat <<'EOF'
feat(operator_ui): parse leader sync_progress text for Sync Coach

EOF
)"
```

---

### Task 2: build_sync_coach visibility helper

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/sync_coach.py`
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_sync_coach.py`

**Interfaces:**
- Consumes: `parse_sync_progress_message` output per side; `teleop_state`; `hitl_mode`; `leader_arms`; `now_s`
- Produces: `build_sync_coach(*, left, right, teleop_state, hitl_mode, leader_arms, now_s, fresh_s=2.0) -> dict` with `visible` and optional `left`/`right` side dicts including `stale`, `disabled`

- [ ] **Step 1: Write failing tests**

```python
# test/test_sync_coach.py
from skye_operator_ui.sync_coach import build_sync_coach

NOW = 1000.0


def _side(stamp=999.5, outcome="active", blockers=None, disabled=False):
    return {
        "side": "left",
        "mode": "HOLD_SETTLING",
        "within": 6,
        "total": 8,
        "blockers": blockers or [],
        "settling": {"current_s": 0.0, "need_s": 0.25},
        "outcome": outcome,
        "stamp_s": stamp,
        "raw_text": "x",
        "parse_ok": True,
    }


def test_hidden_when_synced_even_if_progress_fresh():
    coach = build_sync_coach(
        left=_side(),
        right=_side(),
        teleop_state="SYNCED",
        hitl_mode=None,
        leader_arms={"left_enabled": True, "right_enabled": True},
        now_s=NOW,
    )
    assert coach["visible"] is False


def test_visible_during_teleop_syncing():
    coach = build_sync_coach(
        left=_side(),
        right=None,
        teleop_state="TELEOP_SYNCING",
        hitl_mode=None,
        leader_arms={"left_enabled": True, "right_enabled": True},
        now_s=NOW,
    )
    assert coach["visible"] is True
    assert coach["left"]["stale"] is False


def test_visible_during_handover_sync():
    coach = build_sync_coach(
        left=_side(),
        right=_side(),
        teleop_state="SYNCED",
        hitl_mode="HANDOVER_SYNC",
        leader_arms={"left_enabled": True, "right_enabled": True},
        now_s=NOW,
    )
    assert coach["visible"] is True


def test_disabled_side_when_gate_off():
    coach = build_sync_coach(
        left=_side(),
        right=None,
        teleop_state="TELEOP_SYNCING",
        hitl_mode=None,
        leader_arms={"left_enabled": True, "right_enabled": False},
        now_s=NOW,
    )
    assert coach["right"]["disabled"] is True
    assert coach["left"]["disabled"] is False


def test_stale_flag_when_stamp_old():
    coach = build_sync_coach(
        left=_side(stamp=NOW - 5.0),
        right=None,
        teleop_state="TELEOP_SYNCING",
        hitl_mode=None,
        leader_arms={"left_enabled": True, "right_enabled": True},
        now_s=NOW,
    )
    assert coach["left"]["stale"] is True
```

- [ ] **Step 2: Run tests — expect FAIL**

```bash
python -m pytest skye_ros2_ws/src/skye_operator_ui/test/test_sync_coach.py -v
```

- [ ] **Step 3: Implement**

```python
# skye_operator_ui/sync_coach.py
from __future__ import annotations

from typing import Any

FRESH_S_DEFAULT = 2.0


def _phase_active(teleop_state: str | None, hitl_mode: str | None) -> bool:
    if teleop_state == "TELEOP_SYNCING":
        return True
    if hitl_mode == "HANDOVER_SYNC":
        return True
    return False


def _decorate_side(
    raw: dict[str, Any] | None,
    *,
    side: str,
    enabled: bool,
    now_s: float,
    fresh_s: float,
) -> dict[str, Any]:
    if not enabled:
        return {
            "side": side,
            "disabled": True,
            "stale": False,
            "mode": None,
            "within": None,
            "total": None,
            "blockers": [],
            "settling": None,
            "outcome": None,
            "stamp_s": None,
            "raw_text": "",
            "parse_ok": True,
        }
    if raw is None:
        return {
            "side": side,
            "disabled": False,
            "stale": True,
            "mode": None,
            "within": None,
            "total": None,
            "blockers": [],
            "settling": None,
            "outcome": None,
            "stamp_s": None,
            "raw_text": "",
            "parse_ok": True,
        }
    stamp = raw.get("stamp_s")
    stale = True
    if isinstance(stamp, (int, float)):
        stale = (now_s - float(stamp)) > fresh_s
    out = dict(raw)
    out["side"] = side
    out["disabled"] = False
    out["stale"] = stale
    return out


def build_sync_coach(
    *,
    left: dict[str, Any] | None,
    right: dict[str, Any] | None,
    teleop_state: str | None,
    hitl_mode: str | None,
    leader_arms: dict[str, Any] | None,
    now_s: float,
    fresh_s: float = FRESH_S_DEFAULT,
) -> dict[str, Any]:
    arms = leader_arms or {}
    left_en = arms.get("left_enabled", True) is not False
    right_en = arms.get("right_enabled", True) is not False
    left_view = _decorate_side(
        left, side="left", enabled=left_en, now_s=now_s, fresh_s=fresh_s
    )
    right_view = _decorate_side(
        right, side="right", enabled=right_en, now_s=now_s, fresh_s=fresh_s
    )
    visible = _phase_active(teleop_state, hitl_mode)
    return {"visible": visible, "left": left_view, "right": right_view}
```

- [ ] **Step 4: Run tests — expect PASS**

```bash
python -m pytest skye_ros2_ws/src/skye_operator_ui/test/test_sync_coach.py -v
```

- [ ] **Step 5: Commit** (if requested)

```bash
git add skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/sync_coach.py \
  skye_ros2_ws/src/skye_operator_ui/test/test_sync_coach.py
git commit -m "$(cat <<'EOF'
feat(operator_ui): build Sync Coach visibility from teleop phase

EOF
)"
```

---

### Task 3: RosBridge subscribe + mailbox + clear

**Files:**
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/ros_bridge.py`
- Modify: `skye_ros2_ws/src/skye_operator_ui/test/test_ros_bridge.py` (add focused unit tests on bare bridge helpers)
- Test: preferably call `_ingest_sync_progress` / `clear_session_cache` without live ROS

**Interfaces:**
- Consumes: `parse_sync_progress_message`, String msgs on `/leader_arm/{side}/sync_progress`
- Produces: mailbox keys `sync_progress_left`, `sync_progress_right` (parsed dicts or None); cleared in `clear_session_cache`

- [ ] **Step 1: Write failing tests on bare bridge**

Add to `test_ros_bridge.py` (extend `_bare_bridge` init with new attrs):

```python
def test_ingest_sync_progress_parses_and_stores():
    bridge = _bare_bridge()
    bridge._sync_progress = {"left": None, "right": None}
    import json
    payload = {
        "side": "right",
        "text": "[right] HOLD_SETTLING：6/8 轴当前达标\n[right] J1: |position error|=0.2 rad > 0.16 rad",
        "outcome": "active",
        "stamp_s": 42.0,
    }
    bridge._ingest_sync_progress(json.dumps(payload, ensure_ascii=False))
    assert bridge._sync_progress["right"]["within"] == 6
    assert bridge._sync_progress["right"]["side"] == "right"


def test_clear_session_cache_drops_sync_progress():
    bridge = _bare_bridge()
    bridge._sync_progress = {"left": {"side": "left"}, "right": {"side": "right"}}
    # call real clear after ensuring method clears these keys — implement next
    bridge.clear_session_cache()
    assert bridge._sync_progress == {"left": None, "right": None}
```

Update `_bare_bridge` to set `bridge._sync_progress = {"left": None, "right": None}` so existing tests that call `clear_session_cache` do not AttributeError once implemented.

- [ ] **Step 2: Run — expect FAIL**

```bash
python -m pytest skye_ros2_ws/src/skye_operator_ui/test/test_ros_bridge.py::test_ingest_sync_progress_parses_and_stores -v
```

- [ ] **Step 3: Implement in ros_bridge**

1. Import `json` and `parse_sync_progress_message`.
2. In `__init__` after other state: `self._sync_progress: dict[str, Any] = {"left": None, "right": None}`.
3. Subscribe both topics with QoS matching driver:

```python
progress_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
for side in ("left", "right"):
    node.create_subscription(
        String,
        f"/leader_arm/{side}/sync_progress",
        self._sync_progress_callback,
        progress_qos,
        callback_group=self._cb_group,
    )
```

4. Callbacks:

```python
def _sync_progress_callback(self, msg: String) -> None:
    self._ingest_sync_progress(msg.data)

def _ingest_sync_progress(self, data: str) -> None:
    try:
        payload = json.loads(data)
    except (TypeError, ValueError, json.JSONDecodeError):
        return
    if not isinstance(payload, dict):
        return
    parsed = parse_sync_progress_message(payload)
    side = parsed.get("side")
    if side not in ("left", "right"):
        return
    self._sync_progress[side] = parsed
```

5. `mailbox()` add:

```python
"sync_progress_left": self._sync_progress.get("left"),
"sync_progress_right": self._sync_progress.get("right"),
```

6. `clear_session_cache` add:

```python
self._sync_progress = {"left": None, "right": None}
```

- [ ] **Step 4: Run ros_bridge tests**

```bash
python -m pytest skye_ros2_ws/src/skye_operator_ui/test/test_ros_bridge.py -v
```

Expected: PASS (including new tests; fix `_bare_bridge` / any `clear_session_cache` callers).

- [ ] **Step 5: Commit** (if requested)

```bash
git add skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/ros_bridge.py \
  skye_ros2_ws/src/skye_operator_ui/test/test_ros_bridge.py
git commit -m "$(cat <<'EOF'
feat(operator_ui): subscribe leader sync_progress into mailbox

EOF
)"
```

---

### Task 4: Snapshot exposes sync_coach

**Files:**
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/snapshot.py`
- Modify: `skye_ros2_ws/src/skye_operator_ui/test/test_snapshot.py`
- Modify: `skye_ros2_ws/src/skye_operator_ui/test/fakes.py` (optional defaults)

**Interfaces:**
- Consumes: mailbox `sync_progress_*`, `teleop_state`, `hitl_mode`, `leader_arms`; `time.time()`
- Produces: snapshot key `sync_coach` from `build_sync_coach`

- [ ] **Step 1: Failing snapshot test**

```python
def test_snapshot_includes_sync_coach_when_syncing():
    from skye_operator_ui.snapshot import SnapshotBuilder
    from skye_operator_ui.session_state import SessionLogic, UiMode

    logic = SessionLogic()
    logic.begin_start("orin", UiMode.teleop_record)
    logic.precheck_ok()
    logic.mark_ready()

    class FakeSup:
        cfg = {}
        def snapshot_fields(self):
            return {"state": logic.state().name, "profile": "orin", "mode": "teleop_record", "step": None}

    class Bridge:
        def mailbox(self):
            return {
                "teleop_state": "TELEOP_SYNCING",
                "align_status": None,
                "hitl_mode": None,
                "hitl_source": None,
                "left_joints": [],
                "right_joints": [],
                "left_gripper": None,
                "right_gripper": None,
                "health": {},
                "recording_active": False,
                "leader_arms": {"left_enabled": True, "right_enabled": True, "locked": False},
                "sync_progress_left": {
                    "side": "left",
                    "mode": "HOLD_SETTLING",
                    "within": 8,
                    "total": 8,
                    "blockers": [],
                    "settling": {"current_s": 0.1, "need_s": 0.25},
                    "outcome": "active",
                    "stamp_s": __import__("time").time(),
                    "raw_text": "",
                    "parse_ok": True,
                },
                "sync_progress_right": None,
            }

    FakeSup.logic = logic
    snap = SnapshotBuilder().build(FakeSup(), Bridge(), pending_op=None)
    assert snap["sync_coach"]["visible"] is True
    assert snap["sync_coach"]["left"]["within"] == 8
```

Also assert existing FakeBridge snapshots get `sync_coach` with `visible is False` when state is SYNCED (extend `FakeBridge.mailbox` with `sync_progress_left/right: None` or rely on `.get`).

- [ ] **Step 2: Run — FAIL**

```bash
python -m pytest skye_ros2_ws/src/skye_operator_ui/test/test_snapshot.py::test_snapshot_includes_sync_coach_when_syncing -v
```

- [ ] **Step 3: Wire snapshot.py**

```python
import time
from skye_operator_ui.sync_coach import build_sync_coach
# inside build(), before return:
sync_coach = build_sync_coach(
    left=mailbox.get("sync_progress_left"),
    right=mailbox.get("sync_progress_right"),
    teleop_state=teleop_state,
    hitl_mode=hitl_mode,
    leader_arms=mailbox.get("leader_arms"),
    now_s=time.time(),
)
# add to returned dict:
"sync_coach": sync_coach,
```

- [ ] **Step 4: Run snapshot + related tests**

```bash
python -m pytest skye_ros2_ws/src/skye_operator_ui/test/test_snapshot.py skye_ros2_ws/src/skye_operator_ui/test/test_api_app.py -v
```

Expected: PASS.

- [ ] **Step 5: Commit** (if requested)

```bash
git add skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/snapshot.py \
  skye_ros2_ws/src/skye_operator_ui/test/test_snapshot.py \
  skye_ros2_ws/src/skye_operator_ui/test/fakes.py
git commit -m "$(cat <<'EOF'
feat(operator_ui): expose sync_coach on WS snapshot

EOF
)"
```

---

### Task 5: Frontend SyncCoach + panel mount + docs

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/web/src/components/SyncCoach.svelte`
- Modify: `skye_ros2_ws/src/skye_operator_ui/web/src/components/TeleopPanel.svelte`
- Modify: `skye_ros2_ws/src/skye_operator_ui/web/src/components/DaggerPanel.svelte`
- Modify: `skye_ros2_ws/src/skye_operator_ui/web/src/styles.css`
- Modify: `docs/Operator_UI使用说明.md`

**Interfaces:**
- Consumes: `snapshot.sync_coach` (`visible`, `left`, `right`)
- Produces: UI only

- [ ] **Step 1: Add SyncCoach.svelte**

```svelte
<script>
  export let coach = null;
  export let enterLabel = '开遥操';

  $: visible = !!coach?.visible;
  $: left = coach?.left;
  $: right = coach?.right;

  function sideTitle(side) {
    return side === 'left' ? 'LEFT' : 'RIGHT';
  }

  function formatBlocker(b) {
    const kind = b.kind === 'vel' ? '|vel|' : '|pos err|';
    const unit = b.kind === 'vel' ? 'rad/s' : 'rad';
    return `${b.axis} · ${kind} ${Number(b.value).toFixed(3)} > ${Number(b.limit).toFixed(3)} ${unit}`;
  }
</script>

{#if visible}
  <section class="sync-coach" aria-live="polite">
    <div class="sync-coach-head">
      <strong>同步校正</strong>
      <span class="sync-coach-pill mono">
        {left?.mode || right?.mode || 'SYNCING'}
      </span>
    </div>
    <div class="sync-coach-grid">
      {#each [['left', left], ['right', right]] as [name, side] (name)}
        <div
          class="sync-coach-side"
          class:sync-coach-side-warn={side && !side.disabled && (side.blockers?.length || side.stale)}
          class:sync-coach-side-ok={side && !side.disabled && !side.stale && !(side.blockers?.length)}
          class:sync-coach-side-off={side?.disabled}
        >
          <div class="sync-coach-side-head">
            <span>{sideTitle(name)}</span>
            {#if side?.disabled}
              <span class="mono">关</span>
            {:else if side?.within != null && side?.total != null}
              <span class="mono">{side.within}/{side.total}</span>
            {/if}
          </div>
          {#if side?.disabled}
            <p class="sync-coach-muted">已关闭</p>
          {:else if side?.stale}
            <p class="sync-coach-muted">反馈超时</p>
          {:else if !side?.parse_ok && side?.raw_text}
            <pre class="sync-coach-raw mono">{side.raw_text}</pre>
          {:else if side?.blockers?.length}
            <ul class="sync-coach-blockers mono">
              {#each side.blockers as b, i (i)}
                <li>{formatBlocker(b)}</li>
              {/each}
            </ul>
          {:else}
            <p class="sync-coach-ok">瞬时达标，保持中…</p>
          {/if}
          {#if side?.settling && !side?.disabled}
            <p class="sync-coach-muted mono">
              settling {Number(side.settling.current_s).toFixed(2)} / {Number(side.settling.need_s).toFixed(2)} s
            </p>
          {/if}
        </div>
      {/each}
    </div>
    <p class="sync-coach-hint">手动微调超限关节，左右均 SYNCED 后再{enterLabel}</p>
  </section>
{/if}
```

- [ ] **Step 2: Mount in TeleopPanel**

Import SyncCoach. After the `leader-gate-block` `{/if}` that follows `switch_sync` (still inside the `{#each BUTTONS}`), insert coach once—not once per button. Cleaner: **close the each for buttons that need coach after sync controls**, or place coach **after** the entire `action-grid` but visually “under sync”. Spec: under sync/gate, before other actions.

Preferred structure change in TeleopPanel:

1. Keep status grid.
2. Render sync button + gate in a dedicated block.
3. `<SyncCoach coach={snapshot?.sync_coach} enterLabel="开遥操" />`
4. Then remaining buttons grid (开启遥操 / 停止 / 录制…).

Minimal diff alternative: after `</div>` of `action-grid`, still OK visually if CSS `order` not used—prefer **insert SyncCoach immediately after the `{#each BUTTONS}` loop’s sync+gate section** by splitting BUTTONS rendering:

```svelte
import SyncCoach from './SyncCoach.svelte';
// after leader-gate / sync button cluster:
<SyncCoach coach={snapshot?.sync_coach} enterLabel="开遥操" />
```

If leaving single `{#each}`, add after entire `action-grid`:

```svelte
</div>
<SyncCoach coach={snapshot?.sync_coach} enterLabel="开遥操" />
```

Use the post-`action-grid` placement if splitting is too invasive; still matches “主栏同步控件下方”.

- [ ] **Step 3: Mount in DaggerPanel**

After the three-button `action-grid-dagger` `</div>`, before HITL 录制:

```svelte
import SyncCoach from './SyncCoach.svelte';
…
<SyncCoach coach={snapshot?.sync_coach} enterLabel="进入遥操" />
```

- [ ] **Step 4: CSS in styles.css**

```css
.sync-coach {
  margin: 0.75rem 0 1rem;
  padding: 0.75rem 0.85rem;
  border: 1px solid var(--accent);
  border-radius: 6px;
  background: var(--accent-tint);
}

.sync-coach-head {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 0.5rem;
  margin-bottom: 0.55rem;
}

.sync-coach-pill {
  font-size: 0.75rem;
  padding: 0.15rem 0.55rem;
  border-radius: 999px;
  background: var(--wait-tint);
  color: var(--wait);
}

.sync-coach-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 0.55rem;
}

.sync-coach-side {
  border: 1px solid var(--line);
  border-radius: 6px;
  padding: 0.55rem 0.65rem;
  background: var(--surface);
}

.sync-coach-side-warn {
  border-color: var(--wait);
  background: var(--wait-tint);
}

.sync-coach-side-ok {
  border-color: var(--accent);
}

.sync-coach-side-off {
  opacity: 0.65;
}

.sync-coach-side-head {
  display: flex;
  justify-content: space-between;
  font-weight: 600;
  margin-bottom: 0.35rem;
}

.sync-coach-blockers {
  margin: 0;
  padding-left: 1.1rem;
  font-size: 0.8rem;
  color: var(--wait);
}

.sync-coach-ok {
  margin: 0;
  font-size: 0.85rem;
  color: var(--accent);
}

.sync-coach-muted {
  margin: 0.25rem 0 0;
  font-size: 0.8rem;
  color: var(--ink-muted);
}

.sync-coach-raw {
  margin: 0;
  white-space: pre-wrap;
  font-size: 0.72rem;
  max-height: 8rem;
  overflow: auto;
}

.sync-coach-hint {
  margin: 0.55rem 0 0;
  font-size: 0.8rem;
  color: var(--ink-muted);
}

@media (max-width: 720px) {
  .sync-coach-grid {
    grid-template-columns: 1fr;
  }
}
```

- [ ] **Step 5: Docs**

In `docs/Operator_UI使用说明.md`, after the 同步前关单侧 section (or under 开启遥操 note), add:

```markdown
## 同步校正面板

点 **同步**（遥操数采）或 DAgger **同步** 进入小臂同步相位时，主栏同步控件下方会显示 **同步校正**：左右分栏列出当前超阈值关节（位置/速度）与 settling 进度，便于手动微调。两侧进入 `SYNCED`（或离开 DAgger `HANDOVER_SYNC`）后面板自动隐藏。

数据来自 teleop 驱动话题 `/leader_arm/{left,right}/sync_progress`，**不**再需要为盯误差单独开 `skye_factr_dual_trial`（该命令仍可用于离线验收录制）。
```

- [ ] **Step 6: Build frontend**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws/src/skye_operator_ui/web
npm ci
npm run build
```

Expected: build succeeds; `web/dist` updated.

- [ ] **Step 7: Regression pytest**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge
python -m pytest skye_ros2_ws/src/skye_operator_ui/test -v
```

Expected: PASS.

- [ ] **Step 8: Manual smoke** (operator)

1. Restart `./scripts/start_operator_ui.sh`, hard-refresh browser.
2. Orin/Thor 遥操数采 → 同步 → 面板出现超限轴 → 到位后消失 → 开启遥操。
3. Optional: DAgger takeover 同步同检；关右臂再同步见右侧「已关闭」。

- [ ] **Step 9: Commit** (if requested)

```bash
git add skye_ros2_ws/src/skye_operator_ui/web \
  docs/Operator_UI使用说明.md
git commit -m "$(cat <<'EOF'
feat(operator_ui): show Sync Coach panel during arm sync

EOF
)"
```

---

## Spec coverage self-check

| Spec item | Task |
|---|---|
| Subscribe sync_progress, no dual_trial | 3 |
| Parse text → blockers / within / settling | 1 |
| Hard hide after SYNCED; HANDOVER_SYNC | 2, 4 |
| Disabled side / stale 2s | 2 |
| Snapshot `sync_coach` | 4 |
| TeleopPanel + DaggerPanel placement, density 2 | 5 |
| Operator_UI docs | 5 |
| Parser + bridge/snapshot tests | 1–4 |
| No driver JSON change / no bars | honored (non-goals) |

**Latch note (spec §10):** progress QoS is TRANSIENT_LOCAL; Task 2 hard phase gate prevents sticky panel after SYNCED.

## Placeholder scan

No TBD/TODO left in task steps; commands and code are concrete.

## Type consistency

- Mailbox: `sync_progress_left` / `sync_progress_right`
- Snapshot: `sync_coach: { visible, left, right }`
- Side: `disabled`, `stale`, `blockers[]`, `settling`, `parse_ok`, `raw_text`, `mode`, `within`, `total`, `outcome`, `stamp_s`
