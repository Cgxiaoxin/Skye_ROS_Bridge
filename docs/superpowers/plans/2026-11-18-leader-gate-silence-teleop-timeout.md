# Leader Gate Silence + TELEOP Follower Timeout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Soft-gate a leader arm so it stops ticking/publishing (no `/teleop/state` pollution), and widen TELEOP-only follower freshness to 2.0s so single-arm teleop does not false-stop.

**Architecture:** Extend `RuntimeConfig` with `teleop_state_timeout` (default 2.0). In `SkyeFactrTeleopNode`, gate OFF early-returns `_tick` and skips `/teleop/state` publish; `_fresh_follower` uses `teleop_state_timeout` only while `ControlMode.TELEOP`, else `state_timeout`. Ingest validation stays on `state_timeout`.

**Tech Stack:** Python 3.10, pytest, rclpy, YAML config under `skye_factr_teleop`.

**Spec:** `docs/superpowers/specs/2026-11-18-leader-gate-silence-teleop-timeout-design.md`

## Global Constraints

- Do not split `/teleop/state` into left/right topics.
- Do not remove the TELEOP follower watchdog; only widen to 2.0s.
- Do not change `skye_robot_driver` callback groups.
- Gate OFF must keep the process alive (no `pkill`); gate ON restores tick + one `_publish_state`.
- `_on_follower` ingest still validates with `state_timeout` (not `teleop_state_timeout`).
- Thor yaml without `runtime` must still get `teleop_state_timeout=2.0` from code defaults.
- Existing dual-arm path must keep working; update any test that assumed TELEOP dies at `state_timeout`.

## File map

| File | Responsibility |
|------|----------------|
| `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/config.py` | `RuntimeConfig.teleop_state_timeout`; parse/validate |
| `skye_ros2_ws/src/skye_factr_teleop/config/orin/grav_comp_m6_left.yaml` | Explicit `teleop_state_timeout: 2.0` |
| `skye_ros2_ws/src/skye_factr_teleop/config/orin/grav_comp_m6_right.yaml` | Explicit `teleop_state_timeout: 2.0` |
| `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/ros_driver.py` | Gate silence; mode-aware `_fresh_follower`; richer expire log |
| `skye_ros2_ws/src/skye_factr_teleop/test/test_config.py` | Parse + reject invalid `teleop_state_timeout` |
| `skye_ros2_ws/src/skye_factr_teleop/test/test_ros_driver.py` | Gate silence + TELEOP 2s + SYNC still strict; fix stale-teleop regression |
| `docs/Operator_UI使用说明.md` | Document silence + 2s TELEOP timeout |

---

### Task 1: Config — `teleop_state_timeout`

**Files:**
- Modify: `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/config.py`
- Modify: `skye_ros2_ws/src/skye_factr_teleop/config/orin/grav_comp_m6_left.yaml`
- Modify: `skye_ros2_ws/src/skye_factr_teleop/config/orin/grav_comp_m6_right.yaml`
- Test: `skye_ros2_ws/src/skye_factr_teleop/test/test_config.py`

**Interfaces:**
- Produces: `RuntimeConfig(state_timeout, future_tolerance, allow_unnamed_joint_states, teleop_state_timeout=2.0)`
- Produces: `_runtime()` accepts key `teleop_state_timeout` (positive finite)

- [ ] **Step 1: Write failing config tests**

Add to `test_config.py`:

```python
def test_runtime_teleop_state_timeout_default_is_two(tmp_path):
    document = _calibrated_document()
    runtime = load_config(_write(tmp_path, document), require_runtime=True).runtime
    assert runtime.teleop_state_timeout == 2.0


def test_runtime_teleop_state_timeout_can_be_set(tmp_path):
    document = _calibrated_document()
    document["runtime"] = {
        "state_timeout": 0.5,
        "teleop_state_timeout": 2.0,
        "future_tolerance": 0.05,
    }
    runtime = load_config(_write(tmp_path, document), require_runtime=True).runtime
    assert runtime.state_timeout == 0.5
    assert runtime.teleop_state_timeout == 2.0


@pytest.mark.parametrize("value", [0.0, -1.0, float("nan")])
def test_runtime_teleop_state_timeout_must_be_positive(tmp_path, value):
    document = _calibrated_document()
    document["runtime"] = {"teleop_state_timeout": value}
    with pytest.raises(ConfigError, match="teleop_state_timeout"):
        load_config(_write(tmp_path, document), require_runtime=True)
```

Also extend `test_runtime_state_policy_remains_optional` assertions if it builds a full runtime object — after implementation it should still pass and expose `teleop_state_timeout == 2.0` when omitted.

- [ ] **Step 2: Run tests — expect FAIL**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
python3 -m pytest src/skye_factr_teleop/test/test_config.py::test_runtime_teleop_state_timeout_default_is_two \
  src/skye_factr_teleop/test/test_config.py::test_runtime_teleop_state_timeout_can_be_set \
  src/skye_factr_teleop/test/test_config.py::test_runtime_teleop_state_timeout_must_be_positive -v
```

Expected: `AttributeError` / unknown field / tests fail before implementation.

- [ ] **Step 3: Implement `RuntimeConfig.teleop_state_timeout`**

In `config.py`:

```python
@dataclass(frozen=True)
class RuntimeConfig:
    state_timeout: float = 0.25
    future_tolerance: float = 0.05
    allow_unnamed_joint_states: bool = False
    teleop_state_timeout: float = 2.0
```

Update `_runtime`:

```python
def _runtime(value: Any) -> RuntimeConfig:
    path = "runtime"
    data = _map(value, path)
    _only(
        data,
        {
            "state_timeout",
            "future_tolerance",
            "allow_unnamed_joint_states",
            "teleop_state_timeout",
        },
        path,
    )
    return RuntimeConfig(
        _positive(data.get("state_timeout", 0.25), f"{path}.state_timeout"),
        _positive(data.get("future_tolerance", 0.05), f"{path}.future_tolerance"),
        _bool(data.get("allow_unnamed_joint_states", False), f"{path}.allow_unnamed_joint_states"),
        _positive(data.get("teleop_state_timeout", 2.0), f"{path}.teleop_state_timeout"),
    )
```

Orin yaml (both left and right) — under existing `runtime:`:

```yaml
runtime:
  state_timeout: 0.50
  teleop_state_timeout: 2.0
  future_tolerance: 0.05
```

- [ ] **Step 4: Run config tests — expect PASS**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
python3 -m pytest src/skye_factr_teleop/test/test_config.py -v
```

Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/config.py \
  skye_ros2_ws/src/skye_factr_teleop/config/orin/grav_comp_m6_left.yaml \
  skye_ros2_ws/src/skye_factr_teleop/config/orin/grav_comp_m6_right.yaml \
  skye_ros2_ws/src/skye_factr_teleop/test/test_config.py
git commit -m "$(cat <<'EOF'
feat(factr_teleop): add teleop_state_timeout runtime (default 2s)

Allow TELEOP follower freshness to be configured separately from
SYNC/HOLD state_timeout; Orin yaml sets 2.0 explicitly.
EOF
)"
```

---

### Task 2: Driver — gate silence (no tick / no `/teleop/state`)

**Files:**
- Modify: `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/ros_driver.py`
- Test: `skye_ros2_ws/src/skye_factr_teleop/test/test_ros_driver.py`

**Interfaces:**
- Consumes: `_leader_gated_off` flag (existing)
- Produces: gated `_tick` no-op; gated `_publish_state` no-op; gate OFF log mentions silence; gate ON calls `_publish_state()`

- [ ] **Step 1: Write failing gate-silence test**

Add near `test_leader_gate_off_mutes_sync_and_keeps_process_alive`:

```python
@pytest.mark.parametrize("side", ["left", "right"])
def test_leader_gate_off_silences_tick_and_teleop_state(make_node, side):
    rig = make_node(side)
    node = rig.node
    node._on_follower(packet(side, stamp=rig.clock.now))
    # Clear startup IDLE publish recorded by fixture replacement timing:
    # after construction, fixture swaps pubs; call gate off from clean counters.
    node._current_pub = Recorder()
    node._state_pub = Recorder()

    node._on_leader_gate(String(data="off"))
    assert node._leader_gated_off
    assert node._state_pub.messages == []  # must not publish while gated

    before_current = len(node._current_pub.messages)
    for _ in range(5):
        rig.clock.now += 0.01
        node._tick()
    assert len(node._current_pub.messages) == before_current
    assert node._state_pub.messages == []
    assert not node._closed

    node._on_sync(String(data="switch_sync"))
    assert node.core.mode == ControlMode.IDLE

    node._on_leader_gate(String(data="on"))
    assert not node._leader_gated_off
    assert node._state_pub.messages[-1].data == "IDLE"

    rig.clock.now += 0.01
    node._on_follower(packet(side, stamp=rig.clock.now))
    node._tick()
    assert len(node._current_pub.messages) > before_current
```

- [ ] **Step 2: Run test — expect FAIL**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
python3 -m pytest src/skye_factr_teleop/test/test_ros_driver.py::test_leader_gate_off_silences_tick_and_teleop_state -v
```

Expected: FAIL because gated tick still publishes `current_state` and/or gate OFF still publishes state.

- [ ] **Step 3: Implement gate silence in `ros_driver.py`**

`_tick` — return before follower watchdog / step when gated:

```python
def _tick(self):
    if self._closed:
        return
    if self._leader_gated_off:
        return
    # ... existing body unchanged ...
```

`_publish_state` — no-op while gated:

```python
def _publish_state(self):
    if self._leader_gated_off:
        return
    # ... existing body unchanged ...
```

`_on_leader_gate` — OFF path: stop, set flag, log silence, **do not** call `_publish_state`. ON path: clear flag, log, then `_publish_state()`.

```python
def _on_leader_gate(self, message):
    if self._closed or message.data not in ("off", "on"):
        return
    try:
        if message.data == "off":
            if not self._leader_gated_off:
                self._follower_session_active = False
                self.core.stop()
                self._leader_gated_off = True
                self.get_logger().info(
                    f"[{self.side}] leader gate OFF：忽略 sync/teleop，"
                    f"力矩已关闭，控制发布已静默")
            return
        if self._leader_gated_off:
            self._leader_gated_off = False
            self.get_logger().info(
                f"[{self.side}] leader gate ON：可再次接收 sync/teleop")
        self._publish_state()
    except Exception as exc:
        self._fault(exc)
```

Keep existing ignore behavior for sync/teleop/joint_move when gated.

- [ ] **Step 4: Run gate tests — expect PASS**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
python3 -m pytest src/skye_factr_teleop/test/test_ros_driver.py::test_leader_gate_off_silences_tick_and_teleop_state \
  src/skye_factr_teleop/test/test_ros_driver.py::test_leader_gate_off_mutes_sync_and_keeps_process_alive -v
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/ros_driver.py \
  skye_ros2_ws/src/skye_factr_teleop/test/test_ros_driver.py
git commit -m "$(cat <<'EOF'
fix(factr_teleop): silence gated leader tick and /teleop/state

Stop control publishes while leader_gate is off so single-arm teleop
is not polluted by the muted side's IDLE/current_state stream.
EOF
)"
```

---

### Task 3: Driver — TELEOP follower limit 2s + expire log

**Files:**
- Modify: `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/ros_driver.py`
- Test: `skye_ros2_ws/src/skye_factr_teleop/test/test_ros_driver.py`

**Interfaces:**
- Consumes: `runtime.teleop_state_timeout`, `runtime.state_timeout`, `core.mode`
- Produces: `_follower_age_limit()`; mode-aware `_fresh_follower()`; expire warning includes `age` / `stamp_age` / `limit` / `mode`

- [ ] **Step 1: Write failing TELEOP / SYNC timeout tests; fix existing teleop stale case**

Replace the teleop branch behavior of `test_stale_follower_stops_all_follower_modes` so TELEOP uses >2.0s. Prefer splitting:

```python
@pytest.mark.parametrize("mode", ["sync", "hold"])
def test_stale_follower_stops_sync_and_hold(make_node, mode):
    rig = make_node(parameters={"state_timeout": 0.5, "teleop_state_timeout": 2.0})
    node = rig.node
    node._on_follower(packet(stamp=rig.clock.now))
    node._on_sync(String(data="switch_sync"))
    if mode == "hold":
        rig.trajectories[0].finished = True
        rig.clock.now += 0.002
        node._on_follower(packet(stamp=rig.clock.now))
        node._tick()
    rig.clock.now += 0.6  # > state_timeout 0.5
    node._tick()
    assert node.core.mode == ControlMode.IDLE
    assert not rig.hardware.torque_enabled


def test_teleop_allows_follower_age_between_state_and_teleop_timeout(make_node):
    rig = make_node(parameters={"state_timeout": 0.5, "teleop_state_timeout": 2.0})
    node = rig.node
    node._on_follower(packet(stamp=rig.clock.now))
    node._on_sync(String(data="switch_sync"))
    rig.trajectories[0].finished = True
    rig.clock.now += 0.002
    node._on_follower(packet(stamp=rig.clock.now))
    node._tick()
    settle(rig)
    node._on_teleop(String(data="switch_teleop"))
    assert node.core.mode == ControlMode.TELEOP

    # 1.0s > state_timeout but < teleop_state_timeout → stay TELEOP
    rig.clock.now += 1.0
    node._tick()
    assert node.core.mode == ControlMode.TELEOP

    # 2.1s from last follower sample → stop
    rig.clock.now += 1.2
    node._tick()
    assert node.core.mode == ControlMode.IDLE
    assert not rig.hardware.torque_enabled
```

If the old parametrized `test_stale_follower_stops_all_follower_modes` still exists with `mode=="teleop"` and `+= 0.3`, remove teleop from that parametrize or delete that case after adding the two tests above (default `teleop_state_timeout=2.0` would otherwise make `+=0.3` keep TELEOP and fail the old assert).

- [ ] **Step 2: Run new tests — expect FAIL**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
python3 -m pytest src/skye_factr_teleop/test/test_ros_driver.py::test_teleop_allows_follower_age_between_state_and_teleop_timeout \
  src/skye_factr_teleop/test/test_ros_driver.py::test_stale_follower_stops_sync_and_hold -v
```

Expected: FAIL — TELEOP still uses `state_timeout` and dies at 1.0s.

- [ ] **Step 3: Implement mode-aware freshness + log**

In `_configure` (after existing runtime load):

```python
self._max_age = runtime.state_timeout
self._teleop_max_age = runtime.teleop_state_timeout
self._future_tolerance = runtime.future_tolerance
```

Helpers:

```python
def _follower_age_limit(self):
    if self.core is not None and self.core.mode == ControlMode.TELEOP:
        return self._teleop_max_age
    return self._max_age

def _fresh_follower(self):
    sample = self._follower
    if sample is None:
        return False
    limit = self._follower_age_limit()
    age = self._monotonic() - sample.received_at
    stamp_age = self._ros_now() - sample.stamp
    return (0 <= age <= limit
            and -self._future_tolerance <= stamp_age <= limit)
```

Keep `_validate` / `_on_follower` on `max_age=self._max_age` (state_timeout only).

In `_tick` expire branch:

```python
if self._follower_session_active and not self._fresh_follower():
    sample = self._follower
    age = (None if sample is None
           else self._monotonic() - sample.received_at)
    stamp_age = (None if sample is None
                 else self._ros_now() - sample.stamp)
    limit = self._follower_age_limit()
    self.get_logger().warning(
        "Follower state expired; stopping "
        f"(age={age if age is not None else float('nan'):.3f}s "
        f"stamp_age={stamp_age if stamp_age is not None else float('nan'):.3f}s "
        f"limit={limit:.3f}s mode={self.core.mode.name})"
    )
    self._on_stop(None)
    return
```

(Use an equivalent safe format if `age is None` — e.g. print `age=None`.)

- [ ] **Step 4: Run driver timeout + gate tests — expect PASS**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
python3 -m pytest src/skye_factr_teleop/test/test_ros_driver.py::test_teleop_allows_follower_age_between_state_and_teleop_timeout \
  src/skye_factr_teleop/test/test_ros_driver.py::test_stale_follower_stops_sync_and_hold \
  src/skye_factr_teleop/test/test_ros_driver.py::test_leader_gate_off_silences_tick_and_teleop_state \
  src/skye_factr_teleop/test/test_ros_driver.py::test_leader_gate_off_mutes_sync_and_keeps_process_alive \
  src/skye_factr_teleop/test/test_ros_driver.py::test_stale_follower_stops_all_follower_modes -v
```

Expected: PASS (or last test removed/updated so it no longer expects TELEOP stop at 0.3s).

- [ ] **Step 5: Commit**

```bash
git add skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/ros_driver.py \
  skye_ros2_ws/src/skye_factr_teleop/test/test_ros_driver.py
git commit -m "$(cat <<'EOF'
fix(factr_teleop): use 2s follower freshness only in TELEOP

Keep SYNC/HOLD on state_timeout; log age/stamp_age/limit on expiry
to distinguish false stops from real follower loss.
EOF
)"
```

---

### Task 4: Docs + full package test

**Files:**
- Modify: `docs/Operator_UI使用说明.md`
- Verify: `skye_factr_teleop` pytest suite

**Interfaces:**
- Produces: operator-facing note under「同步前关单侧小臂」

- [ ] **Step 1: Update Operator UI doc**

In `docs/Operator_UI使用说明.md`, under「同步前关单侧小臂」, after the soft-gate paragraph, add:

```markdown
关侧软门控会**静默**该侧控制循环：不再发布 `/{side}_leader_arm/current_state`，也不再向共用 `/teleop/state` 发 `IDLE`，避免干扰开侧遥操状态。复能后恢复发布。

开侧进入 **TELEOP** 后，大臂反馈过期阈值使用 `runtime.teleop_state_timeout`（默认 / Orin 配置为 **2.0s**）；同步与 HOLD 仍用 `runtime.state_timeout`（Orin **0.50s**）。若日志出现 `Follower state expired ... limit=2.000s mode=TELEOP`，再查大臂 `/gento/<side>_joint_states` 是否真断流。
```

Optionally tick/add a note under 真机状态 for single-arm smoke after this fix.

- [ ] **Step 2: Run full teleop package tests**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
python3 -m pytest src/skye_factr_teleop/test/test_config.py src/skye_factr_teleop/test/test_ros_driver.py -v
```

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add docs/Operator_UI使用说明.md
git commit -m "$(cat <<'EOF'
docs(operator_ui): document gated-arm silence and TELEOP 2s timeout

EOF
)"
```

---

## Spec coverage check

| Spec section | Task |
|---|---|
| §2 / §5.1–5.2 gate silence + no `/teleop/state` | Task 2 |
| §4 `teleop_state_timeout` + Orin yaml | Task 1 |
| §5.3 mode-aware limit + age log | Task 3 |
| §5.4 ingest stays on `state_timeout` | Task 3 (explicit non-change) |
| §6 Operator UI docs | Task 4 |
| §7 tests (silence / teleop window / sync strict / config) | Tasks 1–3 |
| Non-goals (no topic split, no driver CG rewrite) | honored by omission |

## Manual smoke (after merge / on robot)

1. Orin Operator UI：关左 → 同步 → 遥操 ≥30s，开侧无 `Follower state expired`。
2. 关右重复一次。
3. 双臂不关臂：回归遥操正常。
4. 可选：`ros2 topic hz /left_leader_arm/current_state` 在关左后应变 0（无新帧）。
