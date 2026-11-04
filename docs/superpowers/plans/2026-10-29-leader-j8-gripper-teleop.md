# Leader J8 → Follower Gripper Teleop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** In `TELEOP` only, map logical leader J8 through existing barrier `limit_min/max` to a FACTR `[0,1]` gripper command and publish it on `/gripper/ctrl` (remapped to `/left|right_teleop_gripper/ctrl`).

**Architecture:** Add a pure mapping helper next to leader-state transforms in `control_core.py`. `ros_driver._publish_cycle` publishes gripper commands alongside `/joint_control` when `mode == TELEOP`. Launch remaps wire the node-local topic to the driver; Thor/Orin keep using existing grav_comp YAMLs (no new fields).

**Tech Stack:** ROS 2 Humble, `skye_factr_teleop` (Python), `sensor_msgs/JointState`, pytest, existing `command_qos`.

**Spec:** `docs/superpowers/specs/2026-10-29-leader-j8-gripper-teleop-design.md`

## Global Constraints

- Publish gripper **only** in `ControlMode.TELEOP` (not IDLE / sync / hold).
- Mapping: `norm = clip((q - limit_min) / (limit_max - limit_min), 0, 1)` on **logical** `gripper_position`.
- Semantics on the wire: FACTR `1=open`, `0=closed`; driver invert unchanged.
- No new YAML keys; no `/gripper/state` subscription; no force feedback.
- Non-finite `q`: skip that publish + warning; do not fault the arm.
- `limit_max <= limit_min` at publish time: raise so existing `_fault` stops control (config should already reject this at load).
- Test-only modes (gravity/limit/friction) must **not** create or use `/gripper/ctrl`.

## File Structure

| File | Role |
|------|------|
| `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/control_core.py` | Add `gripper_command_norm(q, limit_min, limit_max) -> float \| None` |
| `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/ros_driver.py` | Publisher + TELEOP publish path |
| `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/launch_support.py` | Remap `/gripper/ctrl` |
| `skye_ros2_ws/src/skye_factr_teleop/test/test_control_core.py` | Mapping unit tests |
| `skye_ros2_ws/src/skye_factr_teleop/test/test_ros_driver.py` | Interface + TELEOP publish / non-TELEOP silence |
| `skye_ros2_ws/src/skye_factr_teleop/test/test_launch.py` | Remap expectation |
| `docs/marvin_dev/双臂启动遥操.md` | One-line operator note |
| `docs/marvin_dev/左臂单臂启遥操.md` | One-line operator note |

---

### Task 1: Gripper norm mapping helper

**Files:**
- Modify: `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/control_core.py`
- Test: `skye_ros2_ws/src/skye_factr_teleop/test/test_control_core.py`

**Interfaces:**
- Produces: `gripper_command_norm(q: float, limit_min: float, limit_max: float) -> float | None`
  - Returns `None` when `q` is non-finite (caller skips publish).
  - Returns clamped `[0.0, 1.0]` when limits are valid and `q` is finite.
  - Raises `ValueError` when `limit_max <= limit_min` or limits are non-finite.

- [ ] **Step 1: Write the failing tests**

Append to `test_control_core.py`:

```python
import math
import pytest
from skye_factr_teleop.control_core import gripper_command_norm


def test_gripper_command_norm_endpoints_and_midpoint():
    assert gripper_command_norm(-0.0175, -0.0175, 0.52517) == pytest.approx(0.0)
    assert gripper_command_norm(0.52517, -0.0175, 0.52517) == pytest.approx(1.0)
    mid = (-0.0175 + 0.52517) / 2.0
    assert gripper_command_norm(mid, -0.0175, 0.52517) == pytest.approx(0.5)


def test_gripper_command_norm_clamps_outside_range():
    assert gripper_command_norm(-1.0, -0.0175, 0.52517) == 0.0
    assert gripper_command_norm(2.0, -0.0175, 0.52517) == 1.0


def test_gripper_command_norm_nonfinite_returns_none():
    assert gripper_command_norm(float("nan"), -0.0175, 0.52517) is None
    assert gripper_command_norm(float("inf"), -0.0175, 0.52517) is None


def test_gripper_command_norm_rejects_invalid_limits():
    with pytest.raises(ValueError, match="gripper limits"):
        gripper_command_norm(0.0, 0.5, 0.5)
    with pytest.raises(ValueError, match="gripper limits"):
        gripper_command_norm(0.0, 1.0, 0.0)
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
cd skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/skye_factr_teleop/test/test_control_core.py::test_gripper_command_norm_endpoints_and_midpoint \
  src/skye_factr_teleop/test/test_control_core.py::test_gripper_command_norm_clamps_outside_range \
  src/skye_factr_teleop/test/test_control_core.py::test_gripper_command_norm_nonfinite_returns_none \
  src/skye_factr_teleop/test/test_control_core.py::test_gripper_command_norm_rejects_invalid_limits -v
```

Expected: FAIL with `ImportError` / `cannot import name 'gripper_command_norm'`.

- [ ] **Step 3: Implement the helper**

In `control_core.py`, near `transform_leader_state` / `joint_limit_barrier`, add:

```python
def gripper_command_norm(q: float, limit_min: float, limit_max: float) -> float | None:
    """Map logical gripper angle to FACTR open-norm in [0, 1] (1=open).

    Returns None when ``q`` is non-finite so the publisher can skip one cycle.
    """
    lo = _scalar(limit_min, "gripper_lower")
    hi = _scalar(limit_max, "gripper_upper")
    if not (math.isfinite(lo) and math.isfinite(hi)) or hi <= lo:
        raise ValueError("invalid gripper limits for command mapping")
    value = float(q)
    if not math.isfinite(value):
        return None
    return float(np.clip((value - lo) / (hi - lo), 0.0, 1.0))
```

Ensure `math` is imported at module top if not already.

- [ ] **Step 4: Run tests to verify they pass**

Re-run the same pytest command. Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/control_core.py \
  skye_ros2_ws/src/skye_factr_teleop/test/test_control_core.py
git commit -m "$(cat <<'EOF'
feat(teleop): add logical J8 to FACTR gripper norm mapper

Pure helper clamps barrier limits to [0,1] and skips non-finite angles.
EOF
)"
```

---

### Task 2: Publish `/gripper/ctrl` in TELEOP

**Files:**
- Modify: `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/ros_driver.py`
- Modify: `skye_ros2_ws/src/skye_factr_teleop/test/test_ros_driver.py`

**Interfaces:**
- Consumes: `gripper_command_norm` from Task 1; `result.state.gripper_position`; `config.controller.limit_barrier_control.gripper.limit_min/max`
- Produces: publisher `_gripper_pub` on `/gripper/ctrl` with `command_qos` (normal teleop path only); TELEOP cycles publish `JointState` with `position=[norm]` length 1

- [ ] **Step 1: Extend make_node recorder and write failing tests**

In `test_ros_driver.py` `make_node` fixture, where recorders are installed for `"_state_pub", "_current_pub", "_target_pub", "_command_pub"`, also install `"_gripper_pub"` when present:

```python
for name in ("_state_pub", "_current_pub", "_target_pub", "_command_pub", "_gripper_pub"):
    if getattr(node, name, None) is not None:
        setattr(node, name, Recorder())
```

Update `test_interfaces_qos_and_disabled_startup` to require `/gripper/ctrl`:

```python
assert {"/teleop/state", "/leader_arm/current_state",
        "/leader_arm/target_joint_state", "/joint_control",
        "/gripper/ctrl"} <= set(pubs)
assert pubs["/gripper/ctrl"].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
assert pubs["/gripper/ctrl"].qos_profile.depth == 1
```

Add a focused test (reuse the sync→teleop pattern from `test_follower_sync_and_teleop_publishes_seven_axis_command` or equivalent existing helper name in the file):

```python
def test_teleop_publishes_gripper_ctrl_and_idle_does_not(make_node):
    rig = make_node()
    node = rig.node
    assert node._gripper_pub is not None
    node._on_follower(packet())
    node._on_sync(String(data="switch_sync"))
    rig.trajectories[0].finished = True
    rig.clock.now += 0.002
    node._tick()
    settle(rig)
    assert node.core.mode == ControlMode.SYNCED
    assert node._gripper_pub.messages == []

    node._on_teleop(String(data="switch_teleop"))
    rig.clock.now += 0.002
    node._tick()
    assert node.core.mode == ControlMode.TELEOP
    assert len(node._command_pub.messages) >= 1
    grip = node._gripper_pub.messages[-1]
    assert len(grip.position) == 1
    assert len(grip.velocity) == len(grip.effort) == 0
    assert 0.0 <= grip.position[0] <= 1.0

    # logical = (physical - offset) * sign  ⇒  physical = logical * sign + offset
    # (same convention as settle(): target * joint_signs when offsets are zero).
    gripper = node.config.controller.limit_barrier_control.gripper
    signs = node.core.joint_signs
    offsets = node._offsets
    physical = np.zeros(8)
    physical[-1] = gripper.limit_max * float(signs[-1]) + float(offsets[-1])
    # Keep arm near sync target so TELEOP stays healthy.
    if node.core._target is not None:
        physical[:7] = (
            np.asarray(node.core._target[:7], dtype=float) * signs[:7] + offsets[:7]
        )
    rig.hardware.inject_state(physical, np.zeros(8))
    rig.clock.now += 0.002
    node._on_follower(packet(stamp=rig.clock.now))
    node._tick()
    assert node._gripper_pub.messages[-1].position[0] == pytest.approx(1.0)

    before = len(node._gripper_pub.messages)
    node._on_stop(String(data="switch_stop"))
    rig.clock.now += 0.002
    node._tick()
    assert node.core.mode == ControlMode.IDLE
    assert len(node._gripper_pub.messages) == before
```

Note: after `STOP_MOTION` / teleop entry, `_target` may be cleared — if so, leave `physical[:7]` at zeros (fixture default) or reuse the last synced physical inject from `settle`. Adjust only if the first TELEOP tick already faults.

Also assert gravity-test path does not require `_gripper_pub` (leave `None` or absent); existing gravity tests should keep passing.

- [ ] **Step 2: Run the new/updated tests — expect FAIL**

```bash
cd skye_ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/skye_factr_teleop/test/test_ros_driver.py::test_interfaces_qos_and_disabled_startup \
  src/skye_factr_teleop/test/test_ros_driver.py::test_teleop_publishes_gripper_ctrl_and_idle_does_not -v
```

Expected: FAIL — `/gripper/ctrl` missing and/or `_gripper_pub` is None.

- [ ] **Step 3: Implement publisher + publish path**

1. Import `gripper_command_norm` from `.control_core` (extend existing import).

2. In `_create_ros_interfaces`, after `_command_pub` creation, only on the normal (non-test-only) path — i.e. when creating follower subscriptions in the final `else` branch, also:

```python
self._gripper_pub = self.create_publisher(JointState, "/gripper/ctrl", command_qos())
```

Initialize `self._gripper_pub = None` before the test-mode branches so gravity/limit/friction keep `None`.

3. In `_publish_cycle`, extend the TELEOP block:

```python
if result.mode == ControlMode.TELEOP:
    self._command_pub.publish(self._joint_message(result.state.arm_position))
    if self._gripper_pub is not None:
        gripper = self.config.controller.limit_barrier_control.gripper
        norm = gripper_command_norm(
            result.state.gripper_position, gripper.limit_min, gripper.limit_max,
        )
        if norm is None:
            self.get_logger().warning(
                "Skipping gripper command: non-finite logical gripper position",
            )
        else:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = [norm]
            self._gripper_pub.publish(msg)
```

Invalid limits raise → existing `_tick` `except` → `_fault` (matches spec).

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest src/skye_factr_teleop/test/test_ros_driver.py::test_interfaces_qos_and_disabled_startup \
  src/skye_factr_teleop/test/test_ros_driver.py::test_teleop_publishes_gripper_ctrl_and_idle_does_not \
  src/skye_factr_teleop/test/test_ros_driver.py::test_gravity_test_node_has_isolated_interface_and_stops \
  src/skye_factr_teleop/test/test_ros_driver.py::test_follower_sync_and_teleop_publishes_seven_axis_command -v
```

(Use the actual name of the existing sync/teleop test in the file if it differs.)

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/ros_driver.py \
  skye_ros2_ws/src/skye_factr_teleop/test/test_ros_driver.py
git commit -m "$(cat <<'EOF'
feat(teleop): publish FACTR gripper ctrl during TELEOP

Map leader J8 through barrier limits onto /gripper/ctrl with command QoS.
EOF
)"
```

---

### Task 3: Launch remaps + docs

**Files:**
- Modify: `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/launch_support.py`
- Modify: `skye_ros2_ws/src/skye_factr_teleop/test/test_launch.py`
- Modify: `docs/marvin_dev/双臂启动遥操.md`
- Modify: `docs/marvin_dev/左臂单臂启遥操.md`

**Interfaces:**
- Consumes: node-local `/gripper/ctrl` from Task 2
- Produces: remap `("/gripper/ctrl", f"/{side}_teleop_gripper/ctrl")` for each teleop node

- [ ] **Step 1: Update failing launch expectation**

In `test_launch.py` `test_launch_profiles_sides_and_remaps`, change expected remaps to:

```python
assert node.expanded_remapping_rules == [
    ("/joint_state", f"/gento/{side}_joint_states"),
    ("/joint_control", f"/gento/{side}_joint_control"),
    ("/joint_move", f"/{side}_joint_move"),
    ("/leader_arm/current_state", f"/{side}_leader_arm/current_state"),
    ("/leader_arm/target_joint_state", f"/{side}_leader_arm/target_joint_state"),
    ("/gripper/ctrl", f"/{side}_teleop_gripper/ctrl"),
]
```

- [ ] **Step 2: Run launch test — expect FAIL**

```bash
cd skye_ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest src/skye_factr_teleop/test/test_launch.py::test_launch_profiles_sides_and_remaps -v
```

Expected: FAIL on missing gripper remap.

- [ ] **Step 3: Add remap in `launch_support.py`**

In `_teleop_nodes` remappings list, append:

```python
("/gripper/ctrl", f"/{side}_teleop_gripper/ctrl"),
```

- [ ] **Step 4: Update operator docs**

In `docs/marvin_dev/双臂启动遥操.md`, after the TELEOP bullet / command block, add:

```markdown
TELEOP 后小臂 J8（扳机）会映射到大臂末端夹爪（`/left|right_teleop_gripper/ctrl`）。
若启动时报 FTDI `latency_timer ... expected 1`，先把对应 `ttyUSB*` 设为 1 再启小臂。
```

In `docs/marvin_dev/左臂单臂启遥操.md`, after switch_teleop steps, add the same J8→夹爪 one-liner (left topic only is fine to mention as `/left_teleop_gripper/ctrl`).

- [ ] **Step 5: Run launch tests + full teleop unit subset**

```bash
python3 -m pytest \
  src/skye_factr_teleop/test/test_launch.py::test_launch_profiles_sides_and_remaps \
  src/skye_factr_teleop/test/test_control_core.py::test_gripper_command_norm_endpoints_and_midpoint \
  src/skye_factr_teleop/test/test_ros_driver.py::test_teleop_publishes_gripper_ctrl_and_idle_does_not -v
```

Expected: PASS.

Rebuild package so install/launch pick up remaps:

```bash
cd skye_ros2_ws
./scripts/build.sh skye_factr_teleop
# or: colcon build --packages-select skye_factr_teleop --symlink-install
```

- [ ] **Step 6: Commit**

```bash
git add skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/launch_support.py \
  skye_ros2_ws/src/skye_factr_teleop/test/test_launch.py \
  docs/marvin_dev/双臂启动遥操.md \
  docs/marvin_dev/左臂单臂启遥操.md
git commit -m "$(cat <<'EOF'
feat(teleop): remap gripper ctrl and document J8 teleop path

Wire /gripper/ctrl to /{side}_teleop_gripper/ctrl for Thor/Orin launches.
EOF
)"
```

---

### Task 4: Hardware smoke checklist (manual)

**Files:** none (operator verification)

- [ ] **Step 1: Preflight**

```bash
# FTDI latency = 1 for both leader arms
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
source skye_ros2_ws/scripts/setup_factr_teleop_env.sh
```

- [ ] **Step 2: Start driver + dual teleop (Thor)**

Terminal A: existing `skye_robot_driver` with `robot_profile:=thor`.  
Terminal B: `start_teleop_m6_dual.launch.py robot_profile:=thor`.

Confirm driver log shows gripper enabled (dm4310).

- [ ] **Step 3: Sync → teleop, watch gripper topic**

```bash
ros2 topic echo /left_teleop_gripper/ctrl --qos-reliability best_effort
# separate terminal: right
ros2 topic echo /right_teleop_gripper/ctrl --qos-reliability best_effort
```

`switch_sync` → wait SYNCED → `switch_teleop`. Topics should stay quiet until TELEOP, then update when J8 moves; hardware grippers should open/close.

- [ ] **Step 4: Record result**

If direction is inverted on a side, note `joint_signs[7]` / `gripper_*_invert` for a follow-up (out of this plan’s code scope). Mark this task done in the PR description after a successful Thor run; Orin Robotiq is optional same-day if hardware available.

No commit required unless adding a short validation note under `skye_factr_teleop/validation_records/` (optional, operator discretion).

---

## Spec coverage self-review

| Spec requirement | Task |
|------------------|------|
| TELEOP-only publish | Task 2 |
| B1 limit_min/max linear map | Task 1–2 |
| `/gripper/ctrl` + remap | Task 2–3 |
| Thor/Orin no new yaml fields | (no config task; reuse existing) |
| Non-finite skip + warning | Task 1–2 |
| Invalid limits → fault | Task 1–2 |
| No force feedback / no state sub | Global Constraints |
| Operator docs | Task 3 |
| Hardware verify | Task 4 |

## Placeholder / consistency check

- Function name fixed: `gripper_command_norm` everywhere.
- Topic names fixed: `/gripper/ctrl` → `/{side}_teleop_gripper/ctrl`.
- No TBD/TODO left in steps.
