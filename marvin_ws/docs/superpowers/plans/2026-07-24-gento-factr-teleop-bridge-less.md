# Gento factr Teleop (Bridge-less) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire `keyboard_gripper` + factr built-in sync/teleop to `gento_robot_driver` without `skye_leader_bridge`, applying the original bridge mapping/safety parameters inside the driver.

**Architecture:** Pure mapping/rate-limit/hold-stop logic lives in `DriverCore` (unit-tested). The ROS node loads YAML parameters, applies mapping on each `/left|/right_joint_control` message, exposes `hold_current`/`stop_motion` services, and enforces `command_timeout_s`. The start script remaps factr `/joint_control` to `/gento/*_joint_control` and starts the existing keyboard node.

**Tech Stack:** ROS 2 Humble, C++17, `ament_cmake_gtest`, `std_srvs/srv/Trigger`, bash start script, installed `factr_teleop` / `keyboard_gripper.py`.

## Global Constraints

- Do not launch or depend on `skye_leader_bridge` for Gento teleop.
- Do not implement the safety supervisor / relative clutch / confirm Trigger from `gento-safe-bidirectional-teleop-design.md`.
- Keep all ROS joint values in radians; deg conversion stays only at the Gento SDK boundary.
- Reuse original bridge params: `signs [1,1,1,-1,1,-1,-1]`, bridge limits, `max_delta_per_cycle: 0.05`, `command_timeout_s: 0.20`.
- Invalid mapped targets are rejected (no silent clamp-to-limit then continue).
- Commits only when the user explicitly requests them; skip commit steps during execution unless asked.

## File Structure

| File | Responsibility |
|---|---|
| `gento_ros2_ws/src/gento_robot_driver/include/gento_robot_driver/driver_core.hpp` | Mapping, rate limit, hold/stop, timeout state |
| `gento_ros2_ws/src/gento_robot_driver/src/driver_core.cpp` | Implementations |
| `gento_ros2_ws/src/gento_robot_driver/src/gento_robot_driver_node.cpp` | ROS wiring + services + timeout timer |
| `gento_ros2_ws/src/gento_robot_driver/config/gento_robot.yaml` | Original bridge params |
| `gento_ros2_ws/src/gento_robot_driver/package.xml` | Add `std_srvs` |
| `gento_ros2_ws/src/gento_robot_driver/test/test_driver_core.cpp` | Pure logic tests |
| `marvin_ws/start_gento_dual_arm_sync.sh` | Keyboard + control remaps |
| `marvin_ws/docs/superpowers/specs/2026-07-24-gento-dual-arm-restart-sop.md` | Operator steps for 1/2/3 |

---

### Task 1: Command mapping and rate-limit pure logic

**Files:**
- Modify: `gento_ros2_ws/src/gento_robot_driver/include/gento_robot_driver/driver_core.hpp`
- Modify: `gento_ros2_ws/src/gento_robot_driver/src/driver_core.cpp`
- Test: `gento_ros2_ws/src/gento_robot_driver/test/test_driver_core.cpp`

**Interfaces:**
- Consumes: `JointArray` leader command (7 rad)
- Produces:
  - `static JointArray map_leader_command(const JointArray& leader, const JointArray& order_as_indices_unused, const JointArray& signs, const JointArray& offsets)` — actually use `std::array<int,7> joint_order` + signs + offsets
  - `static std::optional<JointArray> rate_limit(const JointArray& desired, const JointArray& previous, double max_delta)`
  - Prefer signatures:

```cpp
static JointArray apply_joint_mapping(
    const JointArray& leader,
    const std::array<int, 7>& joint_order,
    const JointArray& signs,
    const JointArray& offsets);

static JointArray limit_delta(
    const JointArray& desired,
    const JointArray& previous,
    double max_delta_per_cycle);
```

- [ ] **Step 1: Write the failing tests**

Append to `test/test_driver_core.cpp`:

```cpp
TEST(DriverCore, ApplyJointMappingUsesOrderSignsOffsets) {
  const DriverCore::JointArray leader{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  const std::array<int, 7> order{0, 1, 2, 3, 4, 5, 6};
  const DriverCore::JointArray signs{1, 1, 1, -1, 1, -1, -1};
  const DriverCore::JointArray offsets{0, 0, 0, 0, 0, 0, 0};

  const auto mapped = DriverCore::apply_joint_mapping(leader, order, signs, offsets);

  EXPECT_NEAR(mapped[0], 0.1, 1e-12);
  EXPECT_NEAR(mapped[3], -0.4, 1e-12);
  EXPECT_NEAR(mapped[5], -0.6, 1e-12);
  EXPECT_NEAR(mapped[6], -0.7, 1e-12);
}

TEST(DriverCore, LimitDeltaCapsPerJointStep) {
  const DriverCore::JointArray previous{0, 0, 0, 0, 0, 0, 0};
  const DriverCore::JointArray desired{0.2, -0.2, 0, 0, 0, 0, 0};

  const auto limited = DriverCore::limit_delta(desired, previous, 0.05);

  EXPECT_NEAR(limited[0], 0.05, 1e-12);
  EXPECT_NEAR(limited[1], -0.05, 1e-12);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cd /data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gento_robot_driver
```

Expected: compile failure (`apply_joint_mapping` / `limit_delta` undeclared) or link failure.

- [ ] **Step 3: Write minimal implementation**

In `driver_core.hpp` add the two static method declarations.

In `driver_core.cpp`:

```cpp
DriverCore::JointArray DriverCore::apply_joint_mapping(
    const JointArray& leader,
    const std::array<int, 7>& joint_order,
    const JointArray& signs,
    const JointArray& offsets) {
  JointArray mapped{};
  for (std::size_t out = 0; out < mapped.size(); ++out) {
    const int src = joint_order[out];
    mapped[out] = leader[static_cast<std::size_t>(src)] * signs[out] + offsets[out];
  }
  return mapped;
}

DriverCore::JointArray DriverCore::limit_delta(
    const JointArray& desired,
    const JointArray& previous,
    double max_delta_per_cycle) {
  JointArray limited{};
  for (std::size_t i = 0; i < desired.size(); ++i) {
    const double delta = desired[i] - previous[i];
    if (delta > max_delta_per_cycle) {
      limited[i] = previous[i] + max_delta_per_cycle;
    } else if (delta < -max_delta_per_cycle) {
      limited[i] = previous[i] - max_delta_per_cycle;
    } else {
      limited[i] = desired[i];
    }
  }
  return limited;
}
```

- [ ] **Step 4: Run tests and confirm pass**

Run:

```bash
cd /data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws
colcon test --packages-select gento_robot_driver --ctest-args -R test_driver_core --event-handlers console_direct+
colcon test-result --verbose
```

Expected: `test_driver_core` PASS including the two new tests.

- [ ] **Step 5: Commit** (only if user asked)

```bash
git add gento_ros2_ws/src/gento_robot_driver/include/gento_robot_driver/driver_core.hpp \
  gento_ros2_ws/src/gento_robot_driver/src/driver_core.cpp \
  gento_ros2_ws/src/gento_robot_driver/test/test_driver_core.cpp
git commit -m "$(cat <<'EOF'
feat(gento): add leader joint mapping and per-cycle delta limit

EOF
)"
```

---

### Task 2: hold_current / stop_motion core semantics

**Files:**
- Modify: `gento_ros2_ws/src/gento_robot_driver/include/gento_robot_driver/driver_core.hpp`
- Modify: `gento_ros2_ws/src/gento_robot_driver/src/driver_core.cpp`
- Test: `gento_ros2_ws/src/gento_robot_driver/test/test_driver_core.cpp`

**Interfaces:**
- Produces:
  - `bool hold_current()` — read RT feedback; for each arm `send_position` current joints; keep `position_ready_ == true`
  - `bool stop_motion()` — `FX_L1_Runtime_StopTraj`; switch both arms idle; set `position_ready_ = false`
  - `bool command_allowed() const` — already exists; remains false after `stop_motion` until `hold_current` or successful re-enable path restores it

- [ ] **Step 1: Write failing tests for allowed-state transitions without hardware**

Because SDK link requires hardware for real hold/stop, unit-test the gate with a testable flag path already used by `command_allowed`:

```cpp
TEST(DriverCore, StopMotionClearsCommandAllowedWithoutLink) {
  DriverCore core;
  EXPECT_FALSE(core.command_allowed());
  EXPECT_FALSE(core.stop_motion());  // not linked → false, still not allowed
  EXPECT_FALSE(core.command_allowed());
}
```

Add declarations so the test compiles; unlinked `stop_motion`/`hold_current` return false without crashing.

- [ ] **Step 2: Run to see missing symbols / fail**

```bash
colcon build --packages-select gento_robot_driver 2>&1 | tail -40
```

Expected: compile error until methods exist.

- [ ] **Step 3: Implement**

```cpp
bool DriverCore::hold_current() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  const ROBOT_RT* feedback = FX_L1_Fbk_GetRT();
  if (feedback == nullptr) {
    return false;
  }
  JointArray left_deg{};
  JointArray right_deg{};
  for (std::size_t i = 0; i < left_deg.size(); ++i) {
    left_deg[i] = feedback->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
    right_deg[i] = feedback->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Pos[i];
  }
  // Ensure position mode if recovering from stop_motion — if already idle, SwitchToPositionMode again.
  // Minimal v1: if !position_ready_, re-enter position mode with last known ratios stored as members.
  // For first cut: require position_ready_ already true OR call SwitchToPositionMode with stored ratios.
  ...
  position_ready_ = true;
  return FX_L1_Runtime_SetJointPosCmd(kThreadId, FX_OBJ_ARM0, left_deg.data()) == 0 &&
         FX_L1_Runtime_SetJointPosCmd(kThreadId, FX_OBJ_ARM1, right_deg.data()) == 0;
}

bool DriverCore::stop_motion() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_) {
    return false;
  }
  FX_L1_Runtime_StopTraj(kThreadId, FX_OBJ_ALL_FLAG);
  FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, kModeTimeoutMs);
  FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, kModeTimeoutMs);
  position_ready_ = false;
  return true;
}
```

Store `left_ratio_` / `right_ratio_` during `connect_and_enable` so `hold_current` can re-enter position mode after stop.

- [ ] **Step 4: Rebuild and run `test_driver_core`**

Expected: PASS.

- [ ] **Step 5: Commit** (only if user asked)

---

### Task 3: ROS node applies mapping, timeout, and Trigger services

**Files:**
- Modify: `gento_ros2_ws/src/gento_robot_driver/include/gento_robot_driver/gento_robot_driver_node.hpp`
- Modify: `gento_ros2_ws/src/gento_robot_driver/src/gento_robot_driver_node.cpp`
- Modify: `gento_ros2_ws/src/gento_robot_driver/package.xml` (add `std_srvs`)
- Modify: `gento_ros2_ws/src/gento_robot_driver/CMakeLists.txt` (`find_package(std_srvs)`)
- Modify: `gento_ros2_ws/src/gento_robot_driver/config/gento_robot.yaml`
- Test: `gento_ros2_ws/src/gento_robot_driver/test/test_node_interfaces.cpp`

**Interfaces:**
- Services: `/hold_current`, `/stop_motion` as `std_srvs/srv/Trigger` (start script may remap under `/gento/`)
- Parameters (defaults from original bridge):

```yaml
left_joint_order: [0, 1, 2, 3, 4, 5, 6]
right_joint_order: [0, 1, 2, 3, 4, 5, 6]
left_joint_signs: [1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0]
right_joint_signs: [1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0]
left_joint_offsets: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
right_joint_offsets: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
left_joint_limits_min: [-3.1067, -2.01, -3.1067, -1.0472, -3.1067, -1.0472, -1.5708]
left_joint_limits_max: [3.1067, 2.01, 3.1067, 2.53, 3.1067, 1.0472, 1.5708]
right_joint_limits_min: [-3.1067, -2.01, -3.1067, -1.0472, -3.1067, -1.0472, -1.5708]
right_joint_limits_max: [3.1067, 2.01, 3.1067, 2.53, 3.1067, 1.0472, 1.5708]
max_delta_per_cycle: 0.05
command_timeout_s: 0.20
```

- [ ] **Step 1: Extend `test_node_interfaces.cpp`**

With `connect_on_startup:=false`, assert services exist:

```cpp
EXPECT_EQ(node->count_services("/hold_current") + /* or get_service_names_and_types filter */, ...);
```

Practical check:

```cpp
auto names = node->get_service_names_and_types();
bool has_hold = false;
bool has_stop = false;
for (const auto& entry : names) {
  if (entry.first == "/hold_current" || entry.first.ends_with("/hold_current")) has_hold = true;
  if (entry.first == "/stop_motion" || entry.first.ends_with("/stop_motion")) has_stop = true;
}
EXPECT_TRUE(has_hold);
EXPECT_TRUE(has_stop);
```

- [ ] **Step 2: Build/test expecting failure**

- [ ] **Step 3: Implement node path**

In `handle_command`:

1. Copy 7 positions into `leader`
2. `mapped = apply_joint_mapping(...)`
3. `validate_target(mapped, min, max)` — reject if false
4. If no `last_command_[arm]`, set previous = mapped (first frame ok)
5. Else `mapped = limit_delta(mapped, last, max_delta_)`
6. `send_position`; on success update `last_command_` and `last_command_time_`
7. Timeout timer every 50 ms: if ever received a command and `(now - last_command_time_) > command_timeout_s`, call `core_.hold_current()`, clear streaming flag, log warn

Wire Trigger callbacks to `core_.hold_current()` / `core_.stop_motion()`.

Update `gento_robot.yaml` with the parameter block above.

- [ ] **Step 4: Rebuild and run all package tests**

```bash
colcon test --packages-select gento_robot_driver --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all PASS.

- [ ] **Step 5: Commit** (only if user asked)

---

### Task 4: Start script — keyboard + joint_control remap

**Files:**
- Modify: `marvin_ws/start_gento_dual_arm_sync.sh`
- Modify: `marvin_ws/docs/superpowers/specs/2026-07-24-gento-dual-arm-restart-sop.md`

**Interfaces:**
- Produces running graph:
  - `/keyboard_gripper` publishing `/mode/switch_*`
  - factr left/right publishing to `/gento/left_joint_control` and `/gento/right_joint_control`
  - factr still subscribed to `/gento/joint_states` via `/joint_state`

- [ ] **Step 1: Update `leader_command` remaps and start keyboard**

Replace the left/right run blocks so remaps include:

```bash
  -r /joint_control:=/gento/left_joint_control \
  -r /joint_state:=/gento/joint_states \
  -r /joint_move:=/left_joint_move \
```

and right analogously to `/gento/right_joint_control`.

After both factr nodes, start keyboard inside the same container:

```bash
ros2 run factr_teleop keyboard_gripper.py --ros-args -r __node:=keyboard_gripper &
kb_pid=$!
```

Update trap to kill left/right/keyboard PIDs.

Change step banner text from only enabling `enable_position_sync` to:

1. Optionally publish `enable_position_sync:=false` first for observability (or keep true only as debug note)
2. Print keyboard help: `1 sync / 2 teleop / 3 stop`
3. Keep existing sync enable as an explicit debug option behind a message that formal teleop uses keyboard `1` after arms are up

Recommended final step 4 text:

```bash
say "4/4 Starting keyboard teleop control (1=sync, 2=teleop, 3=stop)"
# Do not auto-enable position sync for formal teleop; operator presses 1.
ros2 topic pub --once /enable_position_sync std_msgs/msg/Bool "{data: false}"
```

If current operators still need auto-sync demo, gate it with env `GENTO_AUTO_SYNC=1`.

- [ ] **Step 2: Dry-run script syntax**

```bash
bash -n /data/coding/tianji/Skye-mutile-arm/marvin_ws/start_gento_dual_arm_sync.sh
```

Expected: exit 0.

- [ ] **Step 3: Update SOP**

In `2026-07-24-gento-dual-arm-restart-sop.md`, replace “4/4 Enabling position sync” with keyboard 1/2/3 flow; note `skye_leader_bridge` is not used; document `/gento/hold_current` and `/gento/stop_motion`.

- [ ] **Step 4: Commit** (only if user asked)

---

### Task 5: Mark deferred supervisor designs

**Files:**
- Modify: `marvin_ws/docs/superpowers/specs/2026-07-24-gento-safe-bidirectional-teleop-design.md` (top note)
- Modify: `marvin_ws/docs/superpowers/specs/2026-07-24-gento-teleop-safety-supervisor-design.md` (top note, if present on disk)

- [ ] **Step 1: Add status banner**

```markdown
> **Status (2026-07-24):** Deferred. Active Gento teleop path is bridge-less factr wiring:
> [2026-07-24-gento-factr-teleop-bridge-less-design.md](./2026-07-24-gento-factr-teleop-bridge-less-design.md)
```

- [ ] **Step 2: Commit** (only if user asked)

---

## Spec coverage checklist

| Requirement | Task |
|---|---|
| No skye_leader_bridge | Already disabled configs + Tasks 4–5 |
| Keep keyboard + factr state machine | Task 4 |
| Original signs/limits/delta/timeout | Tasks 1, 3 |
| hold_current / stop_motion | Tasks 2, 3 |
| Start script remaps joint_control → gento | Task 4 |
| SOP update | Task 4 |
| Supervisor deferred | Task 5 |

## Placeholder / consistency review

- Method names fixed: `apply_joint_mapping`, `limit_delta`, `hold_current`, `stop_motion`
- YAML keys match node declare_parameter names in Task 3
- No supervisor / clutch / confirm scope creep
