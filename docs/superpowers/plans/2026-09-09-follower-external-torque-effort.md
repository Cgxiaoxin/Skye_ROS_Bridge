# Follower External Torque → JointState.effort Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish SDK `ExternalTorEst` as `sensor_msgs/JointState.effort` on `/gento/{,left_,right_}joint_states`, and disable FACTR Pinocchio gravity re-subtraction so arm torque feedback can work.

**Architecture:** Extend `DriverCore::DualArmState` + `read_state()` to copy `m_ARM_FBK_Joint_ExternalTorEst` each GetRT cycle; extend `publish_state()` / `make_arm_joint_state` so 7-DoF and 14-DoF topics carry matching `effort`. FACTR yaml sets `enable_follower_gravity_comp: False` because effort is already external torque.

**Tech Stack:** ROS 2 Humble, C++17 (`skye_robot_driver`), gtest, FACTR yaml under `marvin_ws/configs/{orin,thor}/`.

**Spec:** `docs/superpowers/specs/2026-09-09-follower-external-torque-effort-design.md`

## Global Constraints

- `effort` source is **only** `ROBOT_RT.m_ARMS[*].m_ARM_OUT.m_ARM_FBK_Joint_ExternalTorEst` (not SensorTor, not SG Joint_Tor).
- No unit conversion; SDK `float` → `double` copy only; do **not** multiply by `joint_signs`.
- Same GetRT stamp for position, velocity, and effort; if GetRT fails, publish nothing for that frame.
- 14-DoF `effort` = left[7] + right[7]; side topics length 7.
- Four yaml files: `enable_follower_gravity_comp: False` with comment; keep `torque_feedback.enable: True`.
- Do not change gripper effort topics or FACTR closed-source binaries.
- Work on current `main` unless user says otherwise; commit per task.

## File Map

| Path | Role |
|------|------|
| `skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_core.hpp` | Add `left_effort` / `right_effort` to `DualArmState`; declare `copy_joint_floats` helper |
| `skye_ros2_ws/src/skye_robot_driver/src/driver_core.cpp` | Fill effort in `read_state()`; implement helper |
| `skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp` | `make_arm_joint_state` + `publish_state` write `effort` |
| `skye_ros2_ws/src/skye_robot_driver/test/test_driver_core.cpp` | Unit test for float→JointArray copy |
| `marvin_ws/configs/{orin,thor}/grav_comp_m6_{left,right}.yaml` | Disable follower gravity re-subtraction |
| `docs/ros_interfaces.md` | Document effort semantics |

---

### Task 1: DualArmState + read ExternalTorEst

**Files:**
- Modify: `skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_core.hpp`
- Modify: `skye_ros2_ws/src/skye_robot_driver/src/driver_core.cpp`
- Modify: `skye_ros2_ws/src/skye_robot_driver/test/test_driver_core.cpp`

**Interfaces:**
- Consumes: SDK `FX_L1_Fbk_GetRT()` → `ROBOT_RT` (existing)
- Produces: `DualArmState::{left_effort,right_effort}`; `DriverCore::copy_joint_floats(const float *src, JointArray *dst)`

- [ ] **Step 1: Write the failing test**

Add to `test/test_driver_core.cpp`:

```cpp
TEST(DriverCore, CopyJointFloatsCopiesSevenValues) {
  const float src[7] = {-1.9f, 0.4f, -0.2f, 0.4f, -0.14f, 0.11f, 1.08f};
  DriverCore::JointArray dst{};
  DriverCore::copy_joint_floats(src, &dst);
  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(dst[i], static_cast<double>(src[i]), 1e-6);
  }
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skye_robot_driver --cmake-args -DBUILD_TESTING=ON
# Expect compile error: copy_joint_floats not declared
```

- [ ] **Step 3: Extend DualArmState and declare helper**

In `driver_core.hpp`, update:

```cpp
  struct DualArmState {
    JointArray left_position{};
    JointArray right_position{};
    JointArray left_velocity{};
    JointArray right_velocity{};
    JointArray left_effort{};
    JointArray right_effort{};
  };
```

In `public:` static helpers section (near other static methods), add:

```cpp
  // Copy 7 SDK floats (e.g. ExternalTorEst) into JointArray. No unit/sign change.
  static void copy_joint_floats(const float *src, JointArray *dst);
```

- [ ] **Step 4: Implement helper + read_state effort fill**

In `driver_core.cpp`:

```cpp
void DriverCore::copy_joint_floats(const float *src, JointArray *dst) {
  for (std::size_t i = 0; i < dst->size(); ++i) {
    (*dst)[i] = static_cast<double>(src[i]);
  }
}
```

Inside `read_state()`, after filling position/velocity degrees loops, before return, add (keep existing pos/vel conversion):

```cpp
  DualArmState state;
  state.left_position = sdk_degrees_to_ros_radians(left_position_degrees);
  state.right_position = sdk_degrees_to_ros_radians(right_position_degrees);
  state.left_velocity = sdk_degrees_to_ros_radians(left_velocity_degrees);
  state.right_velocity = sdk_degrees_to_ros_radians(right_velocity_degrees);
  copy_joint_floats(
      feedback->m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_ExternalTorEst,
      &state.left_effort);
  copy_joint_floats(
      feedback->m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_ExternalTorEst,
      &state.right_effort);
  return state;
```

Do **not** convert effort through `sdk_degrees_to_ros_radians`.

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skye_robot_driver --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select skye_robot_driver --event-handlers console_direct+
colcon test-result --verbose
```

Expected: `CopyJointFloatsCopiesSevenValues` PASS; existing DriverCore tests still PASS.

- [ ] **Step 6: Commit**

```bash
git add \
  skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_core.hpp \
  skye_ros2_ws/src/skye_robot_driver/src/driver_core.cpp \
  skye_ros2_ws/src/skye_robot_driver/test/test_driver_core.cpp
git commit -m "$(cat <<'EOF'
feat(skye): read ExternalTorEst into DualArmState effort

Copy SDK joint external torque estimates each GetRT cycle so the ROS
node can publish JointState.effort for FACTR torque feedback.
EOF
)"
```

---

### Task 2: Publish effort on /gento joint_states topics

**Files:**
- Modify: `skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp`

**Interfaces:**
- Consumes: `DualArmState::{left_effort,right_effort}` from Task 1
- Produces: JointState messages with `effort` length matching `position` on `/joint_states`, `/left_joint_states`, `/right_joint_states` (launch-remapped to `/gento/...`)

- [ ] **Step 1: Extend `make_arm_joint_state`**

Replace the helper signature and body so effort is required for state publish, with default empty meaning “omit” only if needed for action_applied. Prefer always writing effort (zeros OK for action_applied):

```cpp
sensor_msgs::msg::JointState make_arm_joint_state(
    const rclcpp::Time &stamp,
    const std::array<std::string, kArmDof> &names,
    const DriverCore::JointArray &position,
    const DriverCore::JointArray &velocity,
    const DriverCore::JointArray &effort = DriverCore::JointArray{}) {
  sensor_msgs::msg::JointState message;
  message.header.stamp = stamp;
  message.name.assign(names.begin(), names.end());
  message.position.assign(position.begin(), position.end());
  message.velocity.assign(velocity.begin(), velocity.end());
  message.effort.assign(effort.begin(), effort.end());
  return message;
}
```

Existing `publish_joint_action_applied` call with 4 args still compiles (effort defaults to zeros). That is fine (applied topics are not FACTR torque source).

- [ ] **Step 2: Fill effort in `publish_state`**

Update `publish_state()`:

```cpp
void DriverNode::publish_state() {
  const auto state = core_.read_state();
  if (state) {
    const auto stamp = now();
    JointState message;
    message.header.stamp = stamp;
    message.name.assign(kJointNames.begin(), kJointNames.end());
    message.position.reserve(kJointNames.size());
    message.velocity.reserve(kJointNames.size());
    message.effort.reserve(kJointNames.size());
    message.position.insert(
        message.position.end(), state->left_position.begin(),
        state->left_position.end());
    message.position.insert(
        message.position.end(), state->right_position.begin(),
        state->right_position.end());
    message.velocity.insert(
        message.velocity.end(), state->left_velocity.begin(),
        state->left_velocity.end());
    message.velocity.insert(
        message.velocity.end(), state->right_velocity.begin(),
        state->right_velocity.end());
    message.effort.insert(
        message.effort.end(), state->left_effort.begin(),
        state->left_effort.end());
    message.effort.insert(
        message.effort.end(), state->right_effort.begin(),
        state->right_effort.end());
    state_publisher_->publish(message);
    left_state_publisher_->publish(make_arm_joint_state(
        stamp, kLeftJointNames, state->left_position, state->left_velocity,
        state->left_effort));
    right_state_publisher_->publish(make_arm_joint_state(
        stamp, kRightJointNames, state->right_position,
        state->right_velocity, state->right_effort));
  }

  std_msgs::msg::Int16MultiArray robot_state;
  robot_state.data = {
      static_cast<int16_t>(core_.current_state(DriverCore::Arm::kLeft)),
      static_cast<int16_t>(core_.current_state(DriverCore::Arm::kRight))};
  robot_state_publisher_->publish(robot_state);
}
```

- [ ] **Step 3: Build package**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skye_robot_driver
```

Expected: build succeeds with no warnings about unused effort.

- [ ] **Step 4: Commit**

```bash
git add skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp
git commit -m "$(cat <<'EOF'
feat(skye): publish ExternalTorEst on joint_states effort

Fill effort on 14-DoF and per-arm 7-DoF state topics so FACTR can
consume follower external torque for leader force feedback.
EOF
)"
```

---

### Task 3: FACTR yaml — disable follower gravity re-subtraction

**Files:**
- Modify: `marvin_ws/configs/orin/grav_comp_m6_left.yaml`
- Modify: `marvin_ws/configs/orin/grav_comp_m6_right.yaml`
- Modify: `marvin_ws/configs/thor/grav_comp_m6_left.yaml`
- Modify: `marvin_ws/configs/thor/grav_comp_m6_right.yaml`

**Interfaces:**
- Consumes: none (config only)
- Produces: `enable_follower_gravity_comp: False` for all four profiles

- [ ] **Step 1: Update each yaml**

Replace the follower-gravity block comment + flag. Example for left (right/thor same flag; keep each file’s `follower_joint_offset`):

```yaml
  # Follower external torque is published by skye as JointState.effort
  # (SDK ExternalTorEst). Do NOT subtract Pinocchio gravity again.
  enable_follower_gravity_comp: False
  follower_urdf_package: "marvin_m6_description"
  follower_urdf: "marvin_m6.urdf"
```

Ensure `controller.torque_feedback.enable` is `True` in all four files (Orin already True from operator; set Thor if still False).

- [ ] **Step 2: Verify all four**

```bash
rg -n 'enable_follower_gravity_comp|torque_feedback:' -A2 \
  marvin_ws/configs/orin/grav_comp_m6_left.yaml \
  marvin_ws/configs/orin/grav_comp_m6_right.yaml \
  marvin_ws/configs/thor/grav_comp_m6_left.yaml \
  marvin_ws/configs/thor/grav_comp_m6_right.yaml
```

Expected: each file shows `enable_follower_gravity_comp: False`; under `torque_feedback:` each has `enable: True`.

- [ ] **Step 3: Commit**

```bash
git add marvin_ws/configs/orin/grav_comp_m6_left.yaml \
  marvin_ws/configs/orin/grav_comp_m6_right.yaml \
  marvin_ws/configs/thor/grav_comp_m6_left.yaml \
  marvin_ws/configs/thor/grav_comp_m6_right.yaml
git commit -m "$(cat <<'EOF'
fix(marvin): disable FACTR follower gravity when effort is ExternalTorEst

Avoid double-subtracting Pinocchio gravity now that skye publishes
controller external torque estimates on JointState.effort.
EOF
)"
```

---

### Task 4: Document effort semantics + live acceptance checklist

**Files:**
- Modify: `docs/ros_interfaces.md`

**Interfaces:**
- Consumes: published topic contracts from Task 2–3
- Produces: operator-facing documentation

- [ ] **Step 1: Update topic table rows**

In `docs/ros_interfaces.md` Topic table, change the three joint_states rows to:

```markdown
| 发布 | `/gento/joint_states` | `sensor_msgs/JointState` | 14 轴 rad / rad·s⁻¹ / **effort=SDK ExternalTorEst (Nm，轴外力矩)**；HITL/录包 |
| 发布 | `/gento/left_joint_states` | `JointState` | 7 轴左大臂；FACTR sync + **torque_feedback effort** |
| 发布 | `/gento/right_joint_states` | `JointState` | 7 轴右大臂；FACTR sync + **torque_feedback effort** |
```

Add a short note under the table:

```markdown
> **臂力反馈：** `effort` 来自控制器 `ExternalTorEst`（非总力矩 `SensorTor`）。
> FACTR yaml 须 `enable_follower_gravity_comp: False`，否则会再减 Pinocchio 重力。
> 探针：`scripts/torque/probe_external_torque.py`（须先停 `skye_robot_driver`）。
```

- [ ] **Step 2: Commit docs**

```bash
git add docs/ros_interfaces.md
git commit -m "$(cat <<'EOF'
docs: document joint_states effort as SDK ExternalTorEst

Clarify FACTR torque-feedback contract and yaml gravity flag.
EOF
)"
```

- [ ] **Step 3: Live acceptance (hardware; operator)**

With rebuilt driver and restarted Docker (so yaml overlays refresh):

```bash
# Host
pkill -f skye_robot_driver || true
cd /data/coding/tianji/Skye_ROS_Bridge
ROBOT_PROFILE=orin ./scripts/start_skye_for_factr.sh
# other terminal:
export ROS_DOMAIN_ID=21
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/marvin_ws/fastrtps_no_shm.xml"
ros2 topic echo /gento/left_joint_states --once
ros2 topic echo /gento/right_joint_states --once
ros2 topic echo /gento/joint_states --once
```

Checklist:

1. Each side `effort` length == 7; 14-DoF `effort` length == 14.  
2. Idle `|effort|` roughly matches probe ExtTorEst (~±2).  
3. `effort[0:7]` / `[7:14]` match left/right topics.  
4. Docker: `ROBOT_PROFILE=orin ./scripts/run_marvin_m6_impedance.sh`, TELEOP (`2`), gently push big-arm tip → leader feels contact (gain tune out of scope).

No commit required for hardware notes unless logging results into `docs/log.md` (optional).

---

## Spec coverage (self-review)

| Spec requirement | Task |
|------------------|------|
| DualArmState + read ExternalTorEst | Task 1 |
| publish 7/14 effort same stamp | Task 2 |
| Four yaml `enable_follower_gravity_comp: False` | Task 3 |
| `torque_feedback.enable: True` | Task 3 Step 2 verify |
| `ros_interfaces.md` effort semantics | Task 4 |
| Live echo + TELEOP feel | Task 4 Step 3 |
| No SensorTor/SG/signs/gripper/FACTR binary | Global Constraints |

No TBD/placeholder steps. Helper name `copy_joint_floats` consistent across Task 1–2.
