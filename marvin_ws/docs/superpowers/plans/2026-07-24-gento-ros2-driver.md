# Gento ROS2 Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an independent C++ ROS 2 Humble driver that uses the Gento C++ SDK to publish dual-arm state and receive 7-axis position commands.

**Architecture:** A small, testable `DriverCore` owns the Gento SDK state machine and exposes pure command validation/mapping. `GentoRobotDriverNode` is the ROS boundary: it subscribes to existing left/right `sensor_msgs/msg/JointState` control topics, publishes the combined 14-axis state, and delegates SDK calls to `DriverCore`. The package links `libGentoSDK.so` directly and never loads, copies, or renames `libMarvinSDK.so`.

**Tech Stack:** C++17, ROS 2 Humble (`rclcpp`, `sensor_msgs`), ament_cmake, GTest, Gento C++ SDK at `/data/coding/tianji/tianji-robot-SDK-Gento_Skye-Luna/C_SDK`.

## Global Constraints

- Create the independent workspace at `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws`; run all `colcon` commands from that directory.
- Link `/data/coding/tianji/tianji-robot-SDK-Gento_Skye-Luna/C_SDK/libGentoSDK.so`; do not overwrite or load `libMarvinSDK.so`.
- Use `FX_OBJ_ARM0` for left and `FX_OBJ_ARM1` for right; use SDK thread ID `1` for all first-version calls.
- Enter position mode with `FX_L1_State_SwitchToPositionMode(obj, 3000, vel_ratio, acc_ratio)` before accepting commands.
- Default right-arm velocity and acceleration ratios are `10`; valid configured ratios are `1..100`.
- Reject commands unless they contain exactly seven finite `position` values and all values lie in configured per-joint limits.
- Publish `/joint_states` with names `l_j1..l_j7,r_j1..r_j7`; source positions and velocities from `ROBOT_RT::m_ARMS[0/1].m_ARM_OUT`.
- On shutdown or SDK error, call `FX_L1_Runtime_StopTraj(1, FX_OBJ_ALL_FLAG)`, switch both arms to idle, and call `FX_L1_System_Unlink()`.
- This workspace is not currently a Git repository; record validation commands in the implementation handoff instead of making commits.

---

### Task 1: Create a standalone ROS 2 package and prove the SDK ABI is usable

**Files:**
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/package.xml`
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/CMakeLists.txt`
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/test/test_sdk_abi.cpp`

**Interfaces:**
- Consumes: `L1Robot.h`, `FXCommon.h`, and `libGentoSDK.so` from the external SDK directory.
- Produces: the `gento_robot_driver` CMake target and `test_sdk_abi` CTest target.

- [ ] **Step 1: Write the failing ABI test**

Create `test/test_sdk_abi.cpp`:

```cpp
#include <gtest/gtest.h>
extern "C" {
#include "L1Robot.h"
}

TEST(GentoSdkAbi, RequiredControlSymbolsLink) {
  EXPECT_NE(FX_L1_System_GetSDKVersion(), 0);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_System_Link), nullptr);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_Fbk_GetRT), nullptr);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_State_SwitchToPositionMode), nullptr);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_Runtime_SetJointPosCmd), nullptr);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_Runtime_SetSpeedRatio), nullptr);
  EXPECT_NE(reinterpret_cast<void*>(&FX_L1_Runtime_StopTraj), nullptr);
}
```

- [ ] **Step 2: Add the minimal package and CMake configuration**

Create `package.xml` with package name `gento_robot_driver`, version `0.1.0`, build tool `ament_cmake`, build dependencies `rclcpp` and `sensor_msgs`, and test dependency `ament_cmake_gtest`.

Create `CMakeLists.txt` containing these required elements:

```cmake
cmake_minimum_required(VERSION 3.16)
project(gento_robot_driver)
set(CMAKE_CXX_STANDARD 17)
set(GENTO_SDK_ROOT "/data/coding/tianji/tianji-robot-SDK-Gento_Skye-Luna/C_SDK")
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(GENTO_SDK_LIBRARY NAMES GentoSDK PATHS ${GENTO_SDK_ROOT} NO_DEFAULT_PATH REQUIRED)

ament_add_gtest(test_sdk_abi test/test_sdk_abi.cpp)
target_include_directories(test_sdk_abi PRIVATE ${GENTO_SDK_ROOT}/L1Robot ${GENTO_SDK_ROOT}/Common)
target_link_libraries(test_sdk_abi ${GENTO_SDK_LIBRARY})
set_target_properties(test_sdk_abi PROPERTIES BUILD_RPATH ${GENTO_SDK_ROOT})
ament_package()
```

- [ ] **Step 3: Run the test to verify the initial configuration fails only before package creation is complete**

Run from `gento_ros2_ws`:

```bash
colcon test --packages-select gento_robot_driver --event-handlers console_direct+
```

Expected before Step 2 is a package-not-found failure. Expected after Step 2 is a compiled test that passes; a link error means the SDK path or ABI is wrong and must be fixed before implementing ROS code.

- [ ] **Step 4: Verify the generated binary resolves Gento, not Marvin**

Run:

```bash
file /data/coding/tianji/tianji-robot-SDK-Gento_Skye-Luna/C_SDK/libGentoSDK.so
nm -D --defined-only /data/coding/tianji/tianji-robot-SDK-Gento_Skye-Luna/C_SDK/libGentoSDK.so | c++filt | rg 'FX_L1_(System_Link|Fbk_GetRT|State_SwitchToPositionMode|Runtime_SetJointPosCmd|Runtime_SetSpeedRatio|Runtime_StopTraj)'
ctest --test-dir build/gento_robot_driver --output-on-failure
```

Expected: x86-64 ELF, all six symbols, and one passing test.

### Task 2: Implement and test the SDK-facing core state machine

**Files:**
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/include/gento_robot_driver/driver_core.hpp`
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/src/driver_core.cpp`
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/test/test_driver_core.cpp`
- Modify: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/CMakeLists.txt`

**Interfaces:**
- Consumes: Gento C API and `std::array<double, 7>` position commands.
- Produces: `gento_robot_driver::DriverCore` with `connect_and_enable()`, `send_position()`, `read_state()`, and `shutdown()`.

- [ ] **Step 1: Write failing pure validation tests**

Create `test/test_driver_core.cpp`:

```cpp
#include <gtest/gtest.h>
#include "gento_robot_driver/driver_core.hpp"

using gento_robot_driver::DriverCore;

TEST(DriverCore, MapsLeftAndRightToDistinctSdkObjects) {
  EXPECT_EQ(DriverCore::sdk_object_for_arm(DriverCore::Arm::kLeft), FX_OBJ_ARM0);
  EXPECT_EQ(DriverCore::sdk_object_for_arm(DriverCore::Arm::kRight), FX_OBJ_ARM1);
}

TEST(DriverCore, RejectsNonFiniteAndOutOfLimitTargets) {
  const std::array<double, 7> valid{0, 0, 0, -0.5, 0, 0, 0};
  EXPECT_TRUE(DriverCore::validate_target(valid, {-3, -2, -3, -2.4, -3, -1, -1}, {3, 2, 3, 1, 3, 1, 1}));
  auto nan_target = valid;
  nan_target[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(DriverCore::validate_target(nan_target, {-3, -2, -3, -2.4, -3, -1, -1}, {3, 2, 3, 1, 3, 1, 1}));
  auto out_of_limit = valid;
  out_of_limit[3] = -2.5;
  EXPECT_FALSE(DriverCore::validate_target(out_of_limit, {-3, -2, -3, -2.4, -3, -1, -1}, {3, 2, 3, 1, 3, 1, 1}));
}

TEST(DriverCore, RejectsCommandsBeforePositionReady) {
  DriverCore core;
  EXPECT_FALSE(core.command_allowed());
}
```

- [ ] **Step 2: Run the test and observe the expected missing-header failure**

Run:

```bash
colcon test --packages-select gento_robot_driver --event-handlers console_direct+
```

Expected: compilation fails because `gento_robot_driver/driver_core.hpp` does not exist.

- [ ] **Step 3: Implement the smallest `DriverCore` API**

Define:

```cpp
enum class Arm { kLeft, kRight };
using JointArray = std::array<double, 7>;
struct DualArmState { JointArray left_position, right_position, left_velocity, right_velocity; };

static FXObjType sdk_object_for_arm(Arm arm);
static bool validate_target(const JointArray&, const JointArray& minimum, const JointArray& maximum);
bool connect_and_enable(const std::array<unsigned char, 4>& ip, int left_ratio, int right_ratio);
bool command_allowed() const;
bool send_position(Arm arm, const JointArray& target);
std::optional<DualArmState> read_state() const;
void shutdown();
```

`connect_and_enable()` must call `FX_L1_System_Link(ip[0], ip[1], ip[2], ip[3], FX_LOG_ALL_FLAG)`, reject a negative return, call `FX_L1_State_SwitchToPositionMode(FX_OBJ_ARM0, 3000, left_ratio, left_ratio)`, call the corresponding ARM1 function with `right_ratio`, then call `FX_L1_Runtime_SetSpeedRatio(1, obj, ratio, ratio)` for both arms. Set `position_ready_ = true` only if every SDK return is zero.

`read_state()` must map `m_ARMS[0/1].m_ARM_OUT.m_ARM_FBK_Joint_Pos` and `m_ARM_FBK_Joint_Vel` to `DualArmState`. `send_position()` must reject if `position_ready_` is false, then call `FX_L1_Runtime_SetJointPosCmd(1, sdk_object_for_arm(arm), target.data())` and return true only for zero.

`shutdown()` must call `FX_L1_Runtime_StopTraj(1, FX_OBJ_ALL_FLAG)`, `FX_L1_State_SwitchToIdle(FX_OBJ_ARM0, 3000)`, `FX_L1_State_SwitchToIdle(FX_OBJ_ARM1, 3000)`, and `FX_L1_System_Unlink()` only if `linked_` is true.

- [ ] **Step 4: Run unit tests and ABI test**

Run:

```bash
colcon test --packages-select gento_robot_driver --event-handlers console_direct+
colcon test-result --verbose
```

Expected: `test_sdk_abi` and `test_driver_core` pass. No hardware connection is made by either test.

### Task 3: Add the ROS node, launch file, and configuration

**Files:**
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/include/gento_robot_driver/gento_robot_driver_node.hpp`
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/src/gento_robot_driver_node.cpp`
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/src/main.cpp`
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/config/gento_robot.yaml`
- Create: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/launch/gento_robot_driver.launch.py`
- Modify: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/CMakeLists.txt`

**Interfaces:**
- Consumes: `DriverCore`, `sensor_msgs/msg/JointState`, ROS parameters.
- Produces: executable `gento_robot_driver`, subscribers `/left_joint_control` and `/right_joint_control`, publisher `/joint_states`.

- [ ] **Step 1: Write a failing ROS graph test**

Create `test/test_node_interfaces.cpp` that constructs `GentoRobotDriverNode` with `connect_on_startup=false`, then asserts:

```cpp
EXPECT_EQ(node->get_name(), "gento_robot_driver");
EXPECT_TRUE(node->get_publishers_info_by_topic("/joint_states").size() == 1U);
EXPECT_TRUE(node->get_subscriptions_info_by_topic("/left_joint_control").size() == 1U);
EXPECT_TRUE(node->get_subscriptions_info_by_topic("/right_joint_control").size() == 1U);
```

- [ ] **Step 2: Run the test and verify it fails because the node header is absent**

Run:

```bash
colcon test --packages-select gento_robot_driver --event-handlers console_direct+
```

Expected: compilation fails on missing `gento_robot_driver_node.hpp`.

- [ ] **Step 3: Implement the ROS boundary**

The node must declare these parameters and defaults:

```yaml
robot_ip: "6.6.7.190"
left_velocity_ratio: 10
right_velocity_ratio: 10
state_publish_hz: 100.0
connect_on_startup: true
left_joint_limits_min: [-3.0, -2.0, -3.0, -2.4, -3.0, -1.0, -1.0]
left_joint_limits_max: [ 3.0,  2.0,  3.0,  1.0,  3.0,  1.0,  1.0]
right_joint_limits_min: [-3.0, -2.0, -3.0, -2.4, -3.0, -1.0, -1.0]
right_joint_limits_max: [ 3.0,  2.0,  3.0,  1.0,  3.0,  1.0,  1.0]
```

Use subscriptions with queue depth 10. Each callback must require `msg->position.size() == 7`, copy positions into `JointArray`, validate against its arm limits, call `core_.send_position()`, and log an error on rejection or SDK failure. Do not use `msg->name` for indexing; the seven-element position order is the package contract.

Use a wall timer at `1.0 / state_publish_hz`; when `core_.read_state()` returns a value, publish names `l_j1` through `l_j7` then `r_j1` through `r_j7`, plus 14 positions and 14 velocities. `main.cpp` must call `rclcpp::init`, spin the node, then `rclcpp::shutdown`. The node destructor must call `core_.shutdown()`.

Install the executable, `config/`, and `launch/`. The launch file must declare `params_file` and start `gento_robot_driver` with `parameters=[LaunchConfiguration('params_file')]` and `output='screen'`.

- [ ] **Step 4: Build and run all non-hardware tests**

Run:

```bash
colcon build --packages-select gento_robot_driver --symlink-install
source install/setup.bash
colcon test --packages-select gento_robot_driver --event-handlers console_direct+
colcon test-result --verbose
ldd install/gento_robot_driver/lib/gento_robot_driver/gento_robot_driver | rg 'GentoSDK|MarvinSDK'
```

Expected: build and all three tests pass; `ldd` shows `libGentoSDK.so` and does not show `libMarvinSDK.so`.

### Task 4: Verify connection and perform a gated B/J4 hardware test

**Files:**
- Modify only if needed after a documented SDK return failure: `/data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/src/gento_robot_driver/config/gento_robot.yaml`

**Interfaces:**
- Consumes: a safe workcell, controller at `6.6.7.190`, and the independent Gento node.
- Produces: evidence that Gento SDK feedback works and that a B/J4 command moves toward the requested target, or a specific SDK return code that blocks motion.

- [ ] **Step 1: Start the new driver without the legacy `robot_servo_driver` connected to the controller**

Stop the old container or otherwise ensure it is not connected, then run from `gento_ros2_ws`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=20
ros2 launch gento_robot_driver gento_robot_driver.launch.py
```

Expected: logs show successful `FX_L1_System_Link`, SDK/controller versions, both position-mode transitions, and right speed ratio 10.

- [ ] **Step 2: Verify actual feedback before motion**

In another terminal with the same ROS environment:

```bash
ros2 topic echo --once /joint_states
ros2 topic hz /joint_states
```

Expected: exactly 14 names, 14 finite positions, 14 finite velocities, and a stable rate close to 100 Hz. Do not issue motion if this check fails or if any position is non-finite.

- [ ] **Step 3: Send one right B/J4 target derived from live feedback**

After confirming the workcell is clear, calculate `target_r_j4 = current_r_j4 - 0.523598776`. Abort if it violates `right_joint_limits_min[3]..right_joint_limits_max[3]`. Publish the full right-arm target at 50 Hz for 2 seconds:

```bash
ros2 topic pub -r 50 --times 100 /right_joint_control sensor_msgs/msg/JointState "{position: [CURRENT_R_J1, CURRENT_R_J2, CURRENT_R_J3, TARGET_R_J4, CURRENT_R_J5, CURRENT_R_J6, CURRENT_R_J7]}"
```

Replace each `CURRENT_R_J*` with the just-read feedback values and `TARGET_R_J4` with the calculated value; do not send the command if the B/J4 target is outside limits.

- [ ] **Step 4: Verify the outcome and stop safely**

Run:

```bash
ros2 topic echo --once /joint_states
```

Expected success: the returned `r_j4` has changed in the negative direction toward the target while all other right-arm joints remain near their initial values. On any SDK error, unexpected movement, or lack of change, stop the node so its shutdown path calls `FX_L1_Runtime_StopTraj`, then capture the node logs and do not repeat the command.

## Plan Self-Review

- Spec coverage: Tasks 1–3 implement independent workspace, direct Gento linkage, topic compatibility, position mode, state feedback, speed ratio, validation, and safe shutdown. Task 4 covers ABI, connection, feedback, and gated hardware verification.
- Placeholder scan: no deferred implementation references remain; the one hardware command uses uppercase live values deliberately because the values must be read at execution time and cannot safely be hard-coded.
- Type consistency: all SDK calls use `FXObjType`, SDK thread ID `1`, seven-element `std::array<double, 7>`, and `sensor_msgs/msg/JointState` in every task.
