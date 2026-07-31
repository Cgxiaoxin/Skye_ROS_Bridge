# Gento Radian Interface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep every ROS joint command and feedback value in radians while converting only at the Gento SDK boundary, then make the leader bridge publish compatible `sensor_msgs/msg/JointState` commands.

**Architecture:** `DriverCore` owns the Gento SDK boundary and converts command position arrays from radian to degree and feedback position/velocity arrays from degree to radian. The leader bridge keeps its input mapping in radians and gains a Gento output mode which publishes `JointState` to the Gento driver's input topics instead of Marvin's `JointcmdArm` topics.

**Tech Stack:** ROS 2 Humble, C++17, `ament_cmake_gtest`, Python 3/rclpy, `sensor_msgs/msg/JointState`.

## Global Constraints

- All ROS arm positions use rad and all ROS arm velocities use rad/s.
- The Gento SDK is supplied and consumed only in deg and deg/s.
- Command limits remain radian limits and must be checked before conversion.
- Real hardware motion occurs only after build, tests, and a live ROS graph check; retain 10% velocity/acceleration ratios.
- The current bridge's Marvin mode must remain compatible with `marvin_msgs/msg/JointcmdArm`.

---

### Task 1: Unit-safe Gento SDK adapter

**Files:**
- Modify: `gento_ros2_ws/src/gento_robot_driver/include/gento_robot_driver/driver_core.hpp`
- Modify: `gento_ros2_ws/src/gento_robot_driver/src/driver_core.cpp`
- Test: `gento_ros2_ws/src/gento_robot_driver/test/test_driver_core.cpp`

**Interfaces:**
- Consumes: `DriverCore::JointArray` with ROS rad/rad/s values.
- Produces: `ros_radians_to_sdk_degrees()` and `sdk_degrees_to_ros_radians()` pure array conversions.

- [ ] Add failing tests that require `-0.5235987755982988 rad` to equal `-30.0 deg`, and the reverse conversion to equal `-0.5235987755982988 rad`.
- [ ] Run `colcon test --packages-select gento_robot_driver --ctest-args -R test_driver_core` and confirm the missing conversion helper produces a compile failure.
- [ ] Add the two pure conversions using `std::numbers::pi_v<double>` and apply them in `send_position()` and `read_state()`.
- [ ] Re-run the package test suite and require all tests to pass.

### Task 2: Gento-compatible leader bridge output

> **DISABLED 2026-07-24:** Gento teleop must not use `skye_leader_bridge`.
> `config/gento_leader_bridge.yaml` and `config/gento_right_j4_test_bridge.yaml`
> are commented out / marked `disabled: true`. Skip this task for the Gento path.

**Files:**
- Modify: `skye_leader_bridge_ws/src/skye_leader_bridge/skye_leader_bridge/node.py`
- Modify: `skye_leader_bridge_ws/src/skye_leader_bridge/config/gento_leader_bridge.yaml`
- Test: `skye_leader_bridge_ws/src/skye_leader_bridge/test/test_mapping.py`

**Interfaces:**
- Consumes: small-arm `sensor_msgs/msg/JointState.position` values in rad.
- Produces: Gento bridge `sensor_msgs/msg/JointState` output in rad to `/gento/left_joint_control` and `/gento/right_joint_control`.

- [ ] Add a failing unit test for the new output mode selection and preserve existing Marvin mapping tests.
- [ ] Run `python3 -m unittest discover -s test` and confirm the new behavior fails before implementation.
- [ ] Add an output message mode that creates `JointState` instead of `JointcmdArm`, stamps it, and copies mapped radian positions unchanged.
- [ ] Add a Gento config that selects `joint_state` output, uses `gento` topics, preserves current order/signs/limits, and retains enable/deadman behavior.
- [ ] Re-run bridge tests and compile checks.

### Task 3: Integration verification

**Files:**
- Modify: `gento_ros2_ws/README.md`
- Modify: `skye_leader_bridge_ws/src/skye_leader_bridge/STARTUP.md`

**Interfaces:**
- Consumes: the two corrected nodes on a shared `ROS_DOMAIN_ID`.
- Produces: a documented test path where `-0.523598776 rad` represents Gento SDK `-30 deg`.

- [ ] Document units at each boundary and command startup order.
- [ ] Build both packages and run their automated tests.
- [ ] With the robot disconnected, launch nodes with hardware connection disabled where available and verify bridge publisher/subscriber message types using `ros2 topic info -v`.
- [ ] After explicit live-test confirmation, connect only the Gento driver, read feedback, issue a full seven-joint current-state target with J4 offset by `-0.523598776 rad`, and confirm feedback changes by approximately that amount.
