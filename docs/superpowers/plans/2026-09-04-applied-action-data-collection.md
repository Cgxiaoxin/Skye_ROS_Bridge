# Applied Action Data Collection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish SDK-bound joint/gripper actions from `skye_robot_driver` and record them (plus arm/gripper state) to mcap via a new `skye_data_recorder` node for teleop training data.

**Architecture:** After successful `send_position` / gripper `tick_control`, driver publishes four RELIABLE `*_action_applied` topics. A separate Python recorder subscribes to those plus observation topics and writes rosbag2 mcap on start/stop services. No remapping of FACTR inputs; no duplicate mapping logic in the recorder.

**Tech Stack:** ROS 2 Humble, C++ `skye_robot_driver`, Python `rclpy` + `rosbag2_py` (mcap), `sensor_msgs/JointState`, `std_srvs/Trigger`.

**Spec:** `docs/superpowers/specs/2026-09-04-applied-action-data-collection-design.md`

**Interface doc (already updated):** `docs/ros_interfaces.md`

## Global Constraints

- Applied joint `position` is **rad**, equal to post-preprocess `mapped` / `last_command_` (not SDK degrees).
- Applied gripper `position[0]` is **motor space** (0=open, 1=close) after invert + `close_limit`.
- Publish applied **only** after successful SDK/hardware send path (joints: `send_position` true; gripper: each `tick_control` when gripper enabled/started).
- Applied QoS: **RELIABLE** + `KeepLast(depth)` default **20** (param `applied_action_qos_depth`, clamp to ≥10). Do **not** reuse command `BEST_EFFORT`+depth1.
- Recorder must **not** reimplement relative/signs/limits; only subscribe and bag.
- Keep HITL `/skye/recorder/*` unchanged; new services are `/skye/data_recorder/start|stop`.
- Work on current branch; commit after each task; do not force-push.
- Default `ROS_DOMAIN_ID` for P4 remains **21** (same machine as driver).

## File Map

| Path | Role |
|------|------|
| `skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp` | Create applied pubs; publish after send |
| `skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_node.hpp` | Publisher members + helper decls |
| `skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py` | Remap internal → `/gento/*_action_applied` |
| `skye_ros2_ws/src/skye_robot_driver/config/skye_robot.yaml` | `applied_action_qos_depth: 20` |
| `skye_ros2_ws/src/skye_data_recorder/` | New ament_python package + node + launch |
| `skye_ros2_ws/scripts/verify_applied_action_topics.sh` | Smoke check topics/QoS/services |
| `docs/ros_interfaces.md` | Already documents topics + recorder (verify only) |

---

### Task 1: Driver — joint `*_joint_action_applied` publishers

**Files:**
- Modify: `skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_node.hpp`
- Modify: `skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp`
- Modify: `skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py`
- Modify: `skye_ros2_ws/src/skye_robot_driver/config/skye_robot.yaml`

**Interfaces:**
- Consumes: existing `handle_command` / `handle_absolute_command` → `core_.send_position(arm, mapped)`
- Produces: publishers on `/left_joint_action_applied` and `/right_joint_action_applied` (node-local names), remapped to `/gento/left_joint_action_applied` and `/gento/right_joint_action_applied`; helper `applied_action_qos(int depth)` → `rclcpp::QoS`

- [ ] **Step 1: Add QoS helper and publishers in `driver_node.cpp` / `.hpp`**

In `driver_node.cpp`, next to `state_qos()`:

```cpp
rclcpp::QoS applied_action_qos(int depth) {
  const int d = std::max(10, depth);
  rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(d)));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}
```

In constructor after existing state publishers, declare param and create pubs:

```cpp
const int applied_depth =
    declare_parameter<int>("applied_action_qos_depth", 20);
const auto applied_qos = applied_action_qos(applied_depth);
left_joint_action_applied_publisher_ =
    create_publisher<JointState>("/left_joint_action_applied", applied_qos);
right_joint_action_applied_publisher_ =
    create_publisher<JointState>("/right_joint_action_applied", applied_qos);
```

Add matching `rclcpp::Publisher<JointState>::SharedPtr` members in `driver_node.hpp`.

Add private helper (declaration in hpp, definition in cpp):

```cpp
void publish_joint_action_applied(
    DriverCore::Arm arm, const JointArray &mapped);
```

Implementation:

```cpp
void DriverNode::publish_joint_action_applied(
    DriverCore::Arm arm, const JointArray &mapped) {
  const auto &names =
      arm == DriverCore::Arm::kLeft ? kLeftJointNames : kRightJointNames;
  auto *pub = arm == DriverCore::Arm::kLeft
                  ? left_joint_action_applied_publisher_.get()
                  : right_joint_action_applied_publisher_.get();
  JointArray zero_vel{};
  pub->publish(make_arm_joint_state(now(), names, mapped, zero_vel));
}
```

- [ ] **Step 2: Publish only after successful `send_position`**

In `handle_command`, replace the success path so publish happens only when send succeeds:

```cpp
  if (!core_.send_position(arm, mapped)) {
    RCLCPP_ERROR(
        get_logger(),
        "%s command failed: SDK not ready / idle / mode rejected command",
        arm_name);
    return;
  }

  publish_joint_action_applied(arm, mapped);
  last_command = mapped;
  last_command_time = now();
  streaming = true;
  // ... existing absolute-session invalidation unchanged ...
```

Apply the same `publish_joint_action_applied(arm, mapped)` immediately after successful `send_position` in `handle_absolute_command`.

Do **not** publish on failed send or when command is rejected earlier (size/NaN).

- [ ] **Step 3: Launch remaps + yaml param**

Append to `_REMAPS` in `skye_robot_driver.launch.py`:

```python
    ("/left_joint_action_applied", "/gento/left_joint_action_applied"),
    ("/right_joint_action_applied", "/gento/right_joint_action_applied"),
```

In `skye_robot.yaml` under `ros__parameters:`:

```yaml
    # Data-collection applied-action pubs (RELIABLE). Depth >=10; default 20.
    applied_action_qos_depth: 20
```

- [ ] **Step 4: Build and smoke-check publishers exist**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
./scripts/build.sh
source install/setup.bash
# With driver running (or ros2 run briefly):
ros2 topic list | grep action_applied
ros2 topic info -v /gento/left_joint_action_applied
```

Expected: topic listed; Reliability `RELIABLE`; History depth ≥10.

- [ ] **Step 5: Commit**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge
git add \
  skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_node.hpp \
  skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp \
  skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py \
  skye_ros2_ws/src/skye_robot_driver/config/skye_robot.yaml
git commit -m "$(cat <<'EOF'
feat(driver): publish joint action_applied after SDK send

Expose post-preprocess last_command_ as RELIABLE JointState topics
for teleop data-collection labels.
EOF
)"
```

---

### Task 2: Driver — gripper `*_gripper_action_applied` publishers

**Files:**
- Modify: `skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_node.hpp`
- Modify: `skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp`
- Modify: `skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py`

**Interfaces:**
- Consumes: `gripper_.started()`, `gripper_.target(arm)`, `tick_gripper()` → `gripper_.tick_control()`
- Produces: `/left_gripper_action_applied` and `/right_gripper_action_applied` (node-local), remapped to `/gento/left_gripper_action_applied` and `/gento/right_gripper_action_applied`; `position[0]` = motor norm from `gripper_.target(arm)`

- [ ] **Step 1: Create gripper applied publishers (same applied QoS)**

In constructor (reuse `applied_qos` from Task 1):

```cpp
left_gripper_action_applied_publisher_ =
    create_publisher<JointState>("/left_gripper_action_applied", applied_qos);
right_gripper_action_applied_publisher_ =
    create_publisher<JointState>("/right_gripper_action_applied", applied_qos);
```

Add publisher members in hpp.

- [ ] **Step 2: Publish motor target after `tick_control`**

Add helper:

```cpp
void DriverNode::publish_gripper_action_applied() {
  if (!gripper_enabled_ || !gripper_.started()) {
    return;
  }
  auto publish_one =
      [this](DriverCore::Arm arm,
             const rclcpp::Publisher<JointState>::SharedPtr &pub) {
        JointState msg;
        msg.header.stamp = now();
        msg.name = {"gripper_joint"};
        msg.position = {gripper_.target(arm)};  // motor space
        msg.velocity = {0.0};
        msg.effort = {0.0};
        pub->publish(msg);
      };
  publish_one(DriverCore::Arm::kLeft, left_gripper_action_applied_publisher_);
  publish_one(DriverCore::Arm::kRight, right_gripper_action_applied_publisher_);
}
```

In `tick_gripper()`:

```cpp
void DriverNode::tick_gripper() {
  if (!gripper_enabled_) {
    return;
  }
  gripper_.tick_control();
  publish_gripper_action_applied();
  gripper_.tick_feedback();
  publish_gripper_state();
}
```

Do **not** convert with `motor_to_factr_norm` on applied topics.

- [ ] **Step 3: Remaps**

Append to `_REMAPS`:

```python
    ("/left_gripper_action_applied", "/gento/left_gripper_action_applied"),
    ("/right_gripper_action_applied", "/gento/right_gripper_action_applied"),
```

- [ ] **Step 4: Build and verify**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
./scripts/build.sh && source install/setup.bash
ros2 topic list | grep gripper_action_applied
```

Expected: both `/gento/left_gripper_action_applied` and `/gento/right_gripper_action_applied` present when driver+gripper enabled.

- [ ] **Step 5: Commit**

```bash
git add \
  skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_node.hpp \
  skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp \
  skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py
git commit -m "$(cat <<'EOF'
feat(driver): publish gripper action_applied in motor space

Emit per-tick motor targets (0=open,1=close) for data-collection
alongside joint applied topics.
EOF
)"
```

---

### Task 3: New package `skye_data_recorder` (mcap episode recorder)

**Files:**
- Create: `skye_ros2_ws/src/skye_data_recorder/package.xml`
- Create: `skye_ros2_ws/src/skye_data_recorder/setup.py`
- Create: `skye_ros2_ws/src/skye_data_recorder/setup.cfg`
- Create: `skye_ros2_ws/src/skye_data_recorder/resource/skye_data_recorder`
- Create: `skye_ros2_ws/src/skye_data_recorder/skye_data_recorder/__init__.py`
- Create: `skye_ros2_ws/src/skye_data_recorder/skye_data_recorder/data_recorder_node.py`
- Create: `skye_ros2_ws/src/skye_data_recorder/test/test_next_episode_path.py`
- Create: `skye_ros2_ws/src/skye_data_recorder/launch/data_recorder.launch.py`

**Interfaces:**
- Consumes: topics listed in DEFAULT_TOPICS below; QoS RELIABLE depth=`applied_qos_depth` for `*_action_applied`, RELIABLE depth=1 for states
- Produces: services `/skye/data_recorder/start` and `/skye/data_recorder/stop` (`std_srvs/Trigger`); mcap under `output_dir/episode_XXXX/`
- Produces: `next_episode_path(output_dir: str, existing: Sequence[Path] = ()) -> Path`

- [ ] **Step 1: Write failing unit test for episode path helper**

Create `test/test_next_episode_path.py`:

```python
from pathlib import Path
from skye_data_recorder.data_recorder_node import next_episode_path


def test_next_episode_path_skips_existing(tmp_path: Path):
    (tmp_path / "episode_0000").mkdir()
    path = next_episode_path(str(tmp_path))
    assert path.name == "episode_0001"
```

- [ ] **Step 2: Run test — expect fail (module missing)**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
PYTHONPATH=src/skye_data_recorder pytest src/skye_data_recorder/test/test_next_episode_path.py -v
```

Expected: FAIL import or missing `next_episode_path`.

- [ ] **Step 3: Implement package scaffolding + node**

`package.xml` (format 3, ament_python):

```xml
<?xml version="1.0"?>
<package format="3">
  <name>skye_data_recorder</name>
  <version>0.1.0</version>
  <description>Teleop mcap recorder for applied joint/gripper actions</description>
  <maintainer email="dev@tianji.local">tianji</maintainer>
  <license>Apache-2.0</license>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_srvs</depend>
  <exec_depend>rosbag2_py</exec_depend>
  <exec_depend>rosbag2_storage_mcap</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <test_depend>pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`setup.cfg`:

```ini
[develop]
script_dir=$base/lib/skye_data_recorder
[install]
install_scripts=$base/lib/skye_data_recorder
```

`setup.py`:

```python
from setuptools import setup

package_name = "skye_data_recorder"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/data_recorder.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tianji",
    maintainer_email="dev@tianji.local",
    description="Teleop mcap recorder for applied actions",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "data_recorder = skye_data_recorder.data_recorder_node:main",
        ],
    },
    tests_require=["pytest"],
)
```

Touch empty `resource/skye_data_recorder` and `skye_data_recorder/__init__.py`.

Implement `data_recorder_node.py` modeled on `skye_hitl_dagger/episode_recorder_node.py`, with these constants and differences:

```python
DEFAULT_TOPICS = (
    "/gento/left_joint_action_applied",
    "/gento/right_joint_action_applied",
    "/gento/left_gripper_action_applied",
    "/gento/right_gripper_action_applied",
    "/gento/joint_states",
    "/left_gripper/state",
    "/right_gripper/state",
)

TOPIC_MESSAGE_TYPES = {t: JointState for t in DEFAULT_TOPICS}

APPLIED_TOPICS = {
    "/gento/left_joint_action_applied",
    "/gento/right_joint_action_applied",
    "/gento/left_gripper_action_applied",
    "/gento/right_gripper_action_applied",
}
```

Node class `DataRecorderNode`:

- Params: `output_dir` default `/tmp/skye_data_bags`, `topics` default list(DEFAULT_TOPICS), `storage_id`=`mcap`, `applied_qos_depth` default `20`
- Applied QoS: `KEEP_LAST(max(10, applied_qos_depth))` + `RELIABLE` + `VOLATILE`
- State QoS: `KEEP_LAST(1)` + `RELIABLE` + `VOLATILE`
- Services: `/skye/data_recorder/start`, `/skye/data_recorder/stop`
- On mcap open failure, message must mention `sudo apt install ros-humble-rosbag2-storage-mcap`
- `_record`: no-op if writer is None; else serialize with message stamp like HITL recorder

Copy `next_episode_path` logic from HITL `episode_recorder_node.py` unchanged.

- [ ] **Step 4: Re-run unit test — expect pass**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
colcon build --packages-select skye_data_recorder --symlink-install
source install/setup.bash
pytest src/skye_data_recorder/test/test_next_episode_path.py -v
# or: python3 -m pytest ...
```

Expected: PASS.

- [ ] **Step 5: Launch file**

`launch/data_recorder.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="/tmp/skye_data_bags"),
        DeclareLaunchArgument("applied_qos_depth", default_value="20"),
        Node(
            package="skye_data_recorder",
            executable="data_recorder",
            name="skye_data_recorder",
            output="screen",
            parameters=[{
                "output_dir": LaunchConfiguration("output_dir"),
                "applied_qos_depth": LaunchConfiguration("applied_qos_depth"),
                "storage_id": "mcap",
            }],
        ),
    ])
```

- [ ] **Step 6: Commit**

```bash
git add skye_ros2_ws/src/skye_data_recorder
git commit -m "$(cat <<'EOF'
feat: add skye_data_recorder mcap node for applied actions

Subscribe to driver action_applied + arm/gripper state; episode
start/stop services write rosbag2 mcap for teleop datasets.
EOF
)"
```

---

### Task 4: Verify script + doc consistency check

**Files:**
- Create: `skye_ros2_ws/scripts/verify_applied_action_topics.sh`
- Verify (no content rewrite unless drift): `docs/ros_interfaces.md`

**Interfaces:**
- Consumes: running `skye_robot_driver` + `skye_data_recorder` on `ROS_DOMAIN_ID`
- Produces: shell exit 0 when four applied topics exist with RELIABLE and recorder services are advertised

- [ ] **Step 1: Add verify script**

```bash
#!/usr/bin/env bash
set -euo pipefail
# Usage: ROS_DOMAIN_ID=21 ./scripts/verify_applied_action_topics.sh
# Requires: sourced workspace, driver (+ optional recorder) running.

need_topics=(
  /gento/left_joint_action_applied
  /gento/right_joint_action_applied
  /gento/left_gripper_action_applied
  /gento/right_gripper_action_applied
)

for t in "${need_topics[@]}"; do
  ros2 topic list | grep -qx "$t" || { echo "missing $t"; exit 1; }
  info=$(ros2 topic info -v "$t")
  echo "$info" | grep -qi Reliable || { echo "$t not RELIABLE"; exit 1; }
done

ros2 service list | grep -qx /skye/data_recorder/start || \
  echo "WARN: recorder not running (start service missing)"
ros2 service list | grep -qx /skye/data_recorder/stop || true

echo "OK: applied action topics present with RELIABLE QoS"
```

Make executable: `chmod +x skye_ros2_ws/scripts/verify_applied_action_topics.sh`

- [ ] **Step 2: Manual integration checklist (document in script header comments)**

```text
1. Terminal A: start skye_robot_driver (imp_joint, gripper on)
2. Terminal B: ros2 launch skye_data_recorder data_recorder.launch.py
3. ros2 service call /skye/data_recorder/start std_srvs/srv/Trigger {}
4. Teleop briefly (or pub joint_control once with driver in position/imp)
5. ros2 service call /skye/data_recorder/stop std_srvs/srv/Trigger {}
6. Inspect mcap under /tmp/skye_data_bags/episode_XXXX for applied + joint_states
```

- [ ] **Step 3: Confirm `docs/ros_interfaces.md` still matches**

Check Topic table has four `*_action_applied` rows and section「遥操数采节点 `skye_data_recorder`」matches services `/skye/data_recorder/start|stop`. If drift, fix in same commit.

- [ ] **Step 4: Commit**

```bash
git add skye_ros2_ws/scripts/verify_applied_action_topics.sh docs/ros_interfaces.md
git commit -m "$(cat <<'EOF'
test: add verify script for applied-action data collection topics
EOF
)"
```

---

## Spec coverage (self-review)

| Spec requirement | Task |
|------------------|------|
| Joint applied after successful SDK send | Task 1 |
| Gripper applied motor space on tick | Task 2 |
| RELIABLE + depth≥10 default 20 | Task 1–3 |
| Launch remap under `/gento/` | Task 1–2 |
| `skye_data_recorder` mcap + start/stop | Task 3 |
| Default topic list (4 applied + states) | Task 3 |
| Independent of HITL recorder | Task 3 (different service names) |
| `ros_interfaces.md` | Done in brainstorm; verified Task 4 |
| Verify / smoke | Task 4 |
| No camera / no pkl / no remapping guess | Non-goals; not in tasks |

## Placeholder scan

No TBD/TODO steps; code blocks are concrete; package path pinned to `skye_ros2_ws/src/skye_data_recorder`.
