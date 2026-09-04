# Follower Align After Sync Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After FACTR `1` sync, let the operator press host `s` (or pub a topic) so the big arms slowly absolute-align to the small-arm pose at 10% vel/acc, then manually `2` into relative teleop.

**Architecture:** Pure-Python align state machine + thin ROS node on the host; `skye_robot_driver` gains `/gento/set_motion_rates` via `FX_L1_Runtime_SetSpeedRatio`. FACTR Docker keyboard stays unchanged.

**Tech Stack:** ROS 2 Humble, C++17 (`skye_robot_driver`), ament_python (`skye_follower_align`), pytest, gtest.

**Spec:** `docs/superpowers/specs/2026-09-04-follower-align-after-sync-design.md`

## Global Constraints

- Align direction: big arm follows small arm via absolute commands.
- During align: left/right vel_ratio and acc_ratio forced to **10**.
- Trigger: host keyboard **`s`** and `/mode/align_follower` (`data: align_follower`).
- Cancel: host **`x`** and `/mode/align_cancel` (`data: align_cancel`).
- Completion: per-joint `|err| < 0.05` rad (default) for consecutive frames → `ALIGNED`; timeout → `TIMEOUT_WARN` only (soft; teleop still allowed).
- Mapping: `q_big_cmd[i] = sign[i] * q_leader[i]` (offsets 0); publish on `/gento/{left,right}_joint_control_abs`.
- Do not auto-publish `switch_teleop`; do not command grippers; do not modify closed-source FACTR.
- Same `ROS_DOMAIN_ID=21` and FastDDS no-SHM xml as teleop.
- Work on current `main` unless user says otherwise; commit per task.

## File Map

| Path | Role |
|------|------|
| `skye_ros2_ws/src/skye_robot_driver/srv/SetMotionRates.srv` | New service definition |
| `skye_ros2_ws/src/skye_robot_driver/src/driver_core.{hpp,cpp}` | `set_speed_ratios()` wrapping SDK |
| `skye_ros2_ws/src/skye_robot_driver/src/driver_node.{hpp,cpp}` | Service `/set_motion_rates` (remap `/gento/...`) |
| `skye_ros2_ws/src/skye_follower_align/` | New ament_python package |
| `.../skye_follower_align/align_logic.py` | Pure state machine (unit-tested) |
| `.../skye_follower_align/follower_align_node.py` | ROS node |
| `.../skye_follower_align/host_keyboard.py` | Host `s`/`x` → mode topics |
| `.../launch/follower_align.launch.py` | Start align + optional keyboard |
| `scripts/start_follower_align.sh` | Host helper (domain/xml) |
| `docs/Thor_Orin_遥操启动.md`, `docs/ros_interfaces.md` | Operator docs |

---

### Task 1: Pure align logic (TDD)

**Files:**
- Create: `skye_ros2_ws/src/skye_follower_align/skye_follower_align/__init__.py`
- Create: `skye_ros2_ws/src/skye_follower_align/skye_follower_align/align_logic.py`
- Create: `skye_ros2_ws/src/skye_follower_align/test/test_align_logic.py`
- Create: package scaffolding (`package.xml`, `setup.py`, `setup.cfg`, `resource/skye_follower_align`)

**Interfaces:**
- Produces:
  - `map_leader_to_follower(leader: Sequence[float], signs: Sequence[float]) -> list[float]`
  - `class AlignPhase(Enum): IDLE, ALIGNING, ALIGNED, TIMEOUT_WARN`
  - `class AlignSession` with `start()`, `cancel()`, `on_tick(...)`, `phase`, `ignore_duplicate_start`

- [ ] **Step 1: Scaffold minimal ament_python package**

```xml
<!-- package.xml -->
<?xml version="1.0"?>
<package format="3">
  <name>skye_follower_align</name>
  <version>0.1.0</version>
  <description>Host-side follower absolute align after FACTR sync</description>
  <maintainer email="dev@tianji.local">tianji</maintainer>
  <license>Apache-2.0</license>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>skye_robot_driver</depend>
  <test_depend>python3-pytest</test_depend>
  <export><build_type>ament_python</build_type></export>
</package>
```

```python
# setup.py (excerpt)
from setuptools import setup
package_name = "skye_follower_align"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/follower_align.launch.py"]),
    ],
    entry_points={
        "console_scripts": [
            "follower_align_node = skye_follower_align.follower_align_node:main",
            "host_keyboard_align = skye_follower_align.host_keyboard:main",
        ],
    },
)
```

Create empty `resource/skye_follower_align`, `setup.cfg` with `[develop] script_dir=$base/lib/skye_follower_align` and `[install] install_scripts=$base/lib/skye_follower_align`.

- [ ] **Step 2: Write failing tests**

```python
# test/test_align_logic.py
from skye_follower_align.align_logic import (
    AlignPhase, AlignSession, map_leader_to_follower,
)

def test_map_leader_applies_signs():
    leader = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    signs = [1, 1, 1, 1, 1, -1, -1]
    out = map_leader_to_follower(leader, signs)
    assert out == [0.1, 0.2, 0.3, 0.4, 0.5, -0.6, -0.7]

def test_aligned_after_hold_frames():
    s = AlignSession(threshold_rad=0.05, hold_frames=3, timeout_s=10.0)
    s.start()
    leader = [0.0] * 7
    big = [0.01] * 7
    signs = [1.0] * 7
    for _ in range(2):
        assert s.on_tick(leader, big, signs) == AlignPhase.ALIGNING
    assert s.on_tick(leader, big, signs) == AlignPhase.ALIGNED

def test_timeout_warn_soft():
    s = AlignSession(threshold_rad=0.01, hold_frames=3, timeout_s=0.0)
    s.start()
    # force elapsed by injecting start_monotonic in test via optional now= param
    phase = s.on_tick([1.0]*7, [0.0]*7, [1.0]*7, now=100.0)
    assert phase == AlignPhase.TIMEOUT_WARN

def test_second_start_ignored_while_aligning():
    s = AlignSession(threshold_rad=0.05, hold_frames=10, timeout_s=10.0)
    assert s.start() is True
    assert s.start() is False  # ignored
    assert s.phase == AlignPhase.ALIGNING

def test_cancel_returns_idle():
    s = AlignSession(threshold_rad=0.05, hold_frames=10, timeout_s=10.0)
    s.start()
    s.cancel()
    assert s.phase == AlignPhase.IDLE
```

- [ ] **Step 3: Run tests — expect FAIL**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
# until package builds, run via PYTHONPATH:
PYTHONPATH=src/skye_follower_align python3 -m pytest src/skye_follower_align/test/test_align_logic.py -v
```

Expected: import / attribute errors.

- [ ] **Step 4: Implement `align_logic.py`**

```python
from __future__ import annotations
from enum import Enum, auto
import time
from typing import Optional, Sequence

DOF = 7

class AlignPhase(Enum):
    IDLE = auto()
    ALIGNING = auto()
    ALIGNED = auto()
    TIMEOUT_WARN = auto()

def map_leader_to_follower(leader: Sequence[float], signs: Sequence[float]) -> list[float]:
    if len(leader) != DOF or len(signs) != DOF:
        raise ValueError("expected 7 joints")
    return [float(signs[i]) * float(leader[i]) for i in range(DOF)]

def max_abs_err(cmd: Sequence[float], measured: Sequence[float]) -> float:
    return max(abs(float(cmd[i]) - float(measured[i])) for i in range(DOF))

class AlignSession:
    def __init__(self, threshold_rad: float = 0.05, hold_frames: int = 5,
                 timeout_s: float = 10.0):
        self.threshold_rad = threshold_rad
        self.hold_frames = hold_frames
        self.timeout_s = timeout_s
        self.phase = AlignPhase.IDLE
        self._ok_frames = 0
        self._t0 = 0.0

    def start(self, now: Optional[float] = None) -> bool:
        if self.phase == AlignPhase.ALIGNING:
            return False
        self.phase = AlignPhase.ALIGNING
        self._ok_frames = 0
        self._t0 = time.monotonic() if now is None else now
        return True

    def cancel(self) -> None:
        self.phase = AlignPhase.IDLE
        self._ok_frames = 0

    def on_tick(
        self,
        leader: Sequence[float],
        big: Sequence[float],
        signs: Sequence[float],
        now: Optional[float] = None,
    ) -> AlignPhase:
        if self.phase != AlignPhase.ALIGNING:
            return self.phase
        t = time.monotonic() if now is None else now
        cmd = map_leader_to_follower(leader, signs)
        if max_abs_err(cmd, big) < self.threshold_rad:
            self._ok_frames += 1
            if self._ok_frames >= self.hold_frames:
                self.phase = AlignPhase.ALIGNED
                return self.phase
        else:
            self._ok_frames = 0
        if (t - self._t0) >= self.timeout_s:
            self.phase = AlignPhase.TIMEOUT_WARN
        return self.phase
```

- [ ] **Step 5: Run tests — expect PASS**

```bash
PYTHONPATH=src/skye_follower_align python3 -m pytest src/skye_follower_align/test/test_align_logic.py -v
```

Expected: all PASS. Fix `test_timeout_warn_soft` to pass `now=self._t0 + 1` after start with `now=0.0`.

- [ ] **Step 6: Commit**

```bash
git add skye_ros2_ws/src/skye_follower_align
git commit -m "$(cat <<'EOF'
feat(align): pure follower-align state machine with tests

EOF
)"
```

---

### Task 2: Driver `SetMotionRates` service

**Files:**
- Create: `skye_ros2_ws/src/skye_robot_driver/srv/SetMotionRates.srv`
- Modify: `CMakeLists.txt` (rosidl list), `driver_core.hpp/cpp`, `driver_node.hpp/cpp`
- Modify: `launch/skye_robot_driver.launch.py` remap `/set_motion_rates` → `/gento/set_motion_rates`
- Test: extend `test/test_driver_core.cpp` if pure logic; otherwise manual service smoke after build

**Interfaces:**
- Produces: `DriverCore::set_speed_rates(left_vel, left_acc, right_vel, right_acc) -> bool`
- Service request fields: `int16 left_vel_ratio`, `left_acc_ratio`, `right_vel_ratio`, `right_acc_ratio`
- Response: `bool success`, `string message`

- [ ] **Step 1: Add srv file**

```
# SetMotionRates.srv
int16 left_vel_ratio
int16 left_acc_ratio
int16 right_vel_ratio
int16 right_acc_ratio
---
bool success
string message
```

Wire into `rosidl_generate_interfaces` next to `SetMode.srv`.

- [ ] **Step 2: Implement `DriverCore::set_speed_rates`**

```cpp
// driver_core.hpp
bool set_speed_rates(int left_vel, int left_acc, int right_vel, int right_acc);

// driver_core.cpp
bool DriverCore::set_speed_rates(int left_vel, int left_acc,
                                 int right_vel, int right_acc) {
  auto ok_r = [](int v) { return v >= 1 && v <= 100; };
  if (!ok_r(left_vel) || !ok_r(left_acc) || !ok_r(right_vel) || !ok_r(right_acc)) {
    last_error_ = "motion rates must be in [1,100]";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!linked_ || !control_ready_) {
    last_error_ = "SDK not ready for SetSpeedRatio";
    return false;
  }
  if (FX_L1_Runtime_SetSpeedRatio(
          kThreadId, FX_OBJ_ARM0, left_vel, left_acc) != 0) {
    last_error_ = "ARM0 SetSpeedRatio failed";
    return false;
  }
  if (FX_L1_Runtime_SetSpeedRatio(
          kThreadId, FX_OBJ_ARM1, right_vel, right_acc) != 0) {
    last_error_ = "ARM1 SetSpeedRatio failed";
    return false;
  }
  config_.left_vel_ratio = left_vel;
  config_.left_acc_ratio = left_acc;
  config_.right_vel_ratio = right_vel;
  config_.right_acc_ratio = right_acc;
  last_error_.clear();
  return true;
}
```

- [ ] **Step 3: Expose ROS service in `DriverNode`**

```cpp
set_motion_rates_service_ = create_service<SetMotionRates>(
    "/set_motion_rates",
    [this](const std::shared_ptr<SetMotionRates::Request> req,
           std::shared_ptr<SetMotionRates::Response> res) {
      const bool ok = core_.set_speed_rates(
          req->left_vel_ratio, req->left_acc_ratio,
          req->right_vel_ratio, req->right_acc_ratio);
      res->success = ok;
      res->message = ok ? "ok" : core_.last_error();
    });
```

Add launch remap: `("/set_motion_rates", "/gento/set_motion_rates")`.

- [ ] **Step 4: Build**

```bash
cd skye_ros2_ws && ./scripts/build.sh skye_robot_driver
```

Expected: success. Smoke (driver running, connected):

```bash
ros2 service call /gento/set_motion_rates skye_robot_driver/srv/SetMotionRates \
  "{left_vel_ratio: 10, left_acc_ratio: 10, right_vel_ratio: 10, right_acc_ratio: 10}"
```

Expected: `success: true`.

- [ ] **Step 5: Commit**

```bash
git add skye_ros2_ws/src/skye_robot_driver
git commit -m "$(cat <<'EOF'
feat(driver): add SetMotionRates service for align vel/acc

EOF
)"
```

---

### Task 3: ROS `follower_align_node`

**Files:**
- Create: `skye_follower_align/follower_align_node.py`
- Create: `launch/follower_align.launch.py`
- Modify: package entry points (already in Task 1 setup)

**Interfaces:**
- Consumes: `AlignSession`, `map_leader_to_follower`, `/gento/set_motion_rates`, `/gento/hold_current`
- Subscribes: align/cancel commands, leader states, `/gento/joint_states`
- Publishes: abs joint commands, `/align/status`

- [ ] **Step 1: Implement node skeleton**

Key behavior in timer (~`align_rate_hz`):

1. If phase ALIGNING and both leaders fresh (`age < 0.5s`) and big state fresh:
   - `cmd_l = map(leader_l, left_signs)`; same for right
   - publish `JointState` 7-position on `/gento/left_joint_control_abs` and right
   - `phase = session.on_tick(...)` using measured big arms from `/gento/joint_states` indices 0:7 and 7:14
2. On ALIGNED or TIMEOUT_WARN: stop streaming → call hold → restore rates → publish status string
3. On start: save restore rates from params; call set_motion_rates(10,10,10,10); `session.start()`
4. Missing either leader while ALIGNING: `session.cancel()`; hold; restore rates; log error
5. Status pub: map phase name to `IDLE`/`ALIGNING`/`ALIGNED`/`TIMEOUT_WARN`

Default params from spec; declare `left_joint_signs` / `right_joint_signs` as double arrays.

- [ ] **Step 2: Launch file**

```python
# launch/follower_align.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_keyboard", default_value="true"),
        DeclareLaunchArgument("robot_profile", default_value="thor"),
        Node(package="skye_follower_align", executable="follower_align_node",
             name="follower_align", output="screen",
             parameters=[{"use_sim_time": False}]),  # signs filled by OpaqueFunction in full impl
        Node(package="skye_follower_align", executable="host_keyboard_align",
             name="host_keyboard_align", output="screen",
             condition=IfCondition(LaunchConfiguration("enable_keyboard"))),
    ])
```

Use `OpaqueFunction` to load signs from profile: if `robot_profile==orin`, right signs `[1,1,1,1,1,-1,-1]`, else all `+1`.

- [ ] **Step 3: Build and import smoke**

```bash
cd skye_ros2_ws && ./scripts/build.sh skye_follower_align
source install/setup.bash
ros2 pkg executables skye_follower_align
```

Expected: lists `follower_align_node` and `host_keyboard_align`.

- [ ] **Step 4: Commit**

```bash
git add skye_ros2_ws/src/skye_follower_align
git commit -m "$(cat <<'EOF'
feat(align): ROS follower_align_node publishing abs targets

EOF
)"
```

---

### Task 4: Host keyboard + start script

**Files:**
- Create: `skye_follower_align/host_keyboard.py`
- Create: `scripts/start_follower_align.sh`

- [ ] **Step 1: Keyboard node**

```python
# Publishes String on keypress; requires TTY.
# s -> /mode/align_follower data=align_follower
# x -> /mode/align_cancel data=align_cancel
# q -> shutdown node
```

Use `sys.stdin` + `tty.setcbreak` pattern like existing HITL keyboard helpers in repo if present; print banner: `s=align x=cancel q=quit`.

- [ ] **Step 2: Start script**

```bash
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROBOT_PROFILE="${ROBOT_PROFILE:-thor}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
unset ROS_LOCALHOST_ONLY || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${REPO_ROOT}/marvin_ws/fastrtps_no_shm.xml"
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/skye_ros2_ws/install/setup.bash"
exec ros2 launch skye_follower_align follower_align.launch.py \
  robot_profile:="${ROBOT_PROFILE}" enable_keyboard:=true
```

`chmod +x scripts/start_follower_align.sh`

- [ ] **Step 3: Commit**

```bash
git add skye_ros2_ws/src/skye_follower_align scripts/start_follower_align.sh
git commit -m "$(cat <<'EOF'
feat(align): host keyboard s/x and start_follower_align.sh

EOF
)"
```

---

### Task 5: Docs + operator checklist

**Files:**
- Modify: `docs/Thor_Orin_遥操启动.md`
- Modify: `docs/ros_interfaces.md`
- Modify: `docs/superpowers/specs/2026-09-04-follower-align-after-sync-design.md` status → implemented-in-progress / ready for HW
- Optionally one paragraph in `docs/小臂大臂启动步骤.md` linking the new flow

- [ ] **Step 1: Update Thor/Orin startup**

Insert after FACTR `1` sync:

```markdown
### 对齐（FACTR sync 之后）

主机另开终端（焦点在该终端）:

```bash
ROBOT_PROFILE=orin ./scripts/start_follower_align.sh   # 或 thor
# 按 s → 大臂 10% 速度绝对跟小臂；ALIGNED / TIMEOUT_WARN 后
# Docker 再按 2 开遥操；x 取消对齐
```

等价:

```bash
ros2 topic pub --once /mode/align_follower std_msgs/msg/String "{data: align_follower}"
```
```

- [ ] **Step 2: ros_interfaces**

Document `/mode/align_follower`, `/mode/align_cancel`, `/align/status`, `/gento/set_motion_rates`.

- [ ] **Step 3: Commit**

```bash
git add docs/
git commit -m "$(cat <<'EOF'
docs: document 1 → s → 2 follower align flow

EOF
)"
```

---

### Task 6: Hardware validation (manual)

**Files:** none (ops)

- [ ] Thor: `1` → `s` → confirm slow motion + ratio 10 → `ALIGNED` → restore → `2`
- [ ] Orin: same; verify right wrist direction with orin signs
- [ ] Force timeout (lower `align_timeout_s`): `TIMEOUT_WARN` then still `2`
- [ ] `x` during align: immediate hold
- [ ] Topic-only path without keyboard

Record results in `docs/superpowers/plans/2026-09-04-follower-align-hw-checklist.md` (offline template ok).

---

## Spec coverage

| Spec item | Task |
|-----------|------|
| Big follows small abs | Task 1 + 3 |
| vel/acc 10% | Task 2 + 3 |
| Host `s` + topic | Task 3 + 4 |
| Soft timeout | Task 1 + 3 |
| No auto teleop / no FACTR edit | Task 3–5 |
| Signs per profile | Task 3 launch |
| Docs `1→s→2` | Task 5 |
| HW accept | Task 6 |

## Placeholder scan

No TBD/TODO left in steps; `align_hold_frames` default **5**, `align_timeout_s` default **10.0**, `align_rate_hz` default **50**.
