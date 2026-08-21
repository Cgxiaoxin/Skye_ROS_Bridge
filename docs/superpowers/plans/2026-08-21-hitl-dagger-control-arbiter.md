# HITL DAgger control_arbiter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在本仓库落地 `skye_hitl_dagger`：策略/遥操双源仲裁、`q`/`w` 接管交还、绝对角透传、mcap 旁路记录；纯遥操 `hitl_enable:=false` 时零插层。

**Architecture:** 新 ament_python 包承载状态机纯逻辑 + ROS 节点（arbiter / keyboard / recorder）；`skye_robot_driver` 增加 `/gento/{left,right}_joint_control_abs` 绝对入口（仍走限位与 `max_delta`）；HITL launch 把 FACTR remap 到 `/skye/teleop_*`，仅 arbiter 写执行口。

**Tech Stack:** ROS2 Humble, Python 3.10, `sensor_msgs`, `std_msgs`, 自定义 `PolicyActionChunk`, rosbag2/mcap, 现有 C++ `skye_robot_driver`

**Spec:** `docs/superpowers/specs/2026-08-21-hitl-dagger-control-arbiter-design.md`

## Global Constraints

- 包名：`skye_hitl_dagger`，路径：`skye_ros2_ws/src/skye_hitl_dagger`
- VLA **禁止**直连 `/gento/*_joint_control`
- 键盘仅 `q`=takeover、`w`=return；不占用 FACTR `1/2/3`；不用 `e` 当急停
- 接管：`AUTONOMOUS → HANDOVER_SYNC → HUMAN`；交还仅手动 `w`
- Chunk：size=16，step0=当前时刻，绝对角 rad，左右同包；夹爪电机语义 0=开 1=闭
- 无新 chunk：hold 末步，不自动切人
- `hitl_enable:=false`：不启 arbiter/recorder，FACTR 直连 `/gento/*`
- 本 plan **不含** P6.5 训练加权、不含 VLA 推理进程本体
- 绝对角方案锁定：**driver 新增 `*_joint_control_abs` topic**（spec §5.2 选项 2 变体，与 relative 入口并存）
- 策略夹爪：arbiter 写现有 `/left|right_teleop_gripper/ctrl` 时，若 driver `gripper_invert:=true`，则发布 `1 - motor`，使 invert 后等于电机目标（避免改夹爪 invert 路径）

---

## File map

| File | Role |
|------|------|
| `skye_ros2_ws/src/skye_hitl_dagger/package.xml` | 包元数据 |
| `skye_ros2_ws/src/skye_hitl_dagger/setup.py` + `setup.cfg` + `resource/skye_hitl_dagger` | ament_python 安装 |
| `skye_ros2_ws/src/skye_hitl_dagger/msg/PolicyActionChunk.msg` | 策略 chunk |
| `skye_ros2_ws/src/skye_hitl_dagger/msg/ControlMode.msg` | 模式广播 |
| `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/control_mode.py` | 状态机纯逻辑 |
| `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/chunk_player.py` | chunk 时间轴展开 / hold 末步 |
| `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/control_arbiter_node.py` | 仲裁 ROS 节点 |
| `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/hitl_keyboard_node.py` | `q`/`w` → intervention |
| `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/episode_recorder_node.py` | mcap 旁路录制 |
| `skye_ros2_ws/src/skye_hitl_dagger/launch/hitl_dagger.launch.py` | HITL on 启动 |
| `skye_ros2_ws/src/skye_hitl_dagger/test/test_control_mode.py` | 状态机单测 |
| `skye_ros2_ws/src/skye_hitl_dagger/test/test_chunk_player.py` | chunk 展开单测 |
| `skye_ros2_ws/src/skye_robot_driver/.../driver_node.hpp/.cpp` | 订 `*_joint_control_abs` |
| `skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py` | abs remap |
| `docs/ros_interfaces.md` | 文档更新 |
| `marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py` | FACTR→teleop 支路 remap |

---

### Task 1: 包骨架 + `PolicyActionChunk` / `ControlMode` 消息

**Files:**
- Create: `skye_ros2_ws/src/skye_hitl_dagger/package.xml`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/setup.py`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/setup.cfg`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/resource/skye_hitl_dagger`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/__init__.py`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/msg/PolicyActionChunk.msg`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/msg/ControlMode.msg`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/CMakeLists.txt`（若用 ament_cmake 生成接口；见下）

**Interfaces:**
- Produces: `skye_hitl_dagger/msg/PolicyActionChunk`, `skye_hitl_dagger/msg/ControlMode`

**Note:** 自定义 `.msg` 需要 `ament_cmake` + `rosidl`。采用 **ament_cmake 主包 + 安装 Python 节点**（与常见混合包一致），或纯 cmake 包内 `install(PROGRAMS ...)`。本任务用混合：`CMakeLists.txt` 生成接口并 `install` Python 模块。

- [ ] **Step 1: 创建消息定义**

`msg/PolicyActionChunk.msg`:
```text
std_msgs/Header header
string policy_version
uint32 chunk_size
float64 dt
float64[] left_joints
float64[] right_joints
float64[] left_gripper
float64[] right_gripper
```

`msg/ControlMode.msg`:
```text
std_msgs/Header header
string mode
string source
string policy_version
```

`mode` 取值：`AUTONOMOUS` | `HANDOVER_SYNC` | `HUMAN`  
`source` 取值：`policy` | `teleop` | `hold`

- [ ] **Step 2: 写 package.xml / CMakeLists.txt**

`package.xml` 关键依赖：`rclpy`, `sensor_msgs`, `std_msgs`, `rosidl_default_generators`, `rosbag2_py`, `launch_ros`。

`CMakeLists.txt` 最小骨架：
```cmake
cmake_minimum_required(VERSION 3.8)
project(skye_hitl_dagger)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolicyActionChunk.msg"
  "msg/ControlMode.msg"
  DEPENDENCIES std_msgs
)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 3: 编译接口**

Run:
```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skye_hitl_dagger --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 interface show skye_hitl_dagger/msg/PolicyActionChunk
```
Expected: 打印字段列表，含 `chunk_size`, `left_joints` 等。

- [ ] **Step 4: Commit**

```bash
git add skye_ros2_ws/src/skye_hitl_dagger
git commit -m "feat(hitl): scaffold skye_hitl_dagger package and action msgs"
```

---

### Task 2: 状态机纯逻辑（无 ROS）

**Files:**
- Create: `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/control_mode.py`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/test/test_control_mode.py`

**Interfaces:**
- Produces:
  - `class ControlModeState: AUTONOMOUS, HANDOVER_SYNC, HUMAN`
  - `class ControlArbiterLogic` with methods:
    - `mode() -> ControlModeState`
    - `request_takeover() -> bool`
    - `sync_completed() -> bool`
    - `request_return() -> bool`
    - `active_source() -> str`  # `policy` | `teleop` | `hold`

- [ ] **Step 1: 写失败单测**

```python
# test/test_control_mode.py
from skye_hitl_dagger.control_mode import ControlArbiterLogic, ControlModeState

def test_starts_autonomous():
    logic = ControlArbiterLogic()
    assert logic.mode() == ControlModeState.AUTONOMOUS
    assert logic.active_source() == "policy"

def test_takeover_enters_handover_then_human():
    logic = ControlArbiterLogic()
    assert logic.request_takeover() is True
    assert logic.mode() == ControlModeState.HANDOVER_SYNC
    assert logic.active_source() == "hold"
    assert logic.sync_completed() is True
    assert logic.mode() == ControlModeState.HUMAN
    assert logic.active_source() == "teleop"

def test_return_only_from_human():
    logic = ControlArbiterLogic()
    assert logic.request_return() is False
    logic.request_takeover()
    logic.sync_completed()
    assert logic.request_return() is True
    assert logic.mode() == ControlModeState.AUTONOMOUS

def test_takeover_ignored_when_not_autonomous():
    logic = ControlArbiterLogic()
    logic.request_takeover()
    assert logic.request_takeover() is False
```

- [ ] **Step 2: 跑测确认失败**

Run:
```bash
cd skye_ros2_ws
source install/setup.bash
python3 -m pytest src/skye_hitl_dagger/test/test_control_mode.py -v
```
Expected: FAIL（模块不存在或类未定义）

- [ ] **Step 3: 最小实现**

```python
# skye_hitl_dagger/control_mode.py
from enum import Enum, auto

class ControlModeState(Enum):
    AUTONOMOUS = auto()
    HANDOVER_SYNC = auto()
    HUMAN = auto()

class ControlArbiterLogic:
    def __init__(self) -> None:
        self._mode = ControlModeState.AUTONOMOUS

    def mode(self) -> ControlModeState:
        return self._mode

    def active_source(self) -> str:
        if self._mode == ControlModeState.AUTONOMOUS:
            return "policy"
        if self._mode == ControlModeState.HANDOVER_SYNC:
            return "hold"
        return "teleop"

    def request_takeover(self) -> bool:
        if self._mode != ControlModeState.AUTONOMOUS:
            return False
        self._mode = ControlModeState.HANDOVER_SYNC
        return True

    def sync_completed(self) -> bool:
        if self._mode != ControlModeState.HANDOVER_SYNC:
            return False
        self._mode = ControlModeState.HUMAN
        return True

    def request_return(self) -> bool:
        if self._mode != ControlModeState.HUMAN:
            return False
        self._mode = ControlModeState.AUTONOMOUS
        return True
```

- [ ] **Step 4: 跑测通过**

Run: 同 Step 2  
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/control_mode.py \
        skye_ros2_ws/src/skye_hitl_dagger/test/test_control_mode.py
git commit -m "feat(hitl): add control arbiter state machine logic"
```

---

### Task 3: Chunk 时间轴展开（无 ROS）

**Files:**
- Create: `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/chunk_player.py`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/test/test_chunk_player.py`

**Interfaces:**
- Produces:
  - `class ChunkPlayer`
  - `load(chunk_size, dt, t0, left_joints, right_joints, left_gripper, right_gripper) -> bool`
  - `sample(t_now: float) -> dict | None` with keys `left`, `right`, `left_gripper`, `right_gripper`, `holding_tail: bool`
  - Invalid size → `load` returns False，保持上一有效 chunk

- [x] **Step 1: 写失败单测**

```python
from skye_hitl_dagger.chunk_player import ChunkPlayer

def _flat(steps, dof, fill):
    return [float(fill)] * (steps * dof)

def test_step0_at_t0():
    p = ChunkPlayer()
    ok = p.load(16, 0.1, 10.0, _flat(16, 7, 1.0), _flat(16, 7, 2.0),
                [0.0]*16, [1.0]*16)
    assert ok
    s = p.sample(10.0)
    assert s["left"][0] == 1.0
    assert s["holding_tail"] is False

def test_hold_last_step_after_horizon():
    p = ChunkPlayer()
    left = _flat(16, 7, 0.0)
    left[15*7] = 3.14  # last step j1
    p.load(16, 0.1, 0.0, left, _flat(16, 7, 0.0), [0.0]*16, [0.0]*16)
    s = p.sample(10.0)
    assert s["left"][0] == 3.14
    assert s["holding_tail"] is True

def test_reject_bad_size_keeps_previous():
    p = ChunkPlayer()
    p.load(16, 0.1, 0.0, _flat(16, 7, 1.0), _flat(16, 7, 1.0), [0.0]*16, [0.0]*16)
    assert p.load(16, 0.1, 1.0, [0.0]*7, _flat(16, 7, 1.0), [0.0]*16, [0.0]*16) is False
    assert p.sample(0.0)["left"][0] == 1.0
```

- [x] **Step 2: 跑测确认失败**

Run: `python3 -m pytest src/skye_hitl_dagger/test/test_chunk_player.py -v`  
Expected: FAIL

- [x] **Step 3: 最小实现**

```python
# skye_hitl_dagger/chunk_player.py
from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional

DOF = 7
DEFAULT_STEPS = 16

@dataclass
class _Chunk:
    t0: float
    dt: float
    steps: int
    left: List[float]
    right: List[float]
    left_gripper: List[float]
    right_gripper: List[float]

class ChunkPlayer:
    def __init__(self) -> None:
        self._chunk: Optional[_Chunk] = None

    def load(self, chunk_size, dt, t0, left_joints, right_joints,
             left_gripper, right_gripper) -> bool:
        if chunk_size != DEFAULT_STEPS or dt <= 0.0:
            return False
        n = chunk_size * DOF
        if (len(left_joints) != n or len(right_joints) != n
                or len(left_gripper) != chunk_size
                or len(right_gripper) != chunk_size):
            return False
        self._chunk = _Chunk(
            t0=t0, dt=dt, steps=chunk_size,
            left=list(left_joints), right=list(right_joints),
            left_gripper=list(left_gripper),
            right_gripper=list(right_gripper),
        )
        return True

    def sample(self, t_now: float):
        if self._chunk is None:
            return None
        c = self._chunk
        if t_now <= c.t0:
            idx = 0
            holding = False
        else:
            idx = int((t_now - c.t0) / c.dt)
            holding = idx >= c.steps
            if idx >= c.steps:
                idx = c.steps - 1
        base = idx * DOF
        return {
            "left": c.left[base:base+DOF],
            "right": c.right[base:base+DOF],
            "left_gripper": c.left_gripper[idx],
            "right_gripper": c.right_gripper[idx],
            "holding_tail": holding,
        }
```

- [x] **Step 4: 跑测通过**

Expected: PASS

- [x] **Step 5: Commit**

**Report (Task 3 polish):** step index uses exact `t_now >= step_t0s[i]` (no eps); eps retained only for `holding_tail` vs `end_t` (`max(1e-12, |dt|·1e-9, |end_t|·1e-9)`). Precomputed `step_t0s`/`end_t` on load for large `t0`. 12/12 pytest pass incl. `step_t0s[1]-1e-15 → step 0`.

```bash
git commit -am "feat(hitl): add policy chunk player with hold-last-step"
```

---

### Task 4: `control_arbiter_node`（模拟策略 + 遥操选源）

**Files:**
- Create: `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/control_arbiter_node.py`
- Modify: `skye_ros2_ws/src/skye_hitl_dagger/setup.py`（entry_points）或 CMake `install(PROGRAMS)`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/scripts/control_arbiter`

**Interfaces:**
- Consumes: `ControlArbiterLogic`, `ChunkPlayer`, `PolicyActionChunk`, teleop `JointState`, `intervention_cmd`
- Produces: `/gento/left_joint_control_abs`, `/gento/right_joint_control_abs`（AUTONOMOUS/hold）, `/gento/left_joint_control`, `/gento/right_joint_control`（HUMAN）, gripper ctrls, `/skye/control_mode`

- [ ] **Step 1: 实现节点核心回调（最小可跑）**

节点参数：
- `gripper_invert_on_driver` (bool, default true) — 策略夹爪写口前做 `1-x`
- `sync_timeout_s` (float, default 5.0)
- `chunk_stale_warn_s` (float, default 1.5)
- `gripper_rate_hz` (float, default 100.0)

订阅：
- `/skye/policy_action` (`PolicyActionChunk`, BEST_EFFORT KeepLast(1))
- `/skye/teleop_action_left`, `/skye/teleop_action_right`
- `/skye/teleop_gripper_left`, `/skye/teleop_gripper_right`
- `/skye/intervention_cmd` (`std_msgs/String`: `takeover`|`return`)
- `/teleop/state`（可选，用于判定 SYNC 完成：`data == "TELEOP"` 且曾发过 sync）

发布：
- `/gento/left_joint_control_abs`, `/gento/right_joint_control_abs`
- `/gento/left_joint_control`, `/gento/right_joint_control`
- `/left_teleop_gripper/ctrl`, `/right_teleop_gripper/ctrl`
- `/mode/switch_sync`, `/mode/switch_teleop` (`std_msgs/String`)
- `/skye/control_mode` (`ControlMode`)

行为摘要：
1. `takeover` → `request_takeover()`；hold 当前采样目标到 abs；pub `switch_sync`
2. 见 `/teleop/state == TELEOP`（或超时仍 warn 不进 HUMAN）→ `sync_completed()`；pub `switch_teleop`
3. `HUMAN`：teleop JointState 回调里直接转发到 `/gento/*_joint_control`；夹爪原样转发 teleop_gripper
4. `AUTONOMOUS`：定时器 ~250 Hz 调 `chunk_player.sample(now)`，发 abs；夹爪独立 100 Hz，motor→`1-x` 若 invert
5. `holding_tail` 持续超过 `chunk_stale_warn_s` → WARN throttle

在 `control_arbiter_node.py` 中实现完整 Node 类（保持单文件 <400 行；过长则拆 `_publish_joints` 辅助函数）。

- [ ] **Step 2: 注册可执行**

`setup.py` entry_points 或：
```cmake
install(PROGRAMS
  scripts/control_arbiter
  DESTINATION lib/${PROJECT_NAME})
```

`scripts/control_arbiter`:
```python
#!/usr/bin/env python3
from skye_hitl_dagger.control_arbiter_node import main
if __name__ == "__main__":
    main()
```

- [ ] **Step 3: 编译并烟雾测（无真机）**

```bash
colcon build --packages-select skye_hitl_dagger
source install/setup.bash
ros2 run skye_hitl_dagger control_arbiter &
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: takeover}"
ros2 topic echo /skye/control_mode --once
```
Expected: `mode` 变为 `HANDOVER_SYNC`（随后若无 teleop state 可停在 SYNC）。

再用假 chunk：
```bash
# 构造最小合法 chunk 的 Python 一次性 publisher（实现时放 scripts/pub_dummy_chunk.py）
```
Expected: `/gento/left_joint_control_abs` 有流量。

- [ ] **Step 4: Commit**

```bash
git commit -am "feat(hitl): add control_arbiter ROS node"
```

---

### Task 5: HITL 键盘节点 `q`/`w`

**Files:**
- Create: `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/hitl_keyboard_node.py`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/scripts/hitl_keyboard`

**Interfaces:**
- Produces: `/skye/intervention_cmd` String `takeover`|`return`

- [ ] **Step 1: 实现**

用 `sys.stdin` 非阻塞或 `pynput`（若环境无 pynput，用 stdin：`q`/`w` + Enter 亦可；文档写清）。优先：`tty` 原始模式单字符（与 FACTR keyboard 类似）。

映射：
- `q` → `takeover`
- `w` → `return`
- 其它忽略

- [ ] **Step 2: 手动验证**

```bash
ros2 run skye_hitl_dagger hitl_keyboard
# 另开终端
ros2 topic echo /skye/intervention_cmd
# 按 q / w
```
Expected: 对应字符串。

- [ ] **Step 3: Commit**

```bash
git commit -am "feat(hitl): add q/w keyboard intervention node"
```

---

### Task 6: Driver 绝对指令入口 `*_joint_control_abs`

**Files:**
- Modify: `skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_node.hpp`
- Modify: `skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp`
- Modify: `skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py`
- Modify: `docs/ros_interfaces.md`
- Test: 扩展或新增 `skye_ros2_ws/scripts/verify_hitl_abs_interfaces.sh`

**Interfaces:**
- Consumes: `JointState` on `/left_joint_control_abs`（节点内名；launch remap 到 `/gento/left_joint_control_abs`）
- Produces: 与 relative 路径相同的 SDK `send_position`，但 **跳过** `apply_relative_joint_mapping`；仍 `clamp_to_limits` + `limit_delta`

- [ ] **Step 1: 在 `driver_node.hpp` 声明**

```cpp
void handle_absolute_command(
    DriverCore::Arm arm, const sensor_msgs::msg::JointState::SharedPtr message);
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    left_abs_command_subscription_;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    right_abs_command_subscription_;
```

- [ ] **Step 2: 实现 `handle_absolute_command`**

逻辑拷贝 `handle_command` 的校验 / clamp / limit_delta / send，但：
- **不要**走 `apply_relative_joint_mapping`
- mapped = `apply_joint_mapping(leader, order, signs, offsets)` 或直接使用 7 轴（若 order 为恒等且 signs 全 +1、offsets 0，等价于直接用 position）
- 独立 `streaming` 标志可选：与 relative 会话隔离，避免 abs 帧污染 leader_ref（推荐 abs 路径使用独立 `left_abs_streaming_` 或每次 abs 不更新 leader_ref）

关键：abs 路径 **不得**写入 `leader_ref` / `gento_ref`。

- [ ] **Step 3: 订阅与 launch remap**

构造函数内：
```cpp
left_abs_command_subscription_ = create_subscription<JointState>(
    "/left_joint_control_abs", cmd_qos,
    [this](JointState::SharedPtr msg) {
      handle_absolute_command(DriverCore::Arm::kLeft, std::move(msg));
    });
// right 同理
```

`skye_robot_driver.launch.py` 增加 remap：
```python
("/left_joint_control_abs", "/gento/left_joint_control_abs"),
("/right_joint_control_abs", "/gento/right_joint_control_abs"),
```

- [ ] **Step 4: 编译 + 接口核验**

```bash
colcon build --packages-select skye_robot_driver
source install/setup.bash
# connect_on_startup:=false
ros2 topic list | grep joint_control_abs
```
Expected: 看到 `/gento/left_joint_control_abs` 等（节点起来后）。

- [ ] **Step 5: 更新 `docs/ros_interfaces.md`**

增加 abs topic 行与一句说明：策略/HITL 绝对角入口；遥操仍用原 `*_joint_control`。

- [ ] **Step 6: Commit**

```bash
git commit -am "feat(driver): add absolute joint_control_abs command path for HITL"
```

---

### Task 7: HITL launch + FACTR remap overlay

**Files:**
- Create: `skye_ros2_ws/src/skye_hitl_dagger/launch/hitl_dagger.launch.py`
- Create: `marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py`
- Modify: `docs/superpowers/specs/2026-08-21-hitl-dagger-control-arbiter-design.md`（状态改为「实现中」）

**Interfaces:**
- `hitl_enable` 由是否 include 本 launch 表达；日常仍用原 `start_teleop_m6_dual_gento.launch.py`

- [ ] **Step 1: 复制 FACTR launch 并改 remap**

相对原 `start_teleop_m6_dual_gento.launch.py`：
```python
("/joint_control", "/skye/teleop_action_left"),   # left node
("/gripper/ctrl", "/skye/teleop_gripper_left"),
# right 对称 → _right
```
`/joint_state` 仍订 `/gento/joint_states`。

- [ ] **Step 2: `hitl_dagger.launch.py`**

启动：`control_arbiter`, `hitl_keyboard`,（可选）`episode_recorder`。  
参数透传 `gripper_invert_on_driver:=true`。

- [ ] **Step 3: 文档启停段**

在设计 spec 或 `docs/小臂大臂启动步骤.md` 增加「HITL 模式」小节：先主机 driver，再 HITL FACTR launch，再 `hitl_dagger.launch.py`；日常遥操勿启 HITL launch。

- [ ] **Step 4: Commit**

```bash
git commit -am "feat(hitl): add HITL launch and FACTR teleop-branch remaps"
```

---

### Task 8: `episode_recorder`（mcap）

**Files:**
- Create: `skye_ros2_ws/src/skye_hitl_dagger/skye_hitl_dagger/episode_recorder_node.py`
- Create: `skye_ros2_ws/src/skye_hitl_dagger/scripts/episode_recorder`

**Interfaces:**
- Consumes: policy / teleop / gento cmds / joint_states / grippers / control_mode
- Produces: `episode_XXXX.mcap` under `output_dir`

- [ ] **Step 1: 实现旁路录制**

使用 `rosbag2_py.SequentialWriter`，storage_id=`mcap`。  
参数：`output_dir`, `topics`（字符串数组，带默认列表）。  
键盘或服务：`start`/`stop` 可用 `std_srvs/Trigger`：`/skye/recorder/start`, `/skye/recorder/stop`。

默认 topics：
```text
/skye/policy_action
/skye/teleop_action_left
/skye/teleop_action_right
/skye/control_mode
/gento/left_joint_control
/gento/right_joint_control
/gento/left_joint_control_abs
/gento/right_joint_control_abs
/gento/joint_states
/left_teleop_gripper/ctrl
/right_teleop_gripper/ctrl
```

不 resample；原样按到达写入。

- [ ] **Step 2: 烟雾测**

```bash
ros2 run skye_hitl_dagger episode_recorder --ros-args -p output_dir:=/tmp/hitl_bags
ros2 service call /skye/recorder/start std_srvs/srv/Trigger {}
# pub 若干 fake topics
ros2 service call /skye/recorder/stop std_srvs/srv/Trigger {}
ls /tmp/hitl_bags
```
Expected: 生成 `.mcap` 文件。

- [ ] **Step 3: Commit**

```bash
git commit -am "feat(hitl): add mcap episode_recorder node"
```

---

### Task 9: 联调验收脚本 + 文档收尾

**Files:**
- Create: `skye_ros2_ws/scripts/verify_hitl_p61_interfaces.sh`
- Modify: `docs/ros_interfaces.md`
- Modify: `docs/HITL_DAgger_集成方案.md`（P6 状态）
- Modify: `docs/superpowers/specs/2026-08-21-hitl-dagger-control-arbiter-design.md`（状态：P6.1–P6.3 已实现待真机）

- [ ] **Step 1: 写 `verify_hitl_p61_interfaces.sh`**

无真机：起 arbiter + 假 chunk publisher + echo control_mode + 检查 abs topic hz。  
断言：`takeover` 后 mode 含 `HANDOVER`；假 chunk 时 abs 有消息。

- [ ] **Step 2: 跑脚本**

Expected: exit 0

- [ ] **Step 3: 真机清单（人工，不自动化）**

写入 spec §12 对照 checklist 到 `docs/小臂大臂启动步骤.md` HITL 节：
1. HITL off 遥操 hz 对比
2. `q` SYNC→HUMAN
3. `w` 回 AUTONOMOUS
4. 断 chunk hold
5. mcap 含 control_mode

- [ ] **Step 4: Commit**

```bash
git commit -am "docs(hitl): add P6.1 verify script and HITL runbook notes"
```

---

### Task 10:（可选本 plan 末）假 VLA publisher 工具

**Files:**
- Create: `skye_ros2_ws/src/skye_hitl_dagger/scripts/pub_dummy_policy_chunk.py`

- [ ] **Step 1: 实现按 `dt` 循环发布 hold 姿态 chunk（从 `/gento/joint_states` 读当前角填满 16 步）**

供无真 VLA 时做 P6.4 前联调。真 VLA 接入仅需改发布源到 `/skye/policy_action`，协议已定。

- [ ] **Step 2: Commit**

```bash
git commit -am "feat(hitl): add dummy policy chunk publisher for bench tests"
```

---

## Spec coverage checklist（自检）

| Spec 项 | Task |
|---------|------|
| 新包 `skye_hitl_dagger` | 1 |
| PolicyActionChunk / control_mode | 1, 2, 4 |
| 状态机 SYNC→HUMAN，`q`/`w` | 2, 4, 5 |
| Chunk 展开 + hold 末步 | 3, 4 |
| 绝对角不经 relative | 6 |
| HITL off 零插层 | 7（沿用旧 launch） |
| 夹爪策略语义 / 双写隔离 | 4, 7 |
| mcap recorder | 8 |
| 验收 | 9, 10 |
| P6.5 训练加权 | **排除**（另 plan） |
| 真 VLA 进程 | **排除**（协议 + Task 10 占位） |

## Placeholder scan

无 TBD；绝对角与夹爪 invert 旁路已在 Global Constraints 锁定。

---

## Execution handoff

Plan complete and saved to `docs/superpowers/plans/2026-08-21-hitl-dagger-control-arbiter.md`.

**Two execution options:**

1. **Subagent-Driven (recommended)** — 每 Task 新开子代理，任务间审查，迭代快  
2. **Inline Execution** — 本会话按 `executing-plans` 连续执行并设检查点  

Which approach?
