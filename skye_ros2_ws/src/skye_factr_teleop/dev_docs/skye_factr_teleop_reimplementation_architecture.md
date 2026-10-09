# `skye_factr_teleop` 开源参考重建架构与决策记录

## 1. 背景与目标

目标是在不反编译以下二进制扩展的前提下，重新实现其功能：

```text
marvin_ws/install/lib/factr_teleop/dynamixel/driver.cpython-310-x86_64-linux-gnu.so
marvin_ws/install/lib/factr_teleop/factr_teleop.cpython-310-x86_64-linux-gnu.so
marvin_ws/install/lib/factr_teleop/factr_teleop_robot_driver_bin.cpython-310-x86_64-linux-gnu.so
```

新 ROS 2 包命名为：

```text
skye_factr_teleop
```

新实现需要脱离 `marvin_ws`，成为 Skye ROS 2 工作区中的独立源码包。

早期架构分析仅使用以下信息：

- ROS 2 package、launch、YAML 和现有设计文档；
- 现有明文 Python 启动脚本；
- Python 模块正常导入后暴露的公开类、方法、签名和文档字符串；
- ELF 类型和动态依赖等文件级元数据。

没有对三个 `.so` 进行反汇编、反编译或伪代码恢复。

后续确认三个 `.so` 是基于本地开源项目
`/home/lyy/projects/FACTR_Teleop` 修改而来。因此，新实现将以该项目的
Apache-2.0 源码为基线，并结合现有明文配置、ROS 接口契约和运行行为实现
Skye/M6 扩展。这不再属于严格意义上的 clean-room 实现，但仍坚持不对三个
`.so` 做逆向工程。

## 2. 新包放置位置

最终选择：

```text
/home/lyy/projects/leader_for_skye/skye_ros2_ws/src/skye_factr_teleop
```

不放在仓库根目录 `/home/lyy/projects/leader_for_skye`，原因是 `skye_ros2_ws` 已经是标准 colcon 工作区，且已有以下相关 ROS 2 包：

```text
skye_robot_driver
skye_follower_align
skye_hitl_dagger
skye_data_recorder
```

新包放入 `skye_ros2_ws/src` 后，可以统一构建和加载：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
colcon build --packages-select skye_factr_teleop
source install/setup.bash
```

## 3. 原三个模块的职责划分

现有公开接口显示原系统由三个层次组成：

```text
factr_teleop_robot_driver_bin
        │ ROS 2 通信和模式状态机
        ▼
factr_teleop.FACTRTeleop
        │ 控制算法、动力学和轨迹
        ▼
dynamixel.driver.DynamixelDriver
          Dynamixel 串口与同步读写
```

新实现应保留这个职责边界，但使用普通 Python 源码实现。

### 3.1 Dynamixel 硬件层

对应原文件：

```text
dynamixel/driver.cpython-310-x86_64-linux-gnu.so
```

已知公开接口包括：

```python
class DynamixelDriver:
    def __init__(
        self,
        ids: Sequence[int],
        servo_types: Sequence[str],
        port: str = "/dev/ttyUSB0",
        baudrate: int = 4_000_000,
    ): ...

    def close(self): ...
    def get_positions_and_velocities(self, tries=10): ...
    def set_current(self, currents: Sequence[float]): ...
    def set_operating_mode(self, mode: int): ...
    def verify_operating_mode(self, expected_mode: int): ...
    def set_torque_mode(self, enable: bool): ...
    def set_torque(self, torques: Sequence[float]): ...

    @property
    def torque_enabled(self) -> bool: ...
```

已知控制表地址：

```python
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
```

已知力矩到电流换算参数：

```python
TORQUE_TO_CURRENT_MAPPING = {
    "XC330-T288": 880.0,
    "XM430-W210": 766.7,
    "XM430-W350": 561.0,
}
```

该层应负责：

- 串口打开、关闭和波特率设置；
- 舵机发现及配置检查；
- Operating Mode 设置和验证；
- Torque Enable/Disable；
- 多舵机位置、速度同步读取；
- 多舵机目标电流同步写入；
- Dynamixel 原始值与 rad、rad/s、mA、Nm 的转换；
- 通信错误和舵机硬件错误报告。

### 3.2 遥操控制核心

对应原文件：

```text
factr_teleop.cpython-310-x86_64-linux-gnu.so
```

核心公开类为：

```python
class FACTRTeleop(rclpy.node.Node, ABC):
    def __init__(self, node_name="factr_teleop"): ...
```

它是可继承的控制框架，负责 leader 小臂的控制算法。具体 follower 通信由子类实现。

主要功能包括：

- 读取 leader 小臂关节位置和速度；
- leader URDF/Pinocchio 模型加载；
- follower URDF/Pinocchio 模型的可选加载；
- RNEA 重力补偿；
- 静摩擦补偿；
- 零空间调节；
- 关节限位屏障；
- follower 外力矩反馈；
- 夹爪反馈；
- 位置同步、保持和 Ruckig 自动走位；
- 正运动学和末端位姿计算；
- 合成目标力矩并转换为 Dynamixel 目标电流。

控制输出可以按以下形式组织：

```python
tau_command = (
    tau_gravity
    + tau_friction
    + tau_nullspace
    + tau_joint_limit
    + tau_force_feedback
    + tau_position_or_hold
)
```

已知主要公开方法：

```python
control_loop_callback()
gravity_compensation(q, dq)
friction_compensation(dq)
null_space_regulation(q, dq)
joint_limit_barrier(q, dq, gripper_q, gripper_dq)
torque_feedback(external_torque, dq)
joint_position_sync(q, dq, target)
move_to_configuration(target_q_full)
calculate_ee_pose(q)
set_leader_joint_torque(arm_torque, gripper_torque)
set_leader_sync(enable=False)
```

需要由具体 ROS 子类实现的通信接口包括：

```python
set_up_communication()
update_communication(leader_arm_pos, leader_gripper_pos, ee_pose)
get_follow_arm_external_joint_torque()
get_follow_gripper_feedback()
gripper_feedback(...)
```

### 3.3 ROS 2 适配和状态机层

对应原文件：

```text
factr_teleop_robot_driver_bin.cpython-310-x86_64-linux-gnu.so
```

核心公开类：

```python
class FACTRTeleopRos(FACTRTeleop):
    ...
```

该层负责：

- ROS 2 参数、publisher、subscriber 和定时器；
- follower 关节和夹爪状态缓存；
- leader 当前状态和目标状态发布；
- 遥操模式状态机；
- 小臂到大臂的关节命令发布；
- 大臂到小臂的同步目标处理；
- ROS 节点入口 `main()`。

公开状态值为：

```python
class ControlMode:
    IDLE = 0
    TELEOP_SYNCING = 1
    SYNCED = 2
    TELEOP = 3
    TELEPOSE = 5
```

以下为早期接口分析的状态转换记录。新包以开发计划任务 4.1 于 2026-09-28 冻结的
规则为准：同步超时回到 `IDLE`，只有正常完成进入 `SYNCED`；基础阶段不实现
`TELEPOSE` 转换。详见[迁移说明](skye_factr_teleop_migration.md)。

早期记录：

```text
             switch_sync
IDLE ─────────────────────────► TELEOP_SYNCING
                                      │
                                      │ Ruckig 完成或同步结束
                                      ▼
                                   SYNCED
                                      │
                                      │ switch_teleop
                                      ▼
                                   TELEOP
                                      │
                                      │ switch_telepose
                                      ▼
                                  TELEPOSE

任意活动状态 ── switch_stop ──► IDLE
```

状态语义：

- `IDLE`：停止自动走位、保持和遥操发布；
- `TELEOP_SYNCING`：根据 follower 大臂当前位置，用 Ruckig 驱动 leader 小臂对齐；
- `SYNCED`：小臂已经到位并保持，等待进入遥操；
- `TELEOP`：周期性把小臂关节位置发布给大臂；
- `TELEPOSE`：实验性的末端位姿遥操模式。

## 4. ROS 接口契约

从现有 launch、文档和公开类接口推导出的主要接口如下：

| 接口 | 方向 | 类型/含义 |
|---|---:|---|
| `/joint_state` | 订阅 | follower 大臂关节状态，`sensor_msgs/JointState` |
| `/joint_control` | 发布 | leader 到 follower 的关节目标，`sensor_msgs/JointState` |
| `/joint_move` | 订阅 | 外部指定的小臂走位目标 |
| `/gripper/state` | 订阅 | follower 夹爪状态 |
| `/gripper/ctrl` | 发布 | follower 夹爪目标 |
| `/mode/switch_sync` | 订阅 | 请求同步 |
| `/mode/switch_teleop` | 订阅 | 请求进入遥操 |
| `/mode/switch_stop` | 订阅 | 请求停止 |
| `/mode/switch_telepose` | 订阅 | 请求进入位姿遥操 |
| `/leader_arm/current_state` | 发布 | leader 小臂当前关节状态 |
| `/leader_arm/target_joint_state` | 发布 | leader 小臂走位或保持目标 |
| `/teleop/state` | 发布 | 当前遥操状态 |

左右臂使用两个相同节点实例，通过独立 YAML 配置和 ROS remap 区分。

## 5. 最终选择的新包结构

采用适合当前项目的精简目录：

```text
skye_ros2_ws/src/skye_factr_teleop/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── skye_factr_teleop
│
├── skye_factr_teleop/
│   ├── __init__.py
│   ├── config.py
│   ├── dynamixel_driver.py
│   ├── control_core.py
│   ├── state_machine.py
│   ├── ros_driver.py
│   └── keyboard_control.py
│
├── config/
│   ├── grav_comp_m6_left.yaml
│   └── grav_comp_m6_right.yaml
│
├── launch/
│   ├── start_teleop_m6_left.launch.py
│   ├── start_teleop_m6_right.launch.py
│   └── start_teleop_m6_dual.launch.py
│
└── test/
    ├── test_config.py
    ├── test_dynamixel_driver.py
    ├── test_control_core.py
    ├── test_state_machine.py
    └── test_ros_interfaces.py
```

文件与原模块的对应关系：

| 新源码 | 职责 | 对应原模块 |
|---|---|---|
| `dynamixel_driver.py` | 串口、同步读写、单位换算、工作模式 | `dynamixel/driver.so` |
| `control_core.py` | 动力学、力矩控制、反馈和轨迹 | `factr_teleop.so` |
| `state_machine.py` | 模式和状态转换 | 原 ROS 驱动中的状态逻辑 |
| `ros_driver.py` | ROS 2 通信和主入口 | `factr_teleop_robot_driver_bin.so` |
| `config.py` | YAML 加载和数据验证 | 原模块中的配置处理 |
| `keyboard_control.py` | 键盘 `1/2/3` 模式请求 | 原 `keyboard_gripper.py` |

依赖方向应保持单向：

```text
ros_driver
   ├── state_machine
   ├── control_core
   │      └── dynamixel_driver
   └── config

keyboard_control
   └── ROS 模式话题
```

其中 `state_machine.py` 应保持纯 Python，不直接依赖 `rclpy`。ROS 回调只负责将消息转换成状态机事件，便于无 ROS、无硬件单元测试。

第一版不创建 `controllers/`、`dynamics/` 或 `trajectory/` 子目录。只有当 `control_core.py` 明显过大，或者某个控制算法需要独立复用时，再进行拆分。

## 6. 构建类型和程序入口

新包采用 `ament_python`，不需要 `CMakeLists.txt`。

`setup.py` 中预计提供两个入口：

```python
entry_points={
    "console_scripts": [
        "skye_factr_teleop_driver = skye_factr_teleop.ros_driver:main",
        "skye_factr_keyboard = skye_factr_teleop.keyboard_control:main",
    ],
}
```

launch 中使用：

```python
Node(
    package="skye_factr_teleop",
    executable="skye_factr_teleop_driver",
)
```

## 7. Dynamixel SDK 技术选择

最终选择直接使用 Python `dynamixel_sdk`，第一版不引入 C++、Cython、pybind11 或 `ros2_control`。

直接导入：

```python
from dynamixel_sdk import (
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)
```

通信职责划分：

```text
PacketHandler
├── ping 和设备发现
├── Operating Mode 配置
├── Torque Enable/Disable
└── 硬件错误诊断

GroupSyncRead
└── 控制周期内批量读取 Position + Velocity

GroupSyncWrite
└── 控制周期内批量写入 Goal Current
```

XM430/XC330 的 Present Velocity 和 Present Position 地址连续：

```text
128–131  Present Velocity
132–135  Present Position
```

因此可以使用一次八字节同步读取：

```python
GroupSyncRead(
    port_handler,
    packet_handler,
    start_address=128,
    data_length=8,
)
```

控制输出使用 Goal Current：

```text
102–103  Goal Current
```

对应：

```python
GroupSyncWrite(
    port_handler,
    packet_handler,
    ADDR_GOAL_CURRENT,
    2,
)
```

初始化顺序建议为：

```text
打开串口
  → 设置波特率
  → ping 所有预期 ID
  → Torque Disable
  → 设置 Current Control Mode
  → 回读验证 Operating Mode
  → Torque Enable
  → 启动控制循环
```

退出顺序建议为：

```text
写入零目标电流
  → Torque Disable
  → 关闭串口
```

## 8. Dynamixel 驱动新接口

新驱动不必机械复制原接口名称，可以提供更清晰的生命周期：

```python
class DynamixelDriver:
    def __init__(
        self,
        ids: Sequence[int],
        servo_types: Sequence[str],
        port: str,
        baudrate: int = 4_000_000,
    ): ...

    def open(self) -> None: ...
    def close(self) -> None: ...

    def set_operating_mode(self, mode: int) -> None: ...
    def verify_operating_mode(self, expected_mode: int) -> None: ...
    def set_torque_enabled(self, enabled: bool) -> None: ...

    def read_state(self) -> tuple[np.ndarray, np.ndarray]: ...
    def write_currents(self, currents_ma: Sequence[float]) -> None: ...

    @property
    def torque_enabled(self) -> bool: ...
```

控制层不应直接依赖 `dynamixel_sdk`，而应依赖可替换协议：

```python
class DynamixelIO(Protocol):
    def read_state(self) -> tuple[np.ndarray, np.ndarray]: ...
    def write_currents(self, currents_ma: Sequence[float]) -> None: ...
    def set_torque_enabled(self, enabled: bool) -> None: ...
    def close(self) -> None: ...
```

测试时使用 `FakeDynamixelIO`，从而不需要真实串口和舵机。

## 9. 脱离 `marvin_ws` 的要求

新包不得继续引用：

```text
/marvin_ws/install/...
factr_teleop
marvin_m6_description
m6_isomorphism_description
```

需要完成以下资源迁移：

- 将左右臂 YAML 放入 `skye_factr_teleop/config/`；
- 将 leader 小臂 URDF 放入独立的 Skye 描述包；
- follower 大臂 URDF 使用 Skye 工作区内的描述包；
- 使用 `ament_index_python.get_package_share_directory()` 查找资源；
- launch、脚本和文档全部改用 `skye_factr_teleop` 包名；
- 不从 `marvin_ws/install` 加载 Python 模块或共享资源。

描述包可以按需创建为：

```text
skye_ros2_ws/src/skye_factr_description
```

建议依赖关系：

```text
skye_factr_description
        │ leader 小臂 URDF
        ▼
skye_factr_teleop
        │ 遥操关节/夹爪目标
        ▼
skye_robot_driver
```

## 10. 如何参考上游 `FACTR_Teleop`

### 10.1 上游版本和许可基线

上游本地仓库：

```text
/home/lyy/projects/FACTR_Teleop
```

Git 来源和当前参考提交：

```text
https://github.com/JasonJZLiu/FACTR_Teleop.git
7a07ab3629af03a91c0198df7c44a0082dca77ab
```

上游采用 Apache License 2.0。由于新实现会直接复制或修改部分上游源码，
`skye_factr_teleop` 建议单独采用 `Apache-2.0`，即使仓库根目录当前采用 MIT
许可证。需要：

- 在新包中保留或附带 Apache-2.0 `LICENSE`；
- 保留被复用源码原有的版权和许可头；
- 在修改过的源文件中明确标注已为 Skye/M6 修改；
- 在包 README 或 NOTICE 中记录上游仓库和参考提交；
- `package.xml` 的 `<license>` 写为 `Apache-2.0`。

最简单的做法是让整个 `skye_factr_teleop` ROS 包使用 Apache-2.0，避免同一包
内部逐文件混合许可带来的维护成本。

### 10.2 参考原则

参考工作分成三类：

1. **直接迁移后重命名**：上游已经实现且与 Skye 需求一致的代码；
2. **在上游实现上适配**：算法相同，但配置、生命周期、接口或硬件参数不同；
3. **按 Skye 契约新增**：上游不存在、只出现在 M6 魔改版本中的功能。

不会把整个上游仓库复制到 `skye_ros2_ws`。只迁移 `factr_teleop` 包中与 leader
控制直接相关的部分；`bc`、`cameras`、ZMQ 工具和上游内嵌的完整
`dynamixel_sdk` 副本不进入新包。

### 10.3 上游文件到新架构的映射

| 上游文件 | 新文件 | 参考方式 |
|---|---|---|
| `factr_teleop/dynamixel/driver.py` | `skye_factr_teleop/dynamixel_driver.py` | 作为硬件层代码基线，适配型号、错误处理和生命周期 |
| `factr_teleop/factr_teleop.py` | `skye_factr_teleop/control_core.py` | 迁移标定、关节变换和控制律，再扩展 Ruckig/保持等功能 |
| `factr_teleop/factr_teleop_franka_zmq.py` | `skye_factr_teleop/ros_driver.py` | 只参考子类化和通信钩子，不复用 ZMQ/Franka 接口 |
| `factr_teleop/factr_teleop_grav_comp_demo.py` | 测试或调试入口 | 参考最小子类和节点生命周期 |
| `configs/*.yaml` | `config/grav_comp_m6_{left,right}.yaml` | 参考配置分组结构，参数值以 M6 已验证配置为准 |
| `setup.py`、`setup.cfg`、`package.xml` | 新包同名文件 | 参考 `ament_python` 包装方式，替换包名、入口和依赖 |
| `urdf/factr_teleop_franka.urdf` | 不迁移 | Franka leader 模型不适用于 M6，同构小臂模型由 Skye 描述包提供 |

### 10.4 `dynamixel_driver.py` 如何参考

上游 `dynamixel/driver.py` 与目标 `.so` 的公开接口高度一致，可以作为最直接的
实现基线。可复用的核心逻辑包括：

- `PortHandler` 和 Protocol 2.0 `PacketHandler` 初始化；
- 从地址 128 开始一次同步读取 8 字节的速度和位置；
- 32 位有符号位置/速度转换；
- position tick 到 rad 的换算；
- velocity unit 到 rad/s 的换算；
- `GroupSyncWrite` 批量写入两字节 Goal Current；
- Nm 到目标电流的逐型号换算；
- Operating Mode 和 Torque Enable 的配置流程；
- 用 `Protocol` 隔离上层控制代码和真实驱动。

不能原样照搬的部分：

- 上游力矩—电流系数和 M6 魔改版不同，必须以实际 XM430/XC330 型号、固件控制表
  和经过验证的参数为准；
- 上游把串口打开和硬件配置放在构造函数内，新实现采用显式 `open()`/`close()`
  生命周期，便于失败清理和测试；
- 所有总线读写都应统一受锁保护，而不只是 Torque Enable；
- `set_current()` 需要先验证长度和有限值，并确保异常时清理
  `GroupSyncWrite` 参数；
- 上游递归重试同步读取，新实现应采用有界循环，并保留最后一次通信错误；
- 上游测试 `main()` 与当前构造签名不一致，不作为实现依据；
- 不复制上游 `dynamixel/python/src/dynamixel_sdk`，新包直接依赖系统安装的 Python
  `dynamixel_sdk`。

### 10.5 `control_core.py` 如何参考

上游 `factr_teleop.py` 是控制核心的主要来源。优先迁移并测试：

1. Dynamixel 连续角度的零位标定算法；
2. `joint_offsets` 和 `joint_signs` 的坐标变换；
3. leader 关节位置、速度和夹爪状态读取；
4. 关节力矩乘以物理舵机方向后下发；
5. Pinocchio RNEA 重力补偿；
6. 静摩擦抖动补偿；
7. Jacobian 零空间投影；
8. 关节限位屏障；
9. follower 外力矩到 leader 反馈力矩的映射；
10. 控制周期内的力矩项合成顺序。

迁移时需要做的结构调整：

- 上游 `FACTRTeleop` 同时继承 `Node` 和 `ABC`；新架构中尽量让
  `control_core.py` 不继承 `rclpy.node.Node`，由 `ros_driver.py` 持有控制核心；
- 上游通过 `python_utils.get_workspace_root()` 拼接源码路径，新实现通过
  `get_package_share_directory()` 或构造参数注入资源路径；
- 上游使用阻塞 `while` 完成初始姿态匹配，新实现用状态机和周期更新，不能阻塞
  ROS executor；
- 上游参数多为标量，M6 配置中的 gain、damping 和 motor scalar 支持逐关节数组；
- 上游 gripper 只有 `actuation_range`，M6 需要 `limit_min`、`limit_setpoint`、
  `limit_max` 以及独立反馈参数；
- 上游默认始终执行零空间调节，新实现遵守 YAML 中的 `enable`；
- 上游没有 `controller.limit_barrier_control.enable`，新实现应显式支持；
- 上游只加载 leader 动力学，新实现还要按配置加载 follower URDF，并计算
  `tau_external = tau_reported - tau_follower_gravity`；
- 上游没有 Ruckig、走位/保持控制和相关积分项，这些属于 Skye 新增实现；
- 上游抽象方法中使用 `get_leader_arm_external_joint_torque`，目标公开接口使用
  `get_follow_arm_external_joint_torque`，新实现统一使用 follower 命名。

### 10.6 `state_machine.py` 如何参考

上游项目没有目标系统所需的模式状态机，因此本文件不能从上游直接迁移。
它应按照现有 M6 明文契约独立实现：

```text
IDLE → TELEOP_SYNCING → SYNCED → TELEOP → TELEPOSE
  ▲                                      │
  └────────────── switch_stop ───────────┘
```

实现时参考：

- 现有键盘节点对按键 `1/2/3` 的定义；
- 现有 launch 中的 topic remap；
- 当前设计文档记录的状态转换；
- `.so` 正常导入后公开的 `ControlMode` 值和回调方法文档字符串。

状态机保持纯 Python，只接收事件、当前时间和必要条件，输出动作；不直接发布 ROS
消息，也不直接访问 Dynamixel。

### 10.7 `ros_driver.py` 如何参考

上游 `FACTRTeleopFrankaZMQ` 只用于理解以下扩展模式：

- 控制核心通过通信钩子与 follower 解耦；
- `JointState` 消息构造；
- follower 状态缓存；
- follower 外力矩和夹爪反馈的获取；
- `main()` 中的 ROS 初始化、spin 和关闭流程。

以下内容不迁移：

- `python_utils` 依赖；
- `ZMQPublisher`、`ZMQSubscriber`；
- Franka 地址和 `/franka/*` 话题；
- behavior cloning 数据转发职责。

Skye ROS 适配层按已确定的话题契约重新实现，并连接
`skye_robot_driver`：

```text
/gento/joint_states
/gento/left_joint_control
/gento/right_joint_control
/left_joint_move
/right_joint_move
/mode/switch_sync
/mode/switch_teleop
/mode/switch_stop
/teleop/state
```

左右实例共享同一个 `ros_driver.py`，通过 YAML 参数、节点名和 remap 区分，不复制
左右两套控制代码。

### 10.8 配置和 URDF 如何参考

上游 YAML 曾使用四个顶层分组：

```text
dynamixel
arm_teleop
gripper_teleop
controller
```

当前 M6 配置改用顶层 `dynamixel`、`controller` 和可选的 `runtime`。`initialization` 与 `end_effector` 位于 `controller` 下，紧随 `null_space_regulation`；夹爪上下限与独立增益在 `controller.limit_barrier_control`，默认同步目标在 `controller.sync_hold_control.gripper_setpoint`；leader URDF 在 `controller.joint_gravity_comp.dynamics`，从臂模型参数在 `controller.force_feedback_control.joint.dynamics`，从臂重力补偿开关在 `controller.force_feedback_control.joint`。不再使用顶层 `arm_teleop` 和 `gripper_teleop`。

但参数值不从 Franka 示例继承。左右 M6 的端口、型号、方向、关节限制、增益、
follower offset 和夹爪参数，应以当前已验证的 M6 YAML 为迁移输入，再由
`config.py` 做类型、长度、有限值和范围检查。

上游 `factr_teleop_franka.urdf` 只描述 Franka 对应的 leader，不迁移到新包。
Skye/M6 leader URDF 和 mesh 应进入 `skye_factr_description`，follower URDF 使用
Skye 工作区的机器人描述包。控制核心只接收解析后的资源路径，不感知工作区目录。

### 10.9 测试参考策略

上游测试主要是 lint 和版权检查，不能作为行为回归测试直接使用。新测试以迁移后的
源码逻辑和 Skye 契约为依据：

- `test_dynamixel_driver.py`：mock SDK，覆盖符号转换、单位换算、同步读写、错误与清理；
- `test_control_core.py`：固定关节输入，覆盖各力矩项和总输出，不依赖 ROS；
- `test_state_machine.py`：覆盖允许和拒绝的全部状态转换、同步完成和超时；
- `test_config.py`：覆盖左右 YAML、逐关节参数维度、型号和关节范围；
- `test_ros_interfaces.py`：验证 topic 类型、remap、左右实例和消息长度。

硬件验证顺序沿用上游的设备准备原则：ID 1–8、4 Mbps、Return Delay Time 0、
FTDI latency timer 1；但实际电流限制、方向和力矩系数必须按 M6 小臂逐关节验证。

### 10.10 推荐的实施顺序

```text
第一步：迁移包骨架、许可证和配置模型
  ↓
第二步：基于上游 driver.py 实现 dynamixel_driver.py，并用 mock SDK 测试
  ↓
第三步：迁移上游控制律到 control_core.py，并去除 ROS/路径耦合
  ↓
第四步：实现纯 Python state_machine.py
  ↓
第五步：实现 Skye ros_driver.py、左右 remap 和键盘节点
  ↓
第六步：加入 follower 重力估计、Ruckig 同步/保持和增强夹爪反馈
  ↓
第七步：无硬件 ROS 集成测试
  ↓
第八步：单臂低电流硬件验证，再进行双臂验证
```

该顺序先建立可从开源源码验证的基线，再逐项加入魔改功能。每个魔改功能都设置独立配置项和回归测试。实现以上游开源源码、M6 配置、公开接口及原 .so 的可观察行为为依据，在不反编译的前提下实现功能等价替代。

## 11. 当前决策汇总

已经确定：

1. 不反编译原三个 `.so`；
2. 新包命名为 `skye_factr_teleop`；
3. 放入 `skye_ros2_ws/src/skye_factr_teleop`；
4. 使用精简的单 ROS 包、分层源码结构；
5. 使用 `ament_python`；
6. 直接使用 Python `dynamixel_sdk`；
7. 实时通信主要采用 `GroupSyncRead` 和 `GroupSyncWrite`；
8. 初始化、模式配置和诊断采用 `PacketHandler`；
9. 状态机与 ROS 通信分离；
10. 控制核心通过 `DynamixelIO` 接口与硬件层解耦；
11. 第一版不引入 C++、Cython、pybind11 或 `ros2_control`；
12. 第一版不对控制器模块进行过度拆分。
13. 以上游 `FACTR_Teleop` 提交
    `7a07ab3629af03a91c0198df7c44a0082dca77ab` 作为源码参考基线；
14. 新包建议采用 Apache-2.0，并保留上游版权、许可和修改说明；
15. 直接迁移上游 Dynamixel 和控制核心的适用部分，Skye 状态机、ROS 接口、
    Ruckig、follower 重力估计和增强夹爪反馈单独实现；
16. 不复制上游内嵌的 `dynamixel_sdk`，继续使用系统安装的 Python SDK。

当前尚未创建 `skye_factr_teleop` ROS 包，也尚未实现或修改控制代码。
