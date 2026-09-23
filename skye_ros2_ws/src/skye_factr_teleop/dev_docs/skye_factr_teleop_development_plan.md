# `skye_factr_teleop` 开发计划

日期：2026-09-16

架构依据：[`skye_factr_teleop_reimplementation_architecture.md`](./skye_factr_teleop_reimplementation_architecture.md)

## 1. 目标

在以下目录实现一个独立的 ROS 2 Humble 包：

```text
/home/lyy/projects/leader_for_skye/skye_ros2_ws/src/skye_factr_teleop
```

该包以开源 `FACTR_Teleop` 的 Apache-2.0 源码为基线，实现：

- 通过 Python `dynamixel_sdk` 驱动 7 个 leader 关节和 1 个 leader 夹爪舵机；
- leader 重力补偿、摩擦补偿、限位、零空间和可选力反馈；
- 大臂到小臂的自动同步；
- `IDLE → TELEOP_SYNCING → SYNCED → TELEOP → TELEPOSE` 状态机；
- 左右小臂独立实例和 Skye/Gento ROS 2 接口；
- 夹爪遥操及反馈；
- 不再从 `marvin_ws` 加载代码、配置、URDF 或运行资源。

## 2. 已确定的实现边界

### 2.1 技术选择

- ROS 2 Humble；
- `ament_python`；
- Python `dynamixel_sdk`；
- NumPy；
- Pinocchio；
- Ruckig Python API；
- pytest；
- 普通 Python 源码，不使用 Cython、pybind11 或新的二进制扩展。

### 2.2 上游参考

```text
仓库：/home/lyy/projects/FACTR_Teleop
提交：7a07ab3629af03a91c0198df7c44a0082dca77ab
许可：Apache-2.0
```

直接参考：

- `factr_teleop/dynamixel/driver.py`；
- `factr_teleop/factr_teleop.py`；
- `factr_teleop/factr_teleop_franka_zmq.py` 的抽象通信模式；
- 上游 YAML 分组方式和 `ament_python` 包结构。

不迁移：

- ZMQ 和 Franka 专用代码；
- `bc`、`cameras`、`python_utils`；
- 上游内嵌的 `dynamixel_sdk` 副本；
- Franka URDF。

### 2.3 与现有包的边界

`skye_factr_teleop` 负责：

```text
大臂状态 → 小臂同步
小臂状态 → 大臂遥操命令
leader 小臂的力矩控制
leader 夹爪和 follower 夹爪映射
```

`skye_robot_driver` 继续负责：

```text
Gento SDK 连接
大臂状态发布
relative/absolute 命令映射
大臂限位、单周期步长、超时和停止
大臂夹爪驱动
```

`skye_follower_align` 保持独立且可选。它实现的是小臂到大臂的绝对残差对齐，方向与
`skye_factr_teleop` 的 `TELEOP_SYNCING` 相反：

```text
skye_factr_teleop TELEOP_SYNCING：大臂 → 小臂
skye_follower_align：             小臂 → 大臂
```

第一版不把 `skye_follower_align` 合并进新包。

## 3. 计划交付结构

```text
skye_ros2_ws/src/skye_factr_teleop/
├── LICENSE
├── README.md
├── THIRD_PARTY_NOTICES.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── skye_factr_teleop
├── skye_factr_teleop/
│   ├── __init__.py
│   ├── config.py
│   ├── dynamixel_driver.py      # 真实 Python dynamixel_sdk 适配器
│   ├── control_core.py
│   ├── state_machine.py
│   ├── ros_driver.py
│   └── keyboard_control.py
├── config/
│   ├── grav_comp_m6_left.yaml
│   └── grav_comp_m6_right.yaml
├── launch/
│   ├── start_teleop_m6_left.launch.py
│   ├── start_teleop_m6_right.launch.py
│   └── start_teleop_m6_dual.launch.py
└── test/
    ├── test_config.py
    ├── test_dynamixel_driver.py
    ├── test_control_core.py
    ├── test_state_machine.py
    └── test_ros_interfaces.py
```

为彻底脱离 `marvin_ws`，还需要一个描述包：

```text
skye_ros2_ws/src/skye_factr_description/
├── CMakeLists.txt
├── LICENSE
├── package.xml
├── urdf/
└── meshes/
```

描述包的创建以 URDF 和 mesh 的来源、版权及再分发许可确认完成为前提。

## 4. 阶段 0：冻结契约和依赖基线

### 任务 0.1：记录源码来源和许可

阶段 0 不创建 `skye_factr_teleop` 功能包，也不要求包内文件存在。该阶段只生成一份
仓库级基线记录：

```text
skye_ros2_ws/src/skye_factr_teleop/dev_docs/skye_factr_teleop_phase0_baseline.md
```

该记录包含：

- 上游仓库 URL 和参考提交；
- 上游 Apache-2.0 许可证结论；
- 新 ROS 包采用 Apache-2.0 的决定；
- 计划迁移、修改和不迁移的上游文件清单；
- 源文件版权头及 Skye/M6 修改声明模板；
- 阶段 1 需要创建的 `LICENSE`、`THIRD_PARTY_NOTICES.md` 和 `package.xml`
  许可字段清单。

验收：

- 基线记录可以追溯计划迁移代码的上游来源；
- 明确区分仓库根目录 MIT 和新包 Apache-2.0；
- 不依赖尚未创建的功能包目录或文件。

### 任务 0.2：冻结 ROS 接口表

以现有 `skye_robot_driver` 为准，确认：

| 新节点接口 | 对接接口 |
|---|---|
| `/joint_state` | 左实例 remap 到 `/gento/left_joint_states`，右实例 remap 到 `/gento/right_joint_states` |
| `/joint_control` | remap 到 `/gento/{left,right}_joint_control` |
| `/gripper/ctrl` | 左实例 remap 到 `/left_teleop_gripper/ctrl`，右实例 remap 到 `/right_teleop_gripper/ctrl` |
| `/gripper/state` | 左实例 remap 到 `/left_gripper/state`，右实例 remap 到 `/right_gripper/state` |
| `/leader_arm/current_state` | remap 到左右独立 leader 状态话题 |
| `/leader_arm/target_joint_state` | remap 到左右独立 leader 目标话题 |
| `/joint_move` | remap 到 `/left_joint_move`、`/right_joint_move` |

同步优先订阅驱动已经提供的单臂 7 轴状态话题，不再让每侧节点共同消费 14 轴状态后
依赖 `follower_joint_offset` 选择实时控制输入。`follower_joint_offset` 只保留给 14
自由度 follower 动力学模型使用。

模式话题保持兼容：

```text
/mode/switch_sync
/mode/switch_teleop
/mode/switch_stop
/mode/switch_telepose
/enable_position_sync
```

验收：

- 在阶段 0 基线记录中形成一份明确的话题类型、QoS、长度、单位表；
- 与 `docs/ros_interfaces.md` 和 `skye_robot_driver` launch remap 一致；
- 不存在左右节点误读对侧状态的可能。

### 任务 0.3：依赖可用性门禁

`skye_factr_teleop` 最终运行在主机，与主机上的 `skye_robot_driver` 通过 ROS 2 直接
通信。Docker 不是新节点的目标运行环境，也不作为依赖验收环境。

当前主机状态：

```text
numpy          已安装
PyYAML         已安装
rclpy          已安装
dynamixel_sdk  已安装
pinocchio      未安装或当前 Python 路径不可见
ruckig         未安装或当前 Python 路径不可见
```

实施前确认：

- 主机 ROS 2 Humble 使用的 Python 可执行文件和版本；
- 主机构建、测试和运行是否统一使用 `/usr/bin/python3`；
- Pinocchio 在主机 Python 3.10 中的安装来源和导入路径；
- Ruckig 在主机 Python 3.10 中的安装来源和版本；
- Python `dynamixel_sdk` 在主机实际运行解释器中可导入；
- 对应 rosdep key 是否存在；
- 主机用户是否具有 `/dev/serial/by-id/*` 和 `/dev/ttyUSB*` 访问权限；
- 主机启动 shell 在 source `/opt/ros/humble/setup.bash` 和
  `skye_ros2_ws/install/setup.bash` 后可以导入全部运行依赖。

确认结果写入阶段 0 基线记录，包括主机上已安装、缺失、安装来源、目标版本、Python
解释器和验证命令。旧 FACTR Docker 中的依赖只能作为版本参考，不能作为新节点依赖
可用的验收证据。

不在源码中硬编码 `/opt/openrobots/...`，也不依赖容器内路径或 `/marvin_ws` 环境。
如果主机依赖位于非默认路径，由主机环境或启动脚本统一配置，而不是由控制代码修改
`sys.path`。

主机依赖门禁的建议验证形式：

```bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/leader_for_skye/skye_ros2_ws/install/setup.bash
/usr/bin/python3 -c \
  "import rclpy, numpy, yaml, dynamixel_sdk, pinocchio, ruckig"
```

该命令在主机成功后，才认为运行依赖门禁通过。

阶段完成条件：`skye_ros2_ws/src/skye_factr_teleop/dev_docs/skye_factr_teleop_phase0_baseline.md`
已完成并覆盖许可、ROS
契约和依赖方案；此时仍不要求 `skye_factr_teleop` 功能包存在。

## 5. 阶段 1：包骨架、配置和描述资源

### 任务 1.1：创建 `ament_python` 包骨架

创建：

- `LICENSE`：根据阶段 0 基线放入 Apache-2.0 全文；
- `THIRD_PARTY_NOTICES.md`：写入 FACTR 上游 URL、提交和版权；
- `package.xml`；
- `setup.py`；
- `setup.cfg`；
- `resource/skye_factr_teleop`；
- Python 包和空模块；
- launch/config 安装规则；
- 基础驱动 console entry point。键盘入口在基础硬件验收完成后再加入。

入口：

```text
skye_factr_teleop_driver = skye_factr_teleop.ros_driver:main
```

阶段 1 将阶段 0 的决定落实为功能包文件：

- `package.xml` 使用 `<license>Apache-2.0</license>`；
- 从上游迁移的文件保留原版权头；
- 修改文件加入 Skye/M6 修改说明；
- 不将仓库根目录 MIT 许可证替代上游 Apache-2.0 声明。

验收：

- 包骨架内不存在来源不明的复制代码；
- 包内许可文件与阶段 0 基线一致；
- `package.xml`、`LICENSE` 和第三方声明相互一致。

### 任务 1.2：实现配置模型

`config.py` 负责：

- 加载 YAML；
- 标量和逐关节数组标准化；
- 校验 7 个臂关节和 1 个夹爪舵机；
- 校验所有浮点值有限；
- 校验关节上下限和安全裕量；
- 校验 servo type、joint sign、增益数组长度；
- 校验控制频率、同步超时和 Ruckig/hold 参数；
- 将配置转换为只读 dataclass，避免控制周期反复访问字典。

迁移左右 M6 YAML 时：

- 保留已验证参数值和左右差异；
- 去除 `/marvin_ws` 绝对路径；
- URDF 使用包名和相对资源名；
- 不自动把 `dynamixel.joint_signs` 当成大臂语义映射。

测试：

- 左右配置均可加载；
- 缺字段、错误数组长度、NaN、上下限反转必须失败；
- 标量增益和 7 元素增益被规范成一致内部形式；
- 夹爪相关 8 元素参数长度正确。

### 任务 1.3：建立描述包

将以下资源迁移到 `skye_factr_description`：

- 左 leader URDF；
- 右 leader URDF；
- 必需 mesh；
- follower 动力学所需 URDF，或改为依赖已有 Skye 描述包。

要求：

- URDF 中 mesh URI 使用 `package://`；
- 不引用 `marvin_m6_description` 或 `m6_isomorphism_description`；
- Pinocchio 可以通过解析后的绝对路径加载；
- description 包可以单独构建和安装。

阶段验收：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
colcon build --packages-select skye_factr_description skye_factr_teleop \
  --symlink-install
```

并确认：

```bash
ros2 pkg prefix skye_factr_teleop
ros2 pkg prefix skye_factr_description
ros2 pkg executables skye_factr_teleop
```

## 6. 阶段 2：Dynamixel 硬件抽象层

真实 Dynamixel SDK 适配代码明确实现在：

```text
skye_ros2_ws/src/skye_factr_teleop/
└── skye_factr_teleop/
    └── dynamixel_driver.py
```

该文件是生产代码，不是 mock。它直接调用系统安装的 Python `dynamixel_sdk`，负责
串口、Protocol 2.0、同步读写、工作模式、torque enable 和硬件错误处理。测试中的
fake/mock 只替代 SDK 对象，不替代或绕过这个生产适配器的逻辑。

### 任务 2.1：先建立 SDK mock 测试

在 `test_dynamixel_driver.py` 中构造假的：

- `PortHandler`；
- `PacketHandler`；
- `GroupSyncRead`；
- `GroupSyncWrite`。

测试先覆盖预期交互，再实现驱动。

### 任务 2.2：实现常量和换算

实现并测试：

- 32 位无符号寄存器到有符号位置/速度；
- 16 位目标电流字节编码；
- tick 到 rad；
- velocity unit 到 rad/s；
- torque Nm 到目标电流；
- 舵机型号别名规范化；
- 每型号电流限制。

力矩—电流换算值需要单独确认来源，不仅因为它们曾出现在二进制模块公开常量中就视为
硬件真值。代码中记录单位和来源。

### 任务 2.3：实现真实 Python SDK 适配器和驱动生命周期

实现文件：

```text
skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/dynamixel_driver.py
```

生产代码直接导入：

```python
from dynamixel_sdk import (
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)
```

该文件包含：

- `DynamixelIO`：供控制核心依赖的 `Protocol`；
- `DynamixelDriver`：真实 SDK 适配实现；
- Dynamixel Control Table 地址和长度常量；
- 有符号寄存器、rad、rad/s、mA、Nm 换算函数；
- `/dev/serial/by-id` 解析和 FTDI latency timer 只读检查；
- 带设备 ID、操作名和 SDK 错误文本的驱动异常。

真实对象初始化关系：

```python
self._port_handler = PortHandler(port)
self._packet_handler = PacketHandler(2.0)
self._sync_read = GroupSyncRead(
    self._port_handler,
    self._packet_handler,
    128,  # Present Velocity
    8,    # Velocity 4 bytes + Position 4 bytes
)
self._sync_write_current = GroupSyncWrite(
    self._port_handler,
    self._packet_handler,
    102,  # Goal Current
    2,
)
```

`DynamixelDriver` 对外实现：

```python
open()
close()
set_operating_mode()
verify_operating_mode()
set_torque_enabled()
read_state()
write_currents()
```

各方法对应的真实 SDK 调用：

| 方法 | Python `dynamixel_sdk` 调用 |
|---|---|
| `open()` | `PortHandler.openPort()`、`setBaudRate()`、`PacketHandler.ping()`、`GroupSyncRead.addParam()` |
| `set_operating_mode()` | 对每个 ID 调用 `write1ByteTxRx(..., address=11, ...)` |
| `verify_operating_mode()` | 对每个 ID 调用 `read1ByteTxRx(..., address=11)` |
| `set_torque_enabled()` | 对每个 ID 调用 `write1ByteTxRx(..., address=64, ...)` |
| `read_state()` | `GroupSyncRead.txRxPacket()`、`isAvailable()`、`getData()` |
| `write_currents()` | `GroupSyncWrite.addParam()`、`txPacket()`、`clearParam()` |
| `close()` | 写零电流、Torque Disable、`PortHandler.closePort()` |

`open()` 的真实初始化顺序：

```text
解析并检查串口
  → 检查 FTDI latency timer
  → openPort
  → setBaudRate(4 Mbps)
  → ping 预期 ID 1..8
  → 为 GroupSyncRead 注册所有 ID
  → Torque Disable
  → 写 Current Control Mode
  → 回读验证 Current Control Mode
  → Torque Enable
```

行为要求：

- `open()` 幂等或明确拒绝重复打开；
- 所有总线访问共用一把锁；
- 打开失败时释放已获得资源；
- 只允许 Torque Disable 状态切换 Operating Mode；
- 开启 torque 前回读并确认 Current Control Mode；
- 同步读取采用有界重试而非递归；
- 写入前检查长度、有限值和电流范围；
- `GroupSyncWrite.clearParam()` 放在 `finally` 路径；
- `close()` 前尝试写零电流和关闭 torque；
- 通信结果和舵机错误使用包含 ID/操作名的异常。

### 任务 2.4：实现 `DynamixelIO` 和 fake

`DynamixelIO` 定义在生产文件 `dynamixel_driver.py` 中，控制核心只依赖这个协议。
`FakeDynamixelIO` 只放在测试代码或测试 fixture 中，支持：

- 注入位置和速度；
- 记录最近电流命令；
- 模拟超时和通信失败；
- 模拟 torque 状态。

阶段验收：

```bash
cd skye_ros2_ws
PYTHONPATH=src/skye_factr_teleop \
  python3 -m pytest src/skye_factr_teleop/test/test_dynamixel_driver.py -v
```

本阶段禁止连接真实硬件。

这里的“禁止连接真实硬件”只限制阶段 2 的自动化测试方式，不表示使用假的生产实现。
阶段 2 完成时，真实 `DynamixelDriver` 已完整实现，并通过 mock SDK 验证调用和错误
路径。阶段 2.5 先完成通信只读验收和低电流无硬件准备；单舵机低电流真机验收延期到
阶段 7，在任何基础硬件运动验收之前执行。

## 7. 阶段 2.5：真实 Dynamixel 驱动冒烟验收

本阶段始于真实 SDK 适配器完成之后。控制核心开发前完成只读验收和低电流无硬件准备；
真实单舵机低电流验收安排在阶段 7 的最前面。目的不是验证遥操算法，而是证明
`dynamixel_driver.py` 能在真实串口、真实总线和真实舵机上安全工作。

硬件操作必须按顺序执行，通信只读未通过时不得进入低电流写入。

### 2.5.1 通信只读验证

- 断开大臂运动链路；
- 确认左右 USB by-id 对应关系；
- 确认 ID 1–8 唯一且顺序正确；
- 确认 4 Mbps；
- 确认 Return Delay Time 为 0；
- 确认 FTDI latency timer 为 1；
- 使用生产文件 `dynamixel_driver.py` 打开真实串口并 ping 全部预期 ID；
- 只读取位置和速度，不开启 torque；
- 验证连续多帧读取、寄存器符号转换和单位范围；
- 验证关节方向和连续角度；
- 拔掉或断开单个设备时，确认错误包含设备 ID 和操作名，且进程可安全退出。

### 2.5.2 单舵机低电流验证

- 每次只连接或只使能一个舵机；
- 使用独立冒烟工具或最小调用入口直接调用 `DynamixelDriver`，不启动控制核心或 ROS
  遥操节点；
- 使用严格低电流限制，并在开始前记录型号、ID 和限制值；
- 验证 Torque Disable → Current Control Mode → Torque Enable 的真实执行顺序；
- 分别发送短时小幅正电流、零电流和负电流；
- 验证正负电流方向；
- 验证 XM430/XC330 电流编码、换算和限幅；
- 验证 `write_currents()` 后同步写参数被清理；
- 验证正常关闭、异常退出和通信失败路径最终写零电流并 Torque Disable。

### 2.5.3 冒烟验收记录

状态：`PENDING`。本节的真实硬件记录和阶段 2.5 完成条件延期至阶段 7 任务 7.1；
阶段 3--7 不以 `PREPARED` 参数预演记录冒充真机验收记录。

每侧至少记录：

```text
USB by-id
舵机型号与 ID
波特率和 latency timer
只读采样结果
低电流限制与测试方向
Torque Disable 结果
异常路径结果
```

阶段完成条件：

- 左右总线只读验证通过；
- 实际使用的 XM430 和 XC330 型号至少各完成一个单舵机低电流验证；
- 生产 `DynamixelDriver` 无需为硬件测试另写旁路实现；
- 所有测试结束后舵机处于 Torque Disable；
- 验收记录已保存。

阶段 2.5.2 进度（2026-09-23）：

- [x] 完成[低电流验证清单](skye_factr_teleop_phase2_5_low_current_checklist.md)第 1--3 节的静态限制核对、无硬件测试和参数预演；
- [ ] 第 4--7 节现场确认、真实单舵机验证、通过条件判定和验收记录仍为 `PENDING`，
  延期至阶段 7 任务 7.1；
- 预演记录为 `PREPARED`，不计入上述真实硬件验收完成条件。

阶段 2.5.3 进度（2026-09-23）：

- [ ] 左右总线只读结果、XM430/XC330 单舵机低电流结果、安全关闭及本节所列现场信息
  汇总成真机冒烟验收记录：`PENDING`，延期至阶段 7 任务 7.1；
- [ ] 本节的阶段 2.5 完成条件：`PENDING`，以阶段 7 任务 7.1 的真实验收结果判定。

阶段 3--6 只推进无硬件实现和测试。阶段 2.5 的完整硬件完成条件延期到阶段 7
任务 7.1 关闭；在此之前不执行阶段 7 的基础硬件运动验收。

阶段 2.5 不验证 leader 重力补偿、Ruckig、同步、遥操或双臂运动。

## 8. 阶段 3：无 ROS 控制核心

### 任务 3.1：迁移状态变换与标定

从上游迁移并重构：

- Dynamixel 多圈角度零位选择；
- `joint_offsets`；
- `joint_signs`；
- leader 关节/夹爪状态拆分；
- 逻辑关节力矩到物理舵机方向映射。

标定算法应成为纯函数或可注入步骤，不能在构造函数中启动阻塞循环。

### 任务 3.2：迁移控制律

依次实现：

1. 关节限位屏障；
2. leader 重力补偿；
3. 静摩擦补偿；
4. 可选零空间调节；
5. 基础总力矩合成和电流下发。

本阶段只保留 follower 力矩反馈和夹爪反馈的接口位置，不实现对应算法；两者在基础
硬件验收完成后进入阶段 8。

每个控制项均：

- 独立可开关；
- 输入维度明确；
- 不原地修改调用者数组；
- 对 NaN/Inf 明确失败；
- 使用配置中的逐关节增益；
- 具有固定输入/固定输出单元测试。

### 任务 3.3：隔离 Pinocchio

虽然保持精简目录，不新增 `dynamics.py`，但在 `control_core.py` 内建立小型动力学接口：

```python
gravity(q) -> ndarray
jacobian(q) -> ndarray
forward_kinematics(q) -> pose
```

测试使用 fake dynamics；Pinocchio 只在运行适配器中出现。这样在主机尚未安装
Pinocchio 时，纯控制测试仍可运行。

### 任务 3.4：实现单周期控制 API

控制核心提供非阻塞接口，例如：

```text
read leader state
compute one control step
write one current command
return observable state/result
```

不得在控制核心中调用 `rclpy.spin()`、sleep 或等待 ROS 消息。

阶段验收：

- 使用 Fake Dynamixel 和 Fake Dynamics 连续运行多个控制周期；
- 每个力矩项可单独验证；
- torque disable 时不会写电流；
- 异常输入不会留下非零输出；
- 不需要 ROS、串口、Pinocchio 或 Ruckig即可完成核心单元测试。

阶段 3 无硬件实现进度（2026-09-23）：

- [x] 3.1 显式纯函数标定、7+1 状态拆分与物理方向映射；
- [x] 3.2 限位、重力、静摩擦、可选零空间和限流电流合成；follower 力矩与夹爪反馈
  仅保留接口，算法仍归阶段 8；
- [x] 3.3 动力学接口隔离，纯控制测试不导入 Pinocchio；
- [x] 3.4 单周期非阻塞 API 与 fake hardware/dynamics 测试；真实硬件运动验收仍为
  `PENDING`，不得据此越过阶段 7 任务 7.1 门禁。

## 9. 阶段 4：轨迹、保持和纯状态机

### 任务 4.1：冻结状态转换规则

实现前把以下行为写成参数化测试：

| 当前状态 | 事件 | 结果 |
|---|---|---|
| `IDLE` | `switch_sync` | `TELEOP_SYNCING` |
| `TELEOP_SYNCING` | Ruckig 完成 | `SYNCED` |
| `TELEOP_SYNCING` | 超时 | 按兼容契约进入 `SYNCED` 并产生 warning，或在实现前明确改为 `IDLE` |
| `SYNCED` | `switch_teleop` | `TELEOP` |
| 非 `SYNCED` | `switch_teleop` | 拒绝 |
| 任意活动状态 | `switch_stop` | `IDLE` |
| `SYNCED`/`TELEOP_SYNCING` | `switch_sync` | 忽略 |

同步超时语义必须在编码前确定。原扩展公开文档表示“Ruckig 完成或超时后进入
SYNCED”，安全性上更保守的方案是超时回到 `IDLE`；若改变兼容语义，需要在 README
和迁移说明中明确记录。

基础状态机只实现 `IDLE`、`TELEOP_SYNCING`、`SYNCED` 和 `TELEOP`。
`TELEPOSE=5` 的兼容枚举值可以预留，但 `switch_telepose` 转换和行为在基础硬件验收
完成后进入阶段 8。

### 任务 4.2：实现 `state_machine.py`

状态机：

- 不导入 `rclpy`；
- 不访问串口；
- 不直接调用 Ruckig；
- 使用显式事件和动作返回值；
- 时间由调用者传入，测试不依赖真实时钟；
- 记录拒绝原因和状态进入时间。

### 任务 4.3：实现 Ruckig 同步和保持

在 `control_core.py` 中封装：

- 以 leader 当前 8 轴状态作为轨迹起点；
- 以 follower 7 轴位置和夹爪 setpoint 作为目标；
- 每周期更新一次，不使用阻塞循环；
- Ruckig 输出通过 PD/重力补偿转为 leader 电流；
- 同步完成后进入 hold；
- hold 使用配置中的 P/I/D、积分阈值和积分限幅；
- stop 清除轨迹、hold 积分和旧目标。

### 任务 4.4：测试状态与轨迹联动

覆盖：

- sync 起点捕获；
- 目标维度和有限值验证；
- Ruckig 完成；
- 超时；
- sync 中 stop；
- hold 积分清零；
- 重复 sync/teleop 请求；
- 进入 TELEOP 后不再输出 leader 自动走位目标。

阶段验收：所有状态机和轨迹测试使用 fake clock、fake Ruckig 和 fake hardware 通过。

## 10. 阶段 5：基础 ROS 2 驱动节点和 launch

### 任务 5.1：实现 `ros_driver.py`

实现：

- 参数声明和 YAML 路径解析；
- left/right 节点名支持；
- follower 7 轴状态订阅；
- `/joint_move` 订阅；
- leader 当前状态和目标状态发布；
- TELEOP 关节命令发布；
- 基础模式话题订阅：sync、teleop、stop；
- `/teleop/state` 状态发布；
- 定时控制循环；
- SIGINT/异常关闭时零电流、Torque Disable、关闭串口。

所有 JointState 输入至少验证：

- position 长度；
- velocity/effort 可选长度；
- 所有数值有限；
- 消息新鲜度；
- 左右侧期望名称或固定顺序。

QoS 与 `skye_robot_driver` 对齐：

- state：RELIABLE、KEEP_LAST 1、VOLATILE；
- control：与驱动 BEST_EFFORT 订阅兼容，KEEP_LAST 1；
- 状态文本可以使用 TRANSIENT_LOCAL，便于后启动观察者读取当前模式。

### 任务 5.2：实现基础 launch

三个 launch：

- 左臂；
- 右臂；
- 双臂。

要求：

- 配置通过包 share 查找；
- 支持 `robot_profile:=thor|orin`；
- 左右节点使用相同 executable；
- remap 到 `/gento/left_joint_states` 和 `/gento/right_joint_states`；
- 不出现 `/marvin_ws`；
- 不修改 `PYTHONPATH` 指向 `/opt/openrobots`；
- 不启动键盘节点。基础验收阶段通过 ROS CLI 发布模式请求。

### 任务 5.3：ROS 接口测试

测试内容：

- 节点在 fake hardware 模式或依赖注入下可实例化；
- publisher/subscriber 的类型与名称正确；
- 左右 remap 不串线；
- TELEOP 只发布 7 个有限 position；
- stop 后停止遥操发布；
- 状态话题反映状态机转换。

阶段验收：

```bash
cd skye_ros2_ws
colcon build --packages-select skye_factr_teleop --symlink-install
colcon test --packages-select skye_factr_teleop
colcon test-result --verbose
```

## 11. 阶段 6：基础功能无硬件集成验证

### 任务 6.1：全包测试

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
colcon build --packages-select skye_factr_description skye_factr_teleop \
  --symlink-install
colcon test --packages-select skye_factr_teleop
colcon test-result --verbose
```

### 任务 6.2：静态资源验证

- `ros2 pkg prefix` 可以找到两个新包；
- config、launch、URDF 和 mesh 均已安装；
- 对源码和安装产物执行搜索，不存在 `/marvin_ws`；
- 不存在对旧 `factr_teleop` Python 模块的导入；
- 不存在对上游 `python_utils` 和 ZMQ 的依赖。

### 任务 6.3：ROS 图验证

在不连接硬件的测试模式下启动左右节点，验证：

- 两个节点名唯一；
- 每侧只订阅自己的 follower 7 轴状态；
- 模式话题同时到达左右节点；
- leader current/target 话题左右独立；
- `/joint_control` remap 正确；
- QoS 与 `skye_robot_driver` 匹配。

模式切换通过 ROS CLI 驱动，不依赖键盘节点：

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String "{data: switch_sync}"
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String "{data: switch_teleop}"
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String "{data: switch_stop}"
```

### 任务 6.4：性能测量

在目标运行环境先测量无硬件的软件周期和计算耗时：

- 500 Hz 定时器实际周期分布；
- Pinocchio/Ruckig 单周期耗时；
- ROS 回调占用；
- 无硬件模拟下的丢帧和最长控制间隔。

Dynamixel 真实同步读取、同步写入耗时和连接硬件后的控制周期分布，在阶段 7 任务 7.1
通过后再测量。

软件性能指标在阶段 6 实测后冻结，硬件通信和控制周期指标在阶段 7 实测后冻结。
若 Python 单线程不能稳定达到要求，先优化回调和分配；只有有数据证明仍不足时，才
重新评估 C++/扩展模块，不在第一版提前引入。

## 12. 阶段 7：延期真机冒烟与基础功能分级硬件验收

本阶段按 7.1 → 7.2--7.5 的顺序执行。阶段 2.5.1 已完成真实 Dynamixel
只读验收；阶段 2.5.2 真机验证与 2.5.3 冒烟验收记录仍为 `PENDING`。
任一硬件门禁失败即停止后续硬件动作，不以 `PREPARED` 记录代替真实验收。

### 7.1 完成阶段 2.5.2 真机验证与 2.5.3 冒烟验收记录

状态：`PENDING`。

执行[阶段 2.5.2 低电流验证清单](skye_factr_teleop_phase2_5_low_current_checklist.md)
第 4--7 节。若 USB、接线或配置与阶段 2.5.1 记录相比已改变，先重新完成受影响侧的
只读验收；任何低电流写入之前都必须确认当前侧只读通信通过。

左侧 XM430-W350 和右侧 XC330-T288 至少各完成一台单舵机低电流测试，保存型号、
ID、USB by-id、波特率、latency timer、只读采样结果、限值、正负电流方向、异常路径
和 Torque Disable 结果。两侧均通过且阶段 2.5 完成条件全部满足后，才进入 7.2。

下面的方案用于验收 leader 重力补偿、关节同步、关节遥操和双臂基础链路。
模式请求统一使用 ROS CLI。键盘、follower 重力估计、增强夹爪反馈和 TELEPOSE 均不
在本阶段实现或启用。

### 7.2 单臂重力补偿

- 无负载、清场、急停可用；
- 先禁用零空间、反馈和自动同步；
- 逐关节验证 gravity gain 和 joint sign；
- 验证限位屏障方向；
- 验证 stop/进程退出行为。

### 7.3 单臂同步

- 大臂保持不动；
- 通过 ROS CLI 发布 `/mode/switch_sync` 进入 TELEOP_SYNCING；
- 小臂低速对齐大臂；
- 检查 Ruckig 速度/加速度限制；
- 检查完成、超时和通过 `/mode/switch_stop` 中止；
- 确认同步过程中没有向大臂发布 teleop 命令。

### 7.4 单臂遥操

- 只有 SYNCED 后通过 `/mode/switch_teleop` 才进入 TELEOP；
- 小幅单关节运动；
- 验证 `/gento/{side}_joint_control` 方向、长度和频率；
- 验证 `skye_robot_driver` relative clutch、限位和超时；
- 发布 `/mode/switch_stop` 后调用或确认大臂受控保持语义。

### 7.5 双臂基础验收

- 左右单臂全部通过后才启动双臂；
- 验证 USB、节点、话题和参数不交叉；
- 通过 ROS CLI 发布 sync/teleop/stop 请求时左右状态一致；
- 任一侧错误时明确报告，不静默继续未知状态；
- 根据需要保留 `skye_follower_align` 作为同步后的可选残差对齐步骤。

阶段 7 完成标志：阶段 2.5.2 真机验证和 2.5.3 冒烟记录有效，且 7.2--7.5 的
左右单臂与双臂关节遥操链路全部通过并保存验收记录。只有达到此标志，才开始阶段 8。

## 13. 阶段 8：基础硬件验收后的扩展功能

### 任务 8.1：实现键盘控制

新增 `keyboard_control.py` 和 console entry point：

```text
skye_factr_keyboard = skye_factr_teleop.keyboard_control:main
```

保留操作语义：

```text
1 → /mode/switch_sync
2 → /mode/switch_teleop
3 → /mode/switch_stop
q → 退出
```

要求：

- 优先 pynput/X11；
- 回退 evdev；
- 输入映射提取为纯函数并测试；
- 键盘节点只发布模式请求，不直接发布运动命令；
- launch 增加 `use_keyboard` 参数，默认值在硬件回归前保持 `false`。

### 任务 8.2：实现 follower 重力估计

实现：

- 从 Skye description 包加载 14 自由度 follower 模型；
- 左臂使用 offset 0，右臂使用 offset 7；
- 用零速度、零加速度 RNEA 计算 follower 重力；
- 若 JointState 提供有效 effort，计算：

  ```text
  tau_external = tau_reported - tau_follower_gravity
  ```

- 若当前 `skye_robot_driver` 未发布臂关节 effort，则 torque feedback 保持关闭，并给出
  明确诊断；不得将零 effort 当成真实外力矩。

### 任务 8.3：实现夹爪命令和增强反馈

对接现有 Skye driver 的：

```text
/left_teleop_gripper/ctrl
/right_teleop_gripper/ctrl
/left_gripper/state
/right_gripper/state
```

当前驱动 JointState 已提供 position、velocity、effort；若 `frame_id=no_feedback`，新包应
禁用该帧反馈力矩，只保留位置目标映射。

实现并测试：

- 夹爪位置映射和范围；
- effort threshold；
- EMA；
- 夹紧方向 fast alpha；
- 速度阻尼；
- 左右 invert 由大臂驱动负责，新包输出 FACTR 规范化夹爪值，不重复反相。

### 任务 8.4：实现 TELEPOSE

- 先冻结 Pose 消息和坐标系；
- 明确 follower 是否存在对应接收接口；
- 测试坐标变换和速度变换；
- 如果当前 Skye driver 没有可达的 Cartesian teleop 接口，则保留状态和计算能力，但不
  在默认 launch 中启用 TELEPOSE。

阶段验收：阶段 2.5.2 真机验证、2.5.3 冒烟记录与基础硬件验收均通过；四项扩展均可
单独关闭，全部关闭时已验收的同步和关节遥操行为不发生变化。

## 14. 阶段 9：扩展功能回归和硬件验收

### 9.1 键盘回归

- `1/2/3` 与此前 ROS CLI 请求产生相同状态转换；
- 左右节点同时收到请求；
- 键盘节点退出不影响控制节点安全关闭。

### 9.2 follower 力反馈回归

- 先使用录制或模拟 effort 验证重力扣除；
- 再进行单臂低增益接触测试；
- 未收到有效 effort 时保持反馈关闭；
- 不因反馈功能改变基础关节命令映射。

### 9.3 夹爪回归

- 先验证位置方向和范围；
- 再低增益开启 effort feedback；
- `no_feedback` 时不得产生触觉力矩；
- 分别验证 Thor/Orin 夹爪 profile。

### 9.4 TELEPOSE 回归

- 先使用模拟 follower 验证坐标变换；
- 只有存在已验收的 Cartesian 下游接口时才连接真实大臂；
- TELEPOSE 退出后能够回到受控停止状态。

### 9.5 最终双臂回归

- 使用键盘完成双臂 `1/2/3` 工作流；
- 夹爪、关节遥操和可选反馈能够分别启停；
- 任意扩展关闭后，基础链路继续工作；
- 保存最终验收记录和运行参数。

## 15. 测试矩阵

| 层级 | 无硬件 | 串口 | 小臂 | 大臂 | 主要工具 |
|---|---:|---:|---:|---:|---|
| 配置 | 是 | 否 | 否 | 否 | pytest |
| Dynamixel 换算 | 是 | 否 | 否 | 否 | pytest + mock SDK |
| Dynamixel 通信 | 否 | 是 | 是 | 否 | 只读/低电流脚本 |
| 控制律 | 是 | 否 | 否 | 否 | pytest + fake dynamics |
| 状态机 | 是 | 否 | 否 | 否 | pytest + fake clock |
| ROS 接口 | 是 | 否 | 否 | 可 fake | launch test/ROS CLI |
| leader 重力补偿 | 否 | 是 | 是 | 否 | 单臂硬件测试 |
| 同步 | 否 | 是 | 是 | 状态输入 | 单臂硬件测试 |
| 遥操 | 否 | 是 | 是 | 是 | 单臂后双臂测试 |
| 键盘 | 是 | 否 | 否 | 否 | pytest + ROS topic 观察 |
| follower 重力/力反馈 | 先是 | 是 | 是 | effort 输入 | 模拟 effort 后单臂接触测试 |
| 夹爪反馈 | 先是 | 是 | 是 | 是 | 模拟反馈后单臂测试 |
| TELEPOSE | 先是 | 可选 | 可选 | 需笛卡尔接口 | 模拟 follower 后硬件测试 |

## 16. 里程碑与依赖关系

| 里程碑 | 内容 | 依赖 | 完成标志 |
|---|---|---|---|
| M0 | 契约、许可、依赖冻结 | 无 | 阶段 0 基线文档完成，不依赖功能包存在 |
| M1 | 包骨架、配置、description | M0 | 许可决定已落实为包文件，两包可构建，配置测试通过 |
| M2 | 真实 Python Dynamixel SDK 适配器 | M1 | `dynamixel_driver.py` 完整实现，mock SDK 调用与错误测试通过 |
| M2.5-P | Dynamixel 只读验收与低电流无硬件准备 | M2 | 左右总线只读通过，低电流清单第 1--3 节通过 |
| M3 | 控制核心 | M2.5-P | fake hardware/dynamics 测试通过，不连接真实硬件 |
| M4 | 状态机、Ruckig、hold | M3 | 全部状态与轨迹测试通过 |
| M5 | 基础 ROS 节点和 launch | M4 | ROS CLI 可驱动基础状态机 |
| M6 | 基础无硬件集成和软件性能 | M5 | colcon 全绿、依赖独立、无硬件性能记录完成 |
| M2.5-H | 阶段 2.5.2 真机验证与 2.5.3 冒烟记录（阶段 7 任务 7.1） | M6 | XM430/XC330 单舵机低电流、安全关闭和左右验收记录通过 |
| M7 | 基础分级硬件验收（阶段 7 的 7.2--7.5） | M2.5-H | 左右单臂和双臂关节链路验收完成 |
| M8 | 键盘、follower gravity、夹爪反馈、TELEPOSE | M7 | 四项扩展可独立启停并通过无硬件测试 |
| M9 | 扩展功能硬件回归 | M8 | 扩展单项和最终双臂验收记录完成 |

## 17. 需要在实施中明确的门禁

以下事项不阻止先写纯逻辑，但会阻止对应里程碑完成：

1. **URDF/mesh 许可和来源**：未确认前不能将安装产物直接复制为新描述包；
2. **Pinocchio/Ruckig 部署方式**：当前主机 Python 3.10 无法导入；
3. **同步超时语义**：兼容原行为进入 SYNCED，还是更保守地回 IDLE；
4. **follower arm effort 来源**：当前 Skye JointState 是否提供可用于外力估计的臂 effort；
5. **TELEPOSE 下游接口**：当前 Skye driver 是否存在可达的笛卡尔命令入口；
6. **力矩—电流参数来源**：按具体舵机型号和硬件实测冻结；
7. **500 Hz 实际要求**：以最终运行主机上的实测周期为准；
8. **单舵机低电流真机验收**：阶段 7 任务 7.1 完成前，不执行基础硬件运动验收或
   扩展功能硬件回归。

## 18. 完成定义

只有满足以下条件，才认为重建完成：

- `skye_factr_teleop` 和描述包可从干净的 `skye_ros2_ws` 构建；
- 运行时不引用 `marvin_ws` 或旧 `factr_teleop`；
- Apache-2.0 归属和修改说明完整；
- 配置、驱动、控制核心、状态机和 ROS 接口测试全部通过；
- 在控制核心开发前完成阶段 2.5.1 真实总线只读验收及阶段 2.5.2 第 1--3 节无硬件准备；
- 阶段 3--6 不执行真实硬件运动验收；阶段 7 先完成阶段 2.5.2 第 4--7 节真机验证
  和阶段 2.5.3 冒烟验收记录，再依次完成左右单臂和双臂基础硬件验收；
- 使用 ROS CLI 完成基础硬件验收后，才实现键盘、follower 重力估计、夹爪反馈和
  TELEPOSE；
- 左右单臂的只读、低电流、重力补偿、同步、遥操和夹爪测试通过；
- 双臂 `1/2/3` 工作流通过；
- stop、通信错误和退出路径均能使 leader 输出归零并关闭 torque；
- 与 `skye_robot_driver` 的话题、QoS、relative 映射和 gripper 语义一致；
- `skye_follower_align` 能作为独立可选步骤继续使用，不与新包命令争用；
- README 包含安装、依赖、设备配置、启动、停机和故障排查说明。

## 19. 非目标

本轮不包括：

- 反编译或反汇编三个 `.so`；
- 修改 Gento SDK；
- 重写 `skye_robot_driver`；
- 合并或删除 `skye_follower_align`；
- 引入 `ros2_control`；
- 为性能提前改写 C++；
- 自动修改系统 FTDI latency timer；
- 在未完成单臂验收前进行双臂运动测试。
