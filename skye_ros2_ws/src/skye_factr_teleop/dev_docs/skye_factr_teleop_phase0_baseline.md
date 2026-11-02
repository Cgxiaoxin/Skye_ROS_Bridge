# `skye_factr_teleop` 阶段 0 源码与许可基线

日期：2026-09-16

范围：开发计划任务 0.1；本记录不创建或依赖 `skye_factr_teleop` 功能包。

## 1. 上游来源冻结

| 项目 | 冻结值 |
|---|---|
| 上游项目 | `FACTR_Teleop` |
| 上游仓库 URL | <https://github.com/JasonJZLiu/FACTR_Teleop.git> |
| 参考提交 | `7a07ab3629af03a91c0198df7c44a0082dca77ab` |
| 提交日期 | 2025-08-03 |
| 本地核验副本 | `/home/lyy/projects/FACTR_Teleop` |
| 上游许可证 | Apache License 2.0（SPDX：`Apache-2.0`） |

参考提交是后续实现唯一的上游代码基准；本地路径只用于核验，不是运行时或构建时依赖。
可用以下命令复核来源：

```bash
git -C /home/lyy/projects/FACTR_Teleop remote get-url origin
git -C /home/lyy/projects/FACTR_Teleop rev-parse HEAD
git -C /home/lyy/projects/FACTR_Teleop show \
  7a07ab3629af03a91c0198df7c44a0082dca77ab:LICENSE
```

在该提交中，仓库根 `LICENSE` 是 Apache License 2.0 全文，
`src/factr_teleop/package.xml` 和 `src/factr_teleop/setup.py` 均声明
`Apache-2.0`。上游没有 `NOTICE` 文件。计划使用的 FACTR Python 源文件带有
“Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li”及 Apache-2.0 许可头。

## 2. 本仓库与新包的许可证边界

- `skye_ros_bridge` 仓库根目录的 `LICENSE` 是 MIT License，版权声明为
  `Copyright (c) 2026 alex_cg`；它继续适用于未另行声明许可的仓库内容。
- 计划创建的 `skye_ros2_ws/src/skye_factr_teleop/` 是独立 ROS 包。该包采用
  Apache-2.0，并在包目录内放置自己的 `LICENSE`、第三方声明和包元数据。
- 包内从 FACTR 派生的文件继续遵守 Apache-2.0，保留上游版权和许可声明，并标明
  Skye/M6 修改。新写的包内源码也按 Apache-2.0 发布。
- 因此，仓库根 MIT 与新包 Apache-2.0 是按目录和文件声明区分的，不应把根目录
  MIT 误记为 FACTR 上游或新包源码的许可证。

## 3. 上游文件处置清单

### 3.1 计划迁移并修改

不存在按原路径原样复制的上游运行文件。以下文件作为派生实现的直接源码基线，迁移
时会改名、拆分或适配 Skye/M6：

| 上游文件（均相对于参考提交） | 计划目标/用途 | 必要修改 |
|---|---|---|
| `src/factr_teleop/factr_teleop/dynamixel/driver.py` | `skye_factr_teleop/dynamixel_driver.py` | 改用主机安装的 Python `dynamixel_sdk`；保留串口、同步读写和单位换算逻辑；适配 7 个 leader 关节和 1 个夹爪舵机。保留文件中对 GELLO driver 的来源说明。 |
| `src/factr_teleop/factr_teleop/factr_teleop.py` | 主要拆入 `control_core.py`，状态相关逻辑进入 `state_machine.py` | 移除 `python_utils`、工作区探测和非 Skye 路径；适配 M6 配置、Pinocchio/Ruckig、可测试的纯 Python 状态逻辑以及 Skye 控制边界。 |
| `src/factr_teleop/factr_teleop/factr_teleop_franka_zmq.py` | 仅将其抽象通信分层模式用于 `ros_driver.py` 的设计 | 不迁移 ZMQ 地址、Franka 消息内容或 `python_utils` 代码；通信实现改为 Skye/Gento ROS 2 接口。若形成派生代码，按上游派生文件处理。 |
| `src/factr_teleop/factr_teleop/configs/*.yaml` | 仅参考分组方式，生成 `config/grav_comp_m6_{left,right}.yaml` | 参数改为 M6 左右臂实测配置；不复制 Franka 数值或资源路径。若复制了受版权保护的表达，则按派生文件处理。 |
| `src/factr_teleop/package.xml`、`setup.py`、`setup.cfg`、`resource/factr_teleop` | 新包的 `ament_python` 元数据和资源布局参考 | 包名、入口、依赖、维护者和资源清单改为 `skye_factr_teleop`；不原样复制元数据。 |

新建的 `config.py`、`keyboard_control.py`、launch 文件和测试以项目需求实现；除非实际
实现时引入上游表达，否则不把它们标成 FACTR 派生文件。每个文件的最终归类以实际
内容为准，而不是仅以目标文件名判断。

### 3.2 明确不迁移

| 上游路径/内容 | 决定 |
|---|---|
| `src/factr_teleop/factr_teleop/dynamixel/python/**` | 不迁移内嵌 Dynamixel SDK、副本许可证和示例；使用主机安装的 `dynamixel_sdk` 依赖。 |
| `src/factr_teleop/factr_teleop/factr_teleop_franka_zmq.py` 中的 ZMQ/Franka 专用实现 | 不迁移；只有上一节所述通信分层模式可供参考。 |
| `src/factr_teleop/factr_teleop/factr_teleop_grav_comp_demo.py` | 不迁移独立演示节点。 |
| `src/factr_teleop/factr_teleop/urdf/**` | 不迁移 Franka URDF 和 mesh。Skye/M6 描述资源必须另行确认来源和再分发许可。 |
| `src/factr_teleop/factr_teleop/configs/franka_example.yaml`、`grav_comp_demo.yaml` 的具体参数 | 不迁移 Franka/演示参数。 |
| `src/bc/**`、`src/cameras/**`、`src/python_utils/**` | 不迁移这些上游包。 |
| `launch/**` | 不迁移数据采集、rollout、Franka 遥操和重力补偿演示启动文件。 |
| `assets/**`、上游 README 图片、`src/factr_teleop/resource/figure_*.png` | 不迁移论文和说明材料。 |
| 上游测试模板 `src/*/test/**` | 不迁移；为新包行为编写独立测试。 |

## 4. 源文件声明模板

从 FACTR 源文件迁移或形成派生代码时，保留上游版权，增加 Skye/M6 修改声明，并使用
以下文件头（年份或版权主体发生变化时据实更新）：

```python
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li
# Modifications Copyright (c) 2026 lyy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified for Skye/M6: <brief description of the material changes>.
```

完全新写且未包含上游表达的包内 Python 文件使用：

```python
# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0
```

若文件保留了 `dynamixel/driver.py` 的相关实现，还必须保留其对 GELLO driver 的
“Based on”来源说明。若未来实际迁移其他第三方内容，必须先保留其原始声明并更新
第三方清单，不能仅套用上述模板。

## 5. 阶段 1 许可文件清单

阶段 1 创建功能包时必须同时创建并填写以下内容：

### `LICENSE`

- 放在 `skye_ros2_ws/src/skye_factr_teleop/LICENSE`；
- 内容为未经删改的 Apache License 2.0 全文；
- 不复制仓库根目录的 MIT License 作为新包许可证。

### `THIRD_PARTY_NOTICES.md`

- 标明 FACTR_Teleop 的项目名、仓库 URL、固定提交和 Apache-2.0；
- 保留 `Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li`；
- 列出实际派生的上游文件及其新文件映射；
- 记录上游 `dynamixel/driver.py` 对 GELLO driver 的来源说明；
- 说明上游参考提交没有 `NOTICE` 文件；
- 只列入实际复制或派生的内容。主机依赖应按其分发要求记录，不能把“依赖”误写成
  “已复制源码”。

### `package.xml`

- 使用 package format 3；
- 包级许可字段必须是 `<license>Apache-2.0</license>`；
- 维护者、作者及其邮箱填写新包的真实责任主体，不沿用上游占位信息；
- 依赖项按实际使用声明，许可证字段不得写成根仓库的 `MIT`，也不得笼统写
  `Apache 2.0` 等非 SPDX 形式。

完成以上文件之前，不发布或分发新包产物。

## 6. ROS 2 接口契约（任务 0.2）

本节以新 `skye_factr_teleop` 单侧节点为视角描述方向。左右节点使用相同的节点内接口，
由双臂 launch 显式 remap 到不同的系统话题。

QoS 缩写：

- `state_qos`：`KEEP_LAST(1)`、`RELIABLE`、`VOLATILE`；
- `command_qos`：`KEEP_LAST(1)`、`BEST_EFFORT`、`VOLATILE`；
- `event_qos`：`KEEP_LAST(10)`、`RELIABLE`、`VOLATILE`；
- `latched_state_qos`：`KEEP_LAST(1)`、`RELIABLE`、`TRANSIENT_LOCAL`。

### 6.1 左右实例数据接口

| 方向 | 节点内接口 | 左实例系统话题 | 右实例系统话题 | 类型 | QoS | 长度和单位 |
|---|---|---|---|---|---|---|
| 订阅 | `/joint_state` | `/gento/left_joint_states` | `/gento/right_joint_states` | `sensor_msgs/msg/JointState` | `state_qos` | `name`、`position`、`velocity` 均为 7；位置 rad，速度 rad/s；`effort` 可为空。左右名称分别为 `l_j1..l_j7`、`r_j1..r_j7`。 |
| 发布 | `/joint_control` | `/gento/left_joint_control` | `/gento/right_joint_control` | `sensor_msgs/msg/JointState` | `command_qos` | `position` 必须恰为 7 个有限值，单位 rad；驱动只以 `position` 作为 leader 遥操输入。 |
| 发布 | `/gripper/ctrl` | `/left_teleop_gripper/ctrl` | `/right_teleop_gripper/ctrl` | `sensor_msgs/msg/JointState` | `command_qos` | `position` 长度 1、无量纲归一化 `[0,1]`；FACTR 扳机语义为 `1=开`、`0=闭`。 |
| 订阅 | `/gripper/state` | `/left_gripper/state` | `/right_gripper/state` | `sensor_msgs/msg/JointState` | `state_qos` | `name=[gripper_joint]`；`position`、`velocity`、`effort` 长度各 1，均为驱动给出的归一化反馈；`position` 使用 FACTR 语义 `1=开`、`0=闭`。无硬件反馈时 `position` 是目标回显，`frame_id=no_feedback`。 |
| 发布 | `/leader_arm/current_state` | `/left_leader_arm/current_state` | `/right_leader_arm/current_state` | `sensor_msgs/msg/JointState` | `state_qos` | leader 小臂 7 轴；`position` 为 7 个 rad，`velocity` 为 7 个 rad/s，夹爪不包含在内。 |
| 发布 | `/leader_arm/target_joint_state` | `/left_leader_arm/target_joint_state` | `/right_leader_arm/target_joint_state` | `sensor_msgs/msg/JointState` | `state_qos` | leader 小臂目标；`position` 恰为 7 个有限 rad 值，夹爪不包含在内。 |
| 订阅 | `/joint_move` | `/left_joint_move` | `/right_joint_move` | `sensor_msgs/msg/JointState` | `state_qos` | 外部 leader 走位目标；`position` 恰为 7 个有限 rad 值。 |

2026-09-28 更正：target topic 的方向按任务 5.1 和架构说明改为发布；旧表中的
“订阅”为笔误，外部走位输入使用 `/joint_move`。

`/joint_control` 和 `/gripper/ctrl` 采用驱动控制 QoS，避免旧指令排队；驱动的
BEST_EFFORT 订阅也可兼容已有 RELIABLE FACTR 发布者。`/joint_state` 和
`/gripper/state` 必须使用 RELIABLE 订阅，与 `skye_robot_driver` 的 RELIABLE 状态
发布者匹配。leader 状态同样固定为 `state_qos`，以匹配现有
`skye_follower_align` 订阅者。

### 6.2 共享模式和状态接口

模式请求是有意共享的全局广播：一次键盘请求同时作用于左右实例，不做左右 remap。

| 方向 | 系统话题 | 类型 | QoS | 内容/单位 |
|---|---|---|---|---|
| 订阅 | `/mode/switch_sync` | `std_msgs/msg/String` | `event_qos` | `data="switch_sync"`；无单位，请求 `IDLE -> TELEOP_SYNCING`。 |
| 订阅 | `/mode/switch_teleop` | `std_msgs/msg/String` | `event_qos` | `data="switch_teleop"`；无单位，请求进入 `TELEOP`。 |
| 订阅 | `/mode/switch_stop` | `std_msgs/msg/String` | `event_qos` | `data="switch_stop"`；无单位，请求返回 `IDLE`。 |
| 订阅 | `/mode/switch_telepose` | `std_msgs/msg/String` | `event_qos` | `data="switch_telepose"`；无单位，请求进入 `TELEPOSE`。 |
| 发布 | `/teleop/state` | `std_msgs/msg/String` | `latched_state_qos` | 单个状态字符串：`IDLE`、`TELEOP_SYNCING`、`SYNCED`、`TELEOP` 或 `TELEPOSE`。 |

模式话题保留现有 `std_msgs/msg/String` 类型和消息值，不改为 service、枚举消息或新的
命名空间。`/teleop/state` 使用 transient-local，保证晚加入的监控或 HITL 节点能读到
最近状态。

### 6.3 单臂状态隔离规则

- 左实例只能订阅 `/gento/left_joint_states`，右实例只能订阅
  `/gento/right_joint_states`；两者输入都必须恰为 7 轴。
- 实时同步和控制路径不得订阅 `/gento/joint_states` 后再按
  `controller.force_feedback_control.joint.dynamics.follower_joint_offset` 切片。这样右实例不可能把 14 轴消息的前 7 轴误当作右臂。
- `controller.force_feedback_control.joint.dynamics.follower_joint_offset` 只用于 14 自由度 follower 动力学模型中的左右关节索引：左为
  `0`、右为 `7`；它不参与 ROS 实时状态消息选边。
- launch 必须显式列出全部左右 remap，不依赖节点名、namespace 或相对话题的隐式
  解析来区分两侧。
- 旧的 `marvin_ws/start_gento_dual_arm_sync.sh` 仍把两个 `/joint_state` 都 remap 到
  14 轴 `/gento/joint_states`，不符合本契约，不能作为新包的启动入口或接口依据；新包
  launch 必须采用本节的分侧 7 轴 remap。

### 6.4 对照依据

- `docs/ros_interfaces.md`：驱动状态/控制话题的类型、7 轴长度、rad/rad/s 单位及 QoS；
- `skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp`：
  `state_qos()`、`control_qos()`、单臂状态发布和 7 轴命令长度校验；
- `skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py`：驱动内部
  `/left|right_joint_states` 与 `/left|right_joint_control` 到 `/gento/*` 的 remap；
- `marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py`：现行 FACTR 双实例的
  分侧 7 轴状态、控制、leader 状态、走位和夹爪 remap；
- `skye_ros2_ws/src/skye_follower_align/skye_follower_align/follower_align_node.py`：
  左右 leader 状态订阅及其 `state_qos`；
- `marvin_ws/install/lib/factr_teleop/keyboard_gripper.py`：兼容模式话题类型和消息值。

## 7. 主机依赖基线（任务 0.3）

核验日期：2026-09-16。目标运行环境是 Ubuntu 22.04 amd64 主机，不是 FACTR
Docker。构建、测试和运行统一使用：

```text
/usr/bin/python3 -> /usr/bin/python3.10
Python 3.10.12
ROS_DISTRO=humble
```

`skye_ros2_ws/scripts/build.sh` 已显式使用 `/usr/bin/python3` 和
`-DPython3_EXECUTABLE=/usr/bin/python3`。新包的验证和启动也必须使用该解释器；不得
以其他 Python、Docker 内环境或旧 FACTR 安装作为通过依据。

### 7.1 已安装依赖

| 导入名 | 版本 | 安装来源 | `/usr/bin/python3` 实际导入路径 | 状态 |
|---|---:|---|---|---|
| `numpy` | 1.21.5 | Ubuntu APT：`python3-numpy` | `/usr/lib/python3/dist-packages/numpy/__init__.py` | 通过 |
| `yaml` | 5.4.1 | Ubuntu APT：`python3-yaml` | `/usr/lib/python3/dist-packages/yaml/__init__.py` | 通过 |
| `rclpy` | 3.3.22 | ROS 2 APT：`ros-humble-rclpy` | `/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py` | 通过 |
| `dynamixel_sdk` | 4.0.3 | ROS 2 APT：`ros-humble-dynamixel-sdk` | `/opt/ros/humble/local/lib/python3.10/dist-packages/dynamixel_sdk/__init__.py` | 通过 |
| `pinocchio` | 4.0.0 | ROS 2 APT：`ros-humble-pinocchio` | `/opt/ros/humble/lib/python3.10/site-packages/pinocchio/__init__.py` | 通过 |
| `ruckig` | 0.19.4 | 官方 PyPI wheel，`/usr/bin/python3 -m pip install --user ruckig==0.19.4` | `/home/lyy/.local/lib/python3.10/site-packages/ruckig.cpython-310-x86_64-linux-gnu.so` | 通过 |

Pinocchio 的目标版本冻结为 ROS Humble 仓库当前提供的 4.0.0。Ruckig Python API 的
目标版本冻结为 0.19.4，来源冻结为官方 PyPI；安装的是 CPython 3.10、Linux x86_64
manylinux wheel。该用户级安装供主机用户 `lyy` 运行新包，不代表其他系统用户也可
导入。

ROS APT 的 `ros-humble-ruckig` 0.9.2 已卸载。该包只提供 C++ 头文件和
`libruckig.so`，不提供 Python 模块；卸载不影响上表中独立安装的 PyPI `ruckig`
0.19.4。

### 7.2 rosdep 映射

| rosdep key | 解析结果 | 结论 |
|---|---|---|
| `pinocchio` | `ros-humble-pinocchio` | 可覆盖 Pinocchio Python API。 |
| `dynamixel_sdk` | `ros-humble-dynamixel-sdk` | 可覆盖 Dynamixel Python API。 |
| `rclpy` | `ros-humble-rclpy` | 可用。 |
| `python3-numpy` | `python3-numpy` | 可用。 |
| `python3-yaml` | `python3-yaml` | 可用。 |
| `ruckig` | `ros-humble-ruckig` | key 存在但当前未满足；该包只满足 C++ 库，不能满足 `import ruckig`。 |
| `python3-ruckig` | 无规则 | Python API 没有可用 rosdep key。 |

因此后续 `package.xml` 不能把 rosdep key `ruckig` 当作 Python API 已安装的证明；主机
部署说明必须单独列出官方 PyPI `ruckig==0.19.4` 前置条件。

### 7.3 运行依赖门禁

以下规定命令已在主机成功执行：

```bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/install/setup.bash
/usr/bin/python3 -c \
  "import rclpy, numpy, yaml, dynamixel_sdk, pinocchio, ruckig"
```

另已用 `/usr/bin/python3` 成功构造 `Ruckig(8, 0.002)`、`InputParameter(8)` 和
`OutputParameter(8)`。Python 运行依赖门禁通过。

不得在控制源码中追加 `/opt/openrobots/...`、`/marvin_ws` 或其他路径到
`sys.path`。ROS 和工作区路径由 source 对应 `setup.bash` 提供；Ruckig 位于 Python
3.10 的标准用户 site-packages。

### 7.4 串口设备和权限状态

2026-09-16 在主机用户 `lyy` 的新登录会话中核验：

- `id` 和 `id -nG` 均确认当前会话已经加载 `dialout`；
- `/etc/group` 的 `dialout` 成员包含 `lyy`；
- 左 leader：
  `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0`
  指向 `/dev/ttyUSB0`；
- 右 leader：
  `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0`
  指向 `/dev/ttyUSB1`；
- 两个 leader 设备节点均为字符设备，所有者/组为 `root:dialout`，权限为
  `crw-rw----`；
- 用户 `lyy` 通过稳定的 `/dev/serial/by-id/*` 链接对两个 leader 设备均可读、可写；
- 另检测到 Milsky dongle
  `/dev/serial/by-id/usb-Milsky_LL_Dongle_A730FDE8F132-if00 -> /dev/ttyACM0`，
  同样可读、可写，但它不是 leader Dynamixel 串口，不计入左右 leader 映射。

核验命令：

```bash
id
id -nG
getent group dialout
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
find /dev -maxdepth 1 -type c -name 'ttyUSB*' \
  -exec stat -Lc '%n mode=%A owner=%U group=%G' {} \;
test -r /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0
test -w /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0
test -r /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0
test -w /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0
```

一次性检查输出 `SERIAL PERMISSION GATE: PASS`。左右 leader 串口设备和主机用户权限
门禁通过；此结论只验证设备识别与文件访问权限，不会打开串口、发送 Dynamixel 指令
或驱动机械臂。
