# Skye 遥操桥接交接文档

本文档用于“公共记忆”。后续如果要继续接手这个项目，优先先看这里，再看 `STARTUP.md` 和 `README.md`。

## 1. 当前目标

目标是把同构遥操小臂的关节角输入，接入 Skye 工控机上的 ROS 2 控制栈，用于双臂 + 夹爪遥操作和数采。

当前采用的正式方案是 **方案 B**：

```text
遥操小臂
  -> skye_leader_bridge
  -> /control/model/joint_cmd_A, /control/model/joint_cmd_B
  -> /joint_cmd_mux
  -> /control/joint_cmd_A, /control/joint_cmd_B
  -> /marvin_robot_node
```

夹爪仍然直接走：

```text
/left_teleop_gripper/ctrl  -> /control/gripperValueL -> /dm_gripper_motor_node
/right_teleop_gripper/ctrl -> /control/gripperValueR -> /dm_gripper_motor_node
```

## 2. 代码位置

开发包已经放在 Skye 工控机上：

```text
/opt/kernelmind/skye_leader_bridge_ws
```

本地开发版源代码来源于：

```text
/data/coding/tianji/Skye-mutile-arm/skye_leader_bridge_ws
```

包名：

```text
skye_leader_bridge
```

主要文件：

- `skye_leader_bridge/skye_leader_bridge/node.py`
- `skye_leader_bridge/skye_leader_bridge/prepare.py`
- `skye_leader_bridge/config/skye_leader_bridge_mux.yaml`
- `skye_leader_bridge/config/skye_leader_bridge.yaml`
- `skye_leader_bridge/STARTUP.md`
- `skye_leader_bridge/README.md`

## 3. 已确认的 Skye ROS 信息

### 3.1 JointcmdArm

Skye 上的 `marvin_msgs/msg/JointcmdArm` 已确认是：

```text
std_msgs/Header header
float64[7] positions
```

所以桥接输出字段优先填 `positions`。

### 3.2 Jointfeedback

`marvin_msgs/msg/Jointfeedback` 已确认包含：

- `arm_positions[14]`
- `arm_velocities[14]`
- `arm_efforts[14]`
- `body_positions[6]`
- `head_positions[3]`

顺序说明里写了：

```text
L1-L7, R1-R7
```

### 3.3 夹爪

当前确认的夹爪节点：

```text
/dm_gripper_motor_node
```

它订阅：

```text
/control/gripperValueL  std_msgs/msg/Float32
/control/gripperValueR  std_msgs/msg/Float32
```

它发布：

```text
/info/gripper_feedback_L
/info/gripper_feedback_R
```

已做过小测试，`0.0` 和 `0.2` 会改变反馈数组，但 `0.0/1.0` 对应开还是闭，仍建议机械上再确认。

## 4. joint_cmd_mux 结论

`/joint_cmd_mux` 已经查清楚，作用是关节命令多路选择器。

当前 source 定义：

```text
-1 = idle
0  = teleop_ik
1  = model
2  = replay
```

当前参数：

```yaml
source_names:
- teleop_ik
- model
- replay

source_prefixes:
- control/teleop_ik
- control/model
- control/replay

output_prefix: control
```

结论：

- `teleop_ik` 留给原始 QP/IK/VR 关节命令源
- `model` 留给新的遥操小臂源
- `replay` 留给回放

正式方案 B 下：

```text
/qp_controller -> /control/teleop_ik/joint_cmd_A/B
skye_leader_bridge -> /control/model/joint_cmd_A/B
/joint_cmd_mux -> /control/joint_cmd_A/B
```

## 5. QP 配置要点

`/qp_controller` 启动参数里需要把：

```text
joint_cmd_output_prefix: control
```

改成：

```text
joint_cmd_output_prefix: control/teleop_ik
```

这个参数必须在启动时生效，运行中 `ros2 param set` 不够，因为 publisher topic 已经在节点启动时创建。

如果不改这个参数，`/qp_controller` 会继续直接发布 `/control/joint_cmd_A/B`，与 `/joint_cmd_mux` 冲突。

## 6. 桥接包配置

当前包里有两套配置：

### 6.1 `skye_leader_bridge.yaml`

用途：

- 直发 `/control/joint_cmd_A/B`
- 仅用于 debug / fallback
- 当前不是正式上机推荐配置

### 6.2 `skye_leader_bridge_mux.yaml`

用途：

- 正式方案 B
- 发布到 `/control/model/joint_cmd_A/B`
- 让 `joint_cmd_mux` 统一选择 source

正式启动桥接时要用 mux 配置：

```bash
ros2 launch skye_leader_bridge skye_leader_bridge.launch.py \
  config_file:=/opt/kernelmind/skye_leader_bridge_ws/install/skye_leader_bridge/share/skye_leader_bridge/config/skye_leader_bridge_mux.yaml
```

## 7. 启动顺序

推荐顺序：

1. NoMachine 上位机只做安全确认。
2. 断开上位机对机器人控制器的连接。
3. 启动 ROS 低层机器人服务。
4. 启动 `joint_cmd_mux`。
5. 启动 `skye_leader_bridge`，并使用 mux 配置。
6. 启动遥操小臂服务。
7. 通过 `/mode/switch_teleop` 使能输出。
8. 最后再切 `joint_cmd_mux` 到 `model` source。

## 8. 一键准备脚本

包里已经有：

```text
prepare_skye_impedance
```

它会尝试：

- 清故障
- ready
- 速度比例 10%
- 加速度比例 10%（如果能找到服务）
- `mode=3` 阻抗模式

注意：

- 当前有些机器人图里未必暴露 `/control/clear_fault`
- 如果 service 不存在，脚本会警告并跳过可选项
- `set_mode` 是必须的

命令示例：

```bash
ros2 run skye_leader_bridge prepare_skye_impedance --vel-ratio 10 --acc-ratio 10 --mode 3
```

## 9. 已踩过的坑

### 9.1 旧 direct bridge 会抢 `/control/joint_cmd_A/B`

如果启动的是默认配置，桥接节点会直接往 `/control/joint_cmd_A/B` 发，这是路线 1 直发模式。

正式方案 B 必须改用 `skye_leader_bridge_mux.yaml`。

### 9.2 `joint_cmd_mux` 退出后链路会断

曾经因为把 `joint_cmd_mux` 启动后又 `Ctrl+C` 退出，导致桥接没法输出到最终 joint topic。

要一直保留：

```text
/joint_cmd_mux
```

### 9.3 小臂输入没到 Skye 工控机，桥接就不会输出

桥接输出依赖：

```text
/left_joint_control
/right_joint_control
```

如果小臂节点在本地笔记本上跑，但 ROS2 跨机器通信没打通，Skye 上桥接就收不到输入。

### 9.4 上位机和 ROS 低层驱动不要同时抢控制器

上位机和工控机上的 ROS 低层驱动是互斥控制关系。

正确理解：

- 上位机先做安全确认
- 然后断开
- 再由 ROS 低层驱动接管

不要依赖“上位机先设置模式，再断开后 ROS 保持模式不变”这种顺序。

## 10. 目前可直接验证的状态

当下比较理想的 ROS 图应包括：

```text
/joint_cmd_mux
/skye_leader_bridge
/marvin_robot_node
/dm_gripper_motor_node
```

以及：

```text
/control/model/joint_cmd_A
/control/model/joint_cmd_B
/control/joint_cmd_A
/control/joint_cmd_B
```

其中：

- `/control/model/joint_cmd_A/B` 的 publisher 应该是 `skye_leader_bridge`
- `/control/joint_cmd_A/B` 的 publisher 应该是 `joint_cmd_mux`
- `/joint_cmd_mux` 的 active source 初始可以是 `-1`

## 11. 后续建议

下一步优先做：

1. 小范围验证小臂输入是否能稳定进入 `/control/model/joint_cmd_A/B`
2. 再切 `joint_cmd_mux` 到 `model`
3. 单臂低速验证 `joint_order/signs/offsets`
4. 确认夹爪开合方向
5. 再扩大到双臂

