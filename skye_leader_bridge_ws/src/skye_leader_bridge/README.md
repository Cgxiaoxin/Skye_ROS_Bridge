# Skye 遥操小臂桥接节点

`skye_leader_bridge` 用来把同构遥操小臂输出的 7 轴关节角和夹爪触发量接入 Skye 的 ROS 2 控制栈。当前方案采用“路线 1”：不走 VR 位姿和 IK，直接把小臂关节角转换成 Skye 双臂关节命令。

完整部署和启动流程请优先看 [`STARTUP.md`](STARTUP.md)。本文档保留架构、配置和关键注意事项。

## 数据流

```text
/left_joint_control              -> skye_leader_bridge -> /control/joint_cmd_A
/right_joint_control             -> skye_leader_bridge -> /control/joint_cmd_B
/left_teleop_gripper/ctrl        -> skye_leader_bridge -> /control/gripperValueL
/right_teleop_gripper/ctrl       -> skye_leader_bridge -> /control/gripperValueR
/mode/switch_teleop|sync|stop    -> 遥操使能/同步/停止
```

这里需要区分两件事：

- 桥接节点发布的是 `marvin_msgs/msg/JointcmdArm.positions`，也就是 7 轴关节**位置目标**。
- 机器人底层以位置模式还是阻抗模式执行这些目标，由 `/marvin_robot_node` 的控制模式决定，不由桥接节点本身决定。

遥操作建议让机器人底层处于 **mode=3 阻抗模式**，这样从臂更柔顺；不要在刚性位置模式下直接做高频主从遥操。

## 编译

在 Skye 机器人或包含 `marvin_msgs` 的 Skye ROS 工作区中执行：

```bash
cd /data/coding/tianji/Skye-mutile-arm/skye_leader_bridge_ws
source /opt/ros/humble/setup.bash
source /opt/kernelmind/apex/install/setup.bash
colcon build --packages-select skye_leader_bridge
source install/setup.bash
```

`marvin_msgs` 来自 Skye 机器人工作区。本仓库只包含桥接代码和 topic 分析，不包含 `marvin_msgs` 的 `.msg/.srv` 源文件。

## 已确认的 Skye 消息和 Topic

当前 Skye 机器人上的 `JointcmdArm` 为：

```text
std_msgs/Header header
float64[7] positions
```

因此 `config/skye_leader_bridge.yaml` 中已配置优先填充：

```yaml
message:
  joint_command_fields: [positions, joint_pos, joint_position, joint_positions, position, pos, data]
```

当前夹爪节点为 `/dm_gripper_motor_node`，订阅：

```text
/control/gripperValueL std_msgs/msg/Float32
/control/gripperValueR std_msgs/msg/Float32
```

发布反馈：

```text
/info/gripper_feedback_L std_msgs/msg/Float32MultiArray
/info/gripper_feedback_R std_msgs/msg/Float32MultiArray
```

已小幅测试 `0.0` 和 `0.2` 会改变夹爪反馈数组，但还需要机械上确认 `0.0/1.0` 分别对应张开还是闭合。

## 方案 B：通过 joint_cmd_mux 接入

正式上机推荐使用 `joint_cmd_mux`，保留 `/qp_controller` 和原有控制服务。目标链路是：

```text
QP/IK 原链路 -> /control/teleop_ik/joint_cmd_A/B -> joint_cmd_mux source 0
遥操小臂     -> /control/model/joint_cmd_A/B     -> joint_cmd_mux source 1
回放         -> /control/replay/joint_cmd_A/B    -> joint_cmd_mux source 2
joint_cmd_mux -> /control/joint_cmd_A/B          -> /marvin_robot_node
```

注意：

- `/qp_controller` 必须在启动时配置 `joint_cmd_output_prefix: control/teleop_ik`。
- `skye_leader_bridge` 必须使用 `skye_leader_bridge_mux.yaml`，发布到 `/control/model/joint_cmd_A/B`。
- `/control/joint_cmd_A/B` 的唯一 publisher 应该是 `/joint_cmd_mux`。

检查：

```bash
ros2 topic info -v /control/joint_cmd_A
ros2 topic info -v /control/teleop_ik/joint_cmd_A
ros2 topic info -v /control/model/joint_cmd_A
```

期望状态：

```text
/control/joint_cmd_A:
  Publisher: joint_cmd_mux
  Subscriber: marvin_robot_node

/control/teleop_ik/joint_cmd_A:
  Publisher: qp_controller
  Subscriber: joint_cmd_mux

/control/model/joint_cmd_A:
  Publisher: skye_leader_bridge
  Subscriber: joint_cmd_mux
```

保留的 direct 配置 `skye_leader_bridge.yaml` 只用于底层 debug；正式 mux 方案使用 `skye_leader_bridge_mux.yaml`。

## 控制模式设置

桥接节点只发关节位置目标；真正的执行模式由 Skye 底层控制器决定。上机遥操时建议让机器人处于 **mode=3 阻抗模式**，这样从臂更柔顺，不建议在刚性位置模式下直接做高频主从遥操。

注意：上位机和 ROS 低层驱动会连接同一个机器人控制器。启动 `bringup_control_gento_sky.launch.py` 后，ROS 驱动会重新接管控制器并可能把上位机预设的模式恢复为默认状态；同时 ROS 驱动连接后，上位机通常也不能再同时连接控制器。

因此推荐流程是：

1. 先通过 NoMachine/上位机确认现场安全、急停、机器人状态和工作空间。
2. 断开上位机对机器人控制器的连接。
3. 启动 ROS 低层驱动。
4. 通过本包提供的一键脚本在 ROS 侧清故障、ready、设置速度比例、设置加速度比例、切到 mode=3。
5. 再启动桥接节点。

当前机器人上存在这些服务：

```text
/control/set_mode [marvin_msgs/srv/Int]
/control/set_ready [std_srvs/srv/Trigger]
/control/clear_fault [std_srvs/srv/Trigger]
/control/set_vel_ratio [marvin_msgs/srv/Int]
```

上位机里有两个比例：速度 speed 和加速度 acceleration。首次遥操建议两者都设为 10%。当前已确认 ROS 侧有 `/control/set_vel_ratio`；加速度比例服务名还需要在机器人上确认。

一键准备命令：

```bash
ros2 run skye_leader_bridge prepare_skye_impedance --vel-ratio 10 --acc-ratio 10 --mode 3
```

这条命令会依次尝试：

```text
/control/clear_fault            可选，缺失则警告并跳过
/control/set_ready              可选，缺失则警告并跳过
/control/set_vel_ratio 10       可选，缺失则警告并跳过
/control/set_acc_* 10           可选，如果找到加速度服务则调用
/control/set_mode 3             必须，用于切换阻抗模式
```

脚本会自动尝试常见服务名：

```text
/control/set_acc_ratio
/control/set_accel_ratio
/control/set_acceleration_ratio
```

如果没有找到，它会打印警告。这种情况下请在上位机里手动把加速度设置为 10%，或者先查询真实服务名：

```bash
ros2 service list -t | grep -Ei "acc|accel|acceleration|ratio|vel"
```

确认服务名后可以显式指定：

```bash
ros2 run skye_leader_bridge prepare_skye_impedance \
  --vel-ratio 10 \
  --acc-ratio 10 \
  --acc-service /control/<真实加速度服务名> \
  --mode 3
```

如果需要跳过某一步：

```bash
ros2 run skye_leader_bridge prepare_skye_impedance --skip-clear-fault
ros2 run skye_leader_bridge prepare_skye_impedance --skip-ready
ros2 run skye_leader_bridge prepare_skye_impedance --skip-vel-ratio
ros2 run skye_leader_bridge prepare_skye_impedance --skip-acc-ratio
```

如果想手动调试服务字段：

```bash
ros2 interface show marvin_msgs/srv/Int
```

如果显示类似 `int64 data` 或 `int32 data`，也可以手动执行：

```bash
ros2 service call /control/clear_fault std_srvs/srv/Trigger "{}"
ros2 service call /control/set_ready std_srvs/srv/Trigger "{}"
ros2 service call /control/set_vel_ratio marvin_msgs/srv/Int "{data: 10}"
# 如果存在加速度比例服务，也设置为 10，例如：
# ros2 service call /control/set_acc_ratio marvin_msgs/srv/Int "{data: 10}"
ros2 service call /control/set_mode marvin_msgs/srv/Int "{data: 3}"
```

其中 `mode=3` 按你们当前约定理解为阻抗模式。不同版本固件可能会变，第一次上机前建议用下面命令确认状态反馈：

```bash
ros2 topic echo /info/robot_state --once
ros2 topic echo /info/robot_cmd_state --once
```

如果没有成功切到阻抗模式，先不要启动桥接。

## 推荐启动流程

### 1. 上位机确认现场安全

通过 NoMachine 打开上位机，确认：

- 急停可用。
- 工作空间安全。
- 机器人无明显硬件报警。
- 之后断开上位机对机器人控制器的连接，避免和 ROS 驱动抢连接。

### 2. 启动低层机器人服务

路线 1 推荐只启动低层控制，不启动全量 VR/QP 栈：

```bash
ros2 launch marvin_ros_control bringup_control_gento_sky.launch.py
```

如果你启动的是全量 VR 栈，需要确保 `/qp_controller` 没有发布 `/control/joint_cmd_A/B`。

### 3. 在 ROS 侧准备阻抗模式

另开终端：

```bash
source /opt/ros/humble/setup.bash
source /opt/kernelmind/apex/install/setup.bash
source /opt/kernelmind/skye_leader_bridge_ws/install/setup.bash
ros2 run skye_leader_bridge prepare_skye_impedance --vel-ratio 10 --acc-ratio 10 --mode 3
```

如果这一步失败，不要继续启动桥接。

### 4. 启动前检查 topic 冲突

```bash
ros2 node list
ros2 topic info -v /control/joint_cmd_A
ros2 topic info -v /control/joint_cmd_B
ros2 topic info -v /control/gripperValueL
ros2 topic info -v /control/gripperValueR
```

期望 `/control/joint_cmd_A/B`：

```text
Publisher count: 0
Subscription count: 1
Node name: marvin_robot_node
```

如果 publisher 是 `/qp_controller`，不要启动桥接；先停掉 QP 或改用只启动低层控制的 launch。

### 5. 启动遥操小臂

启动已有的遥操小臂程序，确认出现：

```text
/left_joint_control
/right_joint_control
/left_teleop_gripper/ctrl
/right_teleop_gripper/ctrl
```

### 6. 启动桥接节点

```bash
ros2 launch skye_leader_bridge skye_leader_bridge.launch.py
```

指定配置文件时使用：

```bash
ros2 launch skye_leader_bridge skye_leader_bridge.launch.py \
  config_file:=/path/to/skye_leader_bridge.yaml
```

### 7. 使能遥操

桥接节点默认需要收到 `/mode/switch_teleop` 后才开始输出。可以使用小臂原有键盘节点，或手动发布：

```bash
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String "{data: switch_teleop}"
```

停止输出：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String "{data: switch_stop}"
```

## 上机前检查清单

1. 上位机确认现场安全，并断开对控制器的连接。
2. ROS 低层驱动已启动。
3. `prepare_skye_impedance --vel-ratio 10 --acc-ratio 10 --mode 3` 执行成功；如果脚本提示没有加速度比例服务，需要确认上位机已将 acceleration 设置为 10%，或提供真实加速度服务名。
4. 确认 `/qp_controller` 没有发布 `/control/joint_cmd_A/B`。
5. 确认 `/marvin_robot_node` 订阅 `/control/joint_cmd_A/B`。
6. 确认 `JointcmdArm` 字段是 `positions`。
7. 确认 `/dm_gripper_motor_node` 订阅 `/control/gripperValueL/R`。
8. 调低 `publish_rate_hz` 或 `max_delta_per_cycle` 做首次测试。
9. 先单臂、低速、小范围验证 `joint_order`、`signs` 和 `offsets`。
10. 使用 `/mode/switch_teleop` 启用桥接输出，使用 `/mode/switch_sync` 或 `/mode/switch_stop` 停止输出。