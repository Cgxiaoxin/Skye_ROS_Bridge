# Skye 遥操小臂桥接启动文档

本文档维护 `skye_leader_bridge` 在 Skye 工控机上的部署、编译、启动和排查流程。代码或启动方式调整后，优先同步更新本文档。

## 0. 目标链路

路线 1：遥操小臂关节角直发到 Skye 低层关节命令，不经过 VR 和 IK。

```text
遥操小臂
  -> /left_joint_control, /right_joint_control
  -> skye_leader_bridge
  -> /control/joint_cmd_A, /control/joint_cmd_B
  -> /marvin_robot_node
```

夹爪链路：

```text
/left_teleop_gripper/ctrl, /right_teleop_gripper/ctrl
  -> skye_leader_bridge
  -> /control/gripperValueL, /control/gripperValueR
  -> /dm_gripper_motor_node
```

## 1. 确认环境

在 Skye 工控机执行：

```bash
cd /opt/kernelmind/skye_leader_bridge_ws
source /opt/ros/humble/setup.bash
source /opt/kernelmind/apex/install/setup.bash

ros2 interface show marvin_msgs/msg/JointcmdArm
which colcon
```

`JointcmdArm` 应显示：

```text
std_msgs/Header header
float64[7] positions
```

如果 `which colcon` 没有输出，需要安装或改用 Python 直接运行方案。

## 2. 编译桥接包

首次拷贝或修改代码后执行：

```bash
cd /opt/kernelmind/skye_leader_bridge_ws
source /opt/ros/humble/setup.bash
source /opt/kernelmind/apex/install/setup.bash

colcon build --packages-select skye_leader_bridge
source install/setup.bash
```

确认包可见：

```bash
ros2 pkg list | grep skye_leader_bridge
ros2 pkg executables skye_leader_bridge
```

应看到：

```text
skye_leader_bridge
skye_leader_bridge leader_to_skye_bridge
skye_leader_bridge prepare_skye_impedance
```

如果出现 `Package 'skye_leader_bridge' not found`，通常是当前终端没有执行：

```bash
source /opt/kernelmind/skye_leader_bridge_ws/install/setup.bash
```

## 3. 上位机安全确认

通过 NoMachine 打开上位机，只做安全确认：

- 急停可用。
- 工作空间安全。
- 机器人无明显硬件报警。
- 遥操小臂处于安全姿态。

注意：上位机和 ROS 低层驱动会连接同一个机器人控制器。启动 ROS 低层驱动后，上位机通常不能再同时连接控制器；而且 ROS 驱动启动时可能覆盖上位机预设的模式和速度。

因此不要依赖“上位机先设置模式再断开”的顺序。正确顺序是：先启动 ROS 低层驱动，再在 ROS 侧设置速度、加速度和模式。

## 4. 启动低层机器人服务

路线 1 推荐只启动低层控制，不启动全量 VR/QP 栈：

```bash
source /opt/ros/humble/setup.bash
source /opt/kernelmind/apex/install/setup.bash
ros2 launch marvin_ros_control bringup_control_gento_sky.launch.py
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /opt/kernelmind/apex/install/setup.bash

ros2 node list
ros2 topic list -t
```

需要看到 `/marvin_robot_node`，以及：

```text
/control/joint_cmd_A [marvin_msgs/msg/JointcmdArm]
/control/joint_cmd_B [marvin_msgs/msg/JointcmdArm]
/joint_states [sensor_msgs/msg/JointState]
```

## 5. 准备阻抗模式和低速低加速度

source 三个环境：

```bash
source /opt/ros/humble/setup.bash
source /opt/kernelmind/apex/install/setup.bash
source /opt/kernelmind/skye_leader_bridge_ws/install/setup.bash
```

先查看当前可用控制服务：

```bash
ros2 service list -t | grep -Ei "clear|ready|mode|vel|acc|ratio"
```

运行一键准备脚本：

```bash
ros2 run skye_leader_bridge prepare_skye_impedance --vel-ratio 10 --acc-ratio 10 --mode 3
```

脚本会尝试：

```text
/control/clear_fault            可选，缺失则警告并跳过
/control/set_ready              可选，缺失则警告并跳过
/control/set_vel_ratio 10       可选，缺失则警告并跳过
/control/set_acc_* 10           可选，自动尝试常见加速度服务名
/control/set_mode 3             必须，用于切到阻抗模式
```

如果脚本提示 `/control/clear_fault` 不存在，这不是消息格式错误，而是当前 ROS 图没有这个 service。脚本新版会跳过它；如果你还在用旧包，需要重新拷贝代码、重新 `colcon build` 并重新 `source install/setup.bash`。

如果提示 `/control/set_mode` 不存在，需要先查明哪个节点提供模式切换：

```bash
ros2 service list -t | grep -Ei "mode|ready|robot|control"
ros2 node list
```

没有 `/control/set_mode` 时，不要启动桥接，因为无法确认机器人已切到阻抗模式。

如果加速度服务没有自动找到，查询真实服务名：

```bash
ros2 service list -t | grep -Ei "acc|accel|acceleration|ratio|vel"
```

找到后显式指定：

```bash
ros2 run skye_leader_bridge prepare_skye_impedance \
  --vel-ratio 10 \
  --acc-ratio 10 \
  --acc-service /control/<真实加速度服务名> \
  --mode 3
```

## 6. 检查 mux 链路

方案 B 要求 `/control/joint_cmd_A/B` 由 `/joint_cmd_mux` 统一发布，`/qp_controller` 输出到 `/control/teleop_ik/joint_cmd_A/B`，桥接节点输出到 `/control/model/joint_cmd_A/B`。

先确认 `/qp_controller` 的启动参数已经改为：

```text
joint_cmd_output_prefix: control/teleop_ik
```

运行时改参数不够，必须重启 `/qp_controller` 所在 launch，因为 publisher topic 通常在节点启动时创建。

检查：

```bash
ros2 topic info -v /control/joint_cmd_A
ros2 topic info -v /control/teleop_ik/joint_cmd_A
ros2 topic info -v /control/model/joint_cmd_A
```

启动桥接前的期望状态：

```text
/control/joint_cmd_A:
  Publisher: joint_cmd_mux
  Subscriber: marvin_robot_node

/control/teleop_ik/joint_cmd_A:
  Publisher: qp_controller
  Subscriber: joint_cmd_mux

/control/model/joint_cmd_A:
  Publisher count: 0
  Subscriber: joint_cmd_mux
```

如果 `/control/joint_cmd_A` 仍然有 `/qp_controller` publisher，说明 QP 没有用新 prefix 重启成功。不要启动桥接。

如果 `/control/teleop_ik/joint_cmd_A` 是 unknown topic，说明 QP 还没有输出到 mux 输入，继续检查 launch/参数。

## 7. 启动遥操小臂

启动已有遥操小臂程序后，确认 topic：

```bash
ros2 topic list -t | grep -E "left_joint_control|right_joint_control|teleop_gripper|mode/switch"
```

需要看到：

```text
/left_joint_control [sensor_msgs/msg/JointState]
/right_joint_control [sensor_msgs/msg/JointState]
/left_teleop_gripper/ctrl [sensor_msgs/msg/JointState]
/right_teleop_gripper/ctrl [sensor_msgs/msg/JointState]
/mode/switch_teleop [std_msgs/msg/String]
/mode/switch_stop [std_msgs/msg/String]
```

## 8. 启动桥接节点

```bash
source /opt/ros/humble/setup.bash
source /opt/kernelmind/apex/install/setup.bash
source /opt/kernelmind/skye_leader_bridge_ws/install/setup.bash

ros2 launch skye_leader_bridge skye_leader_bridge.launch.py \
  config_file:=/opt/kernelmind/skye_leader_bridge_ws/install/skye_leader_bridge/share/skye_leader_bridge/config/skye_leader_bridge_mux.yaml
```

启动后检查：

```bash
ros2 topic info -v /control/model/joint_cmd_A
ros2 topic info -v /control/model/joint_cmd_B
```

期望变为：

```text
Publisher count: 1
Node name: skye_leader_bridge
Subscription count: 1
Node name: joint_cmd_mux
```

然后切换 mux 到 model source：

```bash
ros2 service call /control/joint_cmd_mux/select marvin_msgs/srv/Int "{data: 1}"
ros2 topic echo /info/joint_cmd_mux/active_source --once
```

期望 active source 为：

```text
data: 1
```

切回原 QP/IK source：

```bash
ros2 service call /control/joint_cmd_mux/select marvin_msgs/srv/Int "{data: 0}"
```

切到 idle：

```bash
ros2 service call /control/joint_cmd_mux/select marvin_msgs/srv/Int "{data: -1}"
```

## 9. 使能和停止遥操

桥接节点默认需要收到 `/mode/switch_teleop` 后才开始输出。

使用遥操小臂原有键盘节点，或手动使能：

```bash
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String "{data: switch_teleop}"
```

停止输出：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String "{data: switch_stop}"
```

## 10. 首次上机策略

首次测试建议：

1. 单臂测试，另一侧机械上避让。
2. 小范围移动，观察 `joint_order` 和 `signs` 是否正确。
3. 保持 `--vel-ratio 10` 和 `--acc-ratio 10`。
4. 必要时降低 `config/skye_leader_bridge.yaml` 中：

   ```yaml
   safety:
     max_delta_per_cycle: 0.02
     publish_rate_hz: 100
   ```

5. 确认夹爪 `0.0/1.0` 对应开合方向后，再扩大动作范围。

## 11. 常见问题

### `Package 'skye_leader_bridge' not found`

当前终端没有 source 桥接工作区：

```bash
source /opt/kernelmind/skye_leader_bridge_ws/install/setup.bash
```

如果还不行，重新编译：

```bash
cd /opt/kernelmind/skye_leader_bridge_ws
source /opt/ros/humble/setup.bash
source /opt/kernelmind/apex/install/setup.bash
colcon build --packages-select skye_leader_bridge
source install/setup.bash
```

### `Service /control/clear_fault is not available`

这表示当前 ROS 图没有这个 service，不是消息格式不匹配。新版脚本会把它当可选服务跳过。若旧脚本仍报错，重新部署并编译最新代码。

也可以临时跳过：

```bash
ros2 run skye_leader_bridge prepare_skye_impedance \
  --skip-clear-fault \
  --vel-ratio 10 \
  --acc-ratio 10 \
  --mode 3
```

### `Service /control/set_mode is not available`

这是必须服务。先不要启动桥接，检查：

```bash
ros2 service list -t | grep -Ei "mode|ready|robot|control"
ros2 node list
```

需要找到实际提供模式切换的 service 或启动对应节点。

### `/control/joint_cmd_A/B` 有 `/qp_controller` publisher

路线 1 不能同时运行 `/qp_controller`。请停止 QP，或改用只启动低层控制的 launch。

