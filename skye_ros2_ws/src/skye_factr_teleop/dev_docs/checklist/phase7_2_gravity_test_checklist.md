# 阶段 7.2：单臂重力补偿真机验收清单

状态：`PREPARED`。本清单和验收入口已准备；重力补偿真机结果尚未记录，不得标为 `PASS`。
先完成[阶段 7.1](../skye_factr_teleop_development_plan.md#71-完成阶段-252-真机验证与-253-冒烟验收记录)。

本入口只启动一侧 leader，使用生产 `DynamixelDriver`、`ControlCore` 和动力学模型。
`GRAVITY_TEST` 不订阅从臂反馈、同步或走位请求，也不发布 follower 关节命令。
验收模式只输出所选关节的重力项；其余控制项均为零。
本轮设备 A 配置使用 `position_only`，动力学计算为 `RNEA(q, 0, 0)`；
本清单不验收 `position_velocity` 的速度相关补偿效果。

## 1. 操作前确认

- [ ] 7.1 的该侧低电流真机验收有效。USB、接线、标定或机械装配变更时，先按
  [2.5.1 只读清单](phase2_5_readonly_checklist.md)重验受影响项目。
- [ ] 小臂无负载、由操作者支撑，周围人员与线缆已清场，可立即切断舵机电源。
- [ ] 只启动目标侧；没有其他程序占用该侧串口或自动发布模式请求。
- [ ] 核对实体标签、USB by-id、ID 1--8、该侧 `joint_offsets`、`joint_signs`、
  URDF、配置中的 `current_limit_ma` 和 `joint_gravity_comp.mode: position_only`。
  配置须是现场核准的标定文件。
- [ ] 核准本轮 `joint_id`、`gain_scale`、电流上限、渐增时间和最长通电时间。
  `gain_scale` 在 `(0, 1]`，重力检查以配置中的该关节 gravity gain 为基数。
  电流上限不超过 50 mA，且不能高于配置中任一舵机的上限；渐增时间至少 1 秒，
  最长运行时间不超过 30 秒。当前设备 A 模板电流上限均为 50 mA。

本入口的 `set_torque_enabled(True)` 会使能总线全部 8 个舵机，但只给所选关节
非零电流命令；其余 7 个 ID 命令为零。零电流不等于机械制动，操作者须持续支撑小臂。

## 2. 启动与监视

首次测试或更新源码、YAML 后，从工作区目录构建遥操包及其 URDF 依赖，并加载工作区：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to skye_factr_teleop --symlink-install
source install/setup.bash
ros2 pkg executables skye_factr_teleop
```

确认最后一条命令列出 `skye_factr_teleop_driver`。新开的启动或监视终端也须执行：

```bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/leader_for_skye/skye_ros2_ws/install/setup.bash
```

启动前检查是否仍有程序占用 leader 串口：

```bash
pgrep -af 'factr_teleop|skye_factr_teleop|dynamixel' || true
```

每轮只测一个 ID；切换 ID、参数或 YAML 模式时，先停止节点，再重新启动。以下命令使用
设备 A 的左右臂配置文件；执行前按第 1 节核准对应配置和硬件。
节点启动后应处于 `IDLE`、Torque Disable。电流数组按物理 ID 1--8 排列，
单位 mA；它是**限幅后的命令电流**，不是实测电流。当前状态话题应只报告
该侧 7 个关节的位置、速度。开始前确认监视端能收到消息。

### 2.1 左臂

先列出 USB by-id，核对左臂实体标签与 YAML 中的 `dynamixel_port` 均对应
`FTBEO9L1`，再解析实际 tty：

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0
TTY_PATH=$(readlink -f "$USB_BY_ID")
test -c "$TTY_PATH"
TTY_NAME=$(basename "$TTY_PATH")
echo "USB by-id: $USB_BY_ID -> $TTY_PATH"
```

确认 `test -c` 成功且 tty 对应左臂，然后设置并回读 FTDI latency timer：

```bash
echo 1 | sudo tee "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
cat "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
```

确认回读为 `1`，再启动左臂 ID 1、20 mA 上限的测试：

```bash
ros2 launch skye_factr_teleop gravity_test_m6_left.launch.py \
  config_path:=/home/lyy/projects/leader_for_skye/skye_ros2_ws/src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  joint_id:=1 \
  gain_scale:=0.1 \
  ramp_seconds:=2.0 \
  current_limit_ma:=20.0 \
  max_duration_seconds:=10.0
```

在三个终端分别监视：

```bash
ros2 topic echo /teleop/state std_msgs/msg/String
```

```bash
ros2 topic echo /leader_arm/left/gravity_test_current_ma std_msgs/msg/Float64MultiArray
```

```bash
ros2 topic echo /leader_arm/current_state sensor_msgs/msg/JointState
```

### 2.2 右臂

先列出 USB by-id，核对右臂实体标签与 YAML 中的 `dynamixel_port` 均对应
`FTBEO8WL`，再解析实际 tty：

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0
TTY_PATH=$(readlink -f "$USB_BY_ID")
test -c "$TTY_PATH"
TTY_NAME=$(basename "$TTY_PATH")
echo "USB by-id: $USB_BY_ID -> $TTY_PATH"
```

确认 `test -c` 成功且 tty 对应右臂，然后设置并回读 FTDI latency timer：

```bash
echo 1 | sudo tee "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
cat "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
```

确认回读为 `1`，再启动右臂 ID 1、20 mA 上限的测试：

```bash
ros2 launch skye_factr_teleop gravity_test_m6_right.launch.py \
  config_path:=/home/lyy/projects/leader_for_skye/skye_ros2_ws/src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
  joint_id:=1 \
  gain_scale:=0.1 \
  ramp_seconds:=2.0 \
  current_limit_ma:=20.0 \
  max_duration_seconds:=10.0
```

在三个终端分别监视：

```bash
ros2 topic echo /teleop/state std_msgs/msg/String
```

```bash
ros2 topic echo /leader_arm/right/gravity_test_current_ma std_msgs/msg/Float64MultiArray
```

```bash
ros2 topic echo /leader_arm/current_state sensor_msgs/msg/JointState
```

## 3. 逐关节重力补偿检查

确认姿态处于关节安全范围并扶稳小臂后，仅向当前启动的一侧发送一次显式请求。
左臂：

```bash
ros2 topic pub --once /mode/left/switch_gravity_test std_msgs/msg/String '{data: switch_gravity_test}'
```

右臂：

```bash
ros2 topic pub --once /mode/right/switch_gravity_test std_msgs/msg/String '{data: switch_gravity_test}'
```

观察并记录：

1. `/teleop/state` 从 `IDLE` 变为 `GRAVITY_TEST`；没有同步轨迹或 follower 命令。
2. 所选 ID 的补偿增益在 `ramp_seconds` 内从零渐增；命令电流随姿态变化，
   不要求单调增加，但绝对值不得超过本轮上限；其余 ID 的命令电流为零。
3. 扶稳小臂，轻缓改变所选关节姿态后停住；在相同姿态下比较进入测试前后所需的
   手部支撑力，确认补偿辅助抵消重力，而不是加重下坠趋势。不得出现突然加速、
   反向牵引或其它关节异常动作。2.5.1 的只读 `joint_signs` 结果沿用，
   不重复手动方向引导。本轮不做变速移动测试。
4. `max_duration_seconds` 到期后状态回到 `IDLE`，命令电流归零、Torque Disable。
   在任一时刻均可先发送 `switch_stop` 结束本轮。

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

从较低 `gain_scale` 起逐级提高；每一级都须重新启动节点并保存参数、结果。
某一关节在低上限下无明显辅助力时，记录为 `INCONCLUSIVE`，不得据此判断方向正确，
也不得未经现场批准直接提高电流。按 ID 1--7 逐项完成左右两侧。

## 4. 停止条件与结束检查

方向不符、其它关节动作、超出预期的速度、机械或线缆干涉、通信报错、状态未按时
返回 `IDLE` 时，立即使用现场断电手段并记录为 `FAIL`。正常停止时先发布
`/mode/switch_stop`，确认电流消息为全零、状态为 `IDLE`，再结束进程。Ctrl+C 和
控制异常会调用现有零电流与 Torque Disable 清理路径；进程被强制终止或总线失联时
不能仅凭软件消息断言舵机已失能。退出节点后，按[2.5.2 第 10 节](phase2_5_low_current_checklist.md#10-测试结束后复核-torque-disable)
对本侧 ID 1--8 做只读 Torque Disable 回读，保存独立结果文件。

## 5. 验收记录与判定

每一轮至少保存：日期、操作者、侧别、配置文件、补偿模式、USB by-id、URDF、ID、型号、
`gain_scale`、该 ID 配置 gravity gain、电流上限、渐增时间、最长时间、
起止姿态、命令电流峰值、实际施力方向、`switch_stop` 或超时结果、进程退出结果及
测试后 Torque Disable 回读路径。

单轮 `PASS` 要求状态切换正确、仅目标 ID 非零、命令电流不越界、施力方向正确，
且停止和失能回读通过。`INCONCLUSIVE` 不计为通过。左右两侧全部关节的重力检查
和停止路径均通过，才可将本清单标为 `PASS`。本清单通过仅表示
`position_only` 下的重力补偿验收通过；切换到 `position_velocity` 后须另行验收速度相关效果。
7.2 的其他待办以开发计划为准。7.2 完成前不得进入 7.3 真机同步。
