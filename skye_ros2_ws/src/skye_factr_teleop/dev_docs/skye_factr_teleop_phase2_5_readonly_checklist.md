# 阶段 2.5.1：Dynamixel 通信只读验证清单

本清单只用于阶段 2.5.1。不得启动 `skye_factr_teleop_driver`、控制核心、ROS 遥操节点，
不得调用普通 `DynamixelDriver.open()`，也不得进入阶段 2.5.2 的电流写入。

## 1. 上电和连接前

- [ ] 大臂运动链路已断开或保持停机，现场无其它程序可控制大臂；
- [ ] 左右 leader 均处于可自由移动、不会因重力碰撞人员或设备的位置；
- [ ] 已停止所有可能占用 leader 串口的 FACTR/Dynamixel 进程；
- [ ] 操作者知道本轮验证的侧别，左右总线每次只验证一侧；
- [ ] 准备好紧急断电方式；
- [ ] 不运行阶段 2.5.2 或任何带 torque/current 写入的命令。

只读确认占用情况：

```bash
pgrep -af 'factr_teleop|skye_factr_teleop|dynamixel' || true
```

## 2. 连接后、运行工具前

确认稳定设备名及左右映射，不使用临时 `/dev/ttyUSB*` 名称作为配置：

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
```

已知配置映射：

| 侧别 | USB by-id |
|---|---|
| 左 | `usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0` |
| 右 | `usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0` |

拔插会使 FTDI latency timer 复位。按当前实际 tty 设置为 1，然后回读确认：

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
```

## 3. 左右总线自动只读检查

先构建并加载工作区：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skye_factr_description skye_factr_teleop
source install/setup.bash
mkdir -p /tmp/skye_phase2_5_records
```

左侧：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --frames 500 --interval 0.02 \
  --output /tmp/skye_phase2_5_records/left_readonly.json
```

右侧：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side right \
  --config src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
  --frames 500 --interval 0.02 \
  --output /tmp/skye_phase2_5_records/right_readonly.json
```

每侧的自动检查必须显示 `automated result: PASS`，并确认记录中：

- [ ] 使用了对应侧的 USB by-id；
- [ ] 波特率为 `4000000`，latency timer 为 `1`；
- [ ] ID 1--2 的型号编号为 `1020`（XM430-W350）；
- [ ] ID 3--8 的型号编号为 `1220`（XC330-T288）；
- [ ] 每个设备的 Return Delay Time 为 `0`；
- [ ] 每个设备的 `torque_enabled` 为 `false`；
- [ ] 多帧位置和速度均为有限值，无超限速度或异常位置跳变。

任一项失败即停止，不进入后续步骤。

## 4. 关节顺序、方向和连续角度

每侧重新运行 10 秒采样。在采样期间一次只轻微移动一个关节，操作者记录物理关节与
ID 的对应关系。终端表格和 JSON `analysis.devices` 同时给出原始位移、应用 YAML
`joint_signs` 后的位移以及最大帧间跳变。

- [ ] ID 1--8 与物理关节/夹爪顺序一致；
- [ ] 按约定正方向移动时 `signed_position_delta_rad` 为正；
- [ ] 反向移动时符号相反；
- [ ] 缓慢跨越编码器圈界时位置连续，无接近一整圈的突跳；
- [ ] 每次只有被操作关节出现主要位移。

若一次动作回到起点导致总位移接近零，应分开执行正向和反向采样，不以单次往返结果
判断方向。

## 5. 单设备断连诊断

只在确认 torque 全部关闭后执行。使用预先准备的单设备安全隔离开关或可断开的测试分支；
禁止在总线带电时直接拔插 DYNAMIXEL 接头。启动较长采样后，隔离一个预先记录的舵机：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --frames 1500 --interval 0.02 \
  --output /tmp/skye_phase2_5_records/left_disconnect.json
```

- [ ] 命令非零退出；
- [ ] 错误包含 `read_state` 或 `ping` 操作名；
- [ ] 错误包含被断开设备的 ID；
- [ ] 进程退出且串口已关闭；
- [ ] 记录文件保留失败前的采样和完整错误。

右侧重复同样验证。恢复隔离设备前先断电。

## 6. 验收记录

```text
日期/操作者：
侧别：left / right
USB by-id：
配置文件：
舵机型号与 ID：
波特率：4000000
FTDI latency timer：1
Return Delay Time：0 / FAIL
Torque Enable：全部 0 / FAIL
连续采样记录：
关节顺序结果：
关节方向结果：
连续角度结果：
单设备断连 ID：
断连错误是否含 ID 和操作名：
JSON 记录路径：
结论：PASS / FAIL
备注：
```

只有左右总线全部完成且上述人工项均填写为 PASS，才认为阶段 2.5.1 完成。此结论不
授权进入阶段 2.5.2；低电流验证需单独确认。
