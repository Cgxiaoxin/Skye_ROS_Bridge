# Orin 双臂遥操启动（精简版）

推荐使用[统一启动入口](unified_startup.md)：大臂配置在本机设置一次，小臂 A/B 及左右侧按 USB 自动识别，
支持交叉连接。下文保留 Orin + 小臂 B 的手动启动方式。

大臂和小臂均在 Orin 宿主机运行，使用当前 `skye_factr_teleop` 包。小臂为遥操设备 B；所有终端统一 `ROS_DOMAIN_ID=22`，两个驱动均显式指定 `robot_profile:=orin`。

## 1. 首次准备

- 已安装 ROS 2 Humble，使用系统 `/usr/bin/python3`；运行依赖包含 `numpy`、`yaml`、`dynamixel_sdk`、`pinocchio` 和 Python `ruckig`（项目基线 `0.19.4`）。
- 将 **aarch64 版** `libGentoSDK.so` 放到 `third_party/gento_sdk/lib/aarch64/`（仓库该目录目前只有说明文件），在 Orin 上重新编译：

```bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
export PATH="/usr/bin:$PATH"
/usr/bin/python3 -c 'import rclpy, numpy, yaml, dynamixel_sdk, pinocchio, ruckig'
colcon build --symlink-install --packages-select \
  skye_robot_driver skye_factr_description skye_factr_teleop \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

## 2. 每次启动前

1. 大臂控制器上电，确认 `ping -c 2 6.6.7.190` 成功。
2. 小臂按零位标记摆好后上电，连接两路 USB；确认当前用户有串口读写权限。
3. 退出旧 FACTR Docker、其他小臂控制程序及重复的大臂驱动，释放串口和 SDK 连接。
4. 核对 `config/orin/grav_comp_m6_{left,right}.yaml` 的零位、方向、电流与运动上限；当前 Orin 配置仍标注待实机验证。

设备 B 的 USB 标识：左 `FTB8HNOT`，右 `FTAO51EA`。先查看实际设备，再按标识设置 FTDI 延迟，避免依赖 `ttyUSB0/1` 顺序（重新插拔后需复查）：

```bash
ls -l /dev/serial/by-id/
for serial in FTB8HNOT FTAO51EA; do
  port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_${serial}-if00-port0"
  tty_name=$(basename "$(readlink -f "$port")")
  echo 1 | sudo tee "/sys/class/tty/$tty_name/device/latency_timer"
  cat "/sys/class/tty/$tty_name/device/latency_timer"
done
```

两路回读均应为 `1`。

## 3. 每个终端先加载环境

以下公共命令在终端 A、B、C 中各执行一次：

```bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export PATH="/usr/bin:$PATH"
export ROS_DOMAIN_ID=22 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 4. 启动驱动

**终端 A：大臂与 Robotiq 夹爪。**

```bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py robot_profile:=orin
```

确认连接 `6.6.7.190`，夹爪类型为 `robotiq`。保持此终端运行。

**终端 B：双侧小臂。** 显式使用源码配置，修改 YAML 后须重启节点。

```bash
ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py \
  robot_profile:=orin \
  left_config:="$PWD/src/skye_factr_teleop/config/orin/grav_comp_m6_left.yaml" \
  right_config:="$PWD/src/skye_factr_teleop/config/orin/grav_comp_m6_right.yaml"
```

只启动单侧时，改用 `start_teleop_m6_left.launch.py` 或 `start_teleop_m6_right.launch.py`，仅保留对应侧的配置参数。

## 5. 同步 → 遥操 → 停止

**终端 C：操作。** 先确认两侧大臂反馈持续更新（查看后 `Ctrl+C` 退出）：

```bash
ros2 topic hz /gento/left_joint_states
ros2 topic hz /gento/right_joint_states
```

保持大臂静止，确认小臂到目标姿态的路径无干涉，再请求同步：

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
ros2 topic echo /teleop/state \
  --qos-durability transient_local --qos-reliability reliable
```

等待**两侧均完成** `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED`。该状态话题由两侧共享，不能凭单条 `SYNCED` 判断双侧完成；必要时结合节点日志核对。`Ctrl+C` 退出状态查看后，开始遥操：

```bash
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
```

托稳小臂后停止（会关闭小臂力矩）：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

确认两侧回到 `IDLE`，再依次在终端 B、A 按 `Ctrl+C` 退出。重新遥操需再次同步；异常时先停止，必要时使用实体急停。

当前入口使用上述话题切换模式。同步、保持调参见[单臂同步 SOP](single_arm_sync_tuning_sop_quick.md)。
