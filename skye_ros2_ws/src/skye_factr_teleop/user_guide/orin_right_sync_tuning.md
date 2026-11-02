# Orin 右臂同步调参命令

首次或更新代码后构建一次。

```bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
export PATH="/usr/bin:$PATH"
colcon build --packages-select skye_factr_description skye_factr_teleop --symlink-install
source install/setup.bash
```

## 公共环境：每个终端执行

```bash
source /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/scripts/setup_factr_teleop_env.sh --robot orin --device b --side right --domain 21
SIDE=right CONFIG="$RIGHT_CONFIG"
```

## 终端 A：检查、调参、启动

退出旧驱动，检查右臂串口。

```bash
ls -l /dev/serial/by-id/
rg -n 'dynamixel_port:' "$CONFIG"
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO51EA-if00-port0
TTY_PATH=$(readlink -f "$USB_BY_ID")
```

将串口延迟设为 1 ms 并回读。

```bash
if test -c "$TTY_PATH"; then
  TTY_NAME=$(basename "$TTY_PATH")
  echo 1 | sudo tee "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
  cat "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
else
  echo "串口不存在，请先检查连接"
fi
```

停止驱动后修改 YAML，再启动右臂。

```bash
ros2 launch skye_factr_teleop start_teleop_m6_right.launch.py \
  robot_profile:=orin right_config:="$CONFIG" sync_diagnostics:=true
```

## 终端 B：每轮先记录

```bash
OUTPUT="/tmp/orin_right_sync_$(date +%Y%m%d_%H%M%S).json"
echo "$OUTPUT"
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side "$SIDE" --config "$CONFIG" \
  --output "$OUTPUT" --duration 180
```

## 终端 C：预览、执行

在 IDLE 下预览七轴归零目标。

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial move \
  --side "$SIDE" --config "$CONFIG" --target 0 0 0 0 0 0 0
```

确认路径后执行归零，等待 SYNCED。

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial move \
  --side "$SIDE" --config "$CONFIG" --target 0 0 0 0 0 0 0 --execute
```

## 终端 D：保持空闲，专门停止

托稳小臂并停止。

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

## 停止后：原终端 B 回读失能

记录保存后，在终端 A 按 Ctrl+C 退出驱动；保持供电，在原终端 B 回读。

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side "$SIDE" --config "$CONFIG" --frames 2 --interval 0 \
  --output "${OUTPUT%.json}_torque_disable.json"
```

核对 `automated_result=PASS`、ID1..8 `torque_enabled=false`。
