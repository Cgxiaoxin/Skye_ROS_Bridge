# Gento 双大臂与双小臂：全断开后的恢复启动 SOP

适用场景：Gento 双大臂、小臂控制 USB、Docker 容器或 ROS 2 节点均已停止／断开，需要恢复 **bridge-less factr 遥操** 链路（键盘 `1/2/3` + `gento_robot_driver`）。

本文以当前工作区的 [start_gento_dual_arm_sync.sh](../../../start_gento_dual_arm_sync.sh) 为唯一启动入口。不要手工并行启动另一个 `gento_robot_driver`，也不要把状态发布到旧的 `/left_joint_state` 或 `/right_joint_state`。**不要**为 Gento 启动 `skye_leader_bridge`。

相关设计：[2026-07-24-gento-factr-teleop-bridge-less-design.md](./2026-07-24-gento-factr-teleop-bridge-less-design.md)

## 1. 运行拓扑与单位

```text
keyboard_gripper（容器内）
  └─ /mode/switch_{sync,teleop,stop}   # 键 1 / 2 / 3
        └─ factr_teleop_left / factr_teleop_right

Gento 控制器（6.6.7.190）
  └─ gento_robot_driver（主机；唯一 SDK 客户端）
       ├─ pub /gento/joint_states
       │    sensor_msgs/msg/JointState，14 个 position，rad，500 Hz
       ├─ sub /gento/left_joint_control, /gento/right_joint_control
       └─ srv /gento/hold_current, /gento/stop_motion  (std_srvs/Trigger)

factr remaps（容器内）
  ├─ /joint_state   ← /gento/joint_states
  ├─ /joint_control → /gento/{left|right}_joint_control
  └─ /joint_move、夹爪话题保持现有命名
```

- ROS 侧关节位置与速度统一是 **rad / rad·s⁻¹**。
- Gento SDK 边界内部使用 deg / deg·s⁻¹；`gento_robot_driver` 已完成转换。
- 原 bridge 的符号／限位／步长／超时已迁入 `gento_robot.yaml`（驱动内应用）。
- ROS 2 域固定为 `ROS_DOMAIN_ID=20`。
- 同一个 Gento 控制器只能有一个 SDK 客户端。

## 2. 启动前：物理与安全检查

1. 清空双大臂、双小臂周围区域，确认急停处于允许状态。
2. 给 Gento 控制器、双小臂及小臂 USB Hub 供电。
3. 插好两个小臂 FTDI USB 转串口适配器。脚本按稳定序列号识别，不能只按 `ttyUSB0/1` 判断左右：

   | 小臂 | FTDI 稳定设备名 |
   | --- | --- |
   | 左小臂 | `usb-FTDI_USB__-__Serial_Converter_FTB8HNOT-if00-port0` |
   | 右小臂 | `usb-FTDI_USB__-__Serial_Converter_FTAO51EA-if00-port0` |

4. 推荐先在主机确认两条稳定设备链接均存在：

   ```bash
   ls -l /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_*-if00-port0
   ```

   若缺少任意一条，检查 USB Hub 供电、Hub 上行线、FTDI 转串口线及小臂供电。此时不要启动同步脚本。

5. 确认 Gento 网络连通且没有其他 SDK 客户端：

   ```bash
   ping -c 2 6.6.7.190
   pgrep -af '[g]ento_robot_driver'
   ```

## 3. 完整启动

在主机终端执行：

```bash
cd /data/coding/tianji/Skye-mutile-arm/marvin_ws
./start_gento_dual_arm_sync.sh
```

看到下列提示、确认区域安全后，**直接按 Enter**：

```text
Press Enter to continue (Ctrl+C to cancel):
```

无需输入 `START_SYNC`。已经完成现场安全确认、需要非交互调用时：

```bash
./start_gento_dual_arm_sync.sh --yes
```

> 脚本会调用 Docker，终端请求 sudo 密码属于正常现象。不要将密码写入命令、脚本或日志。

## 4. 脚本的执行顺序

| 阶段 | 脚本动作 | 成功条件 |
| --- | --- | --- |
| 启动前检查 | 验证参数文件，检查不存在旧 Gento 驱动进程或旧 PID | 防止两个 SDK 客户端同时连接 |
| 1/4 小臂 | 设 FTDI `latency_timer=1`；重建 Docker；启动左右 factr + `keyboard_gripper` | `/factr_teleop_left`、`/factr_teleop_right` 在线 |
| 2/4 大臂 | 启动 Gento 驱动，连接 `6.6.7.190`，状态 500 Hz；服务 remap 到 `/gento/*` | 日志含 `Connected to Gento controller 6.6.7.190` |
| 3/4 状态流 | 等待 `/gento/joint_states`，输出 ROS 图和频率 | 14 个 rad 位置；频率接近 500 Hz |
| 4/4 键盘遥操 | 默认发布 `/enable_position_sync=false`；打印 `1/2/3` 说明 | 操作员用键盘控制，而非自动 sync |

**正式遥操流程（脚本结束后）：**

1. 键 **`1`**：小臂同步到大臂（factr `TELEOP_SYNCING → SYNCED`）
2. 键 **`2`**：进入遥操（factr `TELEOP`，经 `/gento/{left,right}_joint_control`）
3. 键 **`3`**：停止遥操输出

遗留调试：若仍需启动时自动 `enable_position_sync=true`，使用：

```bash
GENTO_AUTO_SYNC=1 ./start_gento_dual_arm_sync.sh
```

当前脚本每次启动都会检查并设置 FTDI 低延迟参数。因此 USB 重插后，**不需要**提前手工执行 `echo 1 > .../latency_timer`。

## 5. 启动成功后的核对

另开一个主机终端：

```bash
cd /data/coding/tianji/Skye-mutile-arm/marvin_ws
export ROS_DOMAIN_ID=20
set +u; source /opt/ros/humble/setup.bash; set -u
source gento_ros2_ws/install/setup.bash

ros2 topic info -v /gento/joint_states
ros2 topic info -v /gento/left_joint_control
ros2 topic info -v /gento/right_joint_control
ros2 service list | grep -E 'hold_current|stop_motion'
ros2 topic hz --window 500 /gento/joint_states
```

预期：

- `/gento/joint_states` 有一个 Gento 发布者和两个小臂订阅者；
- 驱动订阅 `/gento/left_joint_control` 与 `/gento/right_joint_control`；
- 存在 `/gento/hold_current` 与 `/gento/stop_motion`；
- 频率接近 500 Hz。

检查 Docker 内节点：

```bash
sudo docker exec gento_leader_teleop bash -lc '
  export ROS_DOMAIN_ID=20
  source /opt/ros/humble/setup.bash
  source /marvin_ws/install/setup.bash
  ros2 node list
'
```

预期至少有：

```text
/factr_teleop_left
/factr_teleop_right
/keyboard_gripper
```

## 6. 正常停止

### 停止遥操输出

优先在键盘焦点下按 **`3`**。也可调用驱动保持／急停：

```bash
cd /data/coding/tianji/Skye-mutile-arm/marvin_ws
export ROS_DOMAIN_ID=20
set +u; source /opt/ros/humble/setup.bash; set -u
source gento_ros2_ws/install/setup.bash
ros2 service call /gento/hold_current std_srvs/srv/Trigger "{}"
# 或：StopTraj + idle，之后需再次 hold_current 才允许命令
ros2 service call /gento/stop_motion std_srvs/srv/Trigger "{}"
```

### 停止 Gento 驱动

```bash
cd /data/coding/tianji/Skye-mutile-arm/marvin_ws
kill "$(cat .runtime/gento_dual_arm_sync/gento_robot_driver.pid)"
```

### 如需停止小臂容器

```bash
sudo docker rm -f gento_leader_teleop
```

下一次执行启动脚本会重新创建小臂容器，并重新连接大臂。

## 7. 常见故障处理

### A. `FTDI port missing: /dev/serial/by-id/...`

**含义：** 对应的小臂串口还未被主机识别。

**处理：** 检查小臂供电、USB Hub、Hub 上行线与 FTDI 转串口线；重新插拔后，确认第 2 节中的两个稳定序列号均存在，再运行脚本。不要靠 `ttyUSB` 临时编号猜测左右臂。

### B. `Please ensure the latency timer of ttyUSB0/ttyUSB1 is 1`

**含义：** 旧版手工启动命令或小臂节点检查 FTDI 低延迟参数失败。FTDI 设备重插或重启后可能恢复为非 1 ms。

**处理：** 使用当前的 `start_gento_dual_arm_sync.sh`；它会自动检查并设置为 1 ms。若脚本报告 `Cannot find latency_timer`，先解决 USB 枚举问题 A。

### C. `Both small-arm nodes did not remain online`

**含义：** 小臂容器虽启动，但至少一个控制节点退出。脚本会在此安全停止，不会连接 Gento。

**诊断：**

```bash
sudo docker logs --tail 160 gento_leader_teleop
```

若出现 Dynamixel `Hardware Error Status=32` 或 `Overload`，表示电机过载／机械卡滞告警。先解除机械顶住、检查负载并依现场安全规程断电重上电；不要反复重启节点或开启遥操。

### D. `Another gento_robot_driver process already exists` 或连接超时

**含义：** 另一个 Gento SDK 客户端正在占用控制器，或控制器网络不可达。

**处理：** 用下列命令确认进程和网络，停止不需要的 SDK 客户端后重试：

```bash
pgrep -af '[g]ento_robot_driver'
ping -c 2 6.6.7.190
```

### E. 话题存在，但按 1 后小臂未同步 / 按 2 后大臂不动

```bash
export ROS_DOMAIN_ID=20
source /opt/ros/humble/setup.bash
source gento_ros2_ws/install/setup.bash
ros2 topic echo --once /gento/joint_states
ros2 topic hz /gento/left_joint_control
ros2 topic hz /gento/right_joint_control
sudo docker logs --tail 160 gento_leader_teleop
tail -n 80 .runtime/gento_dual_arm_sync/gento_robot_driver.log
```

确认：`/keyboard_gripper` 在线；按 `2` 后控制话题有流量；驱动日志无 “outside limits” / “not ready”。**不要**启动 `skye_leader_bridge`。

## 8. 日志与运行状态

启动脚本把运行记录放在：

```text
.runtime/gento_dual_arm_sync/
├── gento_robot_driver.log       # 大臂 SDK/ROS 驱动日志
├── gento_robot_driver.pid       # 本脚本启动的驱动 PID
└── first_joint_state.yaml       # 首条完整 14 轴 rad 状态
```

查看大臂日志：

```bash
tail -n 160 .runtime/gento_dual_arm_sync/gento_robot_driver.log
```

本文覆盖 bridge-less factr 遥操恢复。夹爪控制、末端运动、SDK 独立测试与力控，应在本链路完全停止、且确认没有其他 Gento SDK 客户端时单独操作。
