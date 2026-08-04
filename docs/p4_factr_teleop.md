# P4 · FACTR 小臂 ↔ Skye 大臂（bridge-less）

目标：小臂 Docker 内 `factr_teleop` 与主机 `skye_robot_driver` 对接，**不**再起 `gento_robot_driver`。

## 对齐点（相对旧 launch 的修正）

| 项 | 旧 `start_teleop_m6_dual.launch.py` | P4 `start_teleop_m6_dual_gento.launch.py` |
|----|--------------------------------------|---------------------------------------------|
| 指令 | `/left\|right_joint_control` | `/gento/left\|right_joint_control` |
| 反馈 | `/left\|right_joint_state`（7 轴分侧） | **`/gento/joint_states`（14 轴）** |
| Docker 挂载 | 曾误挂 `scripts/` | **`marvin_ws` → `/marvin_ws`** |
| 大臂驱动 | `gento_robot_driver` | **`skye_robot_driver`** |

小臂 yaml：`follower_joint_offset` 左=0、右=7，与 14 轴 `/gento/joint_states` 一致。

## QoS

| Topic | skye_robot_driver | 说明 |
|-------|-------------------|------|
| `/gento/joint_states` | **RELIABLE** KeepLast(1) | 供 FACTR sync 订阅（默认 Reliable） |
| `/gento/{left,right}_joint_control` | **BEST_EFFORT** KeepLast(1) | 可接收 FACTR Reliable 发布 |

## 启动顺序

### 1) 主机：大臂驱动

```bash
cd /data/coding/tianji/Skye_ROS_Bridge
./scripts/start_skye_for_factr.sh
# 另开终端确认：
source skye_ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=20
ros2 topic hz /gento/joint_states
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 2}"
```

### 2) Docker：小臂

```bash
./scripts/run_marvin_m6_impedance.sh
```

容器内：

```bash
source /marvin_ws/install/setup.bash
export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# FTDI 低延迟（按实际 ttyUSB）
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB1/latency_timer

# 必须用 gento remap 版 launch（不要用旧 start_teleop_m6_dual.launch.py）
ros2 launch factr_teleop start_teleop_m6_dual_gento.launch.py use_keyboard:=true
# 若 share 未同步，可用绝对路径：
# ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py use_keyboard:=true
```
### 3) 键盘

| 键 | 行为 |
|----|------|
| `1` | sync：小臂跟大臂 `/gento/joint_states` |
| `2` | teleop：发 `/gento/{left,right}_joint_control` |
| `3` | stop |

## 图检查（主机）

```bash
export ROS_DOMAIN_ID=20
ros2 node list | grep -E 'skye_robot_driver|factr_teleop'
ros2 topic info -v /gento/joint_states
ros2 topic info -v /gento/left_joint_control
```

期望：`joint_states` 有 skye Publisher + 两个 factr Subscription；  
`left/right_joint_control` 有 factr Publisher + skye Subscription。

## 不要做

- 同时启动 `gento_robot_driver` 与 `skye_robot_driver`
- 使用未 remap 的 `start_teleop_m6_dual.launch.py`（会订错反馈 topic）
- 把 `scripts/` 挂成 `/marvin_ws`（旧 bug，已修）

## 参考

- Topic 表：`docs/小臂rostopic.xlsx`
- 旧一键脚本（仍起 gento 驱动，仅作参考）：`marvin_ws/start_gento_dual_arm_sync.sh`
