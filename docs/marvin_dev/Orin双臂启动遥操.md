# Orin 双臂启动遥操

本机为 **遥操设备 B（Orin）**。用主机包 `skye_factr_teleop`，**不进 Docker、不用 `marvin_ws` 小臂**。

| 项 | 本机取值 |
|---|---|
| `robot_profile` | `orin` |
| `ROS_DOMAIN_ID` | **`21`**（A / B / D 必须相同） |
| 左臂 FTDI | `FTBEO9L1` |
| 右臂 FTDI | `FTBEO8WL` |
| 小臂配置 | `skye_factr_teleop/config/orin/grav_comp_m6_{left,right}.yaml` |

Thor 对照：[`双臂启动遥操.md`](./双臂启动遥操.md)（Thor domain=`20`）。旧 Docker：`docs/Thor_Orin_遥操启动.md`。

日常 **不必** `bind_leader_arms.py`。串口已写在 Orin yaml。

---

## 一分钟流程（照抄）

开 **3 个终端**，每个都先：

```bash
source /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws/scripts/setup_factr_teleop_env.sh orin
unset FASTRTPS_DEFAULT_PROFILES_FILE   # 避免指向已删除的 marvin_ws/fastrtps xml
echo "DOMAIN=$ROS_DOMAIN_ID PROFILE=$ROBOT_PROFILE"   # 期望 21 / orin
```

| 终端 | 做什么 |
|---|---|
| **A 大臂** | 下面「A」 |
| **B 双小臂** | 先做「串口」，再下面「B」 |
| **D 控制** | 托住小臂后再发 sync / teleop；**不要在 A/B 里发** |

### A — 大臂

```bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  robot_profile:=orin "params_file:=$DRIVER_CONFIG"
```

看到连上控制器、切入 `STATE_IMP_JOINT` 即可。

### B — 双小臂（先串口）

```bash
ls -l /dev/serial/by-id/
# 期望含 FTBEO8WL（右）、FTBEO9L1（左）

echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB1/latency_timer

ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py \
  robot_profile:=orin sync_diagnostics:=true
```

**必须** `robot_profile:=orin`。漏写会落到 Thor 配置目录；本机换臂后串口 by-id 虽与 Thor 同名，但 `joint_signs` 等参数仍以 orin yaml 为准。  
启动后是 **IDLE、力矩关**，小臂不会自己动。

### D — 同步 → 遥操（托住双小臂）

```bash
# 先确认大臂反馈在本 domain 可读：
ros2 topic hz /gento/left_joint_states
ros2 topic hz /gento/right_joint_states

ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
# 看 B 日志走到 SYNCED（两侧），再：
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
# 静止 5～10 s，再慢动
```

状态：

```text
IDLE ──switch_sync──► … ──► SYNCED   # 小臂使能，跟大臂；还不发大臂命令
SYNCED ──switch_teleop──► TELEOP     # 开始发 /gento/{left,right}_joint_control
任意 ──switch_stop──► IDLE           # 关小臂力矩；不自动 hold 大臂
```

确认：

```bash
ros2 topic echo /teleop/state --once \
  --qos-durability transient_local --qos-reliability reliable
```

TELEOP 后扳机 J8 → 大臂夹爪：`/left|right_teleop_gripper/ctrl`。

### 停止（托住双小臂）

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
ros2 service call /gento/hold_current std_srvs/srv/Trigger '{}'
```

---

## 串口说明

`ttyUSB*` 编号会变，以 **by-id** 为准：

```text
..._FTBEO8WL-if00-port0 -> ../../ttyUSB?   # 右
..._FTBEO9L1-if00-port0 -> ../../ttyUSB?   # 左
```

没有 by-id：检查供电/线缆，或：

```bash
sudo udevadm trigger && sudo udevadm settle
ls -l /dev/serial/by-id/
```

USB 重插或重启后 `latency_timer` 会复位，启小臂前再写一次 `1`。

---

## 改配置 / 重编注意

- Launch 读的是 `install/.../share/.../config/orin/`（symlink-install 时链到 `src`）。
- 只改 `src` 后若 share **不是** symlink：  
  `colcon build --packages-select skye_factr_teleop --symlink-install`  
  然后 **重启 B**。
- 脚本打印的 `LEFT_CONFIG`/`RIGHT_CONFIG` 仍是 Thor 路径占位；真正用哪份由 **`robot_profile:=orin`** 决定。

---

## 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `serial port ... FTB8HNOT` / `FTAO51EA` | 仍是旧 Orin by-id（换臂前） | 确认 orin yaml 已是 `FTBEO9L1`/`FTBEO8WL`，必要时 rebuild |
| `...FTBEO9L1` / `FTBEO8WL` 不存在 | USB 未起来 | 查 by-id；拔插 / `udevadm` |
| `latency_timer ... expected 1` | 时延未设 | 写 `latency_timer=1` 后重启 B |
| `Waiting for subscription` / sync 无反应 | A/B/D domain 不一致 | 都 `source ... orin`，确认 `21`，**重启** A/B |
| B 狂刷 `Rejected follower state: ... stale` | 大臂反馈时间戳不可用；**sync 不会启动** | 三端同为 21；`unset` 坏的 FASTRTPS 路径；必要时 `ROS_LOCALHOST_ONLY=1` 排除局域网串台后再试 |
| `Synchronization requires fresh follower state` | 同上，没有新鲜 `/gento/*_joint_states` | 先 `ros2 topic hz /gento/left_joint_states` 有约 250 Hz 再 sync |
| `gripper_setpoint` unknown / yaml 字段报错 | install 代码与 yaml 版本不一致 | `colcon build --packages-select skye_factr_teleop --symlink-install` 后重启 B |
| `[XMLPARSER Error] realpath failed` | `FASTRTPS_DEFAULT_PROFILES_FILE` 失效 | `unset FASTRTPS_DEFAULT_PROFILES_FILE` |
| 小臂只轻微动一下就停 | 常见是 stale 拒绝跟随，或旧 install 仍是 50 mA 验证电流 | 先看 B 是否刷 stale；再确认 orin yaml 的 `current_limit_ma` 已是 Thor 基线并已进 install |

杀残留大臂（只能有一个 SDK 客户端）：

```bash
pkill -f skye_robot_driver || true
pkill -f gento_robot_driver || true
```

---

## 相关

- Thor：[`双臂启动遥操.md`](./双臂启动遥操.md)
- 排障总览：[`小臂reboot操作说明.md`](./小臂reboot操作说明.md)
- 环境：`skye_ros2_ws/scripts/setup_factr_teleop_env.sh`
- Orin 小臂参数：`skye_ros2_ws/src/skye_factr_teleop/config/orin/`
- 大臂：`skye_robot_driver/config/skye_robot.yaml` + `profiles/orin.yaml`
