# Orin 双臂启动遥操

本机为 **遥操设备 B（Orin）**。流程用主机包 `skye_factr_teleop`，**不进 Docker / 不用 `marvin_ws` 小臂**。

Thor 侧对照见 [`双臂启动遥操.md`](./双臂启动遥操.md)。旧 Docker 流程见仓库根目录 `docs/Thor_Orin_遥操启动.md`。

---

## 与 Thor 的关键差异

| 项 | Thor | Orin（本机） |
|---|---|---|
| `robot_profile` | `thor` | **`orin`** |
| 小臂配置目录 | `config/grav_comp_m6_*.yaml` | `config/orin/grav_comp_m6_*.yaml` |
| 左臂 FTDI by-id | `FTBEO9L1` | **`FTB8HNOT`** |
| 右臂 FTDI by-id | `FTBEO8WL` | **`FTAO51EA`** |
| Domain | **`20`** | **`21`**（A/B/D 必须一致；两机勿混用） |

日常 **不必** 跑 `bind_leader_arms.py`（那是旧 Docker/`marvin_ws` 用的）。串口已写在 Orin yaml；换 FTDI 再改对应 `dynamixel_port`。

---

## 0. 启动前：串口自检

两路小臂 USB 插好上电后：

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyUSB*
```

期望类似：

```text
..._FTAO51EA-if00-port0 -> ../../ttyUSB0   # 右
..._FTB8HNOT-if00-port0 -> ../../ttyUSB1   # 左
```

`ttyUSB` 编号可变，以 **by-id 序列号** 为准。缺失时：确认线缆/供电，或 `sudo udevadm trigger && sudo udevadm settle` 后再 `ls`。

设 FTDI 低时延（USB 重插或重启后需重做）：

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
```

---

## 1. 每个终端先做

```bash
source /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws/scripts/setup_factr_teleop_env.sh orin
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"   # Orin 期望 21（Thor 用 thor → 20）
```

会 `source` ROS + 工作区，并按 profile 设 `ROS_DOMAIN_ID`（orin=`21`，thor=`20`）与 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。

**注意：**

- 脚本打印的 `LEFT_CONFIG` / `RIGHT_CONFIG` 指向 Thor 路径，只是环境变量占位；**真正用哪份 yaml 由 launch 的 `robot_profile:=orin` 决定**，一般不用再传 `left_config` / `right_config`。
- 若本终端残留了旧流程的 `FASTRTPS_DEFAULT_PROFILES_FILE` 且文件已不存在，会刷 `[XMLPARSER Error] realpath failed`。消息仍可能发出，但建议清掉或改成有效路径：

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
# 或：
# export FASTRTPS_DEFAULT_PROFILES_FILE=/data/coding/tianji/Skye_ROS_Bridge/marvin_ws1/fastrtps_no_shm.xml
```

- **已在跑的节点不会随你事后改 shell 变量换 domain**；改 domain 后须重启 A/B。

---

## 2. 启动（三个终端）

| 终端 | 角色 | 命令 |
|---|---|---|
| A | 大臂 | 见下 |
| B | 双小臂 | 见下 |
| D | 模式切换 | 第 3 节（**不要**在 A/B 发 `switch_*`） |

### 终端 A — 大臂

```bash
source /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws/scripts/setup_factr_teleop_env.sh

ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  robot_profile:=orin "params_file:=$DRIVER_CONFIG"
```

看到连接成功、切入阻抗关节模式即可。日志里应加载 `profiles/orin.yaml`。

### 终端 B — 双小臂

先做第 0 节串口 + `latency_timer`，再：

```bash
source /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws/scripts/setup_factr_teleop_env.sh

ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py \
  robot_profile:=orin sync_diagnostics:=true
```

**必须带 `robot_profile:=orin`。** 漏写会默认 Thor，去找 `FTBEO9L1` / `FTBEO8WL`，报 `serial port does not exist`。

启动后两侧仍是 **IDLE、力矩关闭**。配置来自：

- `skye_ros2_ws/src/skye_factr_teleop/config/orin/grav_comp_m6_left.yaml`
- `skye_ros2_ws/src/skye_factr_teleop/config/orin/grav_comp_m6_right.yaml`

（launch 读的是 `install/.../share/.../config/orin/`；若只改了 `src` 且非 symlink install，需同步到 install 或 `colcon build --packages-select skye_factr_teleop`。）

---

## 3. 同步 → 遥操（终端 D，托住双小臂）

```bash
source /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws/scripts/setup_factr_teleop_env.sh
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"   # 须与 A/B 同为 21
```

`/mode/switch_*` 一次作用左右两侧。

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
# 两侧都 SYNCED 后：
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
# 静止 5～10 s，再慢动
```

状态机要点：

```text
IDLE ──switch_sync──► … ──► SYNCED   （使能；跟大臂反馈；不发 joint_control）
SYNCED ──switch_teleop──► TELEOP     （开始发 /gento/{left,right}_joint_control）
任意 ──switch_stop──► IDLE           （关小臂力矩；不自动 hold 大臂）
```

可选确认：

```bash
ros2 topic echo /teleop/state --once \
  --qos-durability transient_local --qos-reliability reliable
ros2 topic echo --once /gento/left_joint_states
ros2 topic echo --once /gento/right_joint_states
```

TELEOP 后小臂 J8（扳机）映射大臂夹爪：`/left|right_teleop_gripper/ctrl`。

---

## 4. 停止（托住双小臂）

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
ros2 service call /gento/hold_current std_srvs/srv/Trigger '{}'
```

---

## 5. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `serial port does not exist: ...FTBEO*` | 未设 `robot_profile:=orin` | 小臂 launch 加 `robot_profile:=orin` |
| `...FTAO51EA` / `FTB8HNOT` 不存在 | USB 未识别 | 第 0 节自检；拔插或 `udevadm trigger` |
| `ruckig.max_velocity is required` | Orin yaml 缺轨迹上限 | 在 `config/orin/grav_comp_m6_*.yaml` 填 `max_velocity` / `max_acceleration` / `max_jerk`，并同步到 install |
| `latency_timer ... expected 1` | FTDI 时延未设 | 第 0 节写 `latency_timer=1` 后重启小臂 |
| 发了 `switch_sync` 无反应 / `Waiting for subscription` | Domain 不一致 | A/B/D 都 `echo $ROS_DOMAIN_ID`，统一后**重启**对应 launch |
| `Rejected follower state: ... stale or in the future` | 大臂时间戳与小臂对不上（常见 domain/时钟问题） | 确认同 domain；必要时重启 A/B；检查系统时间 |
| `[XMLPARSER Error] realpath failed` | `FASTRTPS_DEFAULT_PROFILES_FILE` 指向失效路径 | `unset` 或改成存在的 xml（见第 1 节） |

杀残留大臂驱动（仅一台 SDK 客户端）：

```bash
pkill -f skye_robot_driver || true
pkill -f gento_robot_driver || true
```

---

## 相关

- Thor 双臂（同 reboot 包）：[`双臂启动遥操.md`](./双臂启动遥操.md)
- 完整说明 / 排障：[`小臂reboot操作说明.md`](./小臂reboot操作说明.md)
- 环境脚本：`skye_ros2_ws/scripts/setup_factr_teleop_env.sh`
- Orin 小臂配置：`skye_ros2_ws/src/skye_factr_teleop/config/orin/`
- 大臂参数：`skye_ros2_ws/src/skye_robot_driver/config/skye_robot.yaml` + `profiles/orin.yaml`
