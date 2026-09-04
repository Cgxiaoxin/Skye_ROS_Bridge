# Thor / Orin 双机遥操启动

一套代码，用 `ROBOT_PROFILE`（或 launch `robot_profile:=`）区分两台机器。  
**主机大臂驱动**与 **小臂 Docker** 必须同一 profile、同一 `ROS_DOMAIN_ID=21`。

更细的排障、HITL、夹爪专题见 `docs/小臂大臂启动步骤.md`；换主臂串口见 `docs/新主臂串口绑定.md`。

---

## 两台差异


|                  | Thor                      | Orin                               |
| ---------------- | ------------------------- | ---------------------------------- |
| Profile          | `thor`（默认，可不写）            | `orin`（必须显式）                       |
| 夹爪               | DM4310（末端 CAN）            | Robotiq Hand-E（RS485）              |
| 大臂 `joint_signs` | 左右全 `+1`                  | 左全 `+1`，右 J6/J7 = `-1`             |
| Robotiq 接线       | —                         | 左 ARM0 / 右 ARM1，均 `485A`，slave=`9` |
| Robotiq 闭合开度     | —                         | 左 `2.0` mm / 右 `13.0` mm           |
| 小臂标定目录           | `marvin_ws/configs/thor/` | `marvin_ws/configs/orin/`          |


Profile 参数文件：

- `skye_ros2_ws/.../config/profiles/thor.yaml`
- `skye_ros2_ws/.../config/profiles/orin.yaml`

---



## 上电前（两台相同）

在仓库根目录 `Skye_ROS_Bridge/` 下操作。

1. 控制器上电，网线通：`ping -c 2 6.6.7.190`
2. 小臂对齐蓝线/红标后再上电；两路 FTDI 已插入
3. 确认没有第二个 SDK 客户端：

```bash
pkill -f skye_robot_driver || true
pkill -f gento_robot_driver || true
```

1. 调试终端若另开，自行保证：

```bash
export ROS_DOMAIN_ID=21
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/marvin_ws/fastrtps_no_shm.xml"
```

（`start_skye_for_factr.sh` / `run_marvin_m6_impedance.sh` 会自己设这些，不必写进 `~/.bashrc`。）

---



## Thor 启动



### 终端 A — 大臂驱动

```bash
export ROS_DOMAIN_ID=21
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/marvin_ws/fastrtps_no_shm.xml"
./scripts/start_skye_for_factr.sh
# 等价: ROBOT_PROFILE=thor ./scripts/start_skye_for_factr.sh
```

日志应出现 profile=`thor`，夹爪类型 `dm4310`。

### 终端 B — 小臂 Docker

```bash
cd /Skye_ROS_Bridge
./scripts/run_marvin_m6_impedance.sh
# 等价: ROBOT_PROFILE=thor ./scripts/run_marvin_m6_impedance.sh
```

进 Docker 后键盘：`1` sync → 等稳 → **对齐（见下）** → `2` teleop（`3` stop）。

---



## Orin 启动



### 终端 A — 大臂驱动

```bash
cd /Skye_ROS_Bridge

export ROS_DOMAIN_ID=21
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/marvin_ws/fastrtps_no_shm.xml"
ROBOT_PROFILE=orin ./scripts/start_skye_for_factr.sh
```

日志应出现 profile=`orin`，夹爪类型 `robotiq`。

### 终端 B — 对齐辅助（需新开）

```bash
ROBOT_PROFILE=orin ./scripts/start_follower_align.sh
# 焦点在该终端，按 s 开始对齐
```

或用命令发布对齐模式：

```bash
ros2 topic pub --once /mode/align_follower std_msgs/msg/String "{data: align_follower}"
```

查看当前对齐状态：

```bash
ros2 topic echo /align/status
```



### 终端 B — 小臂 Docker（必须同一 orin）

```bash
cd /Skye_ROS_Bridge
ROBOT_PROFILE=orin ./scripts/run_marvin_m6_impedance.sh

source /marvin_ws/install/setup.bash
export ROS_DOMAIN_ID=21
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_6FILE=/marvin_ws/fastrtps_no_shm.xml
ls "$FASTRTPS_DEFAULT_PROFILES_FILE"   # 必须存在；须在 launch 之前 export
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
# 推荐：直接 launch overlay（无需 cp，始终最新）
ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py use_keyboard:=false
```

同样：`1` sync → 等稳 → **对齐（见下）** → `2` teleop。

### 同步对齐

```bash
export ROS_DOMAIN_ID=21
ros2 topic pub --once /mode/switch_stop  std_msgs/msg/String "{data: switch_stop}"
sleep 1
ros2 topic pub --once /mode/switch_sync  std_msgs/msg/String "{data: switch_sync}"
# 启动之前需要先另开一个终端（运行该脚本）： cmd2，其他全部cmd1
scripts/start_follower_align.sh
#  大臂与小臂对齐
ros2 topic pub --once /mode/align_follower std_msgs/msg/String "{data: align_follower}"
# 查看同步的状态
ros2 topic echo /align/status
# IDLE → ALIGNING → ALIGNED（或 TIMEOUT_WARN，仍可开遥操）

# 托住小臂，等 SYNCED（可能数秒～十几秒）
ros2 topic echo /teleop/state --once \
  --qos-durability transient_local --qos-reliability reliable
# 看到 SYNCED 后再：
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String "{data: switch_teleop}"
```

---



## 对齐（FACTR sync 之后）

FACTR `1` 让小臂跟大臂；小臂重力补偿偏弱时 sync 后仍可能有位姿残差。在进相对遥操前，主机用 **Docker 外** 终端让大臂以小速度绝对跟小臂。

**推荐流程（可保持 SYNC，小臂不易下垂）：**

1. Docker：`1` sync 完成并稳住
2. 主机：已运行 `start_follower_align.sh`，按 `s`（或 pub `align_follower`）
3. 驱动在 abs 对齐期间会**暂时忽略** FACTR 的相对 `/gento/*_joint_control`（方案 B）
4. `/align/status` → `ALIGNED` / `TIMEOUT_WARN` 后，对齐节点 `hold` 并清会话 → **相对流自动可恢复**
5. Docker：`2` 开相对遥操（首帧会重新 capture leader/gento ref，大臂不会被旧会话带着跑）

不必再强制 `3`/STOP；若仍习惯 `1 → 3 → s → 2` 也可以。

核对对齐在动：

```bash
ros2 topic hz /gento/left_joint_control_abs
ros2 topic echo /align/status --qos-durability transient_local --qos-reliability reliable
```

驱动日志若出现 `relative/teleop cmd ignored while absolute path is active` 属正常（SYNC 仍在发相对、但被忽略）。

主机启动对齐 helper：

```bash
ROBOT_PROFILE=orin ./scripts/start_follower_align.sh   # 或 thor
# 按 s → 大臂 10% 速度绝对跟小臂；ALIGNED / TIMEOUT_WARN 后
# Docker 再按 2 开遥操；x 取消对齐
```

等价:

```bash
ros2 topic pub --once /mode/align_follower std_msgs/msg/String "{data: align_follower}"
```

观察状态:

```bash
ros2 topic echo /align/status --qos-durability transient_local --qos-reliability reliable
# IDLE → ALIGNING → ALIGNED（或 TIMEOUT_WARN，仍可开遥操）
```



### 等价手写 launch

```bash
export ROBOT_PROFILE=orin   # 或 thor，须与 Docker / 大臂一致
source skye_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=21
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/marvin_ws/fastrtps_no_shm.xml"
ros2 launch skye_follower_align follower_align.launch.py \
  robot_profile:=${ROBOT_PROFILE} enable_keyboard:=true
```

---



## 换主臂串口 / sync（须与机台同 profile）

```bash
# Orin
export ROBOT_PROFILE=orin
./scripts/sync_marvin_overlay.sh

# Docker 内绑定（两臂 USB 已插、factr 未占串口）
python3 /scripts/bind_leader_arms.py
source /marvin_ws/.skye/leader_arms.env
```

Thor 可省略或 `export ROBOT_PROFILE=thor`。  
`bind` 写 `marvin_ws/configs/<profile>/grav_comp_m6_*.yaml`；`sync` 从该目录拷到 install。混用 profile 会导致小臂 `joint_signs` 与大臂不一致。

详见 `docs/新主臂串口绑定.md`。

---



## 启动后自检

```bash
export ROS_DOMAIN_ID=21
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/marvin_ws/fastrtps_no_shm.xml"
source skye_ros2_ws/install/setup.bash

ros2 topic hz /gento/joint_states --window 100   # 期望 ~250 Hz
ros2 topic echo /gento/left_joint_states --once  # 7 轴
ros2 topic echo /gento/right_joint_states --once # 7 轴，应对右大臂
```


| 检查项  | Thor      | Orin                         |
| ---- | --------- | ---------------------------- |
| 夹爪类型 | `dm4310`  | `robotiq`                    |
| 右腕遥操 | 同向        | J6/J7 映射后同向（profile 已设 `-1`） |
| 夹爪扳机 | 松开=开、按下=闭 | 同左；闭合限位左约 2 mm、右约 13 mm      |


---



## 不要做


| 错误                             | 后果                 |
| ------------------------------ | ------------------ |
| 主机 `orin`、Docker 默认 `thor`     | 标定 / signs 错乱      |
| Orin 不设 profile，走默认 thor       | 当成 DM4310 + 全 `+1` |
| 同时开 `gento_robot_driver`       | SDK 互抢             |
| 主机与 Docker `ROS_DOMAIN_ID` 不一致 | 互相看不见              |
| 漏设 FastDDS 关 SHM xml           | Docker 侧可能看不到驱动节点  |


---



## 相关文件


| 路径                                   | 作用                            |
| ------------------------------------ | ----------------------------- |
| `scripts/start_skye_for_factr.sh`    | 主机起 `skye_robot_driver`       |
| `scripts/start_follower_align.sh`    | 主机对齐节点 + 键盘 `s`/`x`（Docker 外） |
| `scripts/run_marvin_m6_impedance.sh` | 小臂 Docker + sync overlay      |
| `scripts/sync_marvin_overlay.sh`     | 按 profile 同步 launch/config    |
| `scripts/bind_leader_arms.py`        | 绑定左右主臂 FTDI                   |
| `config/profiles/{thor,orin}.yaml`   | 大臂 / 夹爪机台参数                   |
| `marvin_ws/configs/{thor,orin}/`     | 小臂 FACTR `grav_comp`          |


