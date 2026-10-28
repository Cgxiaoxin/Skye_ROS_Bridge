# 小臂 reboot 操作说明

本文说明 `skye_factr_teleop`（小臂重构包）如何替代旧 `marvin_ws` / `factr_teleop`（`.so`），
以及如何接入现有大臂驱动 `skye_robot_driver`，完成：**连接小臂 → 使能/同步（只读大臂）→ 切入遥操**。

更细的验收步骤与调参记录见包内 checklist；本文只保留日常最短路径。

| 详细文档 | 用途 |
|---|---|
| [README.zh-CN.md](../skye_ros2_ws/src/skye_factr_teleop/README.zh-CN.md) | 配置、话题、launch、模式语义 |
| [phase7_3 连接大臂后同步](../skye_ros2_ws/src/skye_factr_teleop/dev_docs/checklist/phase7_3_follower_sync_checklist.md) | **同步阶段只读大臂、不发大臂命令** |
| [phase7_4 左臂遥操](../skye_ros2_ws/src/skye_factr_teleop/dev_docs/checklist/phase7_4_left_teleop_checklist.md) | `SYNCED → TELEOP`，开始向大臂发命令 |
| [phase7_5 双臂全流程](../skye_ros2_ws/src/skye_factr_teleop/dev_docs/checklist/phase7_5_dual_follower_sync_teleop_checklist.md) | 双臂编译、启动、同步、遥操、失能回读 |

> 注意：仓库根目录 `docs/Thor_Orin_遥操启动.md`、`docs/小臂大臂启动步骤.md` 仍主要描述旧
> Docker + `factr_teleop` 流程。跑本 reboot 包时以本文和包内 checklist 为准。

---

## 1. 新旧对照（你在用哪套）

| 项目 | 旧系统 | reboot 后 |
|---|---|---|
| 小臂包 | `marvin_ws` 内 `factr_teleop`（含 `.so`） | `skye_ros2_ws/src/skye_factr_teleop`（纯源码） |
| 模型 | marvin 资源 | `skye_factr_description` |
| 启动 | `ros2 launch factr_teleop start_teleop_m6_*` / Docker 脚本 | `ros2 launch skye_factr_teleop start_teleop_m6_*` |
| 大臂 | 仍是 `skye_robot_driver` | 不变，配置见 `skye_robot.yaml` / `profiles/thor.yaml` |
| 模式话题 | `/mode/switch_{sync,teleop,stop}` | **同名，契约兼容** |

状态机要点（小臂节点）：

```text
启动后 torque 关闭，状态 IDLE
  │
  ├─ switch_sync  ──► TELEOP_SYNCING → HOLD_SETTLING → SYNCED
  │                   （使能力矩；截取大臂反馈作为目标；不发 /gento/*_joint_control）
  │
  ├─ switch_teleop（仅 SYNCED 允许）──► TELEOP
  │                   （开始发布 7 轴到 /gento/*_joint_control，大臂才会跟）
  │
  └─ switch_stop  ──► IDLE（关力矩；不自动调用大臂 hold_current）
```

**关键区分：**

- **同步阶段**：只**订阅** `/gento/<side>_joint_states`，**不发布** `/gento/<side>_joint_control`。
  这就是「只读大臂数据、不给大臂发数据」。
- **遥操阶段**：才会往大臂发位置命令。若只要练小臂 / 验证同步，**停在 SYNCED，不要发 `switch_teleop`**。

`joint_move` 得到的 SYNCED **不授予**遥操资格；要遥操必须先走真实大臂反馈的 `switch_sync`。

---

## 2. 环境与编译（每个终端先做）

工作区根目录以本仓库为准（示例路径可改）。**终端 A / B / D 各自开壳后都要跑本节**
（`source` 与 `export` 不会跨终端继承；漏做就会「节点已起、命令端看不见」）。

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
# 首次或源码/launch 变更后：
colcon build --packages-up-to skye_factr_teleop --symlink-install
colcon build --packages-select skye_robot_driver \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.bash

# 必须显式 export；不设则默认为 0。A/B/D 必须完全相同。
# 本流程约定 73（与 phase7 checklist 一致）。
# 切勿残留旧 Thor/Orin Docker 流程的 21（见 docs/Thor_Orin_遥操启动.md）。
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"   # 启动前自检，三端应打印同一数字

SIDE=left   # 或 right；双臂见第 6 节（# 前须有空格，勿写成 SIDE=left#）
CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_${SIDE}.yaml"
DRIVER_CONFIG="$PWD/src/skye_robot_driver/config/skye_robot.yaml"
```

串口（设备 A / Thor 示例）：左 `FTBEO9L1`，右 `FTBEO8WL`。核对 by-id、权限、无占用，并把
FTDI `latency_timer` 设为 `1`（USB 重插或重启后需重设）。完整命令见
[同步清单 2.1 节](../skye_ros2_ws/src/skye_factr_teleop/dev_docs/checklist/phase7_3_follower_sync_checklist.md#21-启动前准备编译串口检查与设置)。

启动前停掉旧 `factr_teleop`、另一侧冲突节点、`skye_follower_align` 及其它会发运动命令的进程。

若出现 `Package 'skye_factr_teleop' not found` 且提示只在搜 `/opt/ros/humble`：
说明本终端未 `source install/setup.bash`（或未先 colcon build）。

---

## 3. 连接小臂 + 只读大臂同步（使能，不发大臂命令）

目标：大臂上电并发布关节反馈；小臂 `switch_sync` 后走到目标并保持；**全程 `joint_control` 无有效遥操输出**。

### 3.1 终端分工

| 终端 | 作用 | 注意 |
|---|---|---|
| A | 大臂驱动 | launch 占住该终端，**不要**在此发 `switch_*` |
| B | 本侧小臂 launch | 启动后仍是 **IDLE、力矩关闭**；占住该终端 |
| C（可选） | 同步记录 `skye_factr_sync_trial record` | |
| D | 模式切换 / 查看反馈 | **在此**跑 `topic echo` / `switch_sync` |
| E（可选） | `ros2 topic echo /teleop/state` | |

### 3.2 启动

先确认 A/B/D 都已按第 2 节 `source` + `export ROS_DOMAIN_ID=73`。

**终端 A — 大臂（提供只读反馈）：**

```bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  robot_profile:=thor "params_file:=$DRIVER_CONFIG"
```

**终端 B — 小臂（启动后仍 IDLE，力矩关闭）：**

```bash
ros2 launch skye_factr_teleop "start_teleop_m6_${SIDE}.launch.py" \
  robot_profile:=thor "${SIDE}_config:=$CONFIG" sync_diagnostics:=true
```

**终端 D — 确认大臂反馈可读、姿态安全（先自检再 echo）：**

```bash
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"   # 须与 A/B 相同（约定 73）
ros2 topic list | grep -E 'left_joint_states|switch_sync'   # 应立刻有输出
ros2 topic echo --once "/gento/${SIDE}_joint_states" sensor_msgs/msg/JointState
# 核对 l_j1..l_j7 或 r_j1..r_j7，目标须在小臂安全范围内；大臂保持静止
```

若 `topic list` 几乎只有 `/rosout`，或 `echo` / `pub --once` 一直卡住：多半是本终端
`ROS_DOMAIN_ID` 与 A/B 不一致（常见残留旧流程的 `21`，或一端忘了 export 落在默认 `0`）。
先 `echo $ROS_DOMAIN_ID`，对齐后重开命令；**已 launch 的节点不会随你事后改 shell 变量换 domain**，
必要时停掉 A/B，统一 domain 后再启。

### 3.3 使能 / 同步（截取大臂当前位置，小臂跟上）

仍在**终端 D**（托住小臂）：

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
```

预期状态：

```text
IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED
```

此时：

- 小臂已使能力矩，并保持在截取到的大臂姿态（+ YAML 夹爪目标）；
- **不会**向 `/gento/${SIDE}_joint_control` 发遥操命令；
- 大臂须继续发新鲜 `joint_states`；反馈过期小臂会停并回 IDLE。

可选核对「没在发大臂命令」：

```bash
ros2 topic echo "/gento/${SIDE}_joint_control"   # 同步阶段应无持续遥操流
# 若用 sync_trial record，汇总里 follower_command_count 应为 0
```

### 3.4 停止小臂（关力矩）

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

`switch_stop` **不会**自动调用大臂 `hold_current`。若大臂也需要受控保持，另开：

```bash
ros2 service call /gento/hold_current std_srvs/srv/Trigger '{}'
```

退出小臂节点后，建议用只读工具回读本侧 Torque Disable（见 checklist）。

---

## 4. 切入遥操（会开始给大臂发数据）

前提：已通过**真实大臂反馈**的 `switch_sync`，状态为 `SYNCED`（不是 `joint_move` 假同步）。

```bash
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
```

进入 `TELEOP` 后：

- 小臂开始向 `/gento/<side>_joint_control` 发布 7 轴位置；
- 大臂 `skye_robot_driver` 按 `teleop_mapping_mode`（当前多为 `relative`）跟随；
- 首帧会捕获参考；建议先静止观察 5～10 s，再小幅慢动。

停止仍用 `switch_stop`，再视需要 `hold_current`。

单臂遥操记录可用：

```bash
ros2 run skye_factr_teleop skye_factr_teleop_trial \
  --side left --config "$CONFIG" --driver-config "$DRIVER_CONFIG" \
  --operator <name> --output /tmp/left_teleop_01.json \
  --duration 180 --post-stop-seconds 3
```

> 若你的目标是「小臂进某种操作感、但大臂绝对不动」：  
> **不要发 `switch_teleop`**，停在第 3 节的 `SYNCED`；  
> 或干脆不启动会消费 `joint_control` 的大臂跟踪（仅调试时）。  
> 软件上没有「TELEOP 但不发布 joint_control」的独立模式。

---

## 5. 接入现有遥操系统时要注意什么

1. **Domain / RMW 一致（最常见翻车点）**：  
   - 本 reboot 流程约定 **`ROS_DOMAIN_ID=73`** + `rmw_fastrtps_cpp`。  
   - 旧 Docker / `start_skye_for_factr.sh` / `Thor_Orin_遥操启动.md` 常用 **`21`**——**不要混用**。  
   - 未 `export` 时 ROS 默认为 **`0`**；若 A/B 在 `0`、命令端仍留着 `21`，就会出现
     `Waiting for at least 1 matching subscription(s)...` 或 `topic echo` 一直无输出。  
   - 每个新终端都要重新 `export`；改 domain 必须**重启**已启动的 launch。
2. **话题契约不变**：大臂仍吃 `/gento/*_joint_states` / `*_joint_control`；小臂 reboot 包 launch 已 remap 到这些名字。
3. **模式话题全局共享**：双臂同时起来时，一次 `switch_sync` / `switch_teleop` 两侧都会收到，各自按本侧反馈新鲜度决定是否切换。
4. **配置来源**：
   - 小臂：`skye_factr_teleop/config/grav_comp_m6_{left,right}.yaml`（Orin 用 `config/orin/`）
   - 大臂：`skye_robot_driver/config/skye_robot.yaml` + `profiles/thor.yaml`  
     （本分支常见：IP `6.6.7.191`，速度/加速度比 `10`，`max_delta_per_cycle=0.001`，右臂 J6/J7 signs 为 `+1`）
5. **与 Operator UI / 旧 Docker**：UI 与 `run_marvin_m6_impedance.sh` 默认仍指向旧 `factr_teleop`。  
   要用 reboot 包，需改启动命令为 `skye_factr_teleop` 的 launch，或暂时手动按本文启节点，再让 UI 只负责编排其它部分。
6. **不要混跑**：同一串口不要同时跑旧 FACTR 与 `skye_factr_teleop`；也不要一边按旧文档起 Docker 小臂、一边按本文起 `skye_factr_teleop`。

---

## 6. 双臂最短路径（摘要）

```bash
# 大臂
ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  robot_profile:=thor "params_file:=$DRIVER_CONFIG"

# 双小臂
LEFT_CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_left.yaml"
RIGHT_CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_right.yaml"
ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py \
  robot_profile:=thor \
  left_config:="$LEFT_CONFIG" right_config:="$RIGHT_CONFIG" \
  sync_diagnostics:=true

# 同步（两侧一起）→ 确认 SYNCED → 再遥操
ros2 topic pub --once /mode/switch_sync  std_msgs/msg/String '{data: switch_sync}'
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
ros2 topic pub --once /mode/switch_stop   std_msgs/msg/String '{data: switch_stop}'
```

双臂统一记录：`ros2 run skye_factr_teleop skye_factr_dual_trial --help`。  
完整步骤与失能回读见 [phase7_5 checklist](../skye_ros2_ws/src/skye_factr_teleop/dev_docs/checklist/phase7_5_dual_follower_sync_teleop_checklist.md)。

---

## 7. 常用检查命令

```bash
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
ros2 topic list | grep -E 'gento/|mode/switch_|teleop/state'

ros2 topic echo /teleop/state std_msgs/msg/String --qos-durability transient_local
ros2 topic echo --once /gento/left_joint_states
ros2 topic echo --once /gento/robot_state   # 期望 imp_joint，常见 [2,2]
ros2 topic info --verbose /gento/left_joint_control
ros2 topic info --verbose /mode/switch_sync   # Subscription count 应 ≥ 1（小臂节点）

ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left --config "$CONFIG" --frames 2 --interval 0 \
  --output /tmp/left_torque_disable.json
```

### 7.1 症状速查

| 现象 | 常见原因 | 处理 |
|---|---|---|
| `package 'skye_factr_teleop' not found`（只搜 `/opt/ros/humble`） | 未 `source install/setup.bash` | 第 2 节重新 source；必要时先 colcon build |
| `Waiting for at least 1 matching subscription(s)...` | 本终端 domain 与小臂节点不一致 | `echo $ROS_DOMAIN_ID`，对齐为 `73` 后重发；必要时重启 A/B |
| `topic echo` 一直无数据 / `topic list` 几乎空 | 同上，或 `SIDE` 未设变成 `/gento/_joint_states` | 对齐 domain；显式 `SIDE=left` |
| 小臂 launch 已起但仍软绵绵 | 正常：启动后是 IDLE、力矩关 | 在终端 D 发 `switch_sync` 才使能 |

---

## 8. 文档索引（按场景）

| 你想做的事 | 打开 |
|---|---|
| 理解包能力与参数 | `README.zh-CN.md` |
| **连大臂、只同步、不发命令** | `phase7_3_follower_sync_checklist.md`（本文第 3 节） |
| 无大臂时只练小臂走位 | `phase7_3_sync_trial_checklist.md` |
| 同步成功后切入关节遥操 | `phase7_4_left_teleop_checklist.md`（本文第 4 节） |
| 双臂联调验收 | `phase7_5_dual_follower_sync_teleop_checklist.md` |
| 启动/话题流程图 | `dev_docs/understand/skye_factr_teleop_launch_flow.md` |
