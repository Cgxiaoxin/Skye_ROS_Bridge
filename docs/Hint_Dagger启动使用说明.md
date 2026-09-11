# HITL（Hint）DAgger 启动说明

策略 rollout 失败时人工接管，从失败点继续采到成功数据。日常纯遥操不要走本流程。

更细的遥操上电见 `[Thor_Orin_遥操启动.md](Thor_Orin_遥操启动.md)`；设计见 `[superpowers/specs/2026-08-21-hitl-dagger-control-arbiter-design.md](superpowers/specs/2026-08-21-hitl-dagger-control-arbiter-design.md)`。

**真机状态（2026-09-10，Thor）：** dummy 策略 + `/skye/intervention_cmd` 已跑通  
`AUTONOMOUS` → `takeover` → `HUMAN` → `return` → `AUTONOMOUS`。  
P6.4 真 VLA / P6.5 训练加权仍待做。

---

## 前置条件


| 项                    | 要求                                                |
| -------------------- | ------------------------------------------------- |
| `ROS_DOMAIN_ID`      | `21`，与 driver / Docker / 推理机一致                    |
| `ROS_LOCALHOST_ONLY` | 关闭                                                |
| RMW                  | 建议 FastDDS；跨机需能发现节点                               |
| 策略消息                 | `chunk_size=16`；关节绝对角 rad；夹爪电机语义 0=开、1=闭          |
| Launch               | Docker 必须用 **HITL remap** 版，勿与日常 teleop launch 同开 |


---



## 启动顺序



### ① 上电

控制器上电，`ping 6.6.7.190`；确认无第二个 SDK 客户端（`pkill -f skye_robot_driver` 等）。

### ② 终端 A — 大臂 driver

```bash
# Thor
./scripts/start_skye_for_factr.sh

# Orin
ROBOT_PROFILE=orin ./scripts/start_skye_for_factr.sh
```



### ③ 终端 B — 小臂 Docker（HITL launch）

```bash
./scripts/run_marvin_m6_impedance.sh
# Orin: ROBOT_PROFILE=orin ./scripts/run_marvin_m6_impedance.sh
```

容器内：

```bash
source /marvin_ws/install/setup.bash
export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/marvin_ws/fastrtps_no_shm.xml

ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py use_keyboard:=false
```

小臂：`1` sync → 等稳 → 对齐（Orin：`start_follower_align.sh` / `s`）→ 再按需 teleop。  
接管时 arbiter 也会再发 sync/teleop。

### ④ 同步（可选另开终端）

```bash
export ROS_DOMAIN_ID=21
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String "{data: switch_sync}"
```

策略跑一段时间大臂位姿变了，可再 sync 一次对齐。

### ⑤ 终端 C — HITL arbiter

driver 已在跑时：

```bash
./scripts/start_hitl_host.sh --arbiter-only
# 录 mcap: ENABLE_RECORDER=true ./scripts/start_hitl_host.sh --arbiter-only --enable-recorder
# 残留进程: 加 --cleanup-stale
# 一次起 driver+HITL: ./scripts/start_hitl_host.sh
```



### ⑥ 终端 D — 策略源

```bash
export ROS_DOMAIN_ID=21
source skye_ros2_ws/install/setup.bash

# 假策略（联调）
ros2 run skye_hitl_dagger pub_dummy_policy_chunk

# 真 VLA：任意机器同 domain，发 PolicyActionChunk → /skye/policy_action
```



### ⑦ 观察

```bash
export ROS_DOMAIN_ID=21
source skye_ros2_ws/install/setup.bash
ros2 topic echo /skye/control_mode
# 期望初始: mode=AUTONOMOUS, source=policy（或 hold）
```

---



## 接管 / 交还


| 动作  | 推荐方式           | 期望 `control_mode`                           |
| --- | -------------- | ------------------------------------------- |
| 接管  | `takeover`（见下） | `HANDOVER_SYNC` → `HUMAN` / `source=teleop` |
| 交还  | `return`       | `AUTONOMOUS` / `source=policy`              |


**推荐：用 topic（launch 终端里直接按** `q`**/**`w` **常无效）：**

```bash
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'takeover'}"
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'return'}"
```

**可选：独立键盘终端**（保持 arbiter 在跑）：

```bash
export ROS_DOMAIN_ID=21
source skye_ros2_ws/install/setup.bash
ros2 run skye_hitl_dagger hitl_keyboard
# tty: q=takeover, w=return；非 TTY 则输入后按 Enter
```

说明：`w` 仅在 `HUMAN` 下有效；`AUTONOMOUS` 下按 `w` 无效果。

**接管对齐（`HANDOVER_SYNC`）：** 不会吃旧的 latched `SYNCED`；须先看到非对齐态（如 `TELEOP_SYNCING`）再等新的 `SYNCED`，并默认稳定 **`min_sync_hold_s:=2`** 秒后才 `switch_teleop`。卡死重发间隔为 `sync_timeout_s:=5.0`。

**hold 告警：** chunk 播完只 WARN 一次，之后静默继续 hold。

**return 后新 chunk：** 若 step0 相对当前大臂位姿跳变超过 `chunk_start_max_jump_rad`（默认 0.1 rad），会把整段轨迹平移到当前位姿再执行，避免交还瞬间突变。

---



## 验收要点

1. 初始 `AUTONOMOUS` + 策略（或 dummy）在控。
2. `takeover` → `HUMAN`，主臂可改轨迹。
3. `return` → `AUTONOMOUS`，继续跟 chunk。
4. 停策略发布 → 大臂 hold 末步，不自动切人；需再次 `takeover`。
5. （可选）开 recorder，bag 含 `/skye/control_mode`。

无真机 bench：`skye_ros2_ws/scripts/verify_hitl_p61_interfaces.sh`。

---



## 流程总览

```text
完整策略顺序：
① 上电 / ping / 清掉第二个 SDK 客户端
        ↓
② 终端 A：大臂 driver（同 Thor_Orin）
   Thor:  ./scripts/start_skye_for_factr.sh
   Orin:  ROBOT_PROFILE=orin ./scripts/start_skye_for_factr.sh
        ↓
③ 终端 B：小臂 Docker（同 Thor_Orin 进容器）
   但 launch 换成 HITL 版：
   ros2 launch .../start_teleop_m6_dual_gento_hitl.launch.py use_keyboard:=false
        ↓
④ （Orin）对齐：follower_align / s，和日常一样
   小臂可先 1 sync → 对齐 → 再视情况 2；接管时 q 也会再走 sync/teleop
        ↓
⑤ 终端 C：HITL（前台，用来按 q/w）
   ./scripts/start_hitl_host.sh --arbiter-only
        ↓
⑥ 终端 D：策略（本机 dummy 或远端 VLA → /skye/policy_action）
        ↓
⑦ 测：策略 AUTONOMOUS → 卡住按 q → HUMAN 遥操 → 按 w 交还
注意：不要同时开日常 teleop launch 和 HITL launch（会抢写大臂）。
```

