# HITL DAgger 现场联调步骤（同构遥操 · Thor / Orin）

> 基于 2026-11-04 Thor 同机调试实跑整理。  
> 配套：[`phase1_hitl_isomorphic_startup.md`](phase1_hitl_isomorphic_startup.md)、[`skye-hitl-dagger-refactor-plan.md`](skye-hitl-dagger-refactor-plan.md)

**本轮已验证：** Domain 对齐后，`takeover` → `SYNCED` → `enter_teleop` → `HUMAN` 双臂遥操可通；`gento` 写臂仅 `control_arbiter`。

---

## 0. 每次开跑前固定环境

Thor 示例（Orin 把 `20/thor` 换成 `21/orin`）：

```bash
export ROBOT_PROFILE=thor
export ROS_DOMAIN_ID=20          # 每个终端都必须有，否则会「半通」
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /data/coding/tianji/Skye_ROS_Bridge
source skye_ros2_ws/install/setup.bash
echo "DOMAIN=$ROS_DOMAIN_ID PROFILE=$ROBOT_PROFILE"
```

**禁止：** 与纯遥操 `start_teleop_m6_dual.launch.py` 同 Domain 混开；第二路 `skye_robot_driver`。

缺包时先编：

```bash
cd skye_ros2_ws
colcon build --packages-select skye_factr_teleop skye_hitl_dagger skye_robot_driver --symlink-install
source install/setup.bash
```

---

## 1. 起图（三个终端）

| 终端 | 作用 | 命令 |
|------|------|------|
| A | 大臂 driver | `./scripts/start_skye_for_factr.sh thor`  
确认打印 `ROS_DOMAIN_ID=20`、连上 `6.6.7.191` |
| B | HITL 小臂 | `ros2 launch skye_factr_teleop start_teleop_m6_dual_hitl.launch.py robot_profile:=thor`  
期望两侧 `IDLE` |
| C | Arbiter | `ros2 launch skye_hitl_dagger hitl_dagger.launch.py robot_profile:=thor`  
不应再报缺 `config/profiles/thor.yaml` |

可选 D：假策略（测 AUTONOMOUS 运动时再开）

```bash
ros2 run skye_hitl_dagger pub_dummy_policy_chunk
```

也可用 Operator UI → 模式 **DAgger** → 开始会话（playbook 已主机化）。

---

## 2. 图测（验收 Phase 1）

```bash
export ROS_DOMAIN_ID=20
source skye_ros2_ws/install/setup.bash

ros2 topic echo /teleop/state --once          # IDLE
ros2 topic info /gento/left_joint_control_abs # Publisher=arbiter, Subscription≥1(driver)
ros2 topic info --verbose /skye/teleop_action_left  # 进 TELEOP 前 Publisher 可为 0
```

期望 HITL on：`/gento/*_joint_control(_abs)` 策略/遥操侧发布者 **只有** `control_arbiter`。

---

## 3. 接管 → 遥操 → 交还

### 3.1 同步（takeover）

先确认小臂 J8/限位在安全区内（见 §5），再：

```bash
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'takeover'}"
```

- 期望：`/skye/control_mode` → `HANDOVER_SYNC`；小臂日志出现 `TELEOP_SYNCING` → `SYNCED`
- 观察：`ros2 topic echo /teleop/state`、`/skye/control_mode`
- **两侧都 `SYNCED` 后再往下**；一侧 reject 时不要 `enter_teleop`

### 3.2 进入遥操

```bash
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'enter_teleop'}"
```

期望：`HUMAN` / `source=teleop`；两侧 `TELEOP：遥操中`；轻动小臂大臂跟随。

### 3.3 交还策略（推荐）

```bash
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'return'}"
```

期望：`AUTONOMOUS`；无 dummy 时大臂 hold 当前姿。

### 3.4 停小臂模式

**正确顺序（重要）：**

```bash
# 1) 先让 arbiter 离开 HANDOVER_SYNC / HUMAN（若还在接管链路）
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'return'}"

# 2) 再停小臂状态机
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String "{data: switch_stop}"
```

只发 `switch_stop`、**不** `return`：小臂会回 `IDLE`，但 **arbiter 若仍停在 `HANDOVER_SYNC`，会每 ~5s 重发 `switch_sync`**，表现为「停了又自己同步」。见 §5.2。

### 3.5 收工

Ctrl+C：C arbiter → B 小臂 → A driver；托住小臂再去使能。

---

## 4. 命令速查

| 意图 | Topic | data |
|------|-------|------|
| 同步接管 | `/skye/intervention_cmd` | `takeover` |
| 进遥操 | `/skye/intervention_cmd` | `enter_teleop` |
| 交还策略 | `/skye/intervention_cmd` | `return` |
| 小臂停（须在 return 之后或确认非 HANDOVER） | `/mode/switch_stop` | `switch_stop` |

---

## 5. 本轮已知问题与对策

### 5.1 切 TELEOP 后右臂 J4/J5「外力 / 回弹」

**现象：** 进 `HUMAN` 后右腕（大臂）J4/J5 像被拽；动小臂会弹回原位。

**较可能原因（按优先级）：**

1. **进入遥操前 arbiter 长时间 `re-publishing switch_sync`**  
   即使 `/teleop/state` 已是 `SYNCED`，`HANDOVER_SYNC` 下约每 5s 仍重发 sync。会反复打断/重开同步轨迹，切换瞬间阻抗/相对会话容易「顶牛」。  
   → 对齐后尽快 `enter_teleop`，不要在 `SYNCED` 空等很久。

2. **`hold_current` → relative 首帧 reseed**  
   `enter_teleop` 会调 `/gento/hold_current` 冻姿再切相对遥操；若 reseed 与人手增量不同步，大臂会像弹簧回到 hold 点。  
   → 进 TELEOP 后先轻推确认跟随，再大幅运动；交接时托住大臂。

3. **限位/标定边缘**  
   本轮右臂 follower 目标曾出现 `J1≈-2.94`（超小臂安全界 `±2.9`），说明大臂姿态已贴边；腕部 J4/J5 在阻抗下更容易「自己拧」。  
   → 同步前把大小臂摆到工作区中部再 takeover。

4. **小臂重力补偿手感**（若弹的是小臂本身）  
   TELEOP 下 leader 是力矩重力补偿，不是锁位；J4/J5 偏沉会「往回掉」，与大臂跟随是两件事。

**建议下次抓日志：**

```bash
ros2 topic echo /skye/control_mode
# 进 HUMAN 后确认不再刷 sync timeout / switch_sync
# 同时看小臂是否仍出现 TELEOP_SYNCING
```

代码侧待跟进：`SYNCED` 且未请求 `enter_teleop` 时不应无限重发 `switch_sync`（或应封顶/改等操作员）。

### 5.2 `switch_stop` 后左臂又自己同步、右臂不同

**根因（日志已证实）：**

- **已修（A/B）：** `switch_stop`（或 `intervention_cmd=stop`）会让 **arbiter → AUTONOMOUS** 并取消 handover；`SYNCED` 后不再无限 `switch_sync`。
- 确定性预检失败（超界等）会记入 `sync_progress`，**不拖死**已 SYNCED 的另一侧；双臂都确定性失败或重试耗尽 → `FAULT`。
- 详见 [`hitl_arbiter_state_machine.md`](hitl_arbiter_state_machine.md)。

**停法：** 直接 `switch_stop` 即可（幂等）；不必再记「先 return 再 stop」。

### 5.3 同步预检拒绝（限位 / URDF）

| 日志摘要 | 处理 |
|----------|------|
| 左 `gripper(ID8)=-0.035` 超 `[-0.0175, 0.525]` | 手掰 J8 进范围；后续改 URDF/限位 |
| 右 `target J1=-2.94` 超 `±2.9` | 大臂/目标姿态超界，先把大臂挪回工作区再 sync |
| `leader is moving` | 停稳后再 takeover |

---

## 6. 推荐最小验收清单

- [ ] 三终端 Domain 均为 20（Thor）
- [ ] abs topic：仅 arbiter pub，driver sub ≥ 1
- [ ] `takeover` → 双侧 `SYNCED`（无持续 reject）
- [ ] 尽快 `enter_teleop` → `HUMAN`，双侧 `TELEOP`
- [ ] 轻遥操跟随正常
- [ ] `return` → `AUTONOMOUS`
- [ ] `switch_stop` 后 arbiter **不再**刷 `re-publishing switch_sync`
- [ ] （可选）dummy chunk 下 AUTONOMOUS 有运动

---

## 7. 相关文档

| 文档 | 用途 |
|------|------|
| [`phase1_hitl_isomorphic_startup.md`](phase1_hitl_isomorphic_startup.md) | 正式/调试拓扑与 remap |
| [`项目开发梳理.md`](项目开发梳理.md) | 总进度 |
| [`hitl_arbiter_state_machine.md`](hitl_arbiter_state_machine.md) | 交接状态转移表（A/B/C） |
| [`../Operator_UI使用说明.md`](../Operator_UI使用说明.md) | UI DAgger 主机 playbook |
| [`../Hint_Dagger启动使用说明.md`](../Hint_Dagger启动使用说明.md) | 旧 Marvin 路径（逐步废弃） |
