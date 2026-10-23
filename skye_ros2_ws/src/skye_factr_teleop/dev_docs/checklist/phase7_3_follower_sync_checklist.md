# 阶段 7.3：连接大臂后的单臂同步

设备 A / Thor，先左后右。无大臂同步按操作者结论已完成；本清单验证真实大臂反馈驱动小臂同步。
沿用当前左右 YAML：轨迹上限 `0.8 rad/s、1.0 rad/s²、5.0 rad/s³`，夹爪目标 `0.4 rad`，
保留已调通的轨迹上限。左侧最终参数、正常同步结果和剩余项目见第 16 节；第 6～15 节保留各阶段的历史记录，
其中临时 YAML 和待执行建议不作为当前启动配置。
无大臂历史调参见[无大臂试验清单第 11、12 节](phase7_3_sync_trial_checklist.md#111-与第-10-节慢速记录的区别)。

## 1. 已核对的接口

| 项目 | 左侧 | 右侧 |
|---|---|---|
| 大臂反馈，`sensor_msgs/msg/JointState` | `/gento/left_joint_states` | `/gento/right_joint_states` |
| 七轴名称及顺序 | `l_j1` … `l_j7` | `r_j1` … `r_j7` |
| 小臂逐周期诊断，JSON String | `/leader_arm/left/sync_diagnostics` | `/leader_arm/right/sync_diagnostics` |
| 小臂实测七轴状态 | `/left_leader_arm/current_state` | `/right_leader_arm/current_state` |
| 小臂参考七轴目标 | `/left_leader_arm/target_joint_state` | `/right_leader_arm/target_joint_state` |
| 大臂命令，本轮收到的消息数应为 0 | `/gento/left_joint_control` | `/gento/right_joint_control` |

位置单位 rad，速度单位 rad/s；反馈须带非零、新鲜时间戳，默认新鲜度门限 `0.25 s`。
`switch_sync` 在 IDLE 截取该侧大臂七轴位置，加上夹爪目标后规划八轴轨迹；本轮不追随大臂的新姿态。
大臂须全程保持不动，并持续发布有效反馈；反馈失效会停止小臂并回到 IDLE。
目标角度直接采用反馈值，不额外套用大臂 relative 指令的符号或零点映射，实体零位与方向需现场核对。

## 2. 公共环境与启动

以下为连接真机后的操作命令。所有终端使用同一主机工作区和 ROS 环境，各执行：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
SIDE=left
CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_${SIDE}.yaml"
```

本次使用独立 domain 73；原 domain 21 中可发现其它主机或容器中的采集、对齐节点。
大小臂启动、记录、状态监控及模式请求终端必须使用相同的 domain 73。

操作者先确认大臂姿态、小臂路径、线缆及夹爪范围可行，并支撑小臂，准备好停止与断电。
只运行目标侧小臂节点，停止旧控制器和 `skye_follower_align`，避免其它节点发布运动命令。
USB 侧别及 latency timer 的检查沿用[原清单第 3 节](phase7_3_sync_trial_checklist.md#3-现场准备与启动终端-a)。

终端 A 启动大臂驱动；已有同一驱动运行时不重复启动。该命令会连接真机、进入控制模式，并按现有配置启动夹爪：

本次控制器实际 IP 为 `6.6.7.191`（操作者 2026-10-23 确认），已写入大臂驱动默认 YAML。

```bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py robot_profile:=thor
```

终端 B 仅启动本侧小臂：

```bash
ros2 launch skye_factr_teleop "start_teleop_m6_${SIDE}.launch.py" \
  "${SIDE}_config:=$CONFIG" sync_diagnostics:=true
```

启动后小臂 torque 关闭。终端 D 先查看真实大臂反馈：

```bash
ros2 topic echo --once "/gento/${SIDE}_joint_states" sensor_msgs/msg/JointState
```

核对七轴名称、顺序及数值与实体姿态一致；目标须位于小臂安全范围内。
不能只凭有消息就发送同步；首轮让小臂起点靠近大臂目标，由操作者确认实体路径。

## 3. 记录与终端输出

终端 C 专门记录，沿用实际采用的 `/tmp/`、180 秒方式；每轮更换未占用的文件名：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side "$SIDE" --config "$CONFIG" --operator lyy \
  --output "/tmp/${SIDE}_follower_sync_01.json" --duration 180
```

等记录器显示 `Recording ...; awaiting move, then IDLE`。它也可记录正式 `switch_sync`，
并非只支持 `joint_move`。文件为上述 JSON 及同名 `.trace.jsonl`。
记录命令会占用终端；不要把 stop 排在它后面等待执行，停止始终在终端 D 发出。
到期或 Ctrl+C 只保存记录，不停止小臂。

终端 E 查看状态，供保存终端输出及判断同步进度：

```bash
ros2 topic echo /teleop/state std_msgs/msg/String --qos-durability transient_local
```

本轮仅启动一侧，因此这个全局状态话题对应当前侧。详细误差、电流和力矩由记录文件分析。
正式同步没有 `move --execute` 的逐轴等待提示；CLI 的 `publishing #1` 只表示请求已发布。

## 4. 同步与停止（终端 D）

确认记录器工作、小臂为 IDLE、大臂静止且操作者准备就绪后，执行：

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
```

应观察到 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED`。
左侧最终配置：七轴位置误差不大于 `0.07 rad`、夹爪误差不大于 `0.05 rad`，
八轴速度绝对值不大于 `0.05 rad/s`，连续满足 `0.25 s` 后才进入 SYNCED。
右侧门限以对应 YAML 为准，本次左侧调整不自动应用到右侧。
轨迹超时 `30 s`、收敛超时 `25 s`；成功后持续保持，不因发布命令退出而停止。
观察平顺性、切换冲击和到位后的保持；出现异常立即停止，无需等记录到期。

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

操作者继续支撑小臂。确认状态回到 IDLE，记录器保存文件；终端 B 正常退出并释放串口后，
保持供电、不重启总线，独立回读失能状态：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side "$SIDE" --config "$CONFIG" --frames 2 --interval 0 \
  --output "/tmp/${SIDE}_follower_sync_01_torque_disable.json"
```

核对 `automated_result=PASS` 及 ID1..8 的 `torque_enabled=false`。
把同步/停止终端输出、记录文件路径及现场观察提供给助手，分析后再安排下一轮。
本阶段不发送 `switch_teleop`，也不再用 `move --target 0 ...` 代替正式同步。

## 5. 后续轮次与完成条件

首轮成功后，逐步覆盖多个安全的大臂固定姿态，并补做小幅运动中 stop、受控超时及反馈中断。
异常试验先确定具体方式，不通过卡住机构制造超时。左侧完成后所有终端改为 `SIDE=right`，
重新设置 CONFIG、启动右侧节点，沿用同样记录流程。

每轮检查轨迹与保持的误差、实测速度、力矩分项、电流裁剪及状态切换；
`follower_command_count` 应为 0，诊断丢帧须计入分析。`RECORDED` 不自动等于硬件 PASS。
正常同步、保持、中止和异常停止及独立失能回读完成后，再形成该侧 7.3 的验收结论。

## 6. 连接前软件验证

2026-10-23：以下相关测试共 **287 项通过**，覆盖同步、收敛、保持、停止、反馈失效、
轨迹/收敛超时、左右接口隔离及记录工具。修正了旧测试中调参前的速度、超时和积分阈值假设，
并等待状态订阅者实际收到 SYNCED 后再测试下一次切换；未修改生产控制代码或左右 YAML。

```bash
/usr/bin/python3 -m pytest -q \
  src/skye_factr_teleop/test/test_ros_driver.py \
  src/skye_factr_teleop/test/test_launch.py \
  src/skye_factr_teleop/test/test_sync_hold.py \
  src/skye_factr_teleop/test/test_sync_trial.py \
  src/skye_factr_teleop/test/test_state_machine.py \
  src/skye_factr_teleop/test/test_state_trajectory_integration.py
```

左右两侧均通过真实 DDS 通信验证：模拟七轴反馈 → CLI `switch_sync` →
TELEOP_SYNCING → HOLD_SETTLING → SYNCED → stop → 现有 `record` 保存 JSON 与 trace；
确认七轴有符号目标按原顺序采用、夹爪为 `0.4 rad`、大臂命令数为 0、停止后假设备失能且输出归零。
DDS 测试使用本机独立 domain 193、假舵机与注入状态，不打开串口、不连接 Gento。

另使用左右当前原始 YAML 和真实 Ruckig 离线检查，确认八轴上限仍为 `0.8 / 1.0 / 5.0`。
从八轴全零到示例七轴目标 `[0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.15]`、夹爪 `0.4 rad`，
两侧参考轨迹耗时均为 `1.480625 s`；此为虚拟起点数值验证，实体方向、路径及运动效果留待真机验证。

## 7. 左侧首轮正式同步记录（2026-10-23）

设备 A / Thor，操作者 lyy，ROS domain 73，控制器 `6.6.7.191`。
通过真实大臂反馈及 `switch_sync` 完成同步，沿用当前左侧 YAML，未调整控制参数。

| 证据 | 文件 |
|---|---|
| 汇总及采集时配置 | [left_follower_sync_01.json](/tmp/left_follower_sync_01.json) |
| 逐周期诊断 | [left_follower_sync_01.trace.jsonl](/tmp/left_follower_sync_01.trace.jsonl) |
| 停止后独立失能回读 | [left_follower_sync_01_torque_disable.json](/tmp/left_follower_sync_01_torque_disable.json) |

文件当前保留在本机 `/tmp/`。本轮目标取正式同步时的大臂反馈，七轴均接近零位，
夹爪目标 `0.4 rad`；不是通过 `joint_move` 发布绝对零位。

- 完整观察到 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED → IDLE`。
- 按首次状态诊断的单调时钟计算：轨迹 `3.0761 s`、收敛 `2.3708 s`，总计 `5.4469 s`；
  首次 SYNCED 至停止后 IDLE 为 `36.0295 s`。
- `missing_samples=0`、`sequence_restarts=0`、`follower_command_count=0`；
  轨迹、收敛与保持阶段八轴命令电流裁剪样本均为 0。
- 参考速度不超过 `0.8 rad/s`；轨迹中 J1/J2 实测速度峰值约 `0.8633 rad/s`，
  J2 最大参考跟踪误差约 `0.5311 rad`。保留为后续姿态对比数据。
- 保持阶段八轴位置误差均小于 `0.05 rad`；J3/J4/J5 分别有 `35/17/16` 帧速度超过
  `0.05 rad/s`，峰值约 `0.0959 rad/s`。不记为保持阶段每帧速度均达标。
- 操作者现场反馈：“轨迹平顺，小臂末端有细微抖动，我认为没有太大问题。”
  末端细微抖动保留为现场现象，本轮不因此调参。
- 正常 stop 后记录器确认 IDLE、软件输出归零及缓存 torque 关闭；
  小臂节点退出后的独立回读于 `15:20:15 +08:00` 完成，`automated_result=PASS`，
  ID1..8 的 `torque_enabled` 全部为 `false`，latency timer 为 `1`。
  原始汇总中采集时的 `hardware_torque_disable_verified=false` 保持原值，由关联回读补充证据。

结论：左侧近零目标的首轮真实反馈同步、保持与正常停止流程已完成，保留现有参数。
后续先同配置重复一轮，再扩展安全目标姿态；右侧及中途 stop、受控超时、反馈中断仍待验收，
本轮不作为整个阶段 7.3 已完成的结论。

## 8. 左侧同参数重复同步（2026-10-23，第 02 轮）

汇总 [left_follower_sync_02.json](/tmp/left_follower_sync_02.json)，逐周期记录
[left_follower_sync_02.trace.jsonl](/tmp/left_follower_sync_02.trace.jsonl)，当前位于本机 `/tmp/`。
完整配置快照与第 01 轮一致，目标仍取近零位大臂真实反馈；操作者反馈“现场表现平稳正常”。

| 项目 | 第 01 轮 | 第 02 轮 |
|---|---:|---:|
| 参考轨迹执行 | 3.0761 s | 3.1754 s |
| 到位收敛 | 2.3708 s | 2.1983 s |
| 总同步时间 | 5.4469 s | 5.3737 s |
| SYNCED 诊断首末帧间隔 | 36.0189 s | 13.3162 s |

两轮均完整记录 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED → IDLE`，
无诊断丢帧、序号重启、命令电流裁剪或大臂命令；停止后观察到软件输出归零和缓存失能。
第 02 轮保持阶段八轴位置误差均小于 `0.05 rad`；J3/J7 分别有 `94/15` 帧速度超差，
最长连续超差首末帧间隔约 `0.1000/0.0280 s`，峰值分别约 `0.4556/0.07194 rad/s`。
J3 峰值出现在首次 SYNCED 后约 `0.3760 s`，保留为后续对比现象，未据此调参。
轨迹中 J2 最大跟踪误差约 `0.5361 rad`，实测速度峰值约 `0.8873 rad/s`。

终端 `ros2 topic echo --once /gento/left_joint_states` 报告丢失一条消息，随后正常打印反馈。
该提示来自临时 echo 订阅者，不能等同为控制器收到过期反馈或诊断记录丢帧；本轮未因此停止同步。

结论：近零目标同配置重复同步成功，现场表现正常，继续保留参数。
第 02 轮已补充独立失能回读
[left_follower_sync_02_torque_disable.json](/tmp/left_follower_sync_02_torque_disable.json)，
`automated_result=PASS`，ID1..8 的 `torque_enabled` 全部为 `false`。
两轮近零目标均已完成各自的同步、保持、正常停止和独立失能回读，下一步扩展安全非零目标姿态。

## 9. 左侧非零目标首轮：收敛阶段停止（2026-10-23，第 03 轮）

汇总 [left_follower_sync_03.json](/tmp/left_follower_sync_03.json)，逐周期记录
[left_follower_sync_03.trace.jsonl](/tmp/left_follower_sync_03.trace.jsonl)。
操作者现场反馈“一切正常”，本轮未修改参数。
正式同步截取的七轴大臂目标约为
`[0.004445, -0.073596, -0.018956, -0.110097, 0.159057, -0.136526, 0.252679] rad`。

- 状态为 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → IDLE`，未出现 SYNCED。
  参考轨迹执行 `3.1775 s`，进入收敛后约 `15.6587 s` 回到 IDLE；尚未达到 `25 s` 收敛超时。
  操作者本轮发送过 `switch_stop`，末帧没有 `sync_timeout` 或 `settle_timeout` 原因。
- 末个收敛周期 J7 目标 `0.252679 rad`、实测 `0.188680 rad`，误差 `0.063999 rad`，
  超过 `0.05 rad` 容差；最后约 1 秒内误差仍约 `0.0640～0.0655 rad`。
  J3 末段位置误差约 `-0.04964～-0.05117 rad`，在容差边缘。
- 收敛阶段物理 ID3 命令电流在 `6670/7775` 帧被裁剪至幅值 `120 mA`，
  ID7 在 `7656/7775` 帧被裁剪至幅值 `50 mA`；占比分别约 `85.8%/98.5%`。
  停止前 ID3/ID7 的裁剪前命令约 `-130.10/-58.60 mA`，裁剪后为 `-120/-50 mA`。
  已存在持续电流裁剪，不能只因现场平稳就记为非零目标同步通过。
- 无诊断丢帧、序号重启或大臂命令；停止后软件输出归零、缓存失能。
  `RECORDED` 表示记录流程完成，`synced_observed=false`。
  已补独立回读 [left_follower_sync_03_torque_disable.json](/tmp/left_follower_sync_03_torque_disable.json)，
  `2026-10-23 15:46:18 +08:00` 检查通过，ID1..8 的 `torque_enabled` 全部为 `false`。

进一步分析：停止前 J3 的积分力矩为 `-0.075 N·m`，J7 为 `+0.015 N·m`，
均已达到各自 `ki × i_max` 上限，同时总输出被电流限幅裁剪。
该证据说明继续增大积分请求不能绕过现有电流限制；是否提高电流可改善实测收敛须下一轮验证。

结论：本轮非零目标尚未完成到位收敛，独立失能回读已通过，基线配置不变。

### 9.1 下一轮独立电流配置（已准备，尚未执行）

配置 [grav_comp_m6_left_follower_current_trial.yaml](/tmp/grav_comp_m6_left_follower_current_trial.yaml)
仅将物理 ID3 电流上限从 `120` 改为 `140 mA`，物理 ID7 从 `50` 改为 `65 mA`。
已经通过配置校验及逐字段对比，除此以外与当前左侧基线完全一致。
未改动原 YAML，未连接硬件或激活新配置；轨迹上限、PID、重力增益及同步容差均保持原值。

下一轮若按此建议执行，节点的 `left_config` 和记录器的 `--config` 均使用该 `/tmp/` 文件，
记录到 `/tmp/left_follower_sync_04.json`。大臂继续保持第 03 轮姿态，操作者支撑小臂，
观察额外电流是否使误差收敛；达到 SYNCED 后观察约 5 s 再 stop，异常时立即 stop。
这是一轮电流余量试验，不代表上述电流已完成硬件验收。

## 10. 左侧电流余量试验：仍因 J7 未到位而超时（2026-10-23，第 04 轮）

汇总 [left_follower_sync_04.json](/tmp/left_follower_sync_04.json)，逐周期记录
[left_follower_sync_04.trace.jsonl](/tmp/left_follower_sync_04.trace.jsonl)。
运行诊断确认 ID3/ID7 上限已生效为 `140/65 mA`，其余配置沿用第 03 轮。

- 参考轨迹 `3.1578 s` 后进入 HOLD_SETTLING，未到 SYNCED；
  最终诊断原因明确为 `settle_timeout`，即 `25 s` 收敛超时自动回到 IDLE。
  操作者随后发送的 stop 不能作为本轮回到 IDLE 的原因。
- J3 已不再发生命令电流裁剪，收敛阶段位置误差一直在 `0.05 rad` 内，
  停止前约 `0.048083 rad`；这说明上轮的 ID3 调整确实增加了可用电流余量。
- J7 的 `12285` 个收敛样本均未满足位置容差；最小误差约 `0.050220 rad`，
  停止前目标 `0.252706 rad`、实测 `0.200951 rad`，误差 `0.051754 rad`。
  最后约一秒误差仍在 `0.050220～0.051754 rad`，连续全轴达标时长始终为 0。
- J7 收敛命令在 `5836/12285` 帧被裁剪（约 `47.5%`），
  裁剪前峰值约 `69.615 mA`，停止前约 `67.501 mA`，均高于 `65 mA` 上限。
  本轮 J7 静差较第 03 轮减小，但电流裁剪仍存在。
- 无诊断丢帧、序号重启或大臂命令；软件停止输出归零、缓存失能。
  独立回读 [left_follower_sync_04_torque_disable.json](/tmp/left_follower_sync_04_torque_disable.json)
  已补充，`automated_result=PASS`，ID1..8 的 `torque_enabled` 全部为 `false`。

下一轮建议仅将 ID7 从 `65` 改为 `75 mA`，ID3 保持 `140 mA`。
独立配置 [grav_comp_m6_left_follower_current_trial_v2.yaml](/tmp/grav_comp_m6_left_follower_current_trial_v2.yaml)
已通过配置加载和逐字段对比，只有 ID7 上限改变，尚未在真机激活。
轨迹上限、增益、同步容差和超时均不变；第 04 轮失能回读已通过，下一步安排第 05 轮。

## 11. 第 05 轮：J7 裁剪消除，但 J3/J7 仍未收敛（2026-10-23）

汇总 [left_follower_sync_05.json](/tmp/left_follower_sync_05.json)，逐周期记录
[left_follower_sync_05.trace.jsonl](/tmp/left_follower_sync_05.trace.jsonl)。
运行诊断确认 ID3/ID7 上限为 `140/75 mA`。

- 轨迹约 `3.2261 s` 后进入 HOLD_SETTLING，25 s 收敛超时后自动返回 IDLE，
  原因为 `settle_timeout`，未出现 SYNCED。
- J7 本轮已无命令电流裁剪，请求电流峰值约 `69.09 mA`，低于 `75 mA`；
  停止前目标 `0.252724 rad`、实测 `0.202485 rad`，误差 `0.050239 rad`，
  收敛全程位置未达标。最后约一秒位置固定，停止前积分力矩已到 `0.015 N·m` 上限。
  因此不能再仅靠提高 J7 电流上限解释或解决剩余误差。
- J3 停止前误差约 `-0.091019 rad`，较上轮的约 `-0.048083 rad` 增大；
  收敛全程位置超差，`11864/12356` 帧（约 `96.0%`）发生电流裁剪。
  停止前请求约 `-164.02 mA`、裁剪后 `-140 mA`、实测 `-140 mA`。
- 无诊断丢帧、序号重启或大臂命令；软件输出归零、缓存失能。
  独立回读 [left_follower_sync_05_torque_disable.json](/tmp/left_follower_sync_05_torque_disable.json)
  已补充，`automated_result=PASS`，ID1..8 的 `torque_enabled` 全部为 `false`。
  操作者随后补充“没有接触”，按现场无接触记录。

结论：提高 J7 上限消除了该轴裁剪，尚未使全轴收敛；J3 本轮偏差增大，
需结合现场支撑/外力/干涉情况及保持积分上限分析，暂不继续提高电流或改动基线。

### 11.1 无接触信息补充后的下一轮配置（第 06 轮，待执行）

回看代码与数据：J3 的积分力矩在约 2 s 后到 `-0.075 N·m`，请求电流最高约
`164.77 mA`，实际被裁剪在 `140 mA`；J7 的请求未触及 `75 mA`，但积分力矩
固定在 `+0.015 N·m` 上限，约 `0.05024 rad` 静差保持不变。
第 03～05 轮小臂起点不同，尤其 J3，不能把同目标的静差变化简单归因于单个参数；
这些记录支持继续检查电流余量和保持积分上限，但尚不能区分所有静摩擦与重力残差因素。

已准备并校验独立配置
[grav_comp_m6_left_follower_hold_trial_v3.yaml](/tmp/grav_comp_m6_left_follower_hold_trial_v3.yaml)，
相对 v2 仅两处修改：

- 物理 ID3 电流上限 `140 → 170 mA`，给本轮最高约 `165 mA` 的请求留出余量。
- J7 保持 `i_max` 从 `0.10 → 0.15 rad·s`，`ki=0.15` 不变，
  积分力矩上限从 `0.015 → 0.0225 N·m`；J7 电流上限仍为 `75 mA`。

其余包括 `0.8/1.0/5.0` 轨迹上限、PID 增益、重力项、容差与超时均不变。
原始 YAML、v1、v2 均保留，新配置尚未在硬件激活。
下一轮节点和记录器同用 v3，输出 `/tmp/left_follower_sync_06.json`，大臂保持当前姿态；
观察 SYNCED、切换冲击、电流裁剪与稳定性，再决定是否保留，不能预先记为调参成功。

## 12. 第 06 轮：v3 首次完成非零目标同步（2026-10-23）

汇总 [left_follower_sync_06.json](/tmp/left_follower_sync_06.json)，逐周期记录
[left_follower_sync_06.trace.jsonl](/tmp/left_follower_sync_06.trace.jsonl)。
诊断确认 ID3/ID7 电流上限为 `170/75 mA`、J7 保持 `i_max=0.15 rad·s`。
大臂目标仍近似第 03～05 轮非零姿态，完整观察到
`IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED → IDLE`。

- 参考轨迹 `3.3112 s`，收敛 `2.5869 s`，总同步 `5.8981 s`；
  首次 SYNCED 的连续达标计时为 `0.2520 s`，保持阶段首末诊断间隔 `21.4079 s`。
- 轨迹、收敛、保持均无命令电流裁剪；无诊断丢帧、序号重启或大臂命令。
- SYNCED 阶段八轴位置误差全程均小于 `0.05 rad`；
  J4/J7 分别有 `16/77` 帧速度超过 `0.05 rad/s`，峰值分别约 `0.07194/0.16787 rad/s`。
  停止前 J3/J7 位置误差约 `-0.041918/+0.019576 rad`。
- ID3 本轮三阶段最大裁剪前请求约 `128.72 mA`，未触及原 v2 的 `140 mA` 上限。
  由于起点不同，不能据此证明把 ID3 上限设为 `170 mA` 是本轮成功的必要条件。
- stop 后软件输出归零、缓存失能，小臂节点正常退出。
  独立回读 [left_follower_sync_06_torque_disable.json](/tmp/left_follower_sync_06_torque_disable.json)
  已补充，`automated_result=PASS`，ID1..8 的 `torque_enabled` 全部为 `false`。
- 操作者补充现场“平顺，抖动微弱，未加重”。

结论：v3 已取得该非零目标的首次同步成功，继续作为候选试验配置；
尚未覆盖第 05 轮的不利起点，也未在 v3 下复核近零目标，不将这一次成功直接写回正式基线。
下一步保持大臂目标和 v3 不变，先做第 07 轮重复验证，记录 `/tmp/left_follower_sync_07.json`。
小臂起点尽量复现第 05 轮，尤其 J3 当时约 `+0.376 rad`；不要求精确复刻，
实际八轴起点以本轮诊断为准，并由操作者确认路径可行。

## 13. 第 07 轮：v3 重复未通过，实际 J3 起点较大（2026-10-23）

汇总 [left_follower_sync_07.json](/tmp/left_follower_sync_07.json)，逐周期记录
[left_follower_sync_07.trace.jsonl](/tmp/left_follower_sync_07.trace.jsonl)。
运行配置仍为 v3，参考轨迹 `3.2491 s` 后进入收敛，25 s 后 `settle_timeout` 返回 IDLE。
无诊断丢帧、序号重启或大臂命令；停止后软件输出归零、缓存失能。
独立回读 [left_follower_sync_07_torque_disable.json](/tmp/left_follower_sync_07_torque_disable.json)
已补充，`automated_result=PASS`，ID1..8 的 `torque_enabled` 全部为 `false`。
操作者确认：“轨迹跟踪阶段有明显抖动，进入 hold 阶段抖动基本很轻微。”

- 本轮 J3 实测起点约 `+1.04004 rad`，不是第 05 轮的约 `+0.37583 rad`，
  不能记为准确复现第 05 轮起点。
- J7 本轮收敛阶段位置误差始终在 `0.05 rad` 内，停止前约 `+0.01345 rad`，无电流裁剪。
- J3 全部收敛样本位置超差，停止前约 `-0.06185 rad`，积分力矩已到 `-0.075 N·m` 上限；
  此时请求电流约 `-141.47 mA`、实测约 `-139 mA`，没有触及 `170 mA` 上限。
  收敛阶段仅 `16/12463` 帧裁剪，末段静差不能再仅归因于电流上限。
- J3 轨迹最大参考速度约 `0.40592 rad/s`，实测速度峰值约 `2.66187 rad/s`；
  收敛阶段实测峰值约 `1.84652 rad/s`。高速度附近连续样本有对应位置变化，
  不能只以参考轨迹满足上限就认定实测运动符合速度限制。

结论：v3 在第 06 轮起点可完成同步，尚未通过本轮较大 J3 起点；
本轮独立失能回读已通过，轨迹抖动得到现场确认；优先单独检查轨迹跟踪，
不继续提高电流、不改动轨迹上限或正式基线。

### 13.1 J3 轨迹速度反馈增益对照（v4，待执行）

按当前实现分解 `kp × (q_ref-q)` 与 `kd × (dq_ref-dq)`：第 07 轮 J3 的位置项峰值
约 `0.1384 N·m`，速度项峰值约 `0.2744 N·m`。
最大电流请求周期约在轨迹开始后 `2.8164 s`，位置项 `-0.1341 N·m`、速度项
`-0.2744 N·m`，合成请求约 `-364.12 mA`，被裁剪为 `-170 mA`。
这是检查速度反馈增益及动态响应的依据，并不能单凭分项幅值认定 kd 就是唯一根因。

独立配置
[grav_comp_m6_left_follower_tracking_trial_v4.yaml](/tmp/grav_comp_m6_left_follower_tracking_trial_v4.yaml)
相对 v3 仅将 J3 的 `ruckig.kd` 从 `0.10` 改为 `0.05`，已通过加载与字段对比，尚未激活。
J3 的轨迹 kp、全部保持参数、电流上限、轨迹上限和容差均不变。
下一轮先在 IDLE 读取 `/left_leader_arm/current_state`，确认小臂 J3 起点，
采用约 `+0.35～+0.40 rad` 的受控起点，先不直接重做 `+1.04 rad` 起点。
先看轨迹抖动、实测速度和裁剪是否改善，再评估到位；明显抖动时立即 stop，不等收敛超时。
由于起点也将缩小，首轮结果不能单独证明 kd 调整的因果效果；后续须在同起点做必要对比。
建议记录 `/tmp/left_follower_sync_08.json`，节点和记录器均使用 v4。

## 14. 第 08 轮：受控起点、v4 无电流裁剪，J3 保持静差未消除（2026-10-23）

操作者先回读确认 J3 起点 `+0.392699 rad`；正式轨迹首帧也为该值。
汇总 [left_follower_sync_08.json](/tmp/left_follower_sync_08.json)，逐周期记录
[left_follower_sync_08.trace.jsonl](/tmp/left_follower_sync_08.trace.jsonl)。
运行配置为 v4，J3 轨迹 `kd=0.05`，其余继承 v3。

- 参考轨迹 `3.2272 s`，进入收敛后 25 s 超时，原因为 `settle_timeout`；无 SYNCED。
- 轨迹、收敛全程无命令电流裁剪，无诊断丢帧、序号重启或大臂命令。
- J3 轨迹参考速度峰值 `0.14731 rad/s`，实测峰值 `0.33573 rad/s`，
  轨迹请求电流峰值约 `72.75 mA`；收敛实测速度峰值约 `0.26379 rad/s`。
  本轮起点也已减小，不能把相较第 07 轮的变化全部归因于降低 kd。
  操作者随后确认“现场几乎没有抖动”，仅作为本轮起点和配置下的现场结论。
- J3 全部 `12467` 个收敛样本位置超差，停止前目标 `-0.018887 rad`、实测 `+0.044485 rad`，
  误差约 `-0.063372 rad`，末段误差范围约 `-0.063372～-0.061838 rad`。
  保持积分力矩已到 `-0.075 N·m` 上限；停止前电流请求约 `-141.88 mA`，
  收敛全程请求峰值约 `144.33 mA`，低于 `170 mA` 上限。
- 停止前其它七轴位置误差均在 `0.05 rad` 内，其中 J7 约 `+0.02113 rad`。
- 自动停止后软件输出归零、缓存失能；已发现并核对独立回读
  [left_follower_sync_08_torque_disable.json](/tmp/left_follower_sync_08_torque_disable.json)，
  `automated_result=PASS`，ID1..8 的 `torque_enabled` 全部为 `false`。

结论：当前到位阻碍集中在 J3 的保持静差；继续提高电流上限不能直接增加当前未被裁剪的请求。
现场轨迹抖动表现和本轮独立失能回读均已补齐，下一步安排仅调整 J3 保持积分上限的有界试验。

### 14.1 J3 保持积分上限试验（v5，待执行）

已准备独立配置
[grav_comp_m6_left_follower_hold_trial_v5.yaml](/tmp/grav_comp_m6_left_follower_hold_trial_v5.yaml)，
相对 v4 仅将 J3 保持 `i_max` 从 `0.125` 改为 `0.15 rad·s`。
`ki=0.6` 不变，积分力矩上限从 `0.075` 增至 `0.090 N·m`，增量 `0.015 N·m`。
其余包括 J3 轨迹 `kd=0.05`、电流上限、轨迹上限、其它保持参数、容差与超时均不变。
已通过配置加载和逐字段对比，尚未激活；第 08 轮独立失能回读已通过。
下一轮记录 `/tmp/left_follower_sync_09.json`，节点和记录器均用 v5，尽量复现第 08 轮起点。
重点检查 J3 静差是否下降、是否出现新的裁剪或保持抖动，不预先保证到位。

## 15. 运行中的硬件报警检测（2026-10-23）

后续使用主配置进行了多轮试验；第 14 节的 v5 属于历史候选，不作为当前启动配置。
现场先后出现 ID6、ID3 的 `Hardware Error Status=0x20 (OVERLOAD)`。
独立读取记录 `/tmp/left_hardware_error_04.json` 显示断电重上电后八台故障码均为 0、扭矩均关闭，
但过载原因尚未查明，不能据此认定硬件验收通过。

驱动现在保留并检查每次同步状态响应的错误位，IDLE、轨迹、收敛、SYNCED 和遥操期间均生效，
无需开启 `sync_diagnostics`。正常周期不增加串口事务或状态数据长度。
收到硬件 Alert 后才追加读取地址 70，输出电机 ID、十六进制故障码及故障名称；
即使故障码读取失败也停止控制，不继续使用本周期状态产生运动电流。
启动或开启扭矩时的硬件 Alert 也会输出具体故障信息。

故障日志示例（仅格式示意，不是真机试验结果）：

```text
Control stopped after failure [side=left, phase=HOLD_SETTLING]: read_state failed for device ID 3: ...; Hardware Error Status=0x20 (OVERLOAD)
```

`phase` 是程序检测到报警时的控制阶段；ROS 日志时间可与试验记录对照。
程序随即尝试电流归零、关闭所有电机扭矩、取消控制定时器并关闭串口。
关闭某台电机失败时仍继续尝试其余电机；仅包含 Alert 的成功关闭应答不再被误判为写入失败。
如停机清理也报错，保留最初故障并输出附加错误上下文。
软件缓存失能仍不能代替独立扭矩回读；程序不会自动重启电机或清除保护。

本次只做离线模拟测试，未连接真机，未修改同步参数。
当前 Python 安装通过链接加载源码，下次重启小臂节点即使用修改后的驱动。
下一次现场试验仍需保留小臂驱动终端日志；`record` 的 JSON/trace 不直接保存本条故障日志。

## 16. 左侧正常同步最终配置与收尾结论（2026-10-23）

### 16.1 当前保留配置

使用主配置 [grav_comp_m6_left.yaml](../../config/grav_comp_m6_left.yaml)，不使用历史临时 v1～v5 YAML。
以下是本次已测范围内保留的参数，向量顺序为 J1..J7、夹爪；配置修改后须重启小臂节点。

```yaml
dynamixel:
  current_limit_ma: [500.0, 500.0, 400.0, 200.0, 180.0, 180.0, 180.0, 50.0]
  joint_offsets: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  joint_signs: [1, 1, 1, 1, 1, -1, -1, 1]
controller:
  sync_hold_control:
    gripper_setpoint: 0.4
    ruckig:
      gravity_comp_gain: [0.5, 0.2, 0.7, 0.6, 1.0, 0.8, 1.0]
      kp: [0.50, 0.50, 0.80, 0.50, 0.35, 0.35, 0.35, 0.35]
      kd: [0.08, 0.08, 0.05, 0.08, 0.05, 0.05, 0.05, 0.05]
      max_velocity: [0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80]
      max_acceleration: [1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00]
      max_jerk: [5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00]
      trajectory_timeout: 30.0
    hold:
      gravity_comp_gain: [0.5, 0.2, 0.7, 0.6, 1.0, 0.8, 1.0]
      kp: [0.50, 0.35, 0.80, 0.80, 0.35, 0.50, 0.35, 0.35]
      kd: [0.15, 0.15, 0.05, 0.08, 0.05, 0.05, 0.05, 0.05]
      ki: [0.05, 0.05, 0.60, 0.15, 0.15, 0.15, 0.15, 0.15]
      i_max: [4.5, 2.0, 0.20, 0.80, 0.10, 0.10, 0.10, 0.10]
      i_thresh: [0.50, 1.00, 0.25, 0.25, 0.15, 0.15, 0.15, 0.15]
      sync_position_tolerance: [0.07, 0.07, 0.07, 0.07, 0.07, 0.07, 0.07, 0.05]
      sync_velocity_tolerance: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
      settle_duration: 0.25
      settle_timeout: 25.0
```

重力模式仍为 `position_only`，静摩擦及限位项仍启用，零空间及力反馈项关闭。
J1/J3 保持积分力矩上限分别为 `0.225 / 0.120 N·m`。
J3 的 `i_thresh=0.25 rad` 是积分介入范围，到位容差仍为 `0.07 rad`。
本组未改变已调通的 `0.8 rad/s、1.0 rad/s²、5.0 rad/s³` 参考轨迹上限。

### 16.2 必要的调参结论

- 常用非零目标下，`fore_01/02` 的 J3 持续受 180 mA 限幅；提高至 300 mA 后的
  `fore_11/12` 仍有约一半收敛样本限幅。提高至 400 mA 后，`fore_21/22` 的收敛阶段
  已无软件限幅，仍然超时，说明单独增加电流上限不足以解决到位问题。
- 操作者确认无明显机械受阻，并确认可手动到达目标姿态。保持阶段的阻碍包括
  J1/J3 积分力矩到限，以及 J3 误差超过原 `i_thresh=0.15 rad` 时积分被清零。
  最终采用第 16.1 节的积分范围和上限。
- `fore_31` 是操作者因明显轨迹抖动提前 stop，不是收敛超时。
  该轮 J3 起点约 `-1.266 rad`，实测速度峰值约 `6.475 rad/s`。
  后续将 J3 轨迹 `kd` 从 `0.10` 降至 `0.05`，现场后续轮次确认轨迹平顺；
  各轮起点不同，第 31 轮大偏差起点尚未用最终参数同起点复测。
- 参考轨迹完成后进入 HOLD_SETTLING，实际位置仍可继续收敛。
  部分起点会先停顿、随后补动；本次不再为缩短收敛时间继续调整参数。

### 16.3 最终参数下的正常同步结果

下列四轮配置快照一致，均使用第 16.1 节参数。每个 JSON 均有同名 `.trace.jsonl` 逐周期记录。
同步总耗时从首个 TELEOP_SYNCING 样本计至首个 SYNCED 样本；收敛等待从首个 HOLD_SETTLING 样本计起。

| 记录 | 大臂目标 | 同步总耗时 | 收敛等待 | SYNCED 后保持 | 电流裁剪 |
|---|---|---:|---:|---:|---|
| [fore_51](/tmp/left_follower_sync_fore_51.json) | 常用非零姿态 | 4.71 s | 0.79 s | 9.26 s | 仅轨迹阶段 J4 约 45.5% 样本裁剪至 200 mA |
| [fore_52](/tmp/left_follower_sync_fore_52.json) | 同一常用非零姿态 | 20.35 s | 17.18 s | 8.25 s | 全程无裁剪 |
| [fore_61](/tmp/left_follower_sync_fore_61.json) | 大臂回零 | 4.72 s | 1.50 s | 9.00 s | 全程无裁剪 |
| [fore_62](/tmp/left_follower_sync_fore_62.json) | 大臂回零 | 19.94 s | 16.88 s | 5.61 s | 全程无裁剪 |

常用非零目标七轴约为 `[-0.0924, -0.5922, -0.1141, -1.1051, 0.3289, -0.5867, 0.3447] rad`，
四轮分别覆盖 J1 负/J2 正和 J1 正/J2 负两类小臂起点。
四轮均完成 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED → IDLE`，
无诊断丢帧、序号重启或大臂控制指令；stop 后软件输出归零、缓存失能。
进入 SYNCED 时位置和速度连续满足配置门限，不等同于之后始终完全静止。

现场观察：操作者确认 `fore_51/52` 无抖动、无报警；零位回归 `fore_61` 很平顺，
`fore_62` 轨迹跟踪平顺，进入收敛阶段仍有较大误差，需要继续调整一段时间后才能同步。
操作者接受当前总体表现。第 62 轮收敛耗时距 25 s 上限仍有约 8.12 s 余量，
SYNCED 阶段七轴位置误差最大约 `0.0613 rad`，在 `0.07 rad` 容差内。
收敛较慢及停顿后的补动记为后续性能优化项，当前保留参数。

### 16.4 批次末独立硬件回读

操作者退出小臂节点后执行只读诊断，提供
[left_hardware_error_44.json](/tmp/left_hardware_error_44.json)（2026-10-23 19:12，UTC+8）。
已核对 JSON 中 ID1..8 的寄存器读值，全部通信成功且无硬件 Alert：

- `Hardware Error Status(70)=0x00`，`FAULTS=NONE`。
- `Torque Enable(64)=0`，八台电机均已关闭扭矩。
- 温度为 `[37, 35, 34, 31, 32, 32, 32, 32] ℃`，对应限值 `[80, 80, 70, 70, 70, 70, 70, 70] ℃`。
- 输入电压为 `11.9～12.2 V`；七轴回读角度均在当前配置的安全范围内，无整圈偏移。

这是本批次结束后的独立故障/失能证据，不是各轮分别执行的回读。
原始同步 JSON 的 `hardware_torque_disable_verified=false` 未回写；本次硬件确认以独立记录为依据。
历史 ID3、ID6 过载事件继续保留，当前无故障读值不证明历史原因已消除。

### 16.5 当前结论与待办

- [x] 左侧已测零位与常用非零目标、两类小臂起点：正常同步、保持、正常 stop 通过。
- [x] 同一最终参数完成零位回归；操作者接受当前平顺性和收敛耗时。
- [x] 批次末独立回读确认八台电机无故障、扭矩关闭。
- [ ] 使用最终参数验证受影响的大偏差起点；结论暂限于本节已测范围，不扩展到任意初始姿态或全部工作空间。
- [ ] 左侧最终参数下的运动中 stop、受控超时、反馈中断等异常路径验收；历史调参中的人工中止不替代完整异常路径验收。
- [ ] 右侧正式大臂反馈同步及独立失能回读；右侧参数须单独验证。
- [ ] 阶段 7.4 的 SYNCED → TELEOP 切换和遥操验收，本组未执行。

当前只完成上述左侧已测范围的正常同步验收，不标记整个阶段 7.3 完成，也不将保持稳定代替切遥操验收。
