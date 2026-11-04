# 单臂同步协同调参 SOP（精简版）

适用范围：小臂走位、真实大臂反馈同步、保持与停止。详细准备、历史记录及命令说明见[完整版](single_arm_sync_tuning_sop.md)。

## 1. 协作原则

- **操作者**确认实体姿态与路径，执行命令，报告抖动、停顿、补动、报警及人工碰触；异常时立即停止。
- **助手**分析配置、JSON、trace 和驱动日志，明确下一轮改动、预期结果与回退条件。
- 每轮只解决一个问题，优先只改一个轴的一类参数；尽量固定起点、目标和负载。
- 先分析上一轮，再开始下一轮；试验成功后重复与回归，再决定保留参数。

## 2. 开始前确认

1. 只运行目标侧小臂，退出其他控制或对齐节点；核对串口侧别、零位、方向及关节顺序。
2. FTDI `latency_timer` 回读为 `1`；目标位于安全区，实体路径、线缆及夹爪无干涉。
3. 所有终端使用同一 ROS 环境、domain 和配置。改参后停止、退出并重启小臂节点，再开始新记录。
4. 正式同步时，大臂保持静止并持续发布有效反馈；同步只截取一次目标。

## 3. 每轮操作

各终端先加载环境，统一变量；每轮更换 `RUN_ID`：

```bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
SIDE=left
CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_${SIDE}.yaml"
RUN_ID="${SIDE}_sync_001"
```

**① 启动。** 正式同步时，先在单独终端启动大臂驱动；已有同一驱动运行时无需重复启动：

```bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py robot_profile:=thor
```

大臂驱动就绪后，在另一个终端启动小臂：

```bash
ros2 launch skye_factr_teleop "start_teleop_m6_${SIDE}.launch.py" \
  "${SIDE}_config:=$CONFIG" sync_diagnostics:=true
```

**② 先记录。** 在另一个终端执行，等待记录器就绪：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side "$SIDE" --config "$CONFIG" --operator lyy \
  --output "/tmp/${RUN_ID}.json" --duration 180
```

**③ 请求同步。** 确认小臂为 IDLE、大臂反馈与目标安全后，在独立操作终端执行：

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
```

观察 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED`。轨迹结束后还要等待八轴实测位置、速度连续达标；SYNCED 后继续保持。容差与超时以本轮 YAML 为准。

**④ 停止。** 完成约定保持观察后停止；异常时立即停止：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

**记录器退出不会停止小臂；停止命令必须在独立终端执行。** 人工碰触、支撑的时刻须标注，其后的数据不用于证明无接触保持稳定。

**⑤ 保存与回读。** 确认 IDLE、记录保存，退出小臂节点并释放串口；保持供电、不重启总线，支撑小臂后执行：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side "$SIDE" --config "$CONFIG" --frames 2 --interval 0 \
  --output "/tmp/${RUN_ID}_torque_disable.json"
```

核对 `automated_result=PASS`、ID1～8 的 `torque_enabled=false`。软件缓存失能不能代替硬件回读。

无大臂时改用 `skye_factr_sync_trial move`，先预览再 `--execute`，无需额外发 `switch_sync`；具体命令见完整版。

## 4. 调参判断顺序

先核对配置、起点、目标、人工干预和丢帧，再区分轨迹、收敛、保持阶段，定位未达标轴。

| 现象 | 优先检查与处理 |
|---|---|
| 误差大且电流持续裁剪 | 排除干涉与目标冲突，检查力矩分项和周期，再评估小步增加电流余量 |
| 静差存在但电流未裁剪 | 检查保持 P/I、重力残差与静摩擦；继续提高电流上限不会自动增加请求 |
| 积分不起作用 | 检查误差是否超 `i_thresh` 被清零，以及积分是否达到 `i_max`；积分力矩上限为 `ki × i_max` |
| 速度超差或抖动 | 检查速度反馈、周期与 D 项，必要时回退近期增加的 Kd，避免直接加阻尼 |
| 轨迹结束后停顿、补动 | 检查终点误差、轨迹到保持交接及剩余静差；现有 `0.5 s` 平滑交接不能消除全部停顿 |
| 目标靠近限位 | 检查安全区与屏障力矩是否对抗跟踪，先核对机械行程和映射 |

`i_thresh` 是积分介入范围，位置容差是通过门限。放宽容差或延长超时须单独记录；Ruckig 上限约束参考轨迹，实测速度仍需检查。

## 5. 复盘与收尾

每轮向助手提供：**配置路径、JSON/trace、驱动日志、现场现象、人工干预时刻、停止原因及失能回读。** 助手按“事实 → 判断 → 最小改动 → 验证点 → 回退条件”回复。

同条件重复成功后，再覆盖不同起点、零位与常用非零目标；同步期间 `follower_command_count` 应为 `0`。同时检查平顺性、限幅、保持和停止，不只看是否进入 SYNCED。

结论写明已测范围、保留参数和剩余问题。左右侧、异常停止与切遥操分别验收；历史成功不自动代表当前参数通过。批次结束将需要保留的配置和证据从 `/tmp/` 归档至 `validation_records/`。
