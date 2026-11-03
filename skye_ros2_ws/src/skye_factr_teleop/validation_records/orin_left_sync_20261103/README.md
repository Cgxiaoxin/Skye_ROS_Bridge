# Orin / 设备 B 左侧同步参数冻结记录

冻结日期：2026-11-03。当前保留这组参数作为已测同步工况的调参基线，不继续增加增益或电流。适用设备为 Orin / 小臂 B 左侧，试验使用 ROS_DOMAIN_ID=73。

- 活动配置：[config/orin/grav_comp_m6_left.yaml](../../config/orin/grav_comp_m6_left.yaml)
- 完整配置快照：[grav_comp_m6_left.yaml](grav_comp_m6_left.yaml)
- 两轮记录的完整配置与冻结时活动配置经配置加载器解析后完全一致。

## 关键参数

以下索引均按 J1..J7、夹爪排列。

| 参数 | 冻结值 |
| --- | --- |
| J3 current_limit_ma | 300 mA |
| J3 hold.ki / i_max / i_thresh | 0 / 0.10 rad·s / 0.25 rad |
| J3 hold.kp / kd | 0.80 / 0.08 |
| J4 ruckig.kp / kd | 0.50 / 0.08 |
| J4 hold.kp / kd | 0.80 / 0.08 |
| J4 current_limit_ma | 300 mA |
| sync_position_tolerance | 七轴 0.16 rad，夹爪 0.08 rad |
| sync_velocity_tolerance | 八轴 0.10 rad/s |
| settle_duration / settle_timeout | 0.25 / 25 s |

其余参数以完整配置快照为准。

## 复测结果

数值由对应原始 trace 离线统计。保持时长为 SYNCED 首末样本时间差；收敛时长为 HOLD_SETTLING 到 SYNCED 首样本时间差。

| 指标 | 13:49:17 | 14:02:51 |
| --- | --- | --- |
| 记录摘要 | [left_sync_20261103_134917.json](left_sync_20261103_134917.json) | [left_sync_20261103_140251.json](left_sync_20261103_140251.json) |
| 收敛时长 | 0.252 s | 1.032 s |
| SYNCED 保持时长 | 30.412 s | 25.781 s |
| J4 收敛阶段速度峰值 | 0.024 rad/s | 0.240 rad/s |
| J4 收敛及 SYNCED 阶段命令限幅 | 0% | 0% |
| J3 SYNCED 平均实测电流绝对值 | 278.5 mA | 279.9 mA |
| J3 SYNCED 实测电流峰值 | 291 mA | 292 mA |
| J3 SYNCED 最大位置误差 | 0.1232 rad | 0.1313 rad |
| J3 SYNCED 命令限幅比例 | 50% | 50% |
| 记录序列丢样 | 0 | 0 |
| 收尾 | IDLE，观察到零命令及缓存失能状态 | IDLE，观察到零命令及缓存失能状态 |

13:49:17 一轮由操作者确认无抖动。两轮 trace 均未复现此前 J4 收敛阶段约 4.56 rad/s 的大幅振荡。

## 已知表现与适用范围

- J3 保持电流接近 300 mA，位置基本不动，残余误差约 0.12～0.13 rad；满足当前同步容差，但并未精确到达目标。
- J3 低速摩擦补偿按周期正负交替，约一半命令样本触及软件限幅；这与此前连续顶住 450 mA 的表现不同，仍须保留为后续保持试验的关注项。
- 冻结代表保留当前已测同步参数，不代表任意姿态、长时间保持或后续遥操均已验收。记录中的独立硬件失能验证仍为 false。
- 完整 trace 当前位于 `/tmp/left_sync_20261103_134917.trace.jsonl` 和 `/tmp/left_sync_20261103_140251.trace.jsonl`；本目录保存配置快照及原始记录摘要，未归档 trace。
