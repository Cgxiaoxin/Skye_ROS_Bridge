# 大臂力矩探针 / 力反馈诊断

## `probe_external_torque.py`

独占 `Link`，从 `FX_L1_Fbk_GetRT` 读：

- `ExternalTorEst` — 轴外力矩估计（目标字段）
- `SensorTor` — RT 力矩对照
- `SG Joint_Tor` — 慢组对照（可选）

**不会**写 `/gento/*_joint_states`；只打印，方便接大臂后人工确认。

### 运行前

```bash
pkill -f skye_robot_driver || true
pkill -f gento_robot_driver || true
```

### 命令

```bash
cd /data/coding/tianji/Skye_ROS_Bridge
/usr/bin/python3 scripts/torque/probe_external_torque.py
/usr/bin/python3 scripts/torque/probe_external_torque.py --duration 30 --hz 10
/usr/bin/python3 scripts/torque/probe_external_torque.py --mode imp_joint --arm both
```

空载看 `max|ext|` 是否约在 ±5；轻推末端看对应轴是否变大。

## `record_effort_vs_cmd.py`

驱动**运行中**录 CSV：`effort` vs `/gento/*_joint_control` vs 小臂 `current_state` + `/teleop/state`。

用来区分力环抖（effort 抖 → 小臂抖）还是位环抖（cmd/leader 抖）。

```bash
# 必须与大臂驱动同一 domain（默认 21）；脚本会自行 setdefault
export ROS_DOMAIN_ID=21
unset ROS_LOCALHOST_ONLY
/usr/bin/python3 scripts/torque/record_effort_vs_cmd.py --side left --duration 20
/usr/bin/python3 scripts/torque/record_effort_vs_cmd.py --side right -o /tmp/effort_right.csv
```

若 `no samples`：先确认 `ros2 topic hz /gento/left_joint_states` 有频率，且 `echo $ROS_DOMAIN_ID` 与驱动一致。

看：`teleop_state!=TELEOP` 时 effort 应接近 0；TELEOP 空载 `|effort|` 应远小于 raw 残差；接触大力峰值仍应可见。
