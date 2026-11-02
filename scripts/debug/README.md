# scripts/debug — 遥操现场调试

本目录集中放小臂/大臂调试脚本；**运行输出一律进 `logs/`**。

| 脚本 | 用途 |
|---|---|
| `record_leader_health.py` | 双臂：小臂 `current_state` 频率/掉帧 + 大臂 `joint_states` stamp stale + diagnostics |
| `watch_arm_joints.py` | 1 Hz 对照小臂角 / 大臂角 / `joint_control` 指令角 |
| `move_small_arm_zero.py` | 小臂回零 / sync / telepose 辅助动作 |

说明笔记：`notes/`（例如频率与电机诊断）。

## 常用

```bash
conda deactivate
source skye_ros2_ws/scripts/setup_factr_teleop_env.sh orin
unset FASTRTPS_DEFAULT_PROFILES_FILE

# 小臂懒汉/掉频 + 大臂反馈 stale（推荐遥操时开着）
/usr/bin/python3 scripts/debug/record_leader_health.py

# 角度对照
/usr/bin/python3 scripts/debug/watch_arm_joints.py
```

终端只打 1 Hz 摘要；细节在 `logs/leader_health_*.jsonl` / `logs/watch_arm_joints_*.log`。
