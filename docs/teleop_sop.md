# 遥操 SOP

> 硬件急停优先。禁止与旧 Marvin / 另一 SDK 客户端同时连同一控制器。

## 启动

1. 清场、确认急停回路。
2. 构建并启动：`skye_robot_driver`（见 `dev_plan.md` P0/P1）。
3. 确认 Link 成功、`/gento/joint_states` 有数据。
4. 确认已进 **PD**；`vel_ratio` 先保持 10。
5. 单臂小幅试跑 → 再双臂 → 再开 FACTR teleop（键 `1` sync → `2` teleop）。

## 急停

1. 硬件急停按钮。
2. 软件：`ros2 service call /gento/emergency_stop std_srvs/srv/Trigger`
3. 或 `/gento/stop_motion`。

## 复位

排除故障 → ResetError / hold → 再进 PD。勿急停后立刻高速遥操。

## 退出

停主手 → stop/hold → 节点退出（析构 `Unlink`）。

## 隔离

- 主线：`skye_ros2_ws`
- `marvin_ws` 仅参考；勿与本驱动同时启动连同一 IP。
