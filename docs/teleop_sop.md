# 遥操 SOP

> 硬件急停优先。禁止与旧 Marvin / 另一 SDK 客户端同时连同一控制器。

## 启动

1. 清场、确认急停回路。
2. 构建并启动：
   ```bash
   cd skye_ros2_ws && ./scripts/build.sh && source install/setup.bash
   export ROS_DOMAIN_ID=20
   ros2 launch skye_robot_driver skye_robot_driver.launch.py
   ```
3. 确认 Link 成功；默认 **`control_mode:=imp_joint`（mode=2）**。
4. 查模式：`ros2 topic echo --once /gento/robot_state --qos-reliability best_effort`  
   期望左右为 **`2, 2`**（`FX_STATE_IMP_JOINT`）。切换：
   ```bash
   # 位置模式
   ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 1}"
   # 关节阻抗（遥操）
   ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 2}"
   # 空闲
   ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 0}"
   ```
   回执含 `success` 与左右 `FX` 状态。5. 确认 `/gento/joint_states` @ ~250 Hz。
6. 单臂小幅试跑 → 双臂 → 再开 FACTR（`1` sync → `2` teleop）。

核验脚本：`./scripts/verify_p2_smoke.sh`（需已连真机启动驱动）。

## 急停

1. 硬件急停按钮。
2. 软件：`ros2 service call /gento/emergency_stop std_srvs/srv/Trigger`
3. 或 `/gento/stop_motion`。

## 复位

排除故障 → hold / `set_mode` 再进阻抗 → 勿急停后立刻高速遥操。

## 退出

停主手 → stop/hold → 节点退出（析构 `Unlink`）。

## 隔离

- 主线：`skye_ros2_ws`
- `marvin_ws` 仅参考；勿与本驱动同时启动连同一 IP。

## 备注：rqt 看不到数据

控制流为 BEST_EFFORT；rqt 默认 RELIABLE 会 QoS 不兼容。用 `ros2 topic echo/hz` 验收即可。
