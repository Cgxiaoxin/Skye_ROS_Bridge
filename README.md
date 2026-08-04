# Skye_ROS_Bridge

Skye / Gento 双臂 **C++ ROS2** 遥操驱动（本机开发；Orin/Thor 换 `lib/aarch64/libGentoSDK.so` 后重编即可）。

## 目录

```text
Skye_ROS_Bridge/
├── docs/
│   ├── dev_plan.md          # 开发顺序 + 与 marvin 对齐清单 ★
│   ├── ros_interfaces.md    # topic / service 速查
│   └── teleop_sop.md        # 现场启停
├── third_party/gento_sdk/   # headers + lib/<arch>/libGentoSDK.so
├── skye_ros2_ws/            # 主线工作区
│   └── src/skye_robot_driver/
├── scripts/sync_gento_sdk.sh
├── marvin_ws/               # 旧工程（参考，非主线）
└── 遥操高频优化路线.md      # QoS / 高频优化参考
```

## 当前阶段

按 `docs/dev_plan.md`：**P0/P1/P2(模式+脚本)/P3 已完成** → 现场补齐 P2 小幅运动确认 → **P4 主手** → P5 夹爪。

默认遥操模式：**关节阻抗 mode=2**（`imp_joint`）。模式切换用 service：

```bash
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 1}"  # position
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 2}"  # imp_joint
```

核验：
- P1：`skye_ros2_ws/scripts/verify_p1_interfaces.sh`
- P2：`skye_ros2_ws/scripts/verify_p2_smoke.sh`（需真机已 Link）
- P3：`skye_ros2_ws/scripts/verify_p3_safety.sh`

## 构建

```bash
./scripts/sync_gento_sdk.sh   # 可选：刷新 vendor
cd skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skye_robot_driver
source install/setup.bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py
```

## 安全

- IP 默认 `6.6.7.190`；UDP `50000–50010`
- 只用 `libGentoSDK.so`（不要 PY so）
- 同一控制器同时只允许一个驱动连接
