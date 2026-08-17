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

按 `docs/dev_plan.md`：**P0–P3 完成**；**P4 代码已对齐**（待实机 sync/teleop）→ P5 夹爪。

FACTR 对接文档：`docs/p4_factr_teleop.md`  
主机起驱动：`./scripts/start_skye_for_factr.sh`  
小臂 Docker：`./scripts/run_marvin_m6_impedance.sh`（挂载 `marvin_ws` + `/scripts`）  
换主臂串口：`python3 /scripts/bind_leader_arms.py`（见 `docs/新主臂串口绑定.md`）  
小臂 `marvin_ws/install`（clone 后缺失）：`GITLAB_TOKEN=... ./scripts/bootstrap_marvin_install.sh`（见 `docs/小臂大臂启动步骤.md`）

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
