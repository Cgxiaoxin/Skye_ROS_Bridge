# Skye_ROS_Bridge

Skye / Gento 双臂 **C++ ROS2** 遥操桥接（本机开发；Orin / Thor 换对应架构的 SDK `.so` 后重编本仓库即可）。

## 目录

```text
Skye_ROS_Bridge/
├── third_party/gento_sdk/     # 厂商 SDK：头文件 + lib/<arch>/libGentoSDK.so
├── skye_ros2_ws/              # ROS2 工作区（主线）
│   └── src/skye_robot_driver/ # 唯一驱动包
├── scripts/                   # 同步 SDK 等工具
├── marvin_ws/                 # 旧工程（参考，勿作新主线）
└── 遥操高频优化路线.md
```

## 本机构建

```bash
cd skye_ros2_ws
source /opt/ros/humble/setup.bash   # 按实际 ROS 发行版调整
colcon build --packages-select skye_robot_driver
source install/setup.bash
```

## Orin / Thor

1. 在目标架构上（或交叉）编译 `libGentoSDK.so`，放入 `third_party/gento_sdk/lib/aarch64/`
2. 本机同样 `colcon build`（在目标板上编更稳妥）
3. **无需改业务代码**；CMake 按 `uname -m` 选 `lib/x86_64` 或 `lib/aarch64`

同步 SDK 头文件与 so（从本机 SDK 源仓）：

```bash
./scripts/sync_gento_sdk.sh
```

## 说明

- 链接的是 **`libGentoSDK.so`（C++）**，不是 `libGentoSDKPY.so`
- 控制器默认 IP：`6.6.7.190`；勿与旧 Marvin 驱动同时连同一台控制器
