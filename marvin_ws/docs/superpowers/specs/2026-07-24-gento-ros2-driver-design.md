# Gento ROS2 大臂驱动设计

## 目标

创建一个独立的 C++ ROS 2 Humble 工作区和 `gento_robot_driver` 包，使用 x86-64 的 Gento C++ SDK 控制 7 自由度双臂机器人。该驱动替代旧 `robot_servo_driver` 对 Gento 控制器的依赖，不修改现有 Marvin 工作区的二进制驱动。

首个可交付版本只覆盖当前遥操链路必需的功能：连接、读取左右臂实时关节状态、切换位置模式、发送左右 7 轴位置目标、右臂速度限制，以及安全停止。夹爪、末端位姿、运动学、力控与完整阻抗控制不在本次范围。

## 独立工作区

新工作区命名为 `gento_ros2_ws`。它将包含：

```text
gento_ros2_ws/
  src/gento_robot_driver/
    CMakeLists.txt
    package.xml
    include/gento_robot_driver/gento_robot_driver.hpp
    src/gento_robot_driver.cpp
    src/main.cpp
    test/test_sdk_mapping.cpp
    launch/gento_robot_driver.launch.py
    config/gento_robot.yaml
```

Gento SDK 保持在其现有目录 `/data/coding/tianji/tianji-robot-SDK-Gento_Skye-Luna`，驱动通过头文件和 `libGentoSDK.so` 链接它，而不是复制、改名或覆盖旧 `libMarvinSDK.so`。

运行时使用 `RPATH` 或启动脚本指定 Gento SDK 库目录，确保加载的是 `libGentoSDK.so`。构建前会用 `file`、`ldd` 和 `nm -D` 验证该库为 x86-64，并包含所需 `FX_L1_*` 符号。

## 最小接口映射

| ROS 驱动职责 | ROS 接口 | Gento SDK 接口 |
|---|---|---|
| 连接控制器 | 参数 `robot_ip` | `FX_L1_System_Link()` |
| 读取状态 | 发布 `/joint_states` | `FX_L1_Fbk_GetRT()` |
| 进入位置模式 | 启动时状态机 | `FX_L1_State_SwitchToPositionMode()` |
| 左臂位置目标 | 订阅 `/left_joint_control` | `FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM0, ...)` |
| 右臂位置目标 | 订阅 `/right_joint_control` | `FX_L1_Runtime_SetJointPosCmd(FX_OBJ_ARM1, ...)` |
| 右臂速度限制 | 参数 `right_velocity_ratio`（默认 10） | `FX_L1_Runtime_SetSpeedRatio(thread_id, FX_OBJ_ARM1, vel, acc)` |
| 安全停止 | 退出处理与可选服务 | `FX_L1_Runtime_StopTraj()`，随后安全切回空闲状态 |
| 断开连接 | 节点退出处理 | `FX_L1_System_Unlink()` |

驱动只接受包含 7 个 position 元素的 `sensor_msgs/msg/JointState`。每次新目标都会校验长度、有限数值和配置的关节限位；无效消息不会下发 SDK 命令，并记录 ROS 错误日志。

## 状态机与安全

节点启动顺序为：连接 → 查询 SDK/控制器版本 → 获取一次实时反馈 → 对 ARM0、ARM1 调用 `FX_L1_State_SwitchToPositionMode()` → 通过 `FX_L1_Runtime_SetSpeedRatio()` 配置比例 → 开始发布状态并接收命令。任一失败都会终止启动，不会发送位置指令。比例的合法范围为 1–100，首版将右臂速度、加速度均设为 10。

控制命令的首版只会在 `POSITION_READY` 状态下生效。节点退出、SDK 调用失败或显式停止时，停止轨迹并禁止继续下发命令。初始速度限制配置为右臂 10%，左臂默认不主动运动。

## 测试与验收

先以测试驱动开发验证纯逻辑：ROS 名称到 Gento 臂对象映射、7 元素检查、角度/限位检查、右臂 10% 速度转换，以及在非就绪状态拒绝命令。测试必须先失败，再实现逻辑使其通过。

构建后按三层验证：

1. ABI：新节点链接 `libGentoSDK.so`，不再依赖 `libMarvinSDK.so`；所需 `FX_L1_*` 符号可解析。
2. 连接：启动日志打印 SDK 与控制器版本，`/joint_states` 中左右各有 7 个实际关节值。
3. 硬件：在安全清场、上位机允许外部位置控制且右臂 10% 限速时，基于实时 B/J4 当前值发送 `current - 30°`；回读必须显示 J4 朝目标变化。若状态机、限位或 SDK 返回失败，立即停止并报告错误，不重复发送。

## 非目标与兼容性

不覆盖旧的 `libMarvinSDK.so`，不逆向兼容 `OnSet*` ABI，不修改当前 `marvin_m6_driver` 容器启动链路。新驱动上线前会使用独立节点/容器名，防止两个 SDK 同时连接同一控制器。
