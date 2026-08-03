# ROS 接口（与 marvin `/gento/*` 对齐）

单位：**rad**。控制流 QoS：`KeepLast(1)` + `BEST_EFFORT`。  
细节与开发顺序见 `dev_plan.md`。

## Topic

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 发布 | `/gento/joint_states` | `sensor_msgs/JointState` | `l_j1..l_j7,r_j1..r_j7`，14 轴 |
| 订阅 | `/gento/left_joint_control` | `JointState` | 7 轴 position（rad） |
| 订阅 | `/gento/right_joint_control` | `JointState` | 7 轴 position（rad） |

节点内可用短名，launch remap 到 `/gento/*`。

## Service

| Service | 类型 | 说明 |
|---------|------|------|
| `/gento/hold_current` | `std_srvs/Trigger` | 保持当前位姿 |
| `/gento/stop_motion` | `std_srvs/Trigger` | 停止并拒指令 |
| `/gento/emergency_stop` | `std_srvs/Trigger` | 软件急停（v0.1） |

## 关键参数（首版）

| 参数 | 默认 | 说明 |
|------|------|------|
| `robot_ip` | `6.6.7.190` | 控制器 |
| `state_publish_hz` | `250` | 状态发布（可调 100/250/500） |
| `left/right_velocity_ratio` | `10` | SDK 速度比例 |
| `joint_signs` | `[1,1,1,-1,1,-1,-1]` | 双臂语义符号 |
| `max_delta_per_cycle` | `0.05` | rad/周期 |
| `command_timeout_s` | `0.20` | 超时 hold |
| `pd_cycle_time_ms` | 与频率匹配 | `SetPDCmdCycleTime` |
