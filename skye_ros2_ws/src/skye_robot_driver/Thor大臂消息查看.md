# Thor 大臂消息查看

直接连接 Thor（控制器 IP：`6.6.7.191`），查看双臂发布的数据，不需要小臂。

## 1. 启动大臂驱动

终端 A：

```bash
cd /home/lyy/projects/skye_ros_bridge
ROS_DOMAIN_ID=218 bash scripts/start_force_feedback_test.sh thor --connect
```

此配置关闭夹爪，以 Idle 模式连接；连接时会重置错误并请求双臂进入 Idle。
启动前确认同一控制器没有其他 SDK 客户端连接。

## 2. 查看消息数据

终端 B：

```bash
cd /home/lyy/projects/skye_ros_bridge
source /opt/ros/humble/setup.bash
source skye_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=218
export ROS_LOCALHOST_ONLY=1

# 查看已发布的话题及消息类型
ros2 topic list -t

# 以下命令分别显示一条完整消息
ros2 topic echo --once /gento/joint_states
ros2 topic echo --once /gento/left_joint_states
ros2 topic echo --once /gento/right_joint_states
ros2 topic echo --once /gento/left_joint_external_torque
ros2 topic echo --once /gento/right_joint_external_torque
ros2 topic echo --once /gento/left_flange_wrench
ros2 topic echo --once /gento/right_flange_wrench
ros2 topic echo --once /gento/robot_state
```

去掉某条命令中的 `--once` 可持续查看该话题，按 Ctrl+C 结束。

| 话题 | 数据含义 |
|---|---|
| `/gento/joint_states` | 双臂 14 关节的 `name`、`position`、`velocity`、`effort`；先左后右 |
| `/gento/left_joint_states` | 左臂 7 关节；`effort` 是关节力矩传感器反馈 |
| `/gento/right_joint_states` | 右臂 7 关节；`effort` 是关节力矩传感器反馈 |
| `/gento/left_joint_external_torque` | 左臂 `name` 与 `effort`；此处 `effort` 是外部力矩估计，位置和速度为空 |
| `/gento/right_joint_external_torque` | 右臂对应的外部力矩估计 |
| `/gento/left_flange_wrench` | 左法兰的 `wrench.force.x/y/z` 和 `wrench.torque.x/y/z` |
| `/gento/right_flange_wrench` | 右法兰对应的六维力/力矩反馈 |
| `/gento/robot_state` | `[左臂模式, 右臂模式]`，本次预期为 `[0, 0]`（Idle） |

关节名称为左臂 `l_j1`～`l_j7`、右臂 `r_j1`～`r_j7`。位置单位 rad，速度单位 rad/s。
力矩和六维力数值为 SDK 原值；法兰数组按 `[Fx,Fy,Fz,Tx,Ty,Tz]` 映射，
`frame_id` 默认空，实际单位和坐标轴约定待实机确认。全零只能说明当前收到零值，不能据此判断传感器有效性。

本次夹爪关闭；没有下发运动指令时，`*_action_applied` 话题不会产生指令执行记录。

## 3. 查看发布频率

```bash
ros2 topic hz /gento/joint_states
# 或换成其他需要检查的话题，例如：
ros2 topic hz /gento/left_joint_external_torque
```

驱动配置发布频率为 250 Hz。结束查看后，在终端 A 按 Ctrl+C 退出驱动。
