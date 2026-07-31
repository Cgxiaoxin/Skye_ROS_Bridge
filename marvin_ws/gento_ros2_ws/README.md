# Gento ROS 2 大臂驱动

该工作空间使用 Gento C++ SDK 直接控制 Skye 大臂，不使用或替换 `libMarvinSDK.so`。

## 接口

- 节点：`/gento_robot_driver`
- 订阅：`/left_joint_control`、`/right_joint_control`（`sensor_msgs/msg/JointState`，`position` 必须恰好为 7 个 rad 值）
- 发布：`/joint_states`（`l_j1..l_j7,r_j1..r_j7`，单位 rad / rad/s）
- 单位边界：ROS 侧始终为 rad / rad/s；驱动会在调用 Gento SDK 前转换为 deg，并把 SDK 的 deg / deg/s 反馈转换回 ROS 单位。
- 左臂 SDK 对象：`FX_OBJ_ARM0`
- 右臂 B SDK 对象：`FX_OBJ_ARM1`

默认 IP 是 `6.6.7.190`，左右臂速度/加速度比均为 `10`（10%）。参数在
`src/gento_robot_driver/config/gento_robot.yaml`。

## 构建

```bash
cd /data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gento_robot_driver --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 正常启动

启动前必须满足：

1. 在上位机中断开机械臂连接；控制器只允许一个 SDK 客户端。
2. 停止旧 Marvin 驱动容器，避免并发连接：

```bash
printf '%s\n' 123 | sudo -S docker stop -t 10 marvin_m6_driver
```

3. 检查工作区和机械臂周围无碰撞风险。

启动：

```bash
cd /data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=20
ros2 launch gento_robot_driver gento_robot_driver.launch.py
```

在另一终端使用相同 ROS 环境读取状态：

```bash
ros2 topic echo --once /joint_states
ros2 topic hz /joint_states
```

只有确认存在 14 个有限的关节位置、关节角合理且工作区安全后，才能发送动作。

## 与小臂遥操共存时的安全测试

`factr_teleop` 可能向兼容控制话题发布小臂数据。对大臂单关节测试时，必须临时重映射控制输入，避免误将小臂目标发给大臂：

```bash
cd /data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=20
ros2 run gento_robot_driver gento_robot_driver --ros-args \
  --params-file install/gento_robot_driver/share/gento_robot_driver/config/gento_robot.yaml \
  -r /left_joint_control:=/gento_test/left_joint_control \
  -r /right_joint_control:=/gento_test/right_joint_control
```

使用 `/gento_test/right_joint_control` 发布测试命令。先从 `/joint_states` 读取当前右臂 7 个位置，将仅要测试的关节替换为目标值，其余 6 个值保持为刚读取值。

例如，右臂 B/J4 向负方向移动 30°：

```text
target_r_j4 = current_r_j4 - 0.523598776  # rad
```

必须先验证 `-2.4 <= target_r_j4 <= 1.0`。之后以 50 Hz 发 2 秒：

```bash
ros2 topic pub -r 50 --times 100 /gento_test/right_joint_control \
  sensor_msgs/msg/JointState \
  "{position: [R_J1, R_J2, R_J3, TARGET_R_J4, R_J5, R_J6, R_J7]}"
```

替换占位符为刚读取的真实反馈。结束后再次读取 `/joint_states`，确认只有预期关节向目标变化。

## 停止

在驱动终端按 `Ctrl-C`。节点关闭路径会依次调用：

1. `FX_L1_Runtime_StopTraj(1, FX_OBJ_ALL_FLAG)`；
2. 左右臂切换至 Idle；
3. `FX_L1_System_Unlink()`。

若 SDK 返回 `someone has already linked to the controller`，不要重试运动命令；先在上位机中确认已经执行“断开连接”，再启动节点。
