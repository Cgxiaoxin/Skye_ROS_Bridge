# 启动流程
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
./scripts/build.sh          # 已编过可跳过
source install/setup.bash
export ROS_DOMAIN_ID=20 
# 确认：本机到 6.6.7.190 通；无其他 SDK 客户端连同一控制器
ros2 launch skye_robot_driver skye_robot_driver.launch.py

# 查看关节角
ros2 topic echo --once /gento/joint_states

# 位置
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 1}"
# 关节阻抗
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 2}"
# 空闲
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 0}"

# 这个命令会通过 ros2 service call 方式，请求名为 /gento/hold_current 的服务，服务类型为 std_srvs/srv/Trigger，请求体为空（"{}"）。该命令的作用是让机器人“保持当前电流”，也就是在当前状态下将力矩输出保持不变，经常用于需要暂停控制、让机械臂保持当前力矩输出的场景。
ros2 service call /gento/hold_current std_srvs/srv/Trigger "{}"


# 急停
ros2 service call /gento/emergency_stop std_srvs/srv/Trigger "{}"
# 或
ros2 service call /gento/stop_motion std_srvs/srv/Trigger "{}"
# 恢复遥操：
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 2}"
ros2 service call /gento/hold_current std_srvs/srv/Trigger "{}"

- 2.0741369686131805
- -0.25335740017019365
- -1.9900520514097273
- -1.5366468784529195
- 0.1682262767772595
- 0.1471734723727219
- -0.22461302035285732

ros2 topic pub -r 50 --qos-reliability best_effort \
  /gento/left_joint_control sensor_msgs/msg/JointState "{
  name: ['l_j1','l_j2','l_j3','l_j4','l_j5','l_j6','l_j7'],
  position: [2.07025, -0.17, -1.9418, 1.53576, 0.16950, -0.15009, 0.22490],
  velocity: [],
  effort: []
}"


# todo list
A. 先收尾 P2（半天内）
连续流 + 稍大幅 单臂复测，确认 l_j2 Δ≈指令
右臂同样一次
查清 emergency_stop 失败（符号/返回值/是否需先 Idle）
勾完 dev_plan P2 现场项

## 夹爪测试：
# 开合
ros2 topic pub -1 /left_teleop_gripper/ctrl sensor_msgs/msg/JointState \
  "{name: ['gripper_joint'], position: [0.0]}"   # 开
ros2 topic pub -1 /left_teleop_gripper/ctrl sensor_msgs/msg/JointState \
  "{name: ['gripper_joint'], position: [1.0]}"   # 闭
ros2 topic echo /left_gripper/state

# 驱动节点程序残留，清除
pkill -f skye_robot_driver || true
pkill -f gento_robot_driver || true