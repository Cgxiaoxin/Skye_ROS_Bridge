小臂ros2：
/gento/joint_states                // Gento 大臂14关节反馈（l_j1..l_j7, r_j1..r_j7，单位rad）
/gento/left_joint_control          // Gento 左大臂目标关节控制命令（7个关节，单位rad）
/gento/right_joint_control         // Gento 右大臂目标关节控制命令（7个关节，单位rad）
/joint_control                     // 小臂节点接收的合成关节控制（完整14轴的目标，ROS1遗留/桥接用）
/left_leader_arm/current_state     // 左主控臂当前关节状态（通常为小臂/leader的反馈）
/left_leader_arm/target_joint_state// 左主控臂的目标关节状态
/mode/switch_stop                  // 模式切换：请求停止
/mode/switch_sync                  // 模式切换：小臂请求对齐到大臂当前位姿
/mode/switch_teleop                // 模式切换：进入遥操作模式（大臂跟随小臂，相对映射）
/mode/switch_telepose              // 模式切换：进入定姿遥操作（仅做位姿同步，临时测试用）
/right_joint_control               // Gento 右大臂目标关节控制命令（7个关节，单位rad）（与/gento/right_joint_control重复，区分具体发布/订阅节点）
/right_leader_arm/current_state    // 右主控臂当前关节状态
/right_leader_arm/target_joint_state// 右主控臂的目标关节状态
/teleop/state                      // 当前遥操作系统的主状态（IDLE/SYNCED/TELEOP等，用于状态监控）

大臂的节点：
/gento/left_joint_control、/gento/right_joint_control 
同步小臂前，需要进入teleop_syncing 同步后变成SYNCED，遥操TELEOP 模式


打印小臂的当前状态
当前关节值（rad）：

  左臂: [-1.1060, 1.7810, 0.9833, -0.2715, 0.7026, -0.1411, 0.0353]
  右臂: [ 1.0799, 1.7135,-1.3683, -0.4985,-0.4771, -0.0874, 0.0015]
  模式: IDLE

读取小臂的状态：
sudo docker exec -it gento_leader_teleop bash -lc '
  source /opt/ros/humble/setup.bash
  source /marvin_ws/install/setup.bash
  export ROS_DOMAIN_ID=20
  export ROS2CLI_USE_DAEMON=0

  echo LEFT
  ros2 topic echo --once /left_leader_arm/current_state

  echo RIGHT
  ros2 topic echo --once /right_leader_arm/current_state

  echo MODE
  ros2 topic echo --once \
    --qos-durability transient_local \
    --qos-reliability reliable \
    /teleop/state
  '
小臂的单位是rad 弧度，不是角度。

# 节点日志：
sudo docker logs --tail 240 gento_leader_teleop
# 观察同步过程：
sudo docker logs -f gento_leader_teleop

# 读大臂
加载环境，可不跑
source /opt/ros/humble/setup.bash
source /data/coding/tianji/Skye-mutile-arm/marvin_ws/gento_ros2_ws/
install/setup.bash

export ROS_DOMAIN_ID=20
unset ROS_LOCALHOST_ONLY
export ROS2CLI_USE_DAEMON=0

确认驱动在线：

ros2 node info /gento_robot_driver

确认关节状态只有一个发布者：

ros2 topic info -v /gento/joint_states

读取一帧当前关节角度：

ros2 topic echo --once /gento/joint_states

只看关节名称和位置：

ros2 topic echo --once /gento/joint_states \
--field name --field position

持续观察：
ros2 topic echo /gento/joint_states

# 大臂被连接处理方式：
