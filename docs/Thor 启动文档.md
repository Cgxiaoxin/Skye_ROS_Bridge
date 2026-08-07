source /etc/apex/apex_ros_env.sh

pkill -f 'factr_apex_adapter.py --side left' || true
setsid -f python3 -u /home/user/factr_apex_adapter.py \
  --side left --execute --enable-gripper-bridge \
  --qos-depth 1 --publish-rate 50 --hold-after 0.05 --state-rate 50 \
  --max-step-rad 0.005 \
  --joint6-ema-alpha 0.30 \
  --max-enable-error 1.5 --max-following-error 0.35 \
  --control-timeout 1.0 --feedback-timeout 2.0 \
  > /home/user/factr_apex_adapter_left.log 2>&1

pkill -f 'factr_apex_adapter.py --side right' || true
setsid -f python3 -u /home/user/factr_apex_adapter.py \
  --side right --execute --enable-gripper-bridge \
  --qos-depth 1 --publish-rate 50 --hold-after 0.05 --state-rate 50 \
  --max-step-rad 0.005 \
  --joint6-ema-alpha 0.30 \
  --max-enable-error 1.5 --max-following-error 0.35 \
  --control-timeout 1.0 --feedback-timeout 2.0 \
  > /home/user/factr_apex_adapter_right.log 2>&1

## 通信对齐方式：
FACTR 小臂 (factr_teleop_*.py)
  pub  /gento/left|right_joint_control   ← JointState, 7 轴, rad
  sub  /gento/joint_states               ← JointState, 14 轴, rad（sync）

skye_robot_driver (C++)
  sub  /gento/left|right_joint_control
  pub  /gento/joint_states


右臂
- -0.49310757848770914
- -1.4848920710927556
- 1.636903308743661
- -1.063645488082531
- -1.4404777346451771
- -0.28158817272655134
- -1.0464549953056563

# 注意：这里的 position 字段填写的是“绝对值”，不是增量。
# 右臂
ros2 topic pub --once /gento/right_joint_control sensor_msgs/msg/JointState \
"{position: [-0.49, -1.48, 1.64, -1.06, -1.44, -0.28, -1.05]}"

# 左臂
ros2 topic pub --once /gento/left_joint_control sensor_msgs/msg/JointState \
"{position: [0.69, -1.55, -1.70, -1.23, -1.47, 0.29, -0.62]}"

# 急停 / 停轨
ros2 service call /gento/emergency_stop std_srvs/srv/Trigger
ros2 service call /gento/stop_motion std_srvs/srv/Trigger
```

## 模式切换（已有 service，无单独节点）

```bash
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 0}"  # IDLE
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 1}"  # POSITION
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 2}"  # IMP_JOINT（默认遥操）
```

切模式本身会先 Idle 再进目标模式，**不会**自动插补到远处目标。  
切完后发远距离 `--once` 仍危险：软件只限每步 delta，不是 MoveJ 整段平滑规划。