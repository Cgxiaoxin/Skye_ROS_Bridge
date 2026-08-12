```bash
# 环境准备
source /etc/apex/apex_ros_env.sh

# 启动左臂 factr_apex_adapter
pkill -f 'factr_apex_adapter.py --side left' || true
setsid -f python3 -u /home/user/factr_apex_adapter.py \
  --side left \
  --execute \
  --enable-gripper-bridge \
  --qos-depth 1 \
  --publish-rate 50 \
  --hold-after 0.05 \
  --state-rate 50 \
  --max-step-rad 0.005 \
  --joint6-ema-alpha 0.30 \
  --max-enable-error 1.5 \
  --max-following-error 0.35 \
  --control-timeout 1.0 \
  --feedback-timeout 2.0 \
  > /home/user/factr_apex_adapter_left.log 2>&1

# 启动右臂 factr_apex_adapter
pkill -f 'factr_apex_adapter.py --side right' || true
setsid -f python3 -u /home/user/factr_apex_adapter.py \
  --side right \
  --execute \
  --enable-gripper-bridge \
  --qos-depth 1 \
  --publish-rate 50 \
  --hold-after 0.05 \
  --state-rate 50 \
  --max-step-rad 0.005 \
  --joint6-ema-alpha 0.30 \
  --max-enable-error 1.5 \
  --max-following-error 0.35 \
  --control-timeout 1.0 \
  --feedback-timeout 2.0 \
  > /home/user/factr_apex_adapter_right.log 2>&1
```

---

## 通信对齐方式

| 模块                    | 主题                                 | 类型                 | 备注                             |
|-------------------------|--------------------------------------|----------------------|----------------------------------|
| FACTR 小臂 (factr_teleop_*.py)  | /gento/left\|right_joint_control   | pub  JointState(7轴,rad) | 控制命令                         |
|                         | /gento/joint_states                  | sub  JointState(14轴,rad, sync) | 大臂反馈              |
| skye_robot_driver (C++) | /gento/left\|right_joint_control     | sub                   | 接收控制                         |
|                         | /gento/joint_states                  | pub                   | 发布反馈                         |

---

## 控制示例

### 右臂示例位置参数

```
- -0.49310757848770914
- -1.4848920710927556
-  1.636903308743661
- -1.063645488082531
- -1.4404777346451771
- -0.28158817272655134
- -1.0464549953056563
```

> ⚠️ 注意：position 字段填写“绝对值”，不是增量。

### 单次发送关节目标

```bash
# 右臂
ros2 topic pub --once /gento/right_joint_control sensor_msgs/msg/JointState \
  "{position: [-0.49, -1.48, 1.64, -1.06, -1.44, -0.28, -1.05]}"

# 左臂
ros2 topic pub --once /gento/left_joint_control sensor_msgs/msg/JointState \
  "{position: [0.69, -1.55, -1.70, -1.23, -1.47, 0.29, -0.62]}"
```

### 急停与停轨

```bash
ros2 service call /gento/emergency_stop std_srvs/srv/Trigger
ros2 service call /gento/stop_motion std_srvs/srv/Trigger
```

---

## 模式切换（已有 service，无单独节点）

```bash
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 0}"  # IDLE
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 1}"  # POSITION
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 2}"  # IMP_JOINT（默认遥操）
```

- 切模式会先 Idle 再进目标模式，**不会**自动插补到远处目标。  
- 切完后直接发远距离 `--once` 命令仍有危险：软件约束只处理每步 delta，而非 MoveJ 整轨迹平滑插补。