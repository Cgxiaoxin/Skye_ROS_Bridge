# `grav_comp_m6_left.yaml` 参数使用情况

检查对象：[config/grav_comp_m6_left.yaml](../config/grav_comp_m6_left.yaml)。判断标准是参数值是否被 `config.py` 和测试以外的代码读取，并影响主遥操运行。配置解析、类型校验本身不算实际使用；辅助检查程序单独注明。结论基于当前仓库的代码调用路径，未进行硬件运行验证。

左臂启动时，[launch_support.py](../skye_factr_teleop/launch_support.py) 将该 YAML 路径传给节点；[ros_driver.py](../skye_factr_teleop/ros_driver.py) 加载配置。主运行中的参数主要由 `ros_driver.py` 和 [control_core.py](../skye_factr_teleop/control_core.py) 消费。

| 结论 | YAML 参数 | 依据 |
| --- | --- | --- |
| 主遥操运行中使用 | `dynamixel.dynamixel_port`;<br>`dynamixel.servo_types`;<br>`dynamixel.joint_signs` | `ros_driver.py` 创建硬件驱动；`control_core.py` 用型号确定电流限制和转换，用方向符号转换关节状态及输出电流。 |
| 主遥操运行中使用 | `arm_teleop.leader_urdf_package`;<br>`arm_teleop.leader_urdf`;<br>`arm_teleop.baudrate`;<br>`arm_teleop.arm_limit_barrier_enable`;<br>`arm_teleop.arm_joint_limits_max`;<br>`arm_teleop.arm_joint_limits_min`;<br>`arm_teleop.arm_joint_limits_safety_margin` | `ros_driver.py` 创建动力学模型与硬件驱动；`control_core.py` 控制关节限位。 |
| 主遥操运行中使用 | `gripper_teleop.limit_min`;<br>`gripper_teleop.limit_max`;<br>`gripper_teleop.limit_setpoint` | `control_core.py` 将上下限用于限位力矩，将设定值用作同步时的默认夹爪目标。 |
| 主遥操运行中使用 | `controller.frequency`;<br>`controller.teleop_sync_timeout`;<br>`controller.gravity_comp.enable`;<br>`controller.gravity_comp.mode`;<br>`controller.gravity_comp.gain`;<br>`controller.static_friction_comp.enable_speed`;<br>`controller.static_friction_comp.gain`;<br>`controller.joint_limit_barrier.kp`;<br>`controller.joint_limit_barrier.kd` | `ros_driver.py` 用频率创建定时器和轨迹；`control_core.py` 使用其余参数计算力矩与同步超时。静摩擦补偿跟随 `gravity_comp.enable` 开关。 |
| 主遥操运行中使用 | `controller.hold_ruckig_control.ruckig_kp`;<br>`controller.hold_ruckig_control.ruckig_kd`;<br>`controller.hold_ruckig_control.hold_kp`;<br>`controller.hold_ruckig_control.hold_kd`;<br>`controller.hold_ruckig_control.hold_ki`;<br>`controller.hold_ruckig_control.hold_i_max`;<br>`controller.hold_ruckig_control.hold_i_thresh`;<br>`controller.hold_ruckig_control.ruckig_gravity_comp_gain`;<br>`controller.hold_ruckig_control.hold_gravity_comp_gain` | `control_core.py` 在同步、保持模式计算跟踪和重力补偿力矩。 |
| 开关被读取；当前配置下其余值不生效 | `controller.null_space_regulation.enable`;<br>`controller.null_space_regulation.null_space_joint_target`;<br>`controller.null_space_regulation.kp`;<br>`controller.null_space_regulation.kd` | `enable: false`；`control_core.py` 只在启用且不处于同步／保持运动模式时读取目标和增益。 |
| 仅配置校验或辅助检查使用 | `name`;<br>`arm_teleop.num_arm_joints` | `name` 在 `readonly_check.py`、`low_current_check.py` 中用于核对所选侧；`num_arm_joints` 仅在 `config.py` 中校验为 7，主运行代码使用固定的关节数常量。 |
| 未发现实际消费 | `arm_teleop.leader_name`;<br>`arm_teleop.use_suction_cup`;<br>`arm_teleop.initialization.calibration_joint_pos`;<br>`arm_teleop.initialization.initial_match_joint_pos`;<br>`arm_teleop.enable_follower_gravity_comp`;<br>`arm_teleop.follower_urdf_package`;<br>`arm_teleop.follower_urdf`;<br>`arm_teleop.follower_joint_offset`;<br>`arm_teleop.follower_gravity_in_world` | 除配置解析、校验和测试外，未发现这些字段被读取。`select_joint_offsets()` 虽接受校准位置，但主遥操未调用；启动时读取的是 ROS 参数 `joint_offsets`。 |
| 未发现实际消费 | `gripper_teleop.kp`;<br>`gripper_teleop.kd`;<br>`controller.torque_feedback.enable`;<br>`controller.torque_feedback.gain`;<br>`controller.torque_feedback.damping`;<br>`controller.torque_feedback.motor_scalar`;<br>`controller.joint_position_control.interpolation_step_size`;<br>`controller.joint_position_control.kp`;<br>`controller.joint_position_control.kd`;<br>`controller.gripper_feedback.enable`;<br>`controller.gripper_feedback.gain`;<br>`controller.gripper_feedback.ema_beta`;<br>`controller.gripper_feedback.effort_threshold`;<br>`controller.gripper_feedback.damping`;<br>`controller.gripper_feedback.ema_fast_alpha` | 除配置解析、校验和测试外，未发现这些字段被读取。 |

## 运行行为补充

- `controller.gravity_comp.gain` 用于普通模式；同步和保持模式分别改用 `ruckig_gravity_comp_gain`、`hold_gravity_comp_gain`，见 `control_core.py` 的 `_calculate_torques()`。
- 当前 `gripper_teleop.limit_setpoint` 为 `0.55`，高于 `limit_max` 的 `0.52517`。同步目标采用前者，启用的夹爪限位采用后者，因此夹爪向目标运动时会与限位控制相互作用，见 `control_core.py` 的 `_apply_actions()` 和 `_calculate_torques()`。
