# `grav_comp_m6_left.yaml` 参数使用情况

检查对象：[config/grav_comp_m6_left.yaml](../config/grav_comp_m6_left.yaml)。判断标准是参数值是否被 `config.py` 和测试以外的代码读取，并影响主遥操运行。配置解析、类型校验本身不算实际使用；辅助检查程序单独注明。结论基于当前仓库的代码调用路径，未进行硬件运行验证。

左臂启动时，[launch_support.py](../skye_factr_teleop/launch_support.py) 将该 YAML 路径传给节点；[ros_driver.py](../skye_factr_teleop/ros_driver.py) 加载配置。主运行中的参数主要由 `ros_driver.py` 和 [control_core.py](../skye_factr_teleop/control_core.py) 消费。

| 结论 | YAML 参数 | 依据 |
| --- | --- | --- |
| 主遥操运行中使用 | `dynamixel.dynamixel_port`;<br>`dynamixel.servo_types`;<br>`dynamixel.joint_signs` | `ros_driver.py` 创建硬件驱动；`control_core.py` 用型号确定电流限制和转换，用方向符号转换关节状态及输出电流。 |
| 主遥操运行中使用 | `controller.joint_gravity_comp.dynamics.leader_urdf_package`;<br>`controller.joint_gravity_comp.dynamics.leader_urdf`;<br>`dynamixel.baudrate` | `ros_driver.py` 创建动力学模型与硬件驱动。 |
| 主遥操运行中使用 | `controller.limit_barrier_control.gripper.limit_max`;<br>`controller.limit_barrier_control.gripper.limit_min`;<br>`controller.limit_barrier_control.gripper.kp`;<br>`controller.limit_barrier_control.gripper.kd`;<br>`controller.sync_hold_control.gripper_setpoint` | `control_core.py` 用夹爪上下限和独立增益计算限位力矩，将设定值用作同步时的默认夹爪目标。 |
| 主遥操运行中使用 | `controller.frequency`;<br>`controller.joint_gravity_comp.enable`;<br>`controller.joint_gravity_comp.mode`;<br>`controller.joint_gravity_comp.gain`;<br>`controller.static_friction_comp.enable_speed`;<br>`controller.static_friction_comp.gain` | `ros_driver.py` 用频率创建定时器和轨迹；`control_core.py` 使用其余参数计算力矩。静摩擦补偿跟随 `joint_gravity_comp.enable` 开关。 |
| 主遥操运行中使用 | `controller.limit_barrier_control.enable`;<br>`controller.limit_barrier_control.joint.limit_max`;<br>`controller.limit_barrier_control.joint.limit_min`;<br>`controller.limit_barrier_control.joint.safety_margin`;<br>`controller.limit_barrier_control.joint.kp`;<br>`controller.limit_barrier_control.joint.kd` | `control_core.py` 用开关、边界、裕量和增益计算关节限位回推力矩。 |
| 主遥操运行中使用 | `controller.sync_hold_control.ruckig.kp`;<br>`controller.sync_hold_control.ruckig.kd`;<br>`controller.sync_hold_control.ruckig.gravity_comp_gain`;<br>`controller.sync_hold_control.ruckig.trajectory_timeout`;<br>`controller.sync_hold_control.hold.kp`;<br>`controller.sync_hold_control.hold.kd`;<br>`controller.sync_hold_control.hold.ki`;<br>`controller.sync_hold_control.hold.i_max`;<br>`controller.sync_hold_control.hold.i_thresh`;<br>`controller.sync_hold_control.hold.gravity_comp_gain`;<br>`controller.sync_hold_control.hold.sync_position_tolerance`;<br>`controller.sync_hold_control.hold.sync_velocity_tolerance`;<br>`controller.sync_hold_control.hold.settle_duration`;<br>`controller.sync_hold_control.hold.settle_timeout` | `control_core.py` 在同步、保持模式计算跟踪与重力补偿力矩，检查超时与收敛。`ruckig.max_velocity/max_acceleration/max_jerk` 由操作员标定后供 `ros_driver.py` 创建轨迹。 |
| 开关被读取；当前配置下其余值不生效 | `controller.null_space_regulation.enable`;<br>`controller.null_space_regulation.null_space_joint_target`;<br>`controller.null_space_regulation.kp`;<br>`controller.null_space_regulation.kd` | `enable: false`；`control_core.py` 只在启用且不处于同步／保持运动模式时读取目标和增益。 |
| 仅配置校验或辅助检查使用 | `name` | `name` 在 `readonly_check.py`、`low_current_check.py` 中用于核对所选侧。 |
| 未发现实际消费 | `controller.force_feedback_control.joint.dynamics.follower_urdf_package`;<br>`controller.force_feedback_control.joint.dynamics.follower_urdf`;<br>`controller.force_feedback_control.joint.dynamics.follower_joint_offset`;<br>`controller.force_feedback_control.joint.dynamics.follower_gravity_in_world` | 除配置解析、校验和测试外，未发现这些字段被读取。启动时读取的是 YAML 中的 `dynamixel.joint_offsets`。 |
| 未发现实际消费 | `controller.force_feedback_control.joint.enable`;<br>`controller.force_feedback_control.joint.follower_gravity_comp_enable`;<br>`controller.force_feedback_control.joint.gain`;<br>`controller.force_feedback_control.joint.damping`;<br>`controller.force_feedback_control.joint.motor_scalar`;<br>`controller.force_feedback_control.gripper.enable`;<br>`controller.force_feedback_control.gripper.gain`;<br>`controller.force_feedback_control.gripper.ema_beta`;<br>`controller.force_feedback_control.gripper.effort_threshold`;<br>`controller.force_feedback_control.gripper.damping`;<br>`controller.force_feedback_control.gripper.ema_fast_alpha` | 除配置解析、校验和测试外，未发现这些字段被读取。 |

## 运行行为补充

- `controller.joint_gravity_comp.gain` 用于普通模式；同步和保持模式分别改用 `ruckig_gravity_comp_gain`、`hold_gravity_comp_gain`，见 `control_core.py` 的 `_calculate_torques()`。
- 当前 `controller.sync_hold_control.gripper_setpoint` 为 `0.55`，高于 `controller.limit_barrier_control.gripper.limit_max` 的 `0.52517`。同步目标采用前者，启用的夹爪限位采用后者，因此夹爪向目标运动时会与限位控制相互作用，见 `control_core.py` 的 `_apply_actions()` 和 `_calculate_torques()`。
