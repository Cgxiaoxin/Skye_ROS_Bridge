# 开发计划与 marvin 对齐

当前主线：`skye_ros2_ws/src/skye_robot_driver`（C++ + `libGentoSDK.so`）。  
参考实现：`marvin_ws/gento_ros2_ws/src/gento_robot_driver`（Position）；本驱动默认 **ImpJoint 阻抗遥操**（可选 PD）。  
夹爪后续补齐，本计划不挡臂控闭环。

---

## 开发顺序

### P0 — 先能编过
- [x] `cd skye_ros2_ws && source /opt/ros/humble/setup.bash`
- [x] `colcon build --packages-select skye_robot_driver`（`./scripts/build.sh`，避开 conda Python）
- [x] 修好 CMake / include / `CMPL_LIN` / RPATH（`third_party/gento_sdk/lib/x86_64`）
- [x] `./scripts/check_sdk_abi.sh`（或手工 `file` / `ldd` / `nm -D`）

### P1 — 最小可跑闭环（v0.1）
- [x] 启动：Link →（Error 则 ResetError）→ Idle → **PD**
- [x] 定时 `GetRT` → 发 `/gento/joint_states`（14 轴，**rad**）
- [x] 订 `/gento/left_joint_control`、`/gento/right_joint_control` → `SetJointPosPDCmd`
- [x] `/gento/emergency_stop` + `/gento/stop_motion` / `/gento/hold_current` + 析构 `Unlink`
- [x] QoS：控制流 `KeepLast(1)` + `BEST_EFFORT`
- [x] 逻辑迁移自 `gento_robot_driver`，默认控制模式 **Position → ImpJoint**（可选 PD）
- [x] 无真机接口核验：`scripts/verify_p1_interfaces.sh`（`connect_on_startup:=false`）

### P2 — 真机冒烟
- [x] 默认控制模式改为 **关节阻抗 ImpJoint**（对齐 Apex `set_mode=3`）
- [x] `/gento/robot_state` + `/gento/set_mode`（3→ImpJoint）
- [x] 核验脚本：`scripts/verify_p2_smoke.sh`（反馈 + 模式；小幅运动仍需人工确认）
- [ ] 低速、小幅、**单臂**人工运动确认（现场）
- [ ] 确认 `/gento/joint_states` 与实机动作一致（现场）
- [ ] 再开双臂；`vel_ratio` 先 10（现场）

### P3 — 安全层
- [x] 关节限位（rad）— 超限 **逐轴 clamp**（NaN 仍整帧拒绝）
- [x] `max_delta_per_cycle`（默认 0.05 rad）
- [x] `vel_ratio` / `acc_ratio`（独立参数）
- [x] `SetPDCmdCycleTime`（默认 4 ms @ 250 Hz）
- [x] `command_timeout` → hold
- [x] 核验脚本：`scripts/verify_p3_safety.sh`

### P4 — 主手对接
- [x] FACTR remap 对齐 `/gento/*`：`start_teleop_m6_dual_gento.launch.py`
- [x] Docker 挂载改为 `marvin_ws`（`scripts/run_marvin_m6_impedance.sh`）
- [x] 主机启动脚本：`scripts/start_skye_for_factr.sh`（不起旧 gento 驱动）
- [x] `/gento/joint_states` 改为 **RELIABLE**（FACTR sync 可订阅）
- [x] 文档：`docs/小臂大臂启动步骤.md`
- [ ] 实机：Docker 小臂 + 主机驱动，键 `1` sync → `2` teleop

### P5 — 夹爪（DM4310 Terminal CANFD）
- [x] 扩 `skye_robot_driver`：MIT + `FX_L1_Terminal_*`（对齐 Thor `gripper_bridge.py`）
- [x] 订 `/left|right_teleop_gripper/ctrl`，发 `/left|right_gripper/state`
- [ ] 实机：与 FACTR 联调开合 + 力反馈

---

## 与 marvin_ws 必须对齐的信息

本地现状：**小臂（FACTR 主手）+ 大臂（Gento 从臂）双臂**，同机 / 同 `ROS_DOMAIN_ID`（P4 固定 `21`）。  
活跃链路（bridge-less）：

```text
factr_teleop_{left,right}
  ← /gento/joint_states          # 14-DOF 反馈，sync 用
  → /gento/{left,right}_joint_control
gento_robot_driver / skye_robot_driver   # 唯一 SDK 客户端
  IP 6.6.7.190
```

启动参考（旧）：`marvin_ws/start_gento_dual_arm_sync.sh`；P4 日常：`docs/小臂大臂启动步骤.md`  
参数参考：`marvin_ws/gento_ros2_ws/src/gento_robot_driver/config/gento_robot.yaml`

### 1. Topic / Service（对外契约优先兼容 `/gento/*`）

| 方向 | Topic / Service | 类型 | 约定 |
|------|-----------------|------|------|
| 反馈 | `/gento/joint_states` | `sensor_msgs/JointState` | **14** position(+velocity)，rad |
| 指令 | `/gento/left_joint_control` | `JointState` | **7** position，rad；可忽略 `name` |
| 指令 | `/gento/right_joint_control` | `JointState` | 同上 |
| 服务 | `/gento/hold_current` | `std_srvs/Trigger` | 保持当前反馈位，仍可接新指令 |
| 服务 | `/gento/stop_motion` | `std_srvs/Trigger` | 停轨/切 Idle，拒绝指令直到 hold |
| 建议 | `/gento/emergency_stop` | `Trigger` | v0.1 软件急停（可与 stop 合并实现） |

节点内部可用 `/joint_states`、`/left_joint_control`…，**launch remap 到 `/gento/*`**，与旧 FACTR 脚本兼容。

### 2. 关节名 / 顺序 / 单位

| 项 | 值 |
|----|-----|
| 名字 | `l_j1..l_j7, r_j1..r_j7` |
| 顺序 | 左 `[0..6]`，右 `[7..13]` |
| ROS 单位 | **一律 rad / rad·s⁻¹** |
| SDK 边界 | 仅在 `driver_core` 内 °↔rad |
| 左臂对象 | `FX_OBJ_ARM0` |
| 右臂对象 | `FX_OBJ_ARM1` |

FACTR：`follower_joint_offset` 左=0、右=7（`grav_comp_m6_{left,right}.yaml`）。

### 3. 语义映射（驱动侧，不是舵机接线符号）

默认 **`teleop_mapping_mode: relative`**（相对增量）。切 TELEOP 首帧大臂保持当前位，只跟手小臂相对位移；sync 未完成也不会猛追绝对角。

`absolute` 为旧行为：`mapped[i] = leader[order[i]] * signs[i] + offsets[i]`。

相对模式公式：

```text
q_cmd[i] = q_gento_ref[i] + sign[i] * (q_leader[i] - q_leader_ref[i])
```

J4 现场 `sign=-1` 外形反了，改回全 `+1`。不改 `joint_states`。  
限位对齐大臂 URDF（左右相同）。超出大臂行程走逐轴 clamp。  
**不要**照搬 FACTR Dynamixel 的 `joint_signs`（那是小臂舵机方向）。

### 4. 安全默认（先抄后调）

```text
signs:      [1, 1, 1, 1, 1, 1, 1]
limits_min: [-3.1067,-2.0944,-3.1067,-2.5307,-3.1067,-1.0472,-1.5708]
limits_max: [ 3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708]
max_delta_per_cycle: 0.05   # rad / 周期
command_timeout_s:   0.50   # 超时 → 按臂 hold
left/right_velocity_ratio: 20
```

超限：**逐轴 clamp**。NaN / 非 7 轴仍整帧拒绝。  
不要把 FACTR 小臂 J4 min 收到 `-1.0`。

### 5. 主手模式（FACTR）

| 键 | 行为 |
|----|------|
| `1` sync | 读大臂 `/gento/joint_states`，小臂跟到同步 |
| `2` teleop | 发 `/gento/{left,right}_joint_control` |
| `3` stop | 停主手输出 |

### 6. 夹爪（DM4310，FACTR 对齐）

| Topic | 说明 |
|-------|------|
| `/left_teleop_gripper/ctrl`、`/right_teleop_gripper/ctrl` | 主手夹爪指令，`JointState.position[0]∈[0,1]` |
| `/left_gripper/state`、`/right_gripper/state` | 夹爪状态（归一化位 + 电机 vel/effort） |

实现：同进程 Terminal CANFD + MIT；设计见 `docs/superpowers/specs/2026-08-07-dm-gripper-terminal-design.md`。

### 7. 与旧驱动差异（本仓库主动改）

| 项 | marvin `gento_robot_driver` | 新 `skye_robot_driver` |
|----|----------------------------|-------------------------|
| 模式 | Position + `SetJointPosCmd` | 默认 **ImpJoint** + `SetJointPosCmd`（可切 Position/PD） |
| QoS | 默认 Reliable，depth 10 | 指令 **BEST_EFFORT** KeepLast(1)；**状态 RELIABLE** KeepLast(1)（FACTR sync） |
| SDK 路径 | 外链绝对路径 | `third_party/gento_sdk` |
| 工作区 | `marvin_ws/gento_ros2_ws` | `skye_ros2_ws` |

对外 topic 契约尽量不变 → 小臂 FACTR **可少改**。

### 8. 关键参考文件

- `marvin_ws/start_gento_dual_arm_sync.sh`
- `marvin_ws/gento_ros2_ws/src/gento_robot_driver/config/gento_robot.yaml`
- `marvin_ws/gento_ros2_ws/src/gento_robot_driver/src/{driver_core,gento_robot_driver_node}.cpp`
- `marvin_ws/docs/superpowers/specs/2026-07-24-gento-factr-teleop-bridge-less-design.md`
- `marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_{left,right}.yaml`

---

## 文档保留说明

| 文件 | 用途 |
|------|------|
| `docs/dev_plan.md` | 本文件：开发顺序 + 对齐清单 |
| `docs/ros_interfaces.md` | 对外接口速查 |
| `docs/小臂大臂启动步骤.md` | 现场启停 / 急停 |
| 根目录 `遥操高频优化路线.md` | QoS / 高频优化（Orin 侧参考，勿重复开文档） |
| `third_party/gento_sdk/README.md` | SDK vendor / 换架构 so |
