# Follower external torque → JointState.effort（FACTR 臂力反馈）

**日期:** 2026-09-09  
**状态:** Approved for planning  
**范围:** 最小可用（方案 A / 实现路线 1）

## 问题

FACTR `controller.torque_feedback` 已实现：从 follower `/joint_state`（remap 后为 `/gento/{left,right}_joint_states`）读 `effort`，经增益打到小臂舵机。

当前 `skye_robot_driver` 的 `/gento/*_joint_states` **只发 position/velocity，不发 effort**。打开 `torque_feedback.enable` 也无臂力手感。

夹爪力反馈走 `/left|right_gripper/state.effort`，与本设计无关。

## 背景结论（探针已验证）

`scripts/torque/probe_external_torque.py` 独占 SDK `GetRT` 实机结果：

| 字段 | 空载表现 | 用途 |
|------|----------|------|
| `m_ARM_FBK_Joint_ExternalTorEst` | `max\|τ\|` ≈ 1.4–2.0 | **采用** → `effort` |
| `m_ARM_FBK_Joint_SensorTor` | J2≈10 等，明显更大 | 更像总/传感力矩，**不用** |
| `ROBOT_SG` `Joint_Tor` | 量级异常（±80～117） | **不用** |

上位机已设末端工具参数时，`ExternalTorEst` 为空载残差小的轴外力估计。`marvin_m6.urdf` 末端与实机夹爪+相机不一致，不宜再让 FACTR Pinocchio 二次扣重力。

因此：

- `effort` = **轴外力矩**（ExternalTorEst，单位与 SDK 一致，按 Nm 文档化）
- yaml：`enable_follower_gravity_comp: False`（避免双重扣重力）
- `torque_feedback.enable: True`（保持/启用）

## 目标

1. `skye_robot_driver` 同周期发布 effort 到 14 轴与左右 7 轴 JointState。  
2. Orin/Thor 四份 `grav_comp_m6_*.yaml` 关闭 follower 重力再减。  
3. `docs/ros_interfaces.md` 写明 effort 语义。  
4. FACTR 在 TELEOP 下能使用臂力反馈（手感再调 gain）。

## 非目标

- 不新增独立力矩 topic  
- 不提供运行时切换 SensorTor / ExternalTorEst  
- 不对 effort 应用 `joint_signs`（与 position 反馈原始系一致；signs 只用于指令映射）  
- 不改夹爪 effort / FACTR 闭源  
- 不在本变更中调优 `torque_feedback.gain`（实机手感后调）

## 架构与数据流

```text
FX_L1_Fbk_GetRT (一次)
  Pos/Vel              → position / velocity（现状）
  ExternalTorEst[7×2]  → effort（新增）
        │
        ├─ /gento/left_joint_states   name×7 + pos/vel/effort×7
        ├─ /gento/right_joint_states  同上
        └─ /gento/joint_states        14 轴：effort = left[7]+right[7]

factr_teleop_left  /joint_state → /gento/left_joint_states
factr_teleop_right /joint_state → /gento/right_joint_states
  torque_feedback 使用 effort；不再减 Pinocchio 重力
```

与现有 7-DoF sync workaround（`2026-09-02-factr-sync-7dof-joint-states-design.md`）兼容：FACTR 仍只订分侧 topic；effort 与 position 同消息、同 stamp。

## 组件改动

### 1. `DriverCore`

- `DualArmState` 增加 `left_effort` / `right_effort`（`JointArray`）。  
- `read_state()` 从  
  `feedback->m_ARMS[i].m_ARM_OUT.m_ARM_FBK_Joint_ExternalTorEst[j]`  
  拷贝为 `double`（SDK 为 `float`）。不做单位换算、不乘 signs。

### 2. `DriverNode`

- `make_arm_joint_state` 增加 effort 参数（或重载），写入 `message.effort`。  
- `publish_state()`：  
  - 14 轴消息：`effort` 长度 14 = 左+右；  
  - 左右 7 轴消息：各带对应 effort；  
  - GetRT 失败则整帧不发（与现网一致，禁止半包）。

### 3. FACTR yaml（四份）

路径：`marvin_ws/configs/{orin,thor}/grav_comp_m6_{left,right}.yaml`

```yaml
enable_follower_gravity_comp: False  # effort 已是 ExternalTorEst；勿再减 Pinocchio
```

注释说明原因。`follower_urdf*` 可保留无害。`torque_feedback.enable` 保持 `True`。

### 4. 文档

`docs/ros_interfaces.md`：三路 joint_states 的 `effort` = 大臂 SDK `ExternalTorEst`（轴外力矩），供 FACTR `torque_feedback`；非总力矩。

## 错误处理

- `GetRT == nullptr`：不发布。  
- effort 长度必须与 position 一致（7 或 14）。  
- 不因 effort 全零而抑制发布（空载残差接近零是合法的）。

## 测试 / 验收

1. 编译 `skye_robot_driver` 通过。  
2. `ROBOT_PROFILE=orin ./scripts/start_skye_for_factr.sh`（或 thor）。  
3. `ros2 topic echo /gento/left_joint_states --once`：`effort` 长度 7，空载量级与探针 ExtTorEst 相近。  
4. 右臂同理；`/gento/joint_states.effort[0:7]` / `[7:14]` 与分侧一致。  
5. Docker 使用更新后的 yaml（`enable_follower_gravity_comp: False`）；TELEOP 下轻推大臂末端，小臂有接触感。

探针回归（可选）：停驱动后仍可用 `scripts/torque/probe_external_torque.py` 对照 SDK 原始值。

## 风险

| 风险 | 缓解 |
|------|------|
| 某轴 effort 符号与小臂手感相反 | 实机记录；必要时再开 signs 设计，本版不做 |
| 空载 ±2 残差造成轻飘偏置 | 降低 `torque_feedback.gain` 或后续加死区 |
| 忘记关 `enable_follower_gravity_comp` | 四份 yaml 一并改 + 文档 |

## 实现顺序（供 plan）

1. DualArmState + read_state effort  
2. publish_state / make_arm_joint_state  
3. 四份 yaml  
4. ros_interfaces.md  
5. 实机 echo + TELEOP 手感抽检  
