# UniJEPA → HITL DAgger 接入说明

> 部署树：`server5090/`（5090 推理机）、`client_thor/`（Thor 机器人侧）。  
> Thor client 源码已落入 `client_thor/vendor/remote_inference_async/`，并完成 `--dagger-mode` 接入（pub `/skye/policy_action`，不写控制器）。  
> **实机验收清单（含「本地 / Thor / 5090 分工」）：** [`unijepa_dagger_实机验证流程.md`](unijepa_dagger_实机验证流程.md)。

## 目标拓扑（HITL on）

```text
[5090]  serve_unijepa（口 5568，可先不动协议）
           ▲ obs / ▼ action（现有私有协议）
[Thor]  cameras + client（采图、组观测、调 5090）
           │  改写点：禁止再写 CONTROLLER_IP
           ▼
        /skye/policy_action   PolicyActionChunk
           ▼
        control_arbiter  →  /gento/*_abs  →  skye_robot_driver → 6.6.7.191
        (+ HITL 小臂；takeover / enter_teleop / return)
```

**禁止：** `run_robot.sh` 与 `skye_robot_driver` 同时直连控制器。

## 启停顺序（联调）

| 顺序 | 机器 | 做什么 |
|------|------|--------|
| 1 | Thor | `skye_robot_driver`（`start_skye_for_factr.sh thor`） |
| 2 | Thor | HITL 小臂 `start_teleop_m6_dual_hitl.launch.py robot_profile:=thor` |
| 3 | Thor | `hitl_dagger.launch.py robot_profile:=thor` |
| 4 | Thor | `client_thor/ensure_cameras.sh`（`ROS_DOMAIN_ID` 与上面一致，Thor 常用 **20**） |
| 5 | 5090 | `server5090/serve_unijepa_dagger.sh` |
| 6 | Thor | `client_thor/run_client_dagger.sh`（`--dagger-mode` → `/skye/policy_action`） |
| 7 | 任意 | `intervention_cmd`: takeover → enter_teleop → return |

Domain / RMW 全链路一致；相机脚本默认曾是 `21`，接 Thor HITL 时请改成与 arbiter 相同。

## 口 A 契约（策略 → arbiter）

- Topic：`/skye/policy_action`
- Type：`skye_hitl_dagger/msg/PolicyActionChunk`
- 关节：**follower / `/gento/joint_states` 空间绝对 rad**，左右各 7，展平 `chunk_size * 7`
- 仅 `AUTONOMOUS` 执行；HUMAN / HANDOVER 时 arbiter 忽略轨迹

参考实现：`skye_hitl_dagger/.../pub_dummy_policy_chunk_node.py`。

## 相对现网脚本的改法（原则）

| 现网 | DAgger |
|------|--------|
| `CONTROLLER_IP=6.6.7.191` + `ARM_COMMAND_STYLE=stream` | **关掉写臂**；改为 pub `PolicyActionChunk` |
| 5090 `serve_b_levjepa.sh` | 可先原样；协议不变则只改 Thor 执行端 |
| 动作若为相对/增量 | 在 client 转成绝对角再组 chunk（需确认语义） |

## 已核实（源码）

1. **5568 动作为绝对关节角**（policy rad；client 下发前转 degree）。非相对增量。  
2. 向量 16 维：`[left_j0..6, right_j0..6, left_grip, right_grip]`（见 `PROTOCOL.md`）。  
3. 仍待现场对齐：chunk 帧数（基础约 16 / action-history 50）与 `dt`（`ACTION_HZ=30` → dt≈1/30）；Domain Thor HITL 建议 **20**。
