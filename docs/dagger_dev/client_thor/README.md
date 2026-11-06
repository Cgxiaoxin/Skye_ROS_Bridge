# client_thor — 机器人侧（部署到 Thor）

## 作用

1. 起相机（`ensure_cameras.sh`）  
2. 调 5090 推理（现有 `remote_inference_async`，**源码待补**）  
3. **DAgger：** 把动作发成 `/skye/policy_action`，**禁止**再写 `CONTROLLER_IP`

HITL 栈（driver / 小臂 / arbiter）仍按 `../hitl_dagger_thor_现场联调步骤.md` 另开终端。

## 本目录

| 文件 | 说明 |
|------|------|
| `ensure_cameras.sh` | 自现场恢复；默认 `ROS_DOMAIN_ID=20` |
| `run_robot_unijepa_env.sh` | 原 env 包装（仍指向旧 `run_robot.sh`，**仅作对照**） |
| `run_client_dagger.sh` | DAgger 入口占位：检查 vendor + 提示改写点 |
| `policy_chunk_bridge.py` | 如何组 `PolicyActionChunk` 的骨架（需接上真实 action） |
| `NEED_FROM_YOU.md` | 缺的 Thor 源码清单 |
| `vendor/` | 请把 `remote_inference_async/` 整树放这里 |

## 推荐联调顺序

```bash
# Domain 与 HITL 一致
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 已起：driver + hitl teleop + hitl_dagger
bash ensure_cameras.sh

# 5090 已 serve 后：
# 1) 把 remote_inference_async 拷到 vendor/
# 2) 改写：禁用 CONTROLLER 写臂 → 调 policy_chunk_bridge 发 chunk
bash run_client_dagger.sh
```

## 改写现网 client 的最小 diff 思路

在「拿到模型 action 轨迹」之后、`CONTROLLER_IP` 下发之前：

1. `if DAGGER_MODE=1: publish PolicyActionChunk; return`  
2. 否则保留旧 stream（仅非 HITL 调试用）

绝对角空间必须与 `/gento/joint_states` 一致；相对动作先加当前姿再填 chunk。
