# client_thor — 机器人侧（部署到 Thor）

## 作用

1. 起相机（`ensure_cameras.sh`）  
2. 调 5090 推理（`vendor/remote_inference_async`）  
3. **DAgger：** 把动作发成 `/skye/policy_action`，**禁止**再写 `CONTROLLER_IP`

HITL 栈（driver / 小臂 / arbiter）仍按 `../hitl_dagger_thor_现场联调步骤.md` 另开终端。

## 本目录

| 文件 | 说明 |
|------|------|
| `ensure_cameras.sh` | 自现场恢复；默认 `ROS_DOMAIN_ID=20` |
| `run_robot_unijepa_env.sh` | 原 env 包装（对照；仍写控制器） |
| `run_client_dagger.sh` | **DAgger 入口** → `vendor/.../run_robot_dagger.sh` |
| `policy_chunk_bridge.py` | 单次 dummy chunk 发布（联调口 A） |
| `NEED_FROM_YOU.md` | 现场路径 / 端口对齐清单 |
| `vendor/remote_inference_async/` | UniJEPA client + DAgger 改写 |

## 推荐联调顺序

```bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 已起：driver + hitl teleop + hitl_dagger
bash ensure_cameras.sh

# 5090 已 serve:5568 后：
bash run_client_dagger.sh
```

## 改写点（已落地）

`--dagger-mode` 时：

1. 读 `/gento/joint_states`（`DaggerRosRobotFacade`），不 `link` Gento SDK  
2. 收到 5568 action chunk（绝对 rad `[T,16]`）→ `PolicyActionChunk` → `/skye/policy_action`  
3. 本地强制 `dry_run`，不向控制器 stream

单测：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest vendor/remote_inference_async/test_dagger_chunk.py -v`
