# Thor 侧状态（DAgger 已接入）

**已解压：** `vendor/remote_inference_async/`（来自 `async.tar.gz`）。

结论（动作语义）：**绝对关节角**（policy 侧 rad → client 转 degree 仅用于观测/限幅；发往 arbiter 的 chunk **保持 rad**）。

## 已完成的 DAgger 改写

| 文件 | 作用 |
|------|------|
| `vendor/.../dagger_chunk.py` | `[T,16]` 绝对 rad → PolicyActionChunk 字段（pad/truncate 到 16） |
| `vendor/.../dagger_publisher.py` | pub `/skye/policy_action` |
| `vendor/.../dagger_ros_facade.py` | 读 `/gento/joint_states`，不链 Gento SDK |
| `vendor/.../async_robot_client.py` | `--dagger-mode`：强制 dry_run，收 chunk 后 pub |
| `vendor/.../run_robot_dagger.sh` | 入口：跳过 UDP 3720 检查，传 `--dagger-mode` |
| `run_client_dagger.sh` | 包装：`DAGGER_MODE=1`、`DOMAIN=20`、source skye ws |

## 联调命令

```bash
# 已起：driver + hitl teleop + hitl_dagger；5090 已 serve:5568
export ROS_DOMAIN_ID=20
cd docs/dagger_dev/client_thor
bash ensure_cameras.sh   # 或 SKIP_CAMERAS=1 若相机已起

# 基线对照（改动前/回滚）：
# DAGGER_PUBLISH_MODE=raw bash run_client_dagger.sh

# 默认 setpoint 微窗（P0-a）：
bash run_client_dagger.sh
```

`DAGGER_PUBLISH_MODE=raw|setpoint`（默认 setpoint）；`DAGGER_ERR_MAX_RAD`（默认 0.15）。
arbiter profile 已设 `chunk_start_max_jump_rad: 0.3`（需重新 source/launch）。

## 仍可能现场要对齐的

1. 相机 ROS setup 路径：`CAMERA_ROS_SETUP`（默认 `/home/user/tianji_capture_camera_ws/...`）
2. Skye ws：`SKYE_WS` / `SKYE_WS_SETUP`（要有 `PolicyActionChunk`）
3. 5090：`INFERENCE_HOST` / `INFERENCE_PORT`（默认 `192.168.201.253:5568`）
