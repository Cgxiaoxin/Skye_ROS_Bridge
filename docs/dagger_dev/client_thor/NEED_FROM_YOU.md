# Thor 侧状态（DAgger 已接入）

**已纳入 git：** `vendor/remote_inference_async/`（见包内 [`README.md`](vendor/remote_inference_async/README.md) 分类）。

结论（动作语义）：**绝对关节角**（policy 侧 rad → client 转 degree 仅用于观测/限幅；发往 arbiter 的 chunk **保持 rad**）。

## 已完成的 DAgger 改写

| 文件 | 作用 |
|------|------|
| `dagger_chunk.py` | `[T,16]` 绝对 rad → PolicyActionChunk；command→hold |
| `dagger_quintic.py` / `dagger_home.py` | quintic 回零 0.5°@20Hz + lead |
| `dagger_publisher.py` | pub `/skye/policy_action` |
| `dagger_ros_facade.py` | 读 `/gento/joint_states`，不链 Gento SDK |
| `async_robot_client.py` | `--dagger-mode`：融合/护栏后再 pub（不 raw 直灌） |
| `run_robot_dagger.sh` | HITL 入口 |
| `run_client_dagger.sh` | 包装：`DAGGER_MODE=1`、`DOMAIN=20` |

## 联调命令（在跑 client 的机器上，通常 Thor）

```bash
export ROS_DOMAIN_ID=20
cd docs/dagger_dev/client_thor
bash ensure_cameras.sh   # 或 SKIP_CAMERAS=1 若相机已起
bash run_client_dagger.sh
```

**不必**把本目录同步到 5090；5090 只跑 `server5090/deploy/`。

## 仍可能现场要对齐的

1. 相机 ROS setup：`CAMERA_ROS_SETUP`
2. Skye ws：`SKYE_WS`（要有 `PolicyActionChunk`）
3. 5090：`INFERENCE_HOST` / `INFERENCE_PORT`（默认 `192.168.201.253:5569`）
