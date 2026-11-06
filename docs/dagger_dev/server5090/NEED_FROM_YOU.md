# 5090 侧状态

**已入库：** `deploy/`（自 `deploy.tar.gz` 解压），含 `serve_b_levjepa.sh`、`inference_server/`、`policy_adapter.py`、`downliao_normalization.json` 等。

协议：`inference_server/PROTOCOL.md` — 16 维 `[L7,R7,Lg,Rg]`，关节 **radian**；动作经 `normalization.model_to_robot` 为**绝对关节角**。

## 现场启动仍需

| 项 | 说明 |
|----|------|
| 权重 | `LEVJEPA_CKPT=.../last.ckpt`（本仓不含大权重） |
| 归一化 | 默认可指向本目录 `deploy/downliao_normalization.json` |
| 端口 | 与 Thor client 对齐 **5568**（`INFERENCE_PORT`） |
| 包装 | 可用仓内 `serve_unijepa_dagger.sh`，或直接 `deploy/serve_b_levjepa.sh` |

路径对照（现场）：`/mnt/sdb/zph/UniJEPA`。
