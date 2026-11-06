# server5090 — UniJEPA 推理服务（部署到 5090）

## 作用

在 5090 上加载 LEVJEPA/UniJEPA checkpoint，对 Thor client 的观测请求返回动作。  
**HITL DAgger 第一阶段建议：服务协议尽量不动**，只改 Thor 侧「写控制器 → 发 `/skye/policy_action`」。

## 本目录文件

| 文件 | 说明 |
|------|------|
| `serve_unijepa_dagger.sh` | 包装启动（对齐你现场命令）；需本机已有 UniJEPA 树 |
| `NEED_FROM_YOU.md` | 缺哪些源码/脚本 |

## 现场启动（你已在用的等价命令）

```bash
cd /mnt/sdb/zph/UniJEPA   # 或你的 UniJEPA 根
PATH=/usr/local/cuda/bin:$PATH \
LEVJEPA_CKPT=/mnt/sdb/zph/UniJEPA_ckpt/.../last.ckpt \
LEVJEPA_NORMALIZATION=deploy/downliao_normalization.json \
LEVJEPA_COMPILE_ENCODER=0 \
LEVJEPA_COMPILE_ACTION_HEAD=0 \
bash deploy/serve_b_levjepa.sh
```

或使用本目录包装脚本（改环境变量路径后）：

```bash
bash serve_unijepa_dagger.sh
```

## 与 DAgger 的边界

- 5090 **不**写 `/gento/*`，**不**装整仓 driver。
- 默认继续对 Thor client 提供 **5568（或脚本内端口）** 私有推理口。
- 若以后要「5090 直接 ROS pub」，再另开方案（需 Domain 打通 + 观测回传）。
