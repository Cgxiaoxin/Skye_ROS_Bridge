# 5090 侧还缺什么

本目录目前**只有包装脚本与说明**。仓库 / 对话里**没有**完整 UniJEPA 工程。

## 已从终端看到的

- 路径：`/mnt/sdb/zph/UniJEPA`
- 启动：`deploy/serve_b_levjepa.sh`
- 环境变量示例：
  - `LEVJEPA_CKPT=.../last.ckpt`
  - `LEVJEPA_NORMALIZATION=deploy/downliao_normalization.json`
  - `LEVJEPA_COMPILE_ENCODER=0`
  - `LEVJEPA_COMPILE_ACTION_HEAD=0`
- 另有：`HANDOVER.md`、`deploy/`、`README.md`（内容未贴全）

## 请补充（拷到本目录或给路径 + 打包）

最低优先：

1. **`deploy/serve_b_levjepa.sh`**（全文）
2. **`deploy/` 下被该脚本调用的 Python/配置**（监听端口、obs/action 编解码）
3. **`HANDOVER.md` / 部署说明**（端口默认是否 5568、观测字段）
4. （可选）`deploy/downliao_normalization.json` 或说明 normalization 路径约定

有了这些才能判断：协议是否原样复用，还是要在 5090 直接发 ROS `PolicyActionChunk`。
