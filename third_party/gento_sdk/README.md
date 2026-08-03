# Gento C++ SDK（vendor）

| 项 | 说明 |
|----|------|
| 版本 | 见 `VERSION` |
| 头文件 | `include/`（保持原相对路径） |
| 本机库 | `lib/x86_64/libGentoSDK.so` |
| Orin/Thor | 编译后放入 `lib/aarch64/libGentoSDK.so` |

**不要**放入 `libGentoSDKPY.so` 或 SDK `.cpp` 源码。

更新：仓库根目录执行 `./scripts/sync_gento_sdk.sh`
