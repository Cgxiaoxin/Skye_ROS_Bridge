# Stop That Shit 日常速查

它用于约束 AI Coding Agent：该做的完整做完；未授权的修改、无用途校验和、无关重构和重复检查则停下。[项目 README](https://github.com/lennney/stop-that-shit#%E5%89%8D%E5%90%8E%E5%AF%B9%E6%AF%94)

## 最常用：声明本次任务模式

```text
$stop-that-shit review -- Review 这个 diff，只报告问题，不要修改。
```

- 适用：代码审查、排查问题、只想获得建议。
- 效果：只读；写文件会被 Guard 阻止。
- 对比：普通 Review 可能“发现问题顺手修掉”；此模式只报告问题。

```text
$stop-that-shit answer -- 为什么这个测试失败？
```

- 适用：技术问答、解释代码。
- 效果：只读回答，不应改文件。

```text
$stop-that-shit change -- 修复配置读取失败，补齐受影响的调用方和检查。
```

- 适用：明确要实施修复或功能。
- 效果：允许完成任务必需的修改，包括真实受影响的调用方和测试；不允许借机重构无关模块。

```text
$stop-that-shit monitor -- 持续检查构建日志，有新失败再报告。
```

- 适用：需要持续观察构建、日志或外部状态。
- 效果：持续检查和汇报，但保持只读。

```text
$stop-that-shit watch -- 观察本次 Agent 动作。
```

- 适用：先了解插件会如何判定，暂不希望拦截。
- 效果：仅观察，不阻断；适合刚安装时试用。

## 精确放行：有正当理由时使用

```text
$stop-that-shit change hash=allow -- 生成发布流程需要的校验和。
```

- 适用：发布流程会读取 checksum，并在不一致时拒绝产物。
- 对比：无下游用途时只写 `report.csv`；有完整性校验时，同时生成 `.sha256` 并显式授权。

```text
$stop-that-shit change deps=allow -- 添加我要求的解析器依赖。
```

- 适用：本次任务确实需要新增依赖。
- 效果：避免 Agent 因“以后可能有用”擅自加包。

```text
$stop-that-shit change agents=1 -- 使用一个独立测试 subagent。
```

- 适用：你明确需要并行或独立代理协作。
- 效果：限制最多一个子代理，避免无意义“再找人确认”。

```text
$stop-that-shit lock change files=src/config.cjs|test/config.test.cjs -- 修复这个行为。
```

- 适用：你已确定完整修改范围，且希望严格锁定文件。
- 效果：写入范围外文件会被阻止。
- 注意：影响文件尚未查清时不要急着 `lock`，否则可能挡住必要调用方或测试修改。

## 状态与追溯

```text
$stop-that-shit status
$stop-that-shit runtime
$stop-that-shit explain evt_...
$stop-that-shit label evt_... correct
```

- `status`：看当前任务模式与边界。
- `runtime`：查看本地运行记录。
- `explain`：解释某个事件为何被判定。
- `label`：给事件标注 `correct`、`incorrect` 或 `inconclusive`，帮助校正判断。

## 写作版：STSS（减少废话）

```text
$stss rewrite -- 把这个提案写直接。
$stss audit -- 找出防御性废话。
```

- `rewrite`：直接改写文本，删去重复犹豫，但保留关键条件。
- `audit`：只指出问题及最小修改建议，不改写全文。
- 示例：
  - 前：`我们也许大概有可能在六到八周内完成迁移，具体取决于访问审批。`
  - 后：`预计六到八周完成迁移，前提是获得访问审批。`

## 一句话选择

| 你的目的 | 用什么 |
| --- | --- |
| 只看问题，不动代码 | `review` |
| 只要解释或答案 | `answer` |
| 修复/开发 | `change` |
| 持续盯状态 | `monitor` |
| 先观察、不拦截 | `watch` |
| 任务需要 hash / 新依赖 / 子代理 | 对应加 `hash=allow` / `deps=allow` / `agents=N` |
| 已知且必须锁定文件范围 | `lock change files=...` |

核心原则：不是“少改就是好”，而是只做当前交付真正需要的工作；例如已发布数据的迁移、受影响调用方和必要测试，仍应完整完成。
