# URDF 分类索引

> 把 marvin_ws 内外散落的大臂 / 小臂 URDF 按"产品型号 + 单/双臂"分类，统一用 **symlink** 指向原文件。
> 原文件不会被改动；如果 `install/` 被 colcon 重建，symlink 自动跟随。

## 目录结构

```
urdf_classification/
├── README.md                                ← 本文件
├── 大臂_Marvin_M6-S-CCS-696-V4.0/           ← 从臂 / 大臂 (follower, Marvin M6 双臂机器人)
│   ├── 双臂_带夹爪_marvin_m6.urdf            ← ROS 包内集成版（带夹爪手指 link）
│   ├── 单臂_左_M6-S-L-CCS-696-V4.0.urdf     ← SolidWorks 导出参考版（左）
│   └── 单臂_右_M6-S-R-CCS-696-V4.0.urdf     ← SolidWorks 导出参考版（右）
└── 小臂_M6_isomorphism_leader/              ← 主臂 / 小臂 (leader, FACTR Dynamixel 主臂)
    ├── 双臂_m6_isomorphism.urdf              ← 双臂同构版
    ├── 单臂_左_m6left_isomorphism.urdf       ← 左臂单臂
    └── 单臂_右_m6right_isomorphism.urdf      ← 右臂单臂
```

## 文件清单与原始路径

### 大臂 (Marvin M6-S-CCS-696-V4.0)

| symlink 名 | 真实路径 | 关节命名 | 特征 |
|------------|---------|---------|------|
| `双臂_带夹爪_marvin_m6.urdf` | `marvin_ws/install/share/marvin_m6_description/urdf/marvin_m6.urdf` | `l_j1..l_j7` / `r_j1..r_j7` | 17+ link，含夹爪手指 `left/right_index_finger_link1/2`、`Lgripper_Link11/22`；mesh `package://marvin_m6_description/meshes/*.STL` |
| `单臂_左_M6-S-L-CCS-696-V4.0.urdf` | `/data/documents/tianji/marvin_urdf_/marvin_urdf_left/urdf/Marvin M6-S-L-CCS-696-V4.0 urdf.urdf` | `Joint1_L..Joint7_L` | 7 关节 + Base_L；mesh 走绝对路径（**已失效**，原机器路径） |
| `单臂_右_M6-S-R-CCS-696-V4.0.urdf` | `/data/documents/tianji/marvin_urdf_/marvin_urdf_right/urdf/Marvin M6-S-R-CCS-696-V4.0 urdf.urdf` | `Joint1_R..Joint7_R` | 7 关节 + Base_R；mesh 走绝对路径（**已失效**） |

**校核结论**：`双臂_带夹爪_marvin_m6.urdf` 的 7 个 joint origin 与两个参考单臂 URDF **逐字一致**（Joint1 origin `0 0 0.1745`、Joint3 `0 0.287 0`、Joint5 `0.018 -0.314 0` …），mesh 文件大小也一致（Link1: 563584B，Link7: ~2.5MB），是同一型号的不同命名副本。

### 小臂 (M6 isomorphism / leader)

| symlink 名 | 真实路径 | 关节命名 | 特征 |
|-----------|---------|---------|------|
| `双臂_m6_isomorphism.urdf` | `marvin_ws/install/share/m6_isomorphism_description/urdf/m6_isomorphism.urdf` | `joint1_left..7_left` + `joint1_right..7_right` | 17 link（双臂+torso），**无夹爪 link** |
| `单臂_左_m6left_isomorphism.urdf` | `marvin_ws/install/share/m6_isomorphism_description/urdf/m6left_isomorphism.urdf` | `joint1_left..7_left` | 9 link，单臂 |
| `单臂_右_m6right_isomorphism.urdf` | `marvin_ws/install/share/m6_isomorphism_description/urdf/m6right_isomorphism.urdf` | `joint1_right..7_right` | 9 link，单臂 |

**为何是小臂**：与上方参考大臂 URDF 对比，7 关节 origin **全部对不上**（小臂 Joint1 origin `0 0 0.039` vs 大臂 `0 0 0.1745`），尺寸约为大臂的 1/4；Link7 mesh 仅 842KB vs 大臂 2.5MB；Link7 质量 0.068 kg vs 大臂多 kg 级；不含夹爪 link。"isomorphism（同构）"命名含义即：**小臂运动学结构与大臂同构**，方便 FACTR 遥操同构主从映射。

## 如何分辨大小臂（速查口诀）

| 看什么 | 大臂 | 小臂 |
|--------|------|------|
| 文件名 / 包名 | `marvin_m6_description` / `Marvin M6-S-*-CCS-696-V4.0` | `m6_isomorphism_description` |
| 关节命名 | `l_j*` `r_j*` 或 `Joint*_L/R` | `joint*_left` `joint*_right` |
| 夹爪 link | 有（`*_index_finger_*`、`*gripper_Link*`） | 无 |
| Joint1 origin z | `0.1745` | `0.039` |
| Link7 STL 大小 | ~2.5 MB | ~842 KB |

## 使用 mesh 的注意事项

- **marvin_ws 内的 URDF**（双臂_带夹爪 + 三个 m6_isomorphism）：mesh 走 `package://<pkg>/meshes/...` URI，需要先 `source marvin_ws/install/setup.bash` 让 ROS 能找到 `marvin_m6_description` / `m6_isomorphism_description` 包，然后才能在 RViz 里正确加载。
- **参考大臂 URDF**（两个单臂 M6-S-*-CCS-696-V4.0）：mesh 走绝对路径 `/home/vitai/wyz/tmp/2.3/...`，是从其他机器导出后未清理的路径，**当前环境无法解析 mesh**，只能看 URDF 文本/运动学结构。要用 mesh 需手工把 mesh 路径改成本地 `package://marvin_m6_description/meshes/...`（注意大臂 Link 命名差异：参考是 `Link1_R`，marvin_m6 是 `r1`）。

## 维护

- 新增 URDF 时，请按"产品型号_角色"目录归档，并在本 README 表格里追加一行。
- symlink 失效时（原文件被删 / 移走），重新 `ln -sf <绝对路径> <symlink名>` 即可。
