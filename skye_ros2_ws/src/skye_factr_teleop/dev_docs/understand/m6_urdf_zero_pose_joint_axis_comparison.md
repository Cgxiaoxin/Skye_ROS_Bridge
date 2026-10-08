# M6 URDF 零姿态与关节正方向对比

## 对比对象与结论

本次对比涉及以下三份 URDF：

- 跟随臂：[skye_m6.urdf](../../skye_factr_description/urdf/skye_m6.urdf)
- 左侧 leader：[m6left_isomorphism.urdf](../../skye_factr_description/urdf/m6left_isomorphism.urdf)
- 右侧 leader：[m6right_isomorphism.urdf](../../skye_factr_description/urdf/m6right_isomorphism.urdf)

**结论：当对应关节均取 `q = 0` 时，三份 URDF 中的关节链均从躯干向对应一侧伸展；关节中心的相对位置不同。对应关节的正转轴基本同向，但左臂 J4、J6、J7 有约 1.2° 的轴向偏差。** 从 URDF 运动学定义看，没有发现需要将某个对应关节的角度整体取反的依据。

## 比较方法

将各 URDF 的七个旋转关节均设为 `q = 0`，逐级累乘 `joint/origin`，计算每个关节中心的位置，以及 `joint/axis` 在 `torso_link` 坐标系中的实际方向。这里比较的是变换后的转轴，而非直接比较 XML 中的局部 `axis` 数值。

三份文件的 torso 坐标定义不同：`skye_m6` 的双臂沿 ±Y 展开，两个 isomorphism URDF 的双臂沿 ±X 展开。以下比较先将 `skye_m6` 的计算结果绕 torso Z 轴旋转 **−90°**，使两者的左右臂展开方向对齐。该旋转仅用于比较，并非 URDF 中声明的固定变换。

## 关节零位的姿态构型

在七个旋转关节均为 `q = 0` 时，按关节中心组成的运动链观察：

| URDF | 相对各自 `torso_link` 的伸展方向 | 关节链形态 |
| --- | --- | --- |
| `skye_m6` 左臂、右臂 | 分别朝 +Y、−Y | 从躯干向左右两侧伸展，关节中心主要位于 torso 的 XY 平面 |
| `m6left_isomorphism` | 朝 +X | 从躯干向左侧伸展，关节中心主要位于 torso 的 XY 平面 |
| `m6right_isomorphism` | 朝 −X | 从躯干向右侧伸展，关节中心主要位于 torso 的 XY 平面；J6 有局部 Z 向偏移 |

将坐标方向按前述方式对齐后，三者在**关节链走向**上都是向躯干外侧伸展的零位构型。这里的“同类构型”只描述关节中心的整体走向，不表示关节中心重合、各段长度相等，或 STL 网格呈现完全相同的外观。

## 关节正方向

| 对应关节 | 对齐后的实际正轴 | 差异 |
| --- | --- | --- |
| 左臂 J1、J2、J3、J5 | 与 leader 对应关节同向 | 数值误差约 0.0002° |
| 左臂 J4、J6、J7 | 与 leader 对应关节大体同向 | 各偏约 1.2° |
| 右臂 J1～J7 | 与 leader 对应关节同向 | 最大数值误差约 0.0013° |

左臂的偏差来自 `m6left_isomorphism.urdf` 中 J3 的 `origin rpy`：其中俯仰角为 `0.020931 rad`，约等于 **1.2°**。此偏转传递到后续部分关节的零位转轴。

**注意：**某些 leader 关节在 XML 中写为局部 `axis="0 0 -1"`，而 `skye_m6` 对应关节写为 `axis="0 0 1"`。由于它们的上游关节坐标系朝向不同，这不代表实际正转方向相反；应按上述方式换算到共同坐标系后判断。

## 零位时关节中心的相对位置

下表比较的是**各 URDF 在 `q = 0` 时的关节中心位置**。每侧分别以各自 URDF 的 J1 中心为位置原点，并先按前述 −90° 旋转对齐 `skye_m6` 的坐标轴。数值是相对 J1 的位移 `(X, Y, Z)`，单位为米；它们不表示关节零点角度或连杆的实际长度。

| 手臂与关节中心 | `skye_m6`：相对 J1 位移 | leader：相对 J1 位移 |
| --- | ---: | ---: |
| 左臂 J2 | `(0, 0, 0)` | `(+0.07625, 0, 0)` |
| 左臂 J7 | `(+0.601, 0, 0)` | `(+0.437, 0, 0)` |
| 右臂 J2 | `(0, 0, 0)` | `(−0.07625, 0, 0)` |
| 右臂 J6 | `(−0.601, 0, 0)` | `(−0.437, 0, +0.0295)` |
| 右臂 J7 | `(−0.601, 0, 0)` | `(−0.437, 0, 0)` |

这些差异说明两种机械臂的**关节布局尺寸**不同。例如，`skye_m6` 的 J1、J2 中心重合，leader 的两者相隔约 0.07625 m；右侧 leader 的 J6 中心在零位还比其 J7 中心高约 0.0295 m。关节中心间距不同本身**不能推出关节零点角度不同**。若要比较网格外观意义上的完整零姿态，还需进一步对齐并检查两套不同的 STL 模型。

## 适用范围

以上结论仅针对三份 URDF 的关节运动学定义。它不验证实物编码器零点、电机转向，也不以 STL 网格外观作为零位一致性的判据。

## 对主从臂关节空间遥操作的意义

**判断：上述零位构型、关节顺序和正轴关系，为逐关节映射提供了运动学依据，但不足以单独证明实际遥操作方向已正确。** 左臂 J4、J6、J7 的约 1.2° 轴偏差，以及两臂关节中心布局的差异，也意味着相同的关节角变化不会产生完全相同的末端空间运动。

当前遥操作链路还包含两步坐标换算：

1. leader 的原始电机位置先由 `joint_offsets` 和 `joint_signs` 转成逻辑关节角；这里的 `joint_signs` 处理的是 leader 电机读数方向，不是主从臂之间的映射符号。参见 [control_core.py](../skye_factr_teleop/control_core.py)。
2. follower 驱动默认使用 `relative` 模式：进入遥操作时记录两侧参考角，此后按照关节顺序及 follower 映射符号，将 leader 的角度增量加到 follower 的参考角。对第 `j` 个 follower 关节，其形式为 `q_f,cmd[j] = q_f,ref[j] + s[j] · (q_l[order[j]] − q_l,ref[order[j]])`，而非直接令 `q_f = q_l`。参见 [driver_core.cpp](../../skye_robot_driver/src/driver_core.cpp) 和 [skye_robot.yaml](../../skye_robot_driver/config/skye_robot.yaml)。因此，URDF 中共同的 `q = 0` 并不意味着运行时两臂会自动处于共同零位。

需要特别核实右臂腕部：当前 [Thor](../../skye_robot_driver/config/profiles/thor.yaml) 和 [Orin](../../skye_robot_driver/config/profiles/orin.yaml) 的 follower 映射均将右臂 J6、J7 的符号设为 `−1`。这一运行配置与前文 URDF 正轴比较属于不同层次；该反号可能补偿实际反馈或驱动方向，仅凭 URDF 无法判断其是否正确。

因此，**可以将现有 URDF 关系作为关节空间相对映射的候选依据**；是否满足“主臂某关节正向运动时，从臂对应关节按预期方向运动”，还需逐关节核对 leader 逻辑角增量、follower 指令增量和实物运动方向，尤其是右臂 J6、J7。
