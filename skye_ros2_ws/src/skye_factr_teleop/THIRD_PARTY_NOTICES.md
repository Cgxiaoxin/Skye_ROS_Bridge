# Third-party notices

## FACTR_Teleop

- Project: `FACTR_Teleop`
- Repository: <https://github.com/JasonJZLiu/FACTR_Teleop.git>
- Reference commit: `7a07ab3629af03a91c0198df7c44a0082dca77ab`
- License: Apache-2.0
- Copyright: Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li

The package layout and metadata were informed by FACTR_Teleop. The stage-3
`skye_factr_teleop/control_core.py` control laws and quarter-turn calibration
were adapted from upstream
`src/factr_teleop/factr_teleop/factr_teleop.py` and rewritten as ROS-free,
validated, single-cycle functions. Other runtime-module provenance is tracked
with each implementation task.

The reference commit does not contain a `NOTICE` file.

The upstream `factr_teleop/dynamixel/driver.py`, which is planned as a source
baseline for a later task, states that it is based on the GELLO driver at:
<https://github.com/wuphilipp/gello_software/blob/main/gello/dynamixel/driver.py>.
No GELLO source is included by task 1.1.

Host-installed Python and ROS dependencies are external dependencies, not
copied source, and are therefore not represented as incorporated third-party
code in this notice.
