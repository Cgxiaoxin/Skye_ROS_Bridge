# Robot Profile + Branch Merge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Merge `robotiq_teleop` into `main` and switch Thor/Orin via `robot_profile:=thor|orin`, keeping Thor-safe defaults.

**Architecture:** Bring Robotiq/DM4310 dual backends from `robotiq_teleop` into `main`; keep `skye_robot.yaml` Thor-safe; add `config/profiles/{thor,orin}.yaml` stacked by launch; mirror leader calibrations under `marvin_ws/configs/{thor,orin}/`.

**Tech Stack:** ROS 2 Humble, `skye_robot_driver` (C++), FACTR Docker overlay launch, bash start scripts.

**Spec:** `docs/superpowers/specs/2026-09-03-robot-profile-merge-design.md`

**Rollback:** `git checkout d591505` (pre-merge Thor tip on `main`).

## Global Constraints

- Work **directly on `main`** (user request); commit frequently after each task.
- Default profile is **`thor`**. Omitting `robot_profile` must behave like pre-merge Thor.
- Never leave Orin gripper type or Orin `right_joint_signs` as the only values in `skye_robot.yaml`.
- Preserve FACTR sync workaround: publish `/gento/{left,right}_joint_states` and remap factr nodes.
- Do not force-push; do not amend published commits.
- `ROS_DOMAIN_ID` default remains `21` for P4.

## File Map

| Path | Role |
|------|------|
| `skye_ros2_ws/.../config/skye_robot.yaml` | Neutral/Thor-safe base params |
| `skye_ros2_ws/.../config/profiles/thor.yaml` | Explicit Thor overlay |
| `skye_ros2_ws/.../config/profiles/orin.yaml` | Orin overlay (robotiq + signs) |
| `skye_ros2_ws/.../launch/skye_robot_driver.launch.py` | `robot_profile` + param stack |
| `skye_ros2_ws/.../src/{dm4310,robotiq,gripper_*}.*` | Dual gripper backends (from robotiq) |
| `marvin_ws/configs/{thor,orin}/grav_comp_m6_*.yaml` | Per-machine leader calibrations |
| `marvin_ws/launch_overlay/start_teleop_m6_dual_gento*.launch.py` | Prefer profile grav_comp path |
| `scripts/sync_marvin_overlay.sh` | Sync launch + profile configs into install |
| `scripts/start_skye_for_factr.sh` | Pass `robot_profile` |
| `scripts/run_marvin_m6_impedance.sh` | Call sync with `ROBOT_PROFILE` |
| `docs/小臂大臂启动步骤.md` | Document Thor/Orin start commands |

---

### Task 1: Merge `robotiq_teleop` into `main` and resolve conflicts

**Files:**
- Merge: all paths from `robotiq_teleop` (see `git diff --name-status main...robotiq_teleop`)
- Expect conflicts in: `docs/dev_plan.md`, `docs/ros_interfaces.md`, `docs/小臂大臂启动步骤.md`, `marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py`, `marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py`, `skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_node.hpp`, `skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py`, `skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp`

**Interfaces:**
- Produces: working tree with robotiq gripper sources present; sync 7-DOF publishers still present; may temporarily have Orin defaults in `skye_robot.yaml` (fixed in Task 2).

- [ ] **Step 1: Confirm clean enough working tree**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge
git status -sb
git branch --show-current   # must be main
git rev-parse HEAD          # expect d591505 or later tip user committed
```

Expected: on `main`. Unrelated dirty files: stash or leave untouched if unrelated to merge.

- [ ] **Step 2: Merge robotiq branch**

```bash
git merge robotiq_teleop -m "$(cat <<'EOF'
merge: bring robotiq_teleop gripper backends into main

Integrate Orin/Robotiq dual-gripper support while preparing robot_profile
overlays. Thor-safe defaults are restored in a follow-up commit.

EOF
)"
```

Expected: conflict markers in the 8 files listed above (or subset).

- [ ] **Step 3: Resolve conflict principles (apply while editing)**

For each conflicted file:

1. **`driver_node.cpp` / `driver_node.hpp` / `skye_robot_driver.launch.py`**
   - Keep **robotiq** gripper loading (`gripper_left_type`, backends, per-arm invert).
   - Keep **both** sides’ 7-DOF state publishers + remaps:
     - publishers: `/left_joint_states`, `/right_joint_states`
     - remaps to `/gento/left_joint_states`, `/gento/right_joint_states`
   - If launch conflict: prefer robotiq `OpaqueFunction` structure; ensure 7-DOF remaps remain.

2. **`start_teleop_m6_dual_gento*.launch.py`**
   - Keep `_grav_comp_config()` from robotiq (prefer overlay configs).
   - Keep 7-DOF `/joint_state` remaps to `/gento/{left,right}_joint_states`.

3. **Docs**
   - Keep sync workaround wording from either side (same intent).
   - Prefer robotiq gripper docs where they describe dual backends; Thor DM4310 remains valid.

4. **Do not “fix” yaml defaults yet** in this task beyond making the merge compile; Task 2 owns Thor-safe defaults.

Quick check after resolving:

```bash
rg -n 'left_state_publisher_|right_state_publisher_|/left_joint_states|/right_joint_states' \
  skye_ros2_ws/src/skye_robot_driver/src/driver_node.cpp \
  skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py \
  marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py
rg -n 'gripper_left_type|GripperArmBackend|robotiq_gripper_arm' \
  skye_ros2_ws/src/skye_robot_driver/
```

Expected: both sync remaps and robotiq type strings present.

- [ ] **Step 4: Finish merge commit**

```bash
git add -A
git status
# if merge still in progress:
git commit --no-edit
# if merge commit already created with conflicts fixed via amend-not-allowed:
# use a new commit only if merge already completed; otherwise --no-edit is fine for concluding merge
```

If `git commit --no-edit` fails because merge already committed, skip.

- [ ] **Step 5: Build driver**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
./scripts/build.sh
```

Expected: `skye_robot_driver` builds. If missing sources in CMakeLists, add:

```cmake
  src/gripper_common.cpp
  src/dm4310_gripper_arm.cpp
  src/robotiq_gripper_arm.cpp
```

to `skye_driver_core` (robotiq branch already has this).

- [ ] **Step 6: Run unit tests**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select skye_robot_driver --event-handlers console_direct+
colcon test-result --verbose
```

Expected: `test_driver_core` passes.

- [ ] **Step 7: Commit any build/CMake fixes if needed**

```bash
git add skye_ros2_ws/src/skye_robot_driver/CMakeLists.txt
git commit -m "$(cat <<'EOF'
fix: ensure robotiq gripper sources build after merge

EOF
)"
```

Only if there were post-merge build fixes.

---

### Task 2: Thor-safe base yaml + `profiles/{thor,orin}.yaml`

**Files:**
- Modify: `skye_ros2_ws/src/skye_robot_driver/config/skye_robot.yaml`
- Create: `skye_ros2_ws/src/skye_robot_driver/config/profiles/thor.yaml`
- Create: `skye_ros2_ws/src/skye_robot_driver/config/profiles/orin.yaml`
- Modify: `skye_ros2_ws/src/skye_robot_driver/CMakeLists.txt` (install profiles dir)
- Keep for compat (optional): `config/skye_robot_robotiq_dual.yaml` — make it a thin include-equivalent of orin gripper keys, or leave as-is and document deprecation in Task 5.

**Interfaces:**
- Consumes: merged param names `gripper_left_type`, `gripper_right_type`, `left_joint_signs`, `right_joint_signs`, robotiq `*_mm` keys.
- Produces: stacked params where later file wins; `orin.yaml` must set all Orin-differing keys.

- [ ] **Step 1: Rewrite `skye_robot.yaml` to Thor-safe base**

Ensure these values exist (keep other existing limits/impedance unchanged):

```yaml
    # signs: 仅指令映射（relative）。不改 joint_states。默认 Thor 全 +1。
    left_joint_signs: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    right_joint_signs: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

    # Per-arm driver: dm4310 | robotiq. Default Thor = dm4310.
    gripper_left_type: "dm4310"
    gripper_right_type: "dm4310"

    enable_gripper: true
    gripper_left_motor_id: 1
    gripper_right_motor_id: 2
    gripper_right_terminal: 1
    gripper_invert: true
    gripper_left_invert: true
    gripper_right_invert: true
    gripper_kp: 3.0
    gripper_kd: 0.12
    gripper_rate_hz: 100.0
    gripper_feedback_timeout_ms: 1
    gripper_pos_min: 0.0
    gripper_pos_max: 1.6
    gripper_close_limit: 0.93

    # Robotiq keys present so declare_parameter defaults are documented;
    # active only when gripper_*_type=robotiq (overridden by profiles/orin.yaml).
    gripper_left_robotiq_slave_id: 9
    gripper_right_robotiq_slave_id: 9
    gripper_left_robotiq_485_channel: "485A"
    gripper_right_robotiq_485_channel: "485A"
    gripper_left_robotiq_terminal: 0
    gripper_right_robotiq_terminal: 1
    gripper_robotiq_speed: 136
    gripper_robotiq_force: 16
    gripper_robotiq_pos_min_mm: 0.0
    gripper_left_robotiq_pos_min_mm: 2.0
    gripper_right_robotiq_pos_min_mm: 13.0
    gripper_robotiq_pos_max_mm: 50.0
    gripper_left_robotiq_pos_max_mm: 50.0
    gripper_right_robotiq_pos_max_mm: 50.0
    gripper_robotiq_modbus_timeout_ms: 200
```

- [ ] **Step 2: Create `profiles/thor.yaml`**

```yaml
# Thor machine overlay (explicit; mostly matches skye_robot.yaml defaults)
skye_robot_driver:
  ros__parameters:
    left_joint_signs: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    right_joint_signs: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    gripper_left_type: "dm4310"
    gripper_right_type: "dm4310"
    gripper_invert: true
    gripper_left_invert: true
    gripper_right_invert: true
    gripper_close_limit: 0.93
```

- [ ] **Step 3: Create `profiles/orin.yaml`**

```yaml
# Orin machine overlay (Robotiq Hand-E + right wrist signs)
skye_robot_driver:
  ros__parameters:
    left_joint_signs: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    right_joint_signs: [1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0]
    gripper_left_type: "robotiq"
    gripper_right_type: "robotiq"
    gripper_invert: true
    gripper_left_invert: true
    gripper_right_invert: true
    gripper_close_limit: 1.0
    gripper_left_robotiq_terminal: 0
    gripper_right_robotiq_terminal: 1
    gripper_left_robotiq_485_channel: "485A"
    gripper_right_robotiq_485_channel: "485A"
    gripper_left_robotiq_slave_id: 9
    gripper_right_robotiq_slave_id: 9
    gripper_left_robotiq_pos_min_mm: 2.0
    gripper_right_robotiq_pos_min_mm: 13.0
    gripper_left_robotiq_pos_max_mm: 50.0
    gripper_right_robotiq_pos_max_mm: 50.0
```

- [ ] **Step 4: Install profiles in CMakeLists**

Find the `install(DIRECTORY config ...)` (or equivalent) and ensure `config/profiles` is included. Example if currently:

```cmake
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
```

`DIRECTORY config` already installs nested `profiles/` — verify after build:

```bash
ls install/share/skye_robot_driver/config/profiles/
# expect thor.yaml orin.yaml
```

- [ ] **Step 5: Commit**

```bash
git add \
  skye_ros2_ws/src/skye_robot_driver/config/skye_robot.yaml \
  skye_ros2_ws/src/skye_robot_driver/config/profiles/thor.yaml \
  skye_ros2_ws/src/skye_robot_driver/config/profiles/orin.yaml \
  skye_ros2_ws/src/skye_robot_driver/CMakeLists.txt
git commit -m "$(cat <<'EOF'
feat(config): add thor/orin robot_profile overlays with Thor-safe defaults

EOF
)"
```

---

### Task 3: Launch `robot_profile` stacking

**Files:**
- Modify: `skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py`
- Modify: `scripts/start_skye_for_factr.sh`

**Interfaces:**
- Consumes: `robot_profile` ∈ {`thor`,`orin`}; base `params_file`; optional legacy `robotiq_dual_gripper`.
- Produces: Node `parameters=[base, profile_yaml, connect_on_startup]`.

- [ ] **Step 1: Replace launch with OpaqueFunction that stacks profile**

Full file target shape:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_REMAPS = [
    ("/joint_states", "/gento/joint_states"),
    ("/left_joint_states", "/gento/left_joint_states"),
    ("/right_joint_states", "/gento/right_joint_states"),
    ("/left_joint_control", "/gento/left_joint_control"),
    ("/right_joint_control", "/gento/right_joint_control"),
    ("/left_joint_control_abs", "/gento/left_joint_control_abs"),
    ("/right_joint_control_abs", "/gento/right_joint_control_abs"),
    ("/robot_state", "/gento/robot_state"),
    ("/set_mode", "/gento/set_mode"),
    ("/hold_current", "/gento/hold_current"),
    ("/stop_motion", "/gento/stop_motion"),
    ("/emergency_stop", "/gento/emergency_stop"),
]


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("skye_robot_driver")
    params_file = LaunchConfiguration("params_file").perform(context)
    connect_on_startup = LaunchConfiguration("connect_on_startup").perform(context)
    robot_profile = LaunchConfiguration("robot_profile").perform(context).strip().lower()
    robotiq_dual = LaunchConfiguration("robotiq_dual_gripper").perform(context)
    robotiq_right = LaunchConfiguration("robotiq_right_gripper").perform(context)

    if robot_profile not in ("thor", "orin"):
        raise RuntimeError(
            f"robot_profile must be 'thor' or 'orin', got: {robot_profile!r}"
        )

    # Legacy flags map to orin/partial overlays if profile still thor.
    node_params = [
        params_file,
        os.path.join(pkg_share, "config", "profiles", f"{robot_profile}.yaml"),
    ]
    if robot_profile == "thor" and robotiq_dual.lower() in ("1", "true", "yes"):
        node_params.append(
            os.path.join(pkg_share, "config", "profiles", "orin.yaml")
        )
    elif robot_profile == "thor" and robotiq_right.lower() in ("1", "true", "yes"):
        node_params.append(
            os.path.join(pkg_share, "config", "skye_robot_robotiq_right.yaml")
        )

    node_params.append(
        {
            "connect_on_startup": ParameterValue(
                connect_on_startup.lower() in ("1", "true", "yes"), value_type=bool
            )
        }
    )

    return [
        Node(
            package="skye_robot_driver",
            executable="skye_robot_driver",
            name="skye_robot_driver",
            parameters=node_params,
            remappings=_REMAPS,
            output="screen",
        )
    ]


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("skye_robot_driver"),
        "config",
        "skye_robot.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Base Skye driver parameter file",
            ),
            DeclareLaunchArgument(
                "connect_on_startup",
                default_value="true",
                description="Link SDK and enter control mode on startup",
            ),
            DeclareLaunchArgument(
                "robot_profile",
                default_value="thor",
                description="Machine profile: thor (DM4310) | orin (Robotiq)",
            ),
            DeclareLaunchArgument(
                "robotiq_dual_gripper",
                default_value="false",
                description="Legacy: if true with robot_profile:=thor, also load orin.yaml",
            ),
            DeclareLaunchArgument(
                "robotiq_right_gripper",
                default_value="false",
                description="Legacy: right-only Robotiq overlay (prefer robot_profile:=orin)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
```

- [ ] **Step 2: Wire `start_skye_for_factr.sh`**

Near the top after `REPO_ROOT`:

```bash
ROBOT_PROFILE="${ROBOT_PROFILE:-thor}"
case "${ROBOT_PROFILE}" in
  thor|orin) ;;
  *)
    echo "ERROR: ROBOT_PROFILE must be thor|orin (got: ${ROBOT_PROFILE})" >&2
    exit 1
    ;;
esac
```

Change the final `exec ros2 launch` to:

```bash
echo "== skye_robot_driver profile=${ROBOT_PROFILE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} =="
exec ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  connect_on_startup:="${CONNECT_ON_STARTUP:-true}" \
  robot_profile:="${ROBOT_PROFILE}"
```

- [ ] **Step 3: Rebuild so install launch updates**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws && ./scripts/build.sh
```

- [ ] **Step 4: Dry-check launch resolution (no hardware)**

```bash
source /opt/ros/humble/setup.bash
source /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  robot_profile:=thor connect_on_startup:=false --show-args
ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  robot_profile:=orin connect_on_startup:=false --show-args
```

Expected: both list `robot_profile`. Invalid profile should error when actually launching OpaqueFunction (optional quick test with `robot_profile:=nope` and expect RuntimeError).

- [ ] **Step 5: Commit**

```bash
git add \
  skye_ros2_ws/src/skye_robot_driver/launch/skye_robot_driver.launch.py \
  scripts/start_skye_for_factr.sh
git commit -m "$(cat <<'EOF'
feat(launch): select thor/orin via robot_profile launch arg

EOF
)"
```

---

### Task 4: Leader grav_comp profiles + sync script

**Files:**
- Create: `marvin_ws/configs/thor/grav_comp_m6_left.yaml`
- Create: `marvin_ws/configs/thor/grav_comp_m6_right.yaml`
- Create: `marvin_ws/configs/orin/grav_comp_m6_left.yaml`
- Create: `marvin_ws/configs/orin/grav_comp_m6_right.yaml`
- Modify: `scripts/sync_marvin_overlay.sh`
- Modify: `marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py`
- Modify: `marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py`
- Modify: `scripts/run_marvin_m6_impedance.sh`
- Modify: `scripts/bind_leader_arms.py` (patch ports under active profile dir)

**Interfaces:**
- Consumes: `ROBOT_PROFILE` / `MARVIN_PROFILE` env (`thor`|`orin`).
- Produces: overlay yaml path `marvin_ws/configs/<profile>/grav_comp_m6_{left,right}.yaml` preferred by `_grav_comp_config`.

- [ ] **Step 1: Seed Orin configs from robotiq + install**

```bash
mkdir -p marvin_ws/configs/thor marvin_ws/configs/orin

# Orin right: tracked on robotiq_teleop
git show robotiq_teleop:marvin_ws/configs/grav_comp_m6_right.yaml \
  > marvin_ws/configs/orin/grav_comp_m6_right.yaml

# Orin left: use current install left (J6/J7=-1, J8=+1) if present
cp -f marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_left.yaml \
  marvin_ws/configs/orin/grav_comp_m6_left.yaml
```

Verify Orin right contains:

```text
joint_signs: [1, -1, 1, -1, 1, -1, -1, -1]
```

- [ ] **Step 2: Seed Thor configs from current Thor working install**

On this Thor machine, copy install calibrations into `configs/thor/` (ports will be rebound by `bind_leader_arms.py`):

```bash
cp -f marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_left.yaml \
  marvin_ws/configs/thor/grav_comp_m6_left.yaml
cp -f marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_right.yaml \
  marvin_ws/configs/thor/grav_comp_m6_right.yaml
```

**Important:** If Thor right currently has Orin-style signs because install was polluted, reset Thor **driver** signs via profile (already Task 2). For **leader** yaml, after copy, open both Thor files and confirm they match **Thor 小臂** physical calibration. If unknown, keep copy and add a header comment:

```yaml
# profile: thor — verify joint_signs on Thor hardware before relying on teleop
```

If Thor right leader should differ from Orin’s `[1,-1,1,-1,1,-1,-1,-1]`, fix signs on Thor during hardware validation (Task 6); do not guess in code without measurement.

- [ ] **Step 3: Remove flat `marvin_ws/configs/grav_comp_m6_right.yaml` if merge brought it**

```bash
# After profile dirs exist, avoid ambiguous flat file:
git rm -f marvin_ws/configs/grav_comp_m6_right.yaml 2>/dev/null || rm -f marvin_ws/configs/grav_comp_m6_right.yaml
```

- [ ] **Step 4: Update `_grav_comp_config` in both launch overlays**

Replace helper with profile-aware version:

```python
def _grav_comp_config(name: str) -> str:
    """Prefer marvin_ws/configs/<ROBOT_PROFILE>/, then configs/, then install."""
    marvin = os.environ.get("MARVIN_WS", "/marvin_ws")
    profile = os.environ.get("ROBOT_PROFILE", os.environ.get("MARVIN_PROFILE", "thor")).strip().lower()
    if profile not in ("thor", "orin"):
        profile = "thor"
    candidates = [
        os.path.join(marvin, "configs", profile, name),
        os.path.join(marvin, "configs", name),
        os.path.join(marvin, "install", "share", "factr_teleop", "configs", name),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    pkg_share = get_package_share_directory("factr_teleop")
    return os.path.join(pkg_share, "configs", name)
```

Apply identically in:
- `marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py`
- `marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py`

- [ ] **Step 5: Update `sync_marvin_overlay.sh` to sync active profile configs into install**

Replace the config copy block with:

```bash
PROFILE="${ROBOT_PROFILE:-${MARVIN_PROFILE:-thor}}"
case "${PROFILE}" in
  thor|orin) ;;
  *)
    echo "ERROR: ROBOT_PROFILE must be thor|orin (got: ${PROFILE})" >&2
    exit 1
    ;;
esac

PROFILE_CFG="${MARVIN_WS}/configs/${PROFILE}"
if [[ ! -d "${PROFILE_CFG}" ]]; then
  echo "ERROR: missing profile configs: ${PROFILE_CFG}" >&2
  exit 1
fi

mkdir -p "${INSTALL_CFG}"
shopt -s nullglob
cfg_files=("${PROFILE_CFG}"/grav_comp_m6_*.yaml)
if ((${#cfg_files[@]} == 0)); then
  echo "ERROR: no grav_comp_m6_*.yaml under ${PROFILE_CFG}" >&2
  exit 1
fi
for src in "${cfg_files[@]}"; do
  cp -f "${src}" "${INSTALL_CFG}/"
  echo "  config[${PROFILE}]: $(basename "${src}")"
done
echo "OK: profile=${PROFILE} overlay synced"
```

Keep existing launch file copy logic unchanged.

- [ ] **Step 6: `run_marvin_m6_impedance.sh` exports profile before sync**

Before calling sync:

```bash
export ROBOT_PROFILE="${ROBOT_PROFILE:-thor}"
bash "${SCRIPT_DIR}/sync_marvin_overlay.sh"
```

Ensure Docker run passes the env:

```bash
  -e "ROBOT_PROFILE=${ROBOT_PROFILE}" \
```

(add next to existing `-e ROS_DOMAIN_ID=...` style flags).

- [ ] **Step 7: Point `bind_leader_arms.py` at profile configs**

In `apply()`, resolve profile:

```python
profile = os.environ.get("ROBOT_PROFILE", os.environ.get("MARVIN_PROFILE", "thor")).strip().lower()
if profile not in ("thor", "orin"):
    profile = "thor"
left_cfg = marvin / "configs" / profile / "grav_comp_m6_left.yaml"
right_cfg = marvin / "configs" / profile / "grav_comp_m6_right.yaml"
# Still also patch install copies if present (sync may overwrite from profile):
install_left = marvin / "install/share/factr_teleop/configs/grav_comp_m6_left.yaml"
install_right = marvin / "install/share/factr_teleop/configs/grav_comp_m6_right.yaml"
```

Patch **profile** files first; if install files exist, patch them too so immediate docker use works before re-sync.

- [ ] **Step 8: Dry-run sync**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge
ROBOT_PROFILE=thor ./scripts/sync_marvin_overlay.sh
ROBOT_PROFILE=orin ./scripts/sync_marvin_overlay.sh
# restore thor for this machine
ROBOT_PROFILE=thor ./scripts/sync_marvin_overlay.sh
```

Expected: logs show `config[thor]` / `config[orin]` and files land under `marvin_ws/install/share/factr_teleop/configs/`.

- [ ] **Step 9: Commit**

```bash
git add \
  marvin_ws/configs/thor \
  marvin_ws/configs/orin \
  scripts/sync_marvin_overlay.sh \
  scripts/run_marvin_m6_impedance.sh \
  scripts/bind_leader_arms.py \
  marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py \
  marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py
# include deletion of flat grav_comp if removed
git add -u marvin_ws/configs/grav_comp_m6_right.yaml 2>/dev/null || true
git commit -m "$(cat <<'EOF'
feat(marvin): per-profile grav_comp configs for thor/orin

EOF
)"
```

---

### Task 5: Docs update

**Files:**
- Modify: `docs/小臂大臂启动步骤.md`
- Modify: `docs/ros_interfaces.md` (mention `robot_profile`)
- Modify: `docs/dev_plan.md` (one line: dual machine via profile)

- [ ] **Step 1: Add a short “双机 profile” section near 日常启动**

Insert:

```markdown
## 双机切换（`robot_profile`）

| 机器 | 夹爪 | 大臂 signs | 启动 |
|------|------|------------|------|
| Thor（默认） | DM4310 | 全 +1 | `ROBOT_PROFILE=thor` 或不设 |
| Orin | Robotiq Hand-E | 右 J6/J7 = -1 | `ROBOT_PROFILE=orin` |

主机:

```bash
ROBOT_PROFILE=thor ./scripts/start_skye_for_factr.sh
# 或
ROBOT_PROFILE=orin ./scripts/start_skye_for_factr.sh
```

小臂 Docker（同一 profile）:

```bash
ROBOT_PROFILE=thor ./scripts/run_marvin_m6_impedance.sh
# 或
ROBOT_PROFILE=orin ./scripts/run_marvin_m6_impedance.sh
```

等价 launch:

```bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py robot_profile:=orin
```

旧参数 `robotiq_dual_gripper:=true` 仍可用，但请优先 `robot_profile:=orin`。
```

- [ ] **Step 2: Commit docs**

```bash
git add docs/小臂大臂启动步骤.md docs/ros_interfaces.md docs/dev_plan.md \
  docs/superpowers/specs/2026-09-03-robot-profile-merge-design.md \
  docs/superpowers/plans/2026-09-03-robot-profile-merge.md
git commit -m "$(cat <<'EOF'
docs: document robot_profile thor/orin startup

EOF
)"
```

---

### Task 6: Hardware validation checklist (manual)

**Files:** none (ops only)

**Thor (this machine, default):**

- [ ] **Step 1:** `ROBOT_PROFILE=thor ./scripts/start_skye_for_factr.sh`
- [ ] **Step 2:** Confirm log / params: `gripper_left_type=dm4310`, signs all `+1`
- [ ] **Step 3:** `ROBOT_PROFILE=thor ./scripts/run_marvin_m6_impedance.sh` → `1` sync → `2` teleop
- [ ] **Step 4:** Wrist/J4 same direction; grippers open on release
- [ ] **Step 5:** `ros2 topic echo /gento/right_joint_states --once` is 7-DoF matching right half of `/gento/joint_states`

**Orin (other machine):**

- [ ] **Step 6:** Pull/sync same `main`, then `ROBOT_PROFILE=orin` for **both** host driver and Docker
- [ ] **Step 7:** Confirm robotiq start + force-open; right J6/J7 follow leader
- [ ] **Step 8:** Right sync targets **right** big arm (not left)

**Rollback drill (optional):**

- [ ] **Step 9:** Know that `git checkout d591505` restores pre-merge tree if needed

---

## Spec coverage self-check

| Spec requirement | Task |
|------------------|------|
| Merge robotiq into main | Task 1 |
| Thor-safe defaults | Task 2 |
| `robot_profile:=thor\|orin` | Task 3 |
| Leader calibrations per machine | Task 4 |
| Docs / startup commands | Task 5 |
| Dual-machine acceptance | Task 6 |
| Keep sync 7-DoF workaround | Task 1 + Task 6 |
| Rollback via `d591505` | Header + Task 6 |

## Placeholder scan

No TBD/TODO placeholders in task steps; hardware sign verification for Thor leader yaml is explicitly deferred to measured validation in Task 6, not left as “fill later” code.
