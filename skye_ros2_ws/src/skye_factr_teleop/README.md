# skye_factr_teleop

[简体中文版](README.zh-CN.md). Keep this README and its Chinese translation in sync when changing behavior, commands, or parameters.

ROS 2 Humble `ament_python` package for Skye/M6 FACTR-style leader-arm
teleoperation.

The package includes validated YAML configuration, the production Dynamixel
adapter, a ROS-free single-cycle control core, the single-arm ROS driver, and
left/right/dual-arm launch files. The low-current hardware smoke check remains
pending in development-plan stage 7.

## Configuration

The uncalibrated Thor M6 control templates are installed as
`config/grav_comp_m6_left.yaml` and `config/grav_comp_m6_right.yaml`. Load and
validate either file with `skye_factr_teleop.config.load_config`. Returned nested
dataclasses and numeric tuples are immutable.

Thor uses leader device A; Orin uses leader device B, with its configuration
installed under `config/orin/`. Their respective servo ports and physical
directions are selected in these YAML files. The existing profile-specific gains
and arm-limit barrier settings remain unchanged. Neither newly assigned pairing
has undergone new-package hardware acceptance. These are leader hardware
profiles, not follower joint-sign mappings.

Set `controller.gravity_comp.mode` in the left/right YAML to choose compensation:

| Mode | Model calculation |
|---|---|
| `position_velocity` (default, including when omitted) | `RNEA(q, dq, 0)`, matching FACTR |
| `position_only` | `RNEA(q, 0, 0)`, gravity only |

Reload the configuration and recreate the control core after changing the mode.
Only the velocity passed to inverse dynamics is zeroed in `position_only` mode;
limit damping, friction speed gating, and nullspace damping still use measured
velocity. Static friction amplitude uses the gain-scaled compensation from the
selected mode, including when gravity output is disabled via `ControlTerms`.

The configured leader and follower URDF resources are provided by the
`skye_factr_description` package; no model is loaded from `marvin_ws`.

## Entry point

```bash
ros2 run skye_factr_teleop skye_factr_teleop_driver --help
```

The keyboard entry point is intentionally deferred until basic hardware
acceptance is complete.

## Task 5.1 ROS driver

Select the arm with executable argument `--side left|right` (default `left`).
Default node names are `skye_factr_teleop_left` and `skye_factr_teleop_right`;
standard ROS node-name remapping is supported. Supply one YAML file with
the calibrated offsets and motion limits for that arm. The only ROS parameter
is the read-only `config_path`; the selected YAML holds the complete per-arm
configuration:

| Parameter | Meaning/default |
|---|---|
| `config_path` | Absolute YAML path, or path relative to the package share; empty selects `config/grav_comp_m6_<side>.yaml` |
| `runtime.joint_offsets` | Required: eight calibrated physical-servo offsets in rad, including gripper |
| `runtime.current_limit_ma` | Required: positive current ceiling; also bounded by servo ratings |
| `runtime.max_velocity`, `runtime.max_acceleration`, `runtime.max_jerk` | Required: eight positive limits each, in logical leader order, rad/s, rad/s², rad/s³ |
| `runtime.state_timeout` | Maximum header age and time since receipt, seconds; default `0.25` |
| `runtime.future_tolerance` | Allowed future header timestamp skew, seconds; default `0.05` |
| `runtime.allow_unnamed_joint_states` | Default `false`; if true, empty names mean fixed J1..J7 order |

Copy the matching Thor or Orin YAML to a per-device file and add a top-level
`runtime` mapping with measured values. The packaged YAML files intentionally
omit this mapping and cannot start teleoperation. For example, the required
shape is:

```yaml
runtime:
  joint_offsets: [<8 measured radians>]
  current_limit_ma: <measured safe ceiling in mA>
  max_velocity: [<8 positive rad/s limits>]
  max_acceleration: [<8 positive rad/s² limits>]
  max_jerk: [<8 positive rad/s³ limits>]
```

Replace each bracketed placeholder with actual YAML numbers before launch.
Missing or invalid values fail startup before a serial device is opened. The node resolves its
leader URDF through `skye_factr_description` and uses the Pinocchio adapter in
`dynamics.py` for RNEA, the local joint Jacobian, and forward kinematics.
Runtime Python dependencies include the baseline Pinocchio and Ruckig versions.

After the stage-7 hardware gates, a left-arm invocation with an operator-supplied
unified YAML has this form:

```bash
ros2 run skye_factr_teleop skye_factr_teleop_driver --side left --ros-args \
  -p config_path:=/path/to/left_calibrated.yaml \
  -r /joint_state:=/gento/left_joint_states \
  -r /joint_control:=/gento/left_joint_control \
  -r /joint_move:=/left_joint_move \
  -r /leader_arm/current_state:=/left_leader_arm/current_state \
  -r /leader_arm/target_joint_state:=/left_leader_arm/target_joint_state
```

Use `--side right`, the right calibrated YAML, and right-side remaps for the
other instance. Mode topics are shared globally. The node creates these base
interfaces, with the QoS frozen in the phase-0 contract:

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/joint_state`, `/joint_move` | `sensor_msgs/msg/JointState` |
| Subscribe | `/mode/switch_sync`, `/mode/switch_teleop`, `/mode/switch_stop` | `std_msgs/msg/String` |
| Publish | `/leader_arm/current_state`, `/leader_arm/target_joint_state` | `sensor_msgs/msg/JointState` |
| Publish | `/joint_control` | `sensor_msgs/msg/JointState` |
| Publish | `/teleop/state` | `std_msgs/msg/String` |

State data uses RELIABLE/VOLATILE/KEEP_LAST(1); joint commands use
BEST_EFFORT/VOLATILE/KEEP_LAST(1). Mode requests use RELIABLE/VOLATILE/depth 10;
state text uses RELIABLE/TRANSIENT_LOCAL/depth 1.

Incoming JointState messages require seven finite positions; velocity/effort
must be empty or contain seven finite values. Names must be exactly
`l_j1..l_j7` or `r_j1..r_j7` in order, unless empty names are explicitly allowed.
Messages require nonzero, fresh timestamps in the node's ROS clock domain.
The node never slices a combined 14-axis message. It also checks monotonic time
since receipt, so a paused ROS clock cannot keep old feedback valid indefinitely.

Startup opens/configures the bus with torque disabled, using
`DynamixelDriver.open(enable_torque=False)`. A valid `switch_sync` request in
`IDLE` requires fresh follower data before enabling torque. Ruckig completion
enters `HOLD_SETTLING`; measured position and velocity must remain within their
configured tolerances for `settle_duration` before `SYNCED` permits
`switch_teleop` and seven-axis command publication.
Invalid or stale follower feedback during follower sync, hold, or teleop stops
motion and returns to `IDLE`.

`/joint_move` is accepted only in `IDLE`: it independently moves the leader to
the supplied seven-axis target and holds it, using the configured gripper
setpoint. After settling it reports `SYNCED`, but the node
does **not** grant follower-teleop permission. Stop, then perform follower sync
before requesting teleop. Repeated motion requests cannot replace an active
trajectory. Target-state publication describes the running reference/hold
target; it is not an input topic.

Callbacks are serialized in the default mutually-exclusive callback group.
The timer uses a steady clock at the configured frequency; core timestamps use
monotonic seconds. Stop disables torque and halts command publication. Control
faults cancel the timer, attempt zero current/Torque Disable, and close the bus;
a faulted node must be restarted. SIGINT and node destruction use the same
idempotent cleanup, attempting close even if zeroing fails.

The task-5.1 tests create real ROS nodes/executor with injected hardware and
trajectories. They use a separate localhost ROS domain and do not open devices:

```bash
python3 -m pytest -q test/test_ros_driver.py test/test_dynamics.py test/test_dynamixel_driver.py test/test_control_core.py test/test_state_machine.py test/test_sync_hold.py test/test_state_trajectory_integration.py
```

Result: 211 passed. The node tests require ROS Humble; model tests require
Pinocchio. Cross-node message/remap acceptance remains task 5.3.

## Task 5.2 basic launch

The three entry points launch only the requested leader nodes, using the same
`skye_factr_teleop_driver` executable. They do not start a keyboard node or the
follower driver. `robot_profile` defaults to `thor` and accepts only `thor|orin`.
Control YAML and relative file paths are resolved from this package's share
directory. No environment or Python module search paths are changed.

| Argument | Meaning |
|---|---|
| `robot_profile` | `thor`: `config/grav_comp_m6_<side>.yaml`; `orin`: `config/orin/grav_comp_m6_<side>.yaml` |
| `left_config`, `right_config` | Per-arm unified YAML path; empty selects the profile template, which lacks calibration and fails teleop startup |

Each config path accepts an absolute path or a path relative to the package
share. A single-arm launch declares only that arm's config. In a dual-arm
launch, both files and their required runtime values are validated before
starting either node. Launch does not supply invented calibration offsets or
motion/current limits.

After hardware acceptance, use operator-supplied calibrated YAML files:

```bash
ros2 launch skye_factr_teleop start_teleop_m6_left.launch.py \
  robot_profile:=thor left_config:=/path/to/left_calibrated.yaml

ros2 launch skye_factr_teleop start_teleop_m6_right.launch.py \
  robot_profile:=orin right_config:=/path/to/right_calibrated.yaml

ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py \
  robot_profile:=thor \
  left_config:=/path/to/left_calibrated.yaml \
  right_config:=/path/to/right_calibrated.yaml
```

Each side is remapped independently:

| Node interface | Left destination | Right destination |
|---|---|---|
| `/joint_state` | `/gento/left_joint_states` | `/gento/right_joint_states` |
| `/joint_control` | `/gento/left_joint_control` | `/gento/right_joint_control` |
| `/joint_move` | `/left_joint_move` | `/right_joint_move` |
| `/leader_arm/current_state` | `/left_leader_arm/current_state` | `/right_leader_arm/current_state` |
| `/leader_arm/target_joint_state` | `/left_leader_arm/target_joint_state` | `/right_leader_arm/target_joint_state` |

Mode requests and `/teleop/state` retain their shared global names. During basic
acceptance, request modes through ROS CLI; in a dual launch both arms receive
these requests and apply their own state/freshness checks:

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

To inspect arguments without starting hardware nodes:

```bash
ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py --show-args
```

`test/test_launch.py` expands the actual launch actions and inspects the resulting
commands/parameters/remaps with process execution intercepted. It covers all
three entry points and both profiles without opening serial devices:

```bash
python3 -m pytest -q test/test_launch.py test/test_config.py
```

## Phase 2.5.1 read-only bus check

`skye_factr_dynamixel_readonly_check` calls the production
`DynamixelDriver` without starting ROS control nodes. Its read-only lifecycle
only opens the serial port, sets the host baud rate, pings IDs 1--8, reads
Return Delay Time and Torque Enable, and reads present position/velocity. It
never writes an actuator register. It refuses to continue unless Return Delay
Time is 0, torque is already disabled, the baud rate is 4 Mbps, and the FTDI
latency timer is 1.

From the `skye_ros2_ws` directory after building and sourcing the workspace:

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --frames 500 --interval 0.02 \
  --output /tmp/skye_phase2_5_left.json
```

Use the corresponding right-side YAML and output path for the right bus. An
automated `PASS` covers serial configuration, device identity, torque-off state,
and sampled numeric ranges. USB side mapping, physical ID order, joint direction,
and continuous-angle motion remain operator checks. Follow the
[phase 2.5.1 read-only checklist](dev_docs/skye_factr_teleop_phase2_5_readonly_checklist.md)
before using the result as phase 2.5.1 acceptance evidence.

For first-time ID and direction acceptance, use guided mode. It keeps the bus
read-only, prompts for one positive movement per device, captures the stationary,
movement, stop-grace, and stable-endpoint windows itself, and writes one combined
JSON record per side:

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --guided --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --baseline-frames 50 --movement-frames 200 \
  --settle-frames 50 --endpoint-frames 100 \
  --interval 0.02 --max-abs-velocity 20.0 --max-position-step 0.2 \
  --max-hold-velocity 0.05 --max-hold-position-span 0.01 \
  --output validation_records/phase2_5_1/left_guided.json
```

Repeat with the right-side YAML. The command succeeds only when the bus check and
all eight guided ID/direction checks pass.

When physical target identification is handled separately, add
`--direction-only`. In that mode, non-target movement and non-target endpoint
instability are recorded as warnings instead of failures. Target endpoint
instability is also a warning; target direction, target movement, implausible
sample velocity, and position discontinuity remain failure conditions. A
successful run with warnings reports `PASS_WITH_WARNINGS`; it verifies joint
direction but does not verify physical ID order or endpoint stability.

For encoder-boundary acceptance, run one guided continuous check per side. Test
safe joints and record an explicit skip reason for unsafe joints:

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --guided-continuous --boundary-only --boundary-mode nonzero \
  --boundary-margin 0.05 --ids 1,2,3,4,5,6,7 --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --baseline-frames 50 --continuous-max-frames 750 \
  --settle-frames 50 --endpoint-frames 100 --interval 0.02 \
  --max-position-step 0.2 --motion-reversal-tolerance 0.02 \
  --max-hold-velocity 0.05 --max-hold-position-span 0.01 \
  --output validation_records/phase2_5_1/left_continuous.json
```

`--boundary-mode nonzero` selects the nearest nonzero `k * 2π` boundary, so a
small crossing around zero is not used as the acceptance evidence. This test does
not require rotating a full `2π`: start safely near the selected boundary and
cross at least `--boundary-margin` beyond it. In `--boundary-only` mode, a target
position discontinuity remains a failure and a missing crossing is inconclusive;
direction, other-joint movement, velocity, reversal, and endpoint stability
findings are recorded as warnings. The tool distinguishes `PASS`,
`PASS_WITH_WARNINGS`, `FAIL`, `INCONCLUSIVE`, and `SKIPPED_UNSAFE` results.

## Phase 2.5.2 single-servo low-current check

`skye_factr_dynamixel_low_current_check` calls the production
`DynamixelDriver` for exactly one configured ID. Before any writable open, it
performs a separate read-only check of the model number, Return Delay Time,
Torque Disable state, baud rate, and FTDI latency timer. Hardware mode requires
`--confirm-single-servo` and applies both an explicit operator limit and a hard
ceiling of `min(100 mA, 10% of the model rated current limit)`.

Use `--prepare-only` before connecting hardware. This validates all parameters,
shows the XM430/XC330 register conversion, and writes a JSON preparation record
without constructing a driver or opening a serial port:

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --prepare-only --side left --id 1 \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --output /tmp/skye_phase2_5_2_left_id1_prepared.json
```

Follow the complete
[phase 2.5.2 low-current checklist](dev_docs/skye_factr_teleop_phase2_5_low_current_checklist.md)
before removing `--prepare-only`. Hardware mode sends short positive, zero,
negative, and zero commands, checks encoder direction, and always invokes the
production close path to write zero current and Torque Disable.

## Stage 3 control core (no hardware)

`control_core.py` contains explicit quarter-turn calibration, physical/logical
joint mapping, individually switchable joint-limit, gravity, static-friction,
and null-space terms, plus one non-blocking `ControlCore.step()` call. A runtime
adapter supplies the dynamics interface (`gravity`, `jacobian`, and
`forward_kinematics`) and an explicit current limit; constructing the core does
not open a serial port or enable torque. The step writes no current when torque
is disabled. On read, computation, or write failure it attempts zero current
and Torque Disable. Follower torque and gripper feedback are interface-only
until stage 8.

The fixed-input and fake-device tests run without ROS, serial hardware,
Pinocchio, or Ruckig:

```bash
python3 -m pytest -q test/test_control_core.py
```

## Stage 4.1/4.2 state machine

The contract treats trajectory and settling timeouts as failures:
`TELEOP_SYNCING -> IDLE` or `HOLD_SETTLING -> IDLE`. Neither permits
`switch_teleop` after timeout. Retry with `switch_sync`, wait for successful
completion (`SYNCED`), then request `switch_teleop`.

This changes the timeout-to-`SYNCED` behavior described by the earlier plan.
See the [migration notes](dev_docs/skye_factr_teleop_migration.md) for source
limitations and caller changes. The pure Python state machine is implemented;
task 4.3 also integrates trajectory cleanup and hold in `ControlCore`.

Construct `StateMachine(now=...)` and pass explicit events to
`handle(event, now=...)`. Events accept `Event` members or the frozen string
names. Supply finite, nondecreasing timestamps from the same clock; no clock is
read internally. The control core emits `trajectory_complete` when Ruckig finishes,
then `sync_complete` only after measured settling. Timeout takes precedence at
each phase's deadline.

Each immutable `TransitionResult` contains `previous_state`, `state`, `outcome`,
`reason`, ordered `actions`, and `entered_at`. `accepted` means the event was
handled, so an accepted timeout still means synchronization failed. The machine
records `last_rejection_reason` across subsequent events. `entered_at` changes
only when the state changes; repeated sync requests do not restart its timer.

The caller executes the returned actions in order:

| Action | Control adapter responsibility |
|---|---|
| `stop_motion` | Stop automatic leader motion, hold, and teleop publication |
| `clear_sync` | Clear the trajectory, hold integral, and previous target |
| `start_sync` | Capture a fresh synchronization start and target |
| `start_hold` | Hold the completed synchronization target |
| `start_teleop` | Enable leader-to-follower command publication |

Stop and timeout return `stop_motion`, then `clear_sync`. Rejected or ignored
events return no actions. `TELEPOSE=5` is reserved; `switch_telepose` is rejected.
This module has no ROS, serial, Ruckig, or hardware side effects.

Run the parameterized contract tests from the package directory:

```bash
python3 -m pytest -q test/test_state_machine.py
```

All 44 state-machine tests pass with explicit test timestamps; the task 4.1
expected-failure marker has been removed.

## Stage 4.3 Ruckig synchronization and hold

`ControlCore.handle()` executes state-machine actions synchronously.
`step(now=...)` reads the leader once, advances the active trajectory at most
once, computes current, and performs the normal current write. There is no
internal loop, sleep, ROS call, or automatic Torque Enable. Serialize all core
calls in one control loop. Call `step` at `controller.frequency` with timestamps
in seconds from one monotonic clock; synchronization and hold require `now`.

The real `RuckigTrajectory` adapter lazily imports the baseline Python package
`ruckig==0.19.4`. It follows the official
[online update example](https://docs.ruckig.com/md_pages_2example__01.html),
including `pass_to_input` after each update. Fake trajectory tests need no
Ruckig installation.

The existing YAML does not define trajectory velocity, acceleration, or jerk
limits. Supply explicit, positive eight-axis limits in rad/s, rad/s² and rad/s³
through a factory; no hardware motion limits are inferred from PD gains:

```python
from skye_factr_teleop.control_core import ControlCore
from skye_factr_teleop.trajectory import RuckigTrajectory

# config, hardware, dynamics, offsets, current_limit_ma and the three limit
# vectors are supplied by the caller for the particular leader arm.
def make_trajectory():
    return RuckigTrajectory(
        frequency=config.controller.frequency,
        max_velocity=velocity_limits,
        max_acceleration=acceleration_limits,
        max_jerk=jerk_limits,
    )

core = ControlCore(
    config, hardware, dynamics, offsets, current_limit_ma,
    trajectory_factory=make_trajectory, now=t0,
)
core.handle(
    "switch_sync", now=t0,
    follower_position=follower_q, gripper_setpoint=gripper_q,
)
cycle = core.step(now=tick_time)  # Once per caller's control cycle.
```

Accepted sync captures the measured eight-axis logical leader position and
velocity and copies the seven follower positions plus gripper setpoint.
Coordinates must already be mapped to the logical leader frame. If omitted,
the gripper target is `gripper_teleop.limit_setpoint`. Starting acceleration
is zero because no acceleration measurement is available; target velocity and
acceleration are zero. A fresh trajectory is created per sync. Repeated sync
requests do not replace the target or restart the timeout.

During synchronization, tracking torque is
`ruckig_kp * (q_ref - q) + ruckig_kd * (dq_ref - dq)`.
Ruckig completion enters `HOLD_SETTLING` on the captured final target. Hold torque is
`hold_kp * error - hold_kd * dq + hold_ki * integral`.
For each axis, integrate `error * dt` only when
`abs(error) <= hold_i_thresh`; otherwise reset that axis's integral.
`hold_i_max` bounds the accumulated error in rad·s, before multiplication by
`hold_ki`. The first hold cycle starts with zero integral.

During `HOLD_SETTLING`, every axis must satisfy `sync_position_tolerance` and
`sync_velocity_tolerance` continuously for `settle_duration`. The timer resets
when any axis leaves its band. Success enters `SYNCED` and hold continues.
`settle_timeout` returns to `IDLE` and disables torque. These four settings are
under `controller.hold_ruckig_control`; the supplied values are initial tuning
values and need validation on the robot.

The two phases use `ruckig_gravity_comp_gain` and `hold_gravity_comp_gain`
respectively, replacing the normal gravity gain. The configured gravity mode
and term switches still apply. Joint-limit protection and static friction
remain available; null-space regulation is suppressed during sync/hold to
avoid competing with their full joint targets. Final torque uses the existing
servo direction conversion and current limits.

At `elapsed >= teleop_sync_timeout`, timeout wins before a Ruckig update. Settling
timeout likewise wins before a hold update. Stop,
timeout, and computation/setup failures clear the trajectory, integral, and
target, return to `IDLE`, and attempt zero current and Torque Disable. An explicit
`stop()` needs no timestamp; `handle("switch_stop", now=...)` also stops when
the supplied time is invalid. Other failures propagate after cleanup. The
caller must deliberately re-enable torque before retrying synchronization.
External completion/timeout events are forbidden at the core API, so old
asynchronous events cannot complete a new synchronization.

Entering `TELEOP` clears automatic tracking/hold output. The normal compensation
terms remain available; ROS publication belongs to stage 5. Observe `mode`,
`last_transition`, and the copied `hold_integral`. `CycleResult` adds
`tracking_torque`, `mode`, `target_position`, and `target_velocity`; target
fields are `None` outside sync/hold. `wrote_current` describes the normal cycle
write; a safety zero write during cleanup is separate.

## Stage 4.4 state/trajectory integration acceptance

`test/test_state_trajectory_integration.py` runs the production core, state
machine, and Ruckig adapter with a fake clock, fake Ruckig Python API, and fake
Dynamixel hardware. Its 22 cases cover both left and right configurations:
the complete sync/hold/teleop/stop sequence, repeated requests without side
effects, completion before/at/after the deadline, fresh start capture after
stop/timeout, integral reset, invalid targets, and Ruckig error results.
Several consecutive TELEOP cycles verify that automatic leader targets remain
absent while normal compensation can continue.

Run the stage 4 acceptance checks and control-core regressions from the package
directory:

```bash
python3 -m pytest -q test/test_state_trajectory_integration.py test/test_sync_hold.py test/test_state_machine.py test/test_control_core.py
```

All 120 checks pass: 119 fake/pure-logic checks plus the retained real Ruckig
0.19.4 trajectory limit check. Only that real-library test skips if Ruckig is
absent; integration acceptance uses a fake module and needs no installed
Ruckig. Task 4.4 and milestone M4 are complete. ROS integration and hardware
acceptance remain stages 5 and 7 respectively.

## License

Apache-2.0. See [LICENSE](LICENSE) and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
