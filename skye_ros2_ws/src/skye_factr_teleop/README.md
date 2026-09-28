# skye_factr_teleop

ROS 2 Humble `ament_python` package for Skye/M6 FACTR-style leader-arm
teleoperation.

The package includes validated YAML configuration, the production Dynamixel
adapter, and a ROS-free single-cycle control core. The ROS node and launch
integration are later tasks; the low-current hardware smoke check remains
pending in development-plan stage 7.

## Configuration

The verified default Thor M6 parameters are installed as
`config/grav_comp_m6_left.yaml` and `config/grav_comp_m6_right.yaml`. Load and
validate either file with `skye_factr_teleop.config.load_config`. Returned nested
dataclasses and numeric tuples are immutable.

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
ros2 run skye_factr_teleop skye_factr_teleop_driver
```

The keyboard entry point is intentionally deferred until basic hardware
acceptance is complete.

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

## Stage 4.1 state transition contract

The frozen contract treats synchronization timeout as failure:
`TELEOP_SYNCING -> IDLE`. It must not enter `SYNCED` or permit
`switch_teleop` after timeout. Retry with `switch_sync`, wait for successful
completion (`SYNCED`), then request `switch_teleop`.

This changes the timeout-to-`SYNCED` behavior described by the earlier plan.
See the [migration notes](dev_docs/skye_factr_teleop_migration.md) for source
limitations and caller changes. State-machine implementation remains task 4.2;
trajectory cleanup and hold integration remain tasks 4.3/4.4.

Run the parameterized contract tests from the package directory:

```bash
python3 -m pytest -q test/test_state_machine.py
```

Until `StateMachine` exists, these tests report expected failures (`xfail`),
not implemented behavior. The marker automatically stops applying when the
class is introduced, so contract violations then fail normally.

## License

Apache-2.0. See [LICENSE](LICENSE) and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
