# skye_factr_teleop

ROS 2 Humble `ament_python` package for Skye/M6 FACTR-style leader-arm
teleoperation.

The package skeleton and immutable YAML configuration model from development-plan
tasks 1.1 and 1.2 are implemented. Control, hardware, and launch behavior are
implemented in later tasks.

## Configuration

The verified default Thor M6 parameters are installed as
`config/grav_comp_m6_left.yaml` and `config/grav_comp_m6_right.yaml`. Load and
validate either file with `skye_factr_teleop.config.load_config`. Returned nested
dataclasses and numeric tuples are immutable.

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

## License

Apache-2.0. See [LICENSE](LICENSE) and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
