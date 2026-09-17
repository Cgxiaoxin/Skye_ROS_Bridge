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

## License

Apache-2.0. See [LICENSE](LICENSE) and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
