# Task 4 Report: Host keyboard + start script

**Status:** Complete  
**Date:** 2026-09-04  
**Commit:** `66bdb28` — `feat(align): host keyboard s/x and start_follower_align.sh`

## Summary

Implemented the host-side keyboard bridge and one-shot start script for follower align after FACTR sync. The keyboard node mirrors the `skye_hitl_dagger` tty/cbreak pattern; the launch file conditionally starts it when `enable_keyboard:=true`.

## Deliverables

### 1. `skye_follower_align/host_keyboard.py`

- Entry point: `host_keyboard_align`
- Node name: `host_keyboard_align`
- **Keys:**
  - `s` → publish `std_msgs/String` on `/mode/align_follower` with `data=align_follower`
  - `x` → publish on `/mode/align_cancel` with `data=align_cancel`
  - `q` → stop reader and call `rclpy.shutdown()`
- TTY: `tty.setcbreak` + `select` when stdin is a TTY; line mode fallback otherwise
- Banner logged on TTY startup: `s=align x=cancel q=quit`
- Pattern aligned with `skye_hitl_dagger/hitl_keyboard_node.py` (`KeyboardReader`, `map_key`, `destroy_node` cleanup)

### 2. `launch/follower_align.launch.py`

- Added `IfCondition(LaunchConfiguration("enable_keyboard"))` around `host_keyboard_align` node
- Existing `OpaqueFunction` for `robot_profile` joint signs unchanged

### 3. `setup.py`

- Added console script: `host_keyboard_align = skye_follower_align.host_keyboard:main`

### 4. `scripts/start_follower_align.sh`

- Executable (`chmod +x`)
- `ROS_DOMAIN_ID=21`, `rmw_fastrtps_cpp`, `FASTRTPS_DEFAULT_PROFILES_FILE` → `marvin_ws/fastrtps_no_shm.xml`
- `ROBOT_PROFILE` default `thor`
- Sources ROS Humble + workspace install
- `exec ros2 launch skye_follower_align follower_align.launch.py robot_profile:=… enable_keyboard:=true`

## Build / smoke

```bash
cd skye_ros2_ws && ./scripts/build.sh skye_follower_align
source install/setup.bash
ros2 pkg executables skye_follower_align
```

**Result:**

```
skye_follower_align follower_align_node
skye_follower_align host_keyboard_align
```

Build: success (0.52s).

## Files in commit

| Path | Change |
|------|--------|
| `skye_ros2_ws/src/skye_follower_align/skye_follower_align/host_keyboard.py` | created |
| `skye_ros2_ws/src/skye_follower_align/launch/follower_align.launch.py` | modified |
| `skye_ros2_ws/src/skye_follower_align/setup.py` | modified |
| `scripts/start_follower_align.sh` | created |

## Concerns / follow-ups

1. **No unit tests** for `host_keyboard.py` — HITL has `test_hitl_keyboard_node.py` for `map_key` only; optional to add `test_host_keyboard.py` in a later task.
2. **TTY required for best UX** — launch must keep the terminal foreground (same as HITL); non-TTY falls back to line mode with Enter.
3. **`q` shuts down keyboard node only** — does not stop `follower_align_node`; launch continues until Ctrl+C or process group kill. Matches brief (“shutdown node”); full stack teardown is operator responsibility.
4. **Runtime integration** not exercised here — needs live `/gento/*` services and leader joint topics on domain 21.

## Usage

```bash
./scripts/start_follower_align.sh
# or
ROBOT_PROFILE=orin ./scripts/start_follower_align.sh
```

After FACTR sync, press `s` to start align, `x` to cancel, `q` to quit keyboard node.
