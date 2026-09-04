# Follower align hardware validation checklist

Manual acceptance for **1 → s → 2** follower absolute-align after FACTR sync.  
**Offline template only** — no hardware run in authoring session; operator completes items `[ ]`.

**Plan:** `docs/superpowers/plans/2026-09-04-follower-align-after-sync.md`  
**Operator docs:** `docs/Thor_Orin_遥操启动.md` §「对齐（FACTR sync 之后）」

**Prerequisites:** `skye_robot_driver` running, FACTR Docker up, `ROS_DOMAIN_ID=21`, FastDDS no-SHM xml, align helper via `scripts/start_follower_align.sh`.

**Defaults:** `align_hold_frames=5`, `align_timeout_s=10.0`, `align_rate_hz=50`, align vel/acc ratio **10**, restore ratios **30** (params on `follower_align_node`).

**Pending from Task 2 (not blocking checklist authoring):** live `/gento/set_motion_rates` service smoke on connected driver — run before or during first align session:

```bash
ros2 service call /gento/set_motion_rates skye_robot_driver/srv/SetMotionRates \
  "{left_vel_ratio: 10, left_acc_ratio: 10, right_vel_ratio: 10, right_acc_ratio: 10}"
# expect success: true
```

---

## Shared setup (both machines)

- [ ] Host driver + FACTR Docker on same `ROBOT_PROFILE` (`thor` or `orin`)
- [ ] Align helper running: `ROBOT_PROFILE=<profile> ./scripts/start_follower_align.sh`
- [ ] Monitor: `ros2 topic echo /align/status` shows `IDLE` before first `s`
- [ ] FACTR `1` sync complete; big/small arms stable before align

---

## Thor — happy path (`ROBOT_PROFILE=thor`)

- [ ] FACTR `1` sync → steady
- [ ] Host `s` → `/align/status` → `ALIGNING`
- [ ] Big arms move slowly toward small-arm pose (absolute commands on `/gento/{left,right}_joint_control_abs`)
- [ ] During align: vel/acc ratio **10** (driver log or `SetMotionRates` state); motion visibly slower than normal teleop
- [ ] `/align/status` → `ALIGNED` (per-joint |err| &lt; 0.05 rad, hold 5 frames)
- [ ] After `ALIGNED`: abs command stream stops; rates restored to **30** (or configured restore params); hold engaged
- [ ] FACTR `2` → relative teleop works; no gripper commands from align node

**Recorded:** date / operator / notes:

---

## Orin — happy path + right wrist signs (`ROBOT_PROFILE=orin`)

- [ ] Same flow as Thor: `1` → `s` → slow motion + ratio 10 → `ALIGNED` → restore → `2`
- [ ] Right wrist J6/J7 on big arm follow small arm with **Orin signs** (`right_joint_signs` = `[1,1,1,1,1,-1,-1]` from launch); no inverted wrist motion vs leader
- [ ] Left arm signs all `+1`; both arms reach `ALIGNED` before teleop

**Recorded:** date / operator / notes:

---

## Soft timeout (`TIMEOUT_WARN` → still `2`)

Force timeout by lowering `align_timeout_s` for one run (e.g. launch param `align_timeout_s:=1.0` or mis-align arms so hold never completes):

- [ ] `/align/status` → `TIMEOUT_WARN` (not a hard fault)
- [ ] Abs streaming stops; hold + rate restore same as `ALIGNED`
- [ ] FACTR `2` → teleop still allowed (soft timeout only)

**Recorded:** timeout param used / operator / notes:

---

## Cancel during align (`x` → immediate hold)

- [ ] Start align (`s`); while `ALIGNING`, host `x` (or `ros2 topic pub --once /mode/align_cancel std_msgs/msg/String "{data: align_cancel}"`)
- [ ] Motion stops promptly; abs commands cease
- [ ] Rates restored; `/align/status` → `IDLE`
- [ ] Re-`s` allowed after cancel

**Recorded:** date / operator / notes:

---

## Topic-only path (no keyboard)

Launch without keyboard helper; trigger/cancel via topics only:

```bash
ROBOT_PROFILE=<profile> ros2 launch skye_follower_align follower_align.launch.py \
  robot_profile:=<profile> enable_keyboard:=false
```

- [ ] `ros2 topic pub --once /mode/align_follower std_msgs/msg/String "{data: align_follower}"` → full align cycle (`ALIGNED` or `TIMEOUT_WARN`)
- [ ] `ros2 topic pub --once /mode/align_cancel ...` cancels mid-align
- [ ] FACTR `2` teleop after completion

**Recorded:** date / operator / notes:

---

## Sign-off

- [ ] All sections above passed on **Thor** hardware
- [ ] All sections above passed on **Orin** hardware
- [ ] Task 2 live `/gento/set_motion_rates` smoke confirmed on robot

**Final sign-off:** _______________  **Date:** _______________
