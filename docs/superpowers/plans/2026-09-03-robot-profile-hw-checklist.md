# Robot profile hardware validation checklist

Manual acceptance for Thor / Orin dual-machine `robot_profile` merge.  
Offline CI checks (2026-09-03) marked `[x]`; operator must complete hardware items `[ ]`.

**Rollback:** `git checkout d591505` restores the pre-merge tree.

**Known risk:** Thor `marvin_ws/configs/thor/grav_comp_m6_right.yaml` `joint_signs` currently match Orin (`[1,-1,1,-1,1,-1,-1,-1]`). Physical teleop on Thor must confirm wrist/gripper directions; adjust Thor yaml if needed.

---

## Offline verification (no robot controller)

- [x] Driver profiles exist: `skye_ros2_ws/src/skye_robot_driver/config/profiles/{thor,orin}.yaml`
- [x] Leader configs exist: `marvin_ws/configs/{thor,orin}/grav_comp_m6_{left,right}.yaml`
- [x] `ros2 launch skye_robot_driver skye_robot_driver.launch.py robot_profile:=thor --show-args` — exposes `robot_profile`, `connect_on_startup`, legacy Robotiq flags
- [x] `ros2 launch … robot_profile:=orin --show-args` — same arg surface
- [x] Brief launch `connect_on_startup:=false` loads `skye_robot.yaml` + `profiles/thor.yaml` (node starts, no SDK connect)
- [x] `ROBOT_PROFILE=thor ./scripts/sync_marvin_overlay.sh` — copies `configs/thor/grav_comp_m6_*.yaml` to install
- [x] `ROBOT_PROFILE=orin ./scripts/sync_marvin_overlay.sh` — copies `configs/orin/grav_comp_m6_*.yaml` to install

Expected offline profile values:

| Profile | Host `gripper_*_type` | Host `right_joint_signs` J6/J7 | Leader right `joint_signs` |
|---------|----------------------|--------------------------------|----------------------------|
| thor | dm4310 / dm4310 | +1 / +1 | `[1,-1,1,-1,1,-1,-1,-1]` ⚠ verify on Thor HW |
| orin | robotiq / robotiq | -1 / -1 | `[1,-1,1,-1,1,-1,-1,-1]` |

---

## Thor (this machine, default)

- [ ] **Step 1:** `ROBOT_PROFILE=thor ./scripts/start_skye_for_factr.sh`
- [ ] **Step 2:** Confirm log / params: `gripper_left_type=dm4310`, signs all `+1`
- [ ] **Step 3:** `ROBOT_PROFILE=thor ./scripts/run_marvin_m6_impedance.sh` → `1` sync → `2` teleop
- [ ] **Step 4:** Wrist/J4 same direction; grippers open on release
- [ ] **Step 5:** `ros2 topic echo /gento/right_joint_states --once` is 7-DoF matching right half of `/gento/joint_states`

---

## Orin (other machine)

- [ ] **Step 6:** Pull/sync same `main`, then `ROBOT_PROFILE=orin` for **both** host driver and Docker
- [ ] **Step 7:** Confirm robotiq start + force-open; right J6/J7 follow leader
- [ ] **Step 7b:** `ros2 topic hz /gento/joint_states` — stable rate with gripper enabled
- [ ] **Step 8:** Right sync targets **right** big arm (not left)

---

## Rollback drill (optional)

- [ ] **Step 9:** Know that `git checkout d591505` restores pre-merge tree if needed
