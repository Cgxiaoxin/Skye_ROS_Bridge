#!/usr/bin/env python3
"""Drag-teach waypoints + impedance stream replay (lag / arrival test).

Requires exclusive Gento link (stop skye_robot_driver first).

  export GENTO_SDK_ROOT=/path/to/tianji-robot-SDK-Gento_Skye-Luna
  python pd_test/imp_traj_test.py --collect --arm left --profile orin
  python pd_test/imp_traj_test.py --play --arm left --profile orin --vel 100 --acc 100 --dt 0.02
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
DEFAULT_SDK = Path("/data/coding/tianji/tianji-robot-SDK-Gento_Skye-Luna")
PROFILE_IP = {"orin": "6.6.7.190", "thor": "6.6.7.191"}

# Match skye_robot_driver imp_joint defaults.
K = [25.0] * 7
D = [6.0] * 7
# Soft gains for drag teach.
K_DRAG = [3.0, 3.0, 3.0, 2.0, 1.0, 1.0, 1.0]
D_DRAG = [0.2] * 7


def _setup_sdk():
    root = Path(os.environ.get("GENTO_SDK_ROOT", DEFAULT_SDK))
    py = root / "PYTHON_SDK"
    if not (py / "GentoRobot.py").is_file():
        raise SystemExit(
            f"GentoRobot not found under {py}. Set GENTO_SDK_ROOT to the SDK tree."
        )
    sys.path.insert(0, str(root))
    from PYTHON_SDK.GentoRobot import (  # noqa: E402
        FXLogMask,
        FXObjMask,
        FXObjType,
        GentoRobot,
        state_map,
    )

    return GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map


def parse_ip(s: str):
    parts = s.strip().split(".")
    if len(parts) != 4:
        raise ValueError(f"bad ip: {s}")
    return [int(p) for p in parts]


def arm_obj(arm: str, FXObjType):
    return (FXObjType.OBJ_ARM0, 0) if arm == "left" else (FXObjType.OBJ_ARM1, 1)


def ensure_idle(robot, obj, state_map, timeout_ms=2000):
    st = state_map[robot.current_state(obj)]
    if st == "Error":
        ret, _ = robot.reset_error(obj, timeout_ms)
        if ret != 0:
            raise RuntimeError(f"reset_error failed: {ret}")
    elif st != "IDLE":
        ret = robot.switch_to_idle(obj, timeout_ms)
        if ret != 0:
            raise RuntimeError(f"switch_to_idle failed: {robot._get_operate_error_msg(ret)}")


def read_waypoints(path: Path):
    pts = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        vals = [float(x) for x in line.replace(",", " ").split()]
        if len(vals) != 7:
            raise ValueError(f"need 7 joints, got {len(vals)}: {line}")
        pts.append(vals)
    if len(pts) < 2:
        raise ValueError(f"need >=2 waypoints in {path}")
    return pts


def interpolate(waypoints, seg_time: float, dt: float):
    traj = [list(waypoints[0])]
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        n = max(1, int(round(seg_time / dt)))
        for i in range(1, n + 1):
            a_ = i / n
            traj.append([(1.0 - a_) * a[j] + a_ * b[j] for j in range(7)])
    return traj


def start_estop(robot, FXObjMask):
    def _run():
        try:
            input()
        except EOFError:
            return
        print("\n[E-STOP] Enter pressed")
        try:
            robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
        except Exception as e:
            print(f"estop error: {e}")
        os._exit(1)

    threading.Thread(target=_run, daemon=True).start()


def cmd_collect(args, robot, FXObjType, FXObjMask, state_map):
    obj, idx = arm_obj(args.arm, FXObjType)
    ensure_idle(robot, obj, state_map)
    ret = robot.switch_to_drag_joint(obj, 2000, K_DRAG, D_DRAG)
    if ret != 0:
        raise RuntimeError(f"DragJoint failed: {robot._get_operate_error_msg(ret)}")
    print(f"DragJoint on {args.arm}. Drag arm, Enter=save, q=quit")

    pts = []
    while True:
        line = input("> ").strip().lower()
        if line in ("q", "quit"):
            break
        fb = list(robot.get_rt_dict()["arms"][idx]["fb"]["fb_pos"])
        pts.append(fb)
        print(f"  #{len(pts)} {[round(x, 2) for x in fb]}")

    args.waypoints.parent.mkdir(parents=True, exist_ok=True)
    with args.waypoints.open("w") as f:
        f.write("# deg, 7 joints per line\n")
        for p in pts:
            f.write(" ".join(f"{x:.6f}" for x in p) + "\n")
    print(f"saved {len(pts)} waypoints → {args.waypoints}")
    ensure_idle(robot, obj, state_map)


def cmd_play(args, robot, FXObjType, FXObjMask, state_map):
    obj, idx = arm_obj(args.arm, FXObjType)
    wps = read_waypoints(args.waypoints)
    traj = interpolate(wps, args.seg_time, args.dt)
    print(f"waypoints={len(wps)} traj={len(traj)} dt={args.dt}s "
          f"vel={args.vel} acc={args.acc} tol={args.tol}deg")

    ensure_idle(robot, obj, state_map)
    time.sleep(0.3)
    ret = robot.switch_to_imp_joint_mode(obj, 2000, args.vel, args.acc, K, D)
    if ret != 0:
        raise RuntimeError(f"ImpJoint failed: {robot._get_operate_error_msg(ret)}")
    # Re-assert speed after mode switch (firmware requirement).
    robot.runtime_set_speed_ratio(obj, args.vel, args.acc)
    sg = robot.get_sg_dict()["arms"][idx]["set"]
    print(f"imp vel={sg['vel_ratio']} acc={sg['acc_ratio']} k={sg['joint_k']}")

    rows = []
    late = 0
    t0 = time.perf_counter()
    for i, cmd in enumerate(traj):
        t_cmd = i * args.dt
        # Pace to wall clock so lag is measurable.
        wait = t0 + t_cmd - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        ret = robot.runtime_set_joint_pos_cmd(obj, cmd)
        if ret != 0:
            print(f"cmd {i} failed: {robot._get_operate_error_msg(ret)}")
            break
        rt = robot.get_rt_dict()["arms"][idx]
        fb = list(rt["fb"]["fb_pos"])
        vel = list(rt["fb"]["fb_vel"])
        err = [abs(fb[j] - cmd[j]) for j in range(7)]
        err_max = max(err)
        vel_max = max(abs(v) for v in vel)
        arrived = int(err_max < args.tol)
        late += 1 - arrived
        rows.append([i, f"{t_cmd:.4f}", f"{err_max:.4f}", f"{vel_max:.4f}", arrived]
                    + [f"{x:.4f}" for x in cmd] + [f"{x:.4f}" for x in fb])
        if i % 25 == 0 or i == len(traj) - 1:
            print(f"  i={i}/{len(traj)-1} t={t_cmd:.2f}s err_max={err_max:.2f} "
                  f"vel_max={vel_max:.2f} arrived={arrived}")

    elapsed = time.perf_counter() - t0
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["i", "t_cmd", "err_max_deg", "fb_vel_max", "arrived"]
            + [f"cmd{j}" for j in range(7)]
            + [f"fb{j}" for j in range(7)]
        )
        w.writerows(rows)

    errs = [float(r[2]) for r in rows]
    vels = [float(r[3]) for r in rows]
    print("--- summary ---")
    print(f"steps={len(rows)} wall={elapsed:.2f}s expected={len(traj)*args.dt:.2f}s")
    print(f"late(arrived=0)={late}/{len(rows)} "
          f"({100.0 * late / max(len(rows), 1):.1f}%)")
    if errs:
        print(f"err_max_deg: mean={sum(errs)/len(errs):.3f} max={max(errs):.3f}")
        print(f"fb_vel_max:  mean={sum(vels)/len(vels):.3f} max={max(vels):.3f}")
    print(f"log → {args.log}")
    ensure_idle(robot, obj, state_map)


def main():
    p = argparse.ArgumentParser(description="Impedance traj follow test")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--collect", action="store_true")
    g.add_argument("--play", action="store_true")
    p.add_argument("--arm", choices=("left", "right"), default="left")
    p.add_argument("--profile", choices=("orin", "thor"), default="orin")
    p.add_argument("--ip", default=None)
    p.add_argument("--waypoints", type=Path, default=HERE / "waypoints.txt")
    p.add_argument("--log", type=Path, default=HERE / "play_log.csv")
    p.add_argument("--vel", type=float, default=100.0)
    p.add_argument("--acc", type=float, default=100.0)
    p.add_argument("--dt", type=float, default=0.02)
    p.add_argument("--seg-time", type=float, default=1.0)
    p.add_argument("--tol", type=float, default=1.0)
    args = p.parse_args()
    args.ip = args.ip or PROFILE_IP[args.profile]

    GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map = _setup_sdk()
    robot = GentoRobot()
    start_estop(robot, FXObjMask)
    ip = parse_ip(args.ip)
    try:
        print(f"link {args.ip} arm={args.arm} ...")
        ret = robot.link(*ip, log_level=FXLogMask.FX_LOG_INFO_FLAG)
        if not robot._connected:
            raise RuntimeError(f"link failed: {robot._get_operate_error_msg(ret)}")
        print(f"sdk={robot.get_sdk_version()} ctrl={robot.get_controller_version()}")
        if args.collect:
            cmd_collect(args, robot, FXObjType, FXObjMask, state_map)
        else:
            cmd_play(args, robot, FXObjType, FXObjMask, state_map)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        if robot._connected:
            try:
                obj, _ = arm_obj(args.arm, FXObjType)
                robot.switch_to_idle(obj, 2000)
            except Exception:
                pass
            robot.unlink()
            print("unlinked")


if __name__ == "__main__":
    main()
