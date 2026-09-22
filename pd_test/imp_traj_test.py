#!/usr/bin/env python3
"""ImpJoint waypoint test — aligned with example_basic_ImpJoint.py.

Official ImpJoint usage (see pd_test/example/example_basic_ImpJoint.py):
  1) link → recover Error/IDLE
  2) switch_to_imp_joint_mode(vel, acc, k, d)
  3) SetJointPosCmd(ONE target) → poll until approx equal → next target
  4) switch_to_idle

NOT the previous wall-clock stream (that changes the target before the arm arrives,
so Imp always lags — large err, and vel/acc ratio looks “useless”).

  # teach (DragJoint)
  python pd_test/imp_traj_test.py --collect --arm right --profile orin

  # replay waypoints point-to-point (like the official example)
  python pd_test/imp_traj_test.py --play --arm right --profile orin --vel 10 --acc 10

  # optional: stream stress test (policy-like; expect larger lag)
  python pd_test/imp_traj_test.py --play --style stream --arm right --profile orin \\
      --vel 100 --acc 100 --seg-time 1.0 --dt 0.02
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
DEFAULT_SDK = Path("/data/coding/tianji/tianji-robot-SDK-Gento_Skye-Luna")
PROFILE_IP = {"orin": "6.6.7.190", "thor": "6.6.7.191"}

# Match official ImpJoint example defaults.
K_IMP = [60.0, 60.0, 60.0, 40.0, 20.0, 20.0, 20.0]
D_IMP = [0.3] * 7
K_DRAG = list(K_IMP)
D_DRAG = list(D_IMP)


def _setup_sdk():
    root = Path(os.environ.get("GENTO_SDK_ROOT", DEFAULT_SDK))
    if not (root / "PYTHON_SDK" / "GentoRobot.py").is_file():
        raise SystemExit(f"GentoRobot not found under {root}/PYTHON_SDK. Set GENTO_SDK_ROOT.")
    sys.path.insert(0, str(root))
    from PYTHON_SDK.GentoRobot import (  # noqa: E402
        FXLogMask,
        FXObjMask,
        FXObjType,
        GentoRobot,
        error_dict,
        state_map,
    )

    return GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map, error_dict


def parse_ip(s: str):
    parts = s.strip().split(".")
    if len(parts) != 4:
        raise ValueError(f"bad ip: {s}")
    return [int(p) for p in parts]


def arm_obj(arm: str, FXObjType):
    return (FXObjType.OBJ_ARM0, 0) if arm == "left" else (FXObjType.OBJ_ARM1, 1)


def recover_idle(robot, obj, state_map, error_dict, timeout_ms=2000):
    """Same recover path as example_basic_ImpJoint.py."""
    st = state_map[robot.current_state(obj)]
    print(f"  state={st}")
    if st == "Error":
        ret, code = robot.reset_error(obj, timeout_ms)
        if ret != 0:
            raise RuntimeError(f"reset_error failed: {error_dict.get(code, code)}")
        print("  reset_error → IDLE")
    elif st != "IDLE":
        ret = robot.switch_to_idle(obj, timeout_ms)
        if ret != 0:
            raise RuntimeError(f"switch_to_idle failed: {robot._get_operate_error_msg(ret)}")
        print("  switch_to_idle ok")


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
        print("\n[E-STOP]")
        try:
            robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
        except Exception as e:
            print(f"estop error: {e}")
        os._exit(1)

    threading.Thread(target=_run, daemon=True).start()


def go_to_wait(robot, obj, idx, target, tol, timeout_s=60.0, log_every_s=0.5):
    """example style: one SetJointPosCmd, then poll until approx equal."""
    ret = robot.runtime_set_joint_pos_cmd(obj, target)
    if ret != 0:
        raise RuntimeError(f"SetJointPosCmd failed: {robot._get_operate_error_msg(ret)}")
    t0 = time.perf_counter()
    t_last_log = t0
    while True:
        fb = list(robot.get_rt_dict()["arms"][idx]["fb"]["fb_pos"])
        err = [abs(fb[j] - target[j]) for j in range(7)]
        err_max = max(err)
        now = time.perf_counter()
        if robot.check_sequences_approx_equal(fb, target, tolerance=tol):
            return now - t0, fb
        if now - t0 > timeout_s:
            raise TimeoutError(f"arrive timeout {timeout_s}s err_max={err_max:.2f}deg")
        if now - t_last_log >= log_every_s:
            print(f"  waiting t={now - t0:.1f}s err_max={err_max:.2f}deg "
                  f"err={[round(e, 2) for e in err]} "
                  f"fb={[round(x, 2) for x in fb]}")
            t_last_log = now
        time.sleep(0.001)


def cmd_collect(args, robot, FXObjType, state_map, error_dict):
    obj, idx = arm_obj(args.arm, FXObjType)
    recover_idle(robot, obj, state_map, error_dict)
    ret = robot.switch_to_drag_joint(obj, 2000, K_DRAG, D_DRAG)
    if ret != 0:
        raise RuntimeError(f"DragJoint failed: {robot._get_operate_error_msg(ret)}")
    print(f"DragJoint on {args.arm}. Enter=save, q=quit (hold terminal drag button)")

    pts = []
    while True:
        line = input("> ").strip().lower()
        if line in ("q", "quit"):
            break
        fb = list(robot.get_rt_dict()["arms"][idx]["fb"]["fb_pos"])
        pts.append(fb)
        print(f"  #{len(pts)} {[round(x, 2) for x in fb]}")

    if not pts:
        print("no waypoints")
        recover_idle(robot, obj, state_map, error_dict)
        return

    args.waypoints.parent.mkdir(parents=True, exist_ok=True)
    with args.waypoints.open("w") as f:
        f.write("# deg, 7 joints per line\n")
        for p in pts:
            f.write(" ".join(f"{x:.6f}" for x in p) + "\n")
    print(f"saved {len(pts)} → {args.waypoints}")
    recover_idle(robot, obj, state_map, error_dict)


def enter_imp(robot, obj, idx, vel, acc, k, d):
    """example_basic_ImpJoint: switch once, print sg readback."""
    ret = robot.switch_to_imp_joint_mode(obj, 2000, vel, acc, k, d)
    if ret != 0:
        raise RuntimeError(f"ImpJoint failed: {robot._get_operate_error_msg(ret)}")
    # Example relies on vel/acc passed into SwitchToImpJointMode (not stream spam).
    robot.runtime_set_speed_ratio(obj, vel, acc)
    sg = robot.get_sg_dict()["arms"][idx]["set"]
    print(f"ImpJoint vel={sg['vel_ratio']} acc={sg['acc_ratio']} "
          f"k={sg['joint_k']} d={sg['joint_d']}")
    return sg


def cmd_play_ptp(args, robot, FXObjType, state_map, error_dict):
    """Point-to-point: same motion pattern as example_basic_ImpJoint."""
    obj, idx = arm_obj(args.arm, FXObjType)
    wps = read_waypoints(args.waypoints)
    k = [args.k] * 7 if args.k is not None else list(K_IMP)
    d = [args.d] * 7 if args.d is not None else list(D_IMP)

    print(f"style=ptp waypoints={len(wps)} vel={args.vel} acc={args.acc} tol={args.tol}")
    recover_idle(robot, obj, state_map, error_dict)
    enter_imp(robot, obj, idx, args.vel, args.acc, k, d)

    rows = []
    for i, target in enumerate(wps):
        print(f"\n### goto wp[{i}] {[round(x, 2) for x in target]}")
        t_arrive, fb = go_to_wait(robot, obj, idx, target, args.tol, args.timeout)
        err = [abs(fb[j] - target[j]) for j in range(7)]
        err_max = max(err)
        print(f"  arrived in {t_arrive:.3f}s  err_max={err_max:.3f}deg  "
              f"fb={[round(x, 2) for x in fb]}")
        rows.append([i, f"{t_arrive:.4f}", f"{err_max:.4f}"]
                    + [f"{x:.4f}" for x in target] + [f"{x:.4f}" for x in fb])
        # time.sleep(0.5)  # example pauses between targets 再等0.5s

    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["i", "t_arrive_s", "err_max_deg"]
                   + [f"cmd{j}" for j in range(7)] + [f"fb{j}" for j in range(7)])
        w.writerows(rows)

    ts = [float(r[1]) for r in rows]
    es = [float(r[2]) for r in rows]
    print("\n--- summary (ptp / example-style) ---")
    print(f"points={len(rows)}  t_arrive: mean={sum(ts)/len(ts):.3f}s max={max(ts):.3f}s")
    print(f"err_max_deg: mean={sum(es)/len(es):.3f} max={max(es):.3f}")
    print(f"log → {args.log}")

    ret = robot.switch_to_idle(obj, 2000)
    if ret != 0:
        print(f"warn IDLE: {robot._get_operate_error_msg(ret)}")


def cmd_play_stream(args, robot, FXObjType, state_map, error_dict):
    """Optional stress: dense stream without wait (policy-like). Expect lag."""
    obj, idx = arm_obj(args.arm, FXObjType)
    wps = read_waypoints(args.waypoints)
    traj = interpolate(wps, args.seg_time, args.dt)
    k = [args.k] * 7 if args.k is not None else list(K_IMP)
    d = [args.d] * 7 if args.d is not None else list(D_IMP)

    print(f"style=stream waypoints={len(wps)} traj={len(traj)} "
          f"seg_time={args.seg_time}s dt={args.dt}s vel={args.vel} acc={args.acc}")
    print("NOTE: stream does NOT wait for arrival — Imp will lag if too fast.")

    recover_idle(robot, obj, state_map, error_dict)
    enter_imp(robot, obj, idx, args.vel, args.acc, k, d)

    # Reach first waypoint before stream (example-style wait).
    print("go to wp[0] before stream...")
    go_to_wait(robot, obj, idx, wps[0], args.tol, args.timeout)
    time.sleep(0.3)

    rows = []
    late = 0
    t0 = time.perf_counter()
    for i, cmd in enumerate(traj):
        t_cmd = i * args.dt
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
        err_max = max(abs(fb[j] - cmd[j]) for j in range(7))
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
        w.writerow(["i", "t_cmd", "err_max_deg", "fb_vel_max", "arrived"]
                   + [f"cmd{j}" for j in range(7)] + [f"fb{j}" for j in range(7)])
        w.writerows(rows)

    errs = [float(r[2]) for r in rows]
    vels = [float(r[3]) for r in rows]
    print("\n--- summary (stream) ---")
    print(f"steps={len(rows)} wall={elapsed:.2f}s expected={len(traj)*args.dt:.2f}s")
    print(f"late={late}/{len(rows)} ({100.0*late/max(len(rows),1):.1f}%)")
    if errs:
        print(f"err_max_deg: mean={sum(errs)/len(errs):.3f} max={max(errs):.3f}")
        print(f"fb_vel_max:  mean={sum(vels)/len(vels):.3f} max={max(vels):.3f}")
    print(f"log → {args.log}")

    ret = robot.switch_to_idle(obj, 2000)
    if ret != 0:
        print(f"warn IDLE: {robot._get_operate_error_msg(ret)}")


def main():
    p = argparse.ArgumentParser(description="ImpJoint waypoint test (example-style)")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--collect", action="store_true")
    g.add_argument("--play", action="store_true")
    p.add_argument("--style", choices=("ptp", "stream"), default="ptp",
                   help="ptp=wait each waypoint (official); stream=timed dense cmds")
    p.add_argument("--arm", choices=("left", "right"), default="right")
    p.add_argument("--profile", choices=("orin", "thor"), default="orin")
    p.add_argument("--ip", default=None)
    p.add_argument("--waypoints", type=Path, default=HERE / "waypoints.txt")
    p.add_argument("--log", type=Path, default=HERE / "play_log.csv")
    p.add_argument("--vel", type=float, default=10.0, help="vel ratio %% (example uses 10)")
    p.add_argument("--acc", type=float, default=10.0, help="acc ratio %% (example uses 10)")
    p.add_argument("--k", type=float, default=None, help="uniform K; default=example [3,3,3,2,1,1,1]")
    p.add_argument("--d", type=float, default=None, help="uniform D; default=example 0.2")
    p.add_argument("--tol", type=float, default=5.0,
                   help="arrive tol deg: |fb-target| per joint must be < tol (default 5)")
    p.add_argument("--timeout", type=float, default=60.0, help="per-waypoint timeout s")
    p.add_argument("--dt", type=float, default=0.02, help="stream only: cmd period s")
    p.add_argument("--seg-time", type=float, default=1.0, help="stream only: s per segment")
    args = p.parse_args()
    args.ip = args.ip or PROFILE_IP[args.profile]

    GentoRobot, FXLogMask, FXObjMask, FXObjType, state_map, error_dict = _setup_sdk()
    robot = GentoRobot()
    if args.play:
        start_estop(robot, FXObjMask)
        print("play: Enter = E-STOP")

    try:
        print(f"link {args.ip} arm={args.arm} ...")
        ret = robot.link(*parse_ip(args.ip), log_level=FXLogMask.FX_LOG_INFO_FLAG)
        if not robot._connected:
            raise RuntimeError(f"link failed: {robot._get_operate_error_msg(ret)}")
        print(f"sdk={robot.get_sdk_version()} ctrl={robot.get_controller_version()}")

        if args.collect:
            cmd_collect(args, robot, FXObjType, state_map, error_dict)
        elif args.style == "stream":
            cmd_play_stream(args, robot, FXObjType, state_map, error_dict)
        else:
            cmd_play_ptp(args, robot, FXObjType, state_map, error_dict)
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
