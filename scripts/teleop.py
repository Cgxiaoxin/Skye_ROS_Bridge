#!/usr/bin/env python3
"""Start native FACTR teleop with independent follower and USB leader selection."""

import argparse
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / "skye_ros2_ws"
PACKAGE = WS / "src/skye_factr_teleop"
BY_ID = Path("/dev/serial/by-id")
SYS_TTY = Path("/sys/bus/usb-serial/devices")


def robot_profile(explicit=None):
    value = explicit or os.environ.get("ROBOT_PROFILE")
    if not value:
        marvin = Path(os.environ.get("MARVIN_WS", str(ROOT / "marvin_ws")))
        for path in (marvin / ".skye/robot_profile", marvin / "robot_profile"):
            if path.is_file():
                value = path.read_text().strip()
                break
    value = (value or "").lower()
    if value not in ("thor", "orin"):
        raise RuntimeError("先执行 bash scripts/teleop.sh configure thor（或 orin），或传入 --robot。")
    return value


def select_leaders(side, device="auto"):
    # Historical directories describe the leader hardware, not the host/follower.
    import yaml

    directories = {"a": PACKAGE / "config", "b": PACKAGE / "config/orin"}
    result = {}
    for arm in (("left", "right") if side == "dual" else (side,)):
        candidates = []
        expected = []
        for name, directory in directories.items():
            if device != "auto" and device != name:
                continue
            config = directory / f"grav_comp_m6_{arm}.yaml"
            document = yaml.safe_load(config.read_text())
            port = BY_ID / document["dynamixel"]["dynamixel_port"]
            expected.append(str(port))
            if port.exists():
                candidates.append((name, config, port))
        if len(candidates) != 1:
            raise RuntimeError(
                f"{arm}: 检测到 {len(candidates)} 个匹配串口。"
                "同时接入两套时用 --device a 或 --device b 选择。\n"
                + "\n".join(expected))
        result[arm] = candidates[0]
    return result


def ros_environment(overlay=True):
    setup = WS / "install/setup.bash"
    if overlay and not setup.is_file():
        raise RuntimeError("尚未编译：请先执行 bash scripts/teleop.sh build")
    env = dict(os.environ)
    env["PATH"] = "/usr/bin:" + env.get("PATH", "")
    result = subprocess.run(
        ["bash", "-c",
         'source /opt/ros/humble/setup.bash >&2 && '
         'if [[ -n "$1" ]]; then source "$1" >&2 || exit; fi && env -0',
         "teleop-env", str(setup) if overlay else ""],
        env=env, stdout=subprocess.PIPE, check=True)
    return dict(item.decode().split("=", 1) for item in result.stdout.split(b"\0") if item)


def validate_configs(leaders):
    sys.path.insert(0, str(PACKAGE))
    from skye_factr_teleop.config import load_config

    for _, config, _ in leaders.values():
        load_config(config, require_runtime=True)


def check_ports(leaders, fix_latency=False):
    if not shutil.which("fuser"):
        raise RuntimeError("缺少 fuser，请安装 psmisc。")
    devices = []
    for side, (_, _, port) in leaders.items():
        tty = port.resolve(strict=True)
        if not tty.is_char_device():
            raise RuntimeError(f"{side}: 不是字符设备：{tty}")
        if not os.access(tty, os.R_OK | os.W_OK):
            raise RuntimeError(f"{side}: 当前用户缺少串口读写权限：{tty}，请核对 dialout 组。")
        occupied = subprocess.run(["fuser", str(tty)], capture_output=True)
        if occupied.returncode != 1:
            raise RuntimeError(f"{side}: 串口占用或检查失败：{tty}\n"
                               + occupied.stdout.decode() + occupied.stderr.decode())
        latency = SYS_TTY / tty.name / "latency_timer"
        value = latency.read_text().strip()
        if value != "1" and not fix_latency:
            raise RuntimeError(f"{side}: latency_timer={value}，请执行 check --fix-latency。")
        devices.append((side, tty, latency, value))
    # Resolve and check both buses before changing either latency timer.
    if len({tty for _, tty, _, _ in devices}) != len(devices):
        raise RuntimeError("左右侧解析到了同一个串口。")
    for side, tty, latency, value in devices:
        if value != "1":
            command = ["tee", str(latency)]
            if not os.access(latency, os.W_OK):
                command.insert(0, "sudo")
            subprocess.run(command, input="1\n", text=True, stdout=subprocess.DEVNULL, check=True)
        if latency.read_text().strip() != "1":
            raise RuntimeError(f"{side}: latency_timer 未读回 1：{latency}")
        print(f"  {side}: {tty}, latency_timer=1", flush=True)


def launch_command(profile, side, leaders):
    command = ["ros2", "launch", str(ROOT / "scripts/teleop_system.launch.py"),
               f"robot_profile:={profile}", f"side:={side}",
               f"driver_config:={WS / 'src/skye_robot_driver/config/skye_robot.yaml'}"]
    command.extend(f"{arm}_config:={config}" for arm, (_, config, _) in leaders.items())
    return command


def shell_environment(profile, domain, leaders):
    setup = WS / "install/setup.bash"
    if not setup.is_file():
        raise RuntimeError("尚未编译：请先执行 bash scripts/teleop.sh build")
    print(f"cd {shlex.quote(str(WS))} || return 1")
    print('export PATH="/usr/bin:$PATH"')
    print("source /opt/ros/humble/setup.bash || return 1")
    print(f"source {shlex.quote(str(setup))} || return 1")
    values = {"ROBOT_PROFILE": profile, "ROS_DOMAIN_ID": str(domain),
              "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "ROS_LOCALHOST_ONLY": "0",
              "DRIVER_CONFIG": str(WS / "src/skye_robot_driver/config/skye_robot.yaml")}
    print("unset LEFT_CONFIG RIGHT_CONFIG")
    values.update({f"{side.upper()}_CONFIG": str(config)
                   for side, (_, config, _) in leaders.items()})
    for key, value in values.items():
        print(f"export {key}={shlex.quote(value)}")


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest="action", required=True)
    configure = commands.add_parser("configure", help="保存本机使用的大臂配置（与 USB 小臂无关）")
    configure.add_argument("profile", choices=("thor", "orin"))
    commands.add_parser("build", help="编译小臂及大臂，不连接硬件")
    for action, help_text in (("check", "检查启动条件"), ("start", "启动大小臂"),
                              ("env", "输出可 source 的 ROS 环境及已识别配置")):
        command = commands.add_parser(action, help=help_text)
        command.add_argument("--robot", choices=("thor", "orin"), help="覆盖本机保存的大臂配置")
        command.add_argument("--side", choices=("dual", "left", "right"), default="dual")
        command.add_argument("--device", choices=("auto", "a", "b"), default="auto", help="USB 小臂，默认自动识别")
        command.add_argument("--domain", type=int, help="默认 Thor=20、Orin=21；支持 ROS_DOMAIN_ID")
        if action == "check":
            command.add_argument("--fix-latency", action="store_true", help="将 FTDI 延迟设置为 1 ms")
        elif action == "start":
            command.add_argument("--dry-run", action="store_true", help="只显示识别结果和启动命令")
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    if args.action == "configure":
        subprocess.run(["bash", str(ROOT / "scripts/set_robot_profile.sh"), args.profile], check=True)
        return 0
    if args.action == "build":
        env = ros_environment(overlay=False)
        subprocess.run(["colcon", "build", "--packages-up-to", "skye_factr_teleop",
                        "--symlink-install"], cwd=WS, env=env, check=True)
        subprocess.run(["colcon", "build", "--packages-select", "skye_robot_driver",
                        "--cmake-args", "-DAMENT_CMAKE_SYMLINK_INSTALL=OFF",
                        "-DPython3_EXECUTABLE=/usr/bin/python3"], cwd=WS, env=env, check=True)
        return 0
    profile = robot_profile(args.robot)
    domain = args.domain
    if domain is None:
        domain = int(os.environ.get("ROS_DOMAIN_ID", "20" if profile == "thor" else "21"))
    if not 0 <= domain <= 232:
        raise RuntimeError("ROS domain 必须在 0..232 内。")
    leaders = select_leaders(args.side, args.device)
    validate_configs(leaders)
    if args.action == "env":
        shell_environment(profile, domain, leaders)
        return 0
    print(f"大臂={profile}  domain={domain}  side={args.side}", flush=True)
    for side, (device, config, port) in leaders.items():
        print(f"  {side}: 小臂 {device.upper()} / {port.name}\n    {config}", flush=True)
    command = launch_command(profile, args.side, leaders)
    if args.action == "start" and args.dry_run:
        print(shlex.join(command))
        return 0
    check_ports(leaders, fix_latency=args.action == "start" or args.fix_latency)
    env = ros_environment()
    env.update(ROBOT_PROFILE=profile, ROS_DOMAIN_ID=str(domain),
               RMW_IMPLEMENTATION="rmw_fastrtps_cpp", ROS_LOCALHOST_ONLY="0")
    # Old Docker shells may export a FastDDS XML path absent on this host.
    for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        if env.get(key) and not Path(env[key]).is_file():
            env.pop(key)
    ip = "6.6.7.191" if profile == "thor" else "6.6.7.190"
    subprocess.run(["ping", "-c", "1", "-W", "1", ip], check=True, stdout=subprocess.DEVNULL)
    existing = subprocess.run(
        ["pgrep", "-f", "lib/skye_robot_driver/[s]kye_robot_driver|[g]ento_robot_driver"],
        capture_output=True)
    if existing.returncode != 1:
        raise RuntimeError("已有大臂驱动运行或进程检查失败，请先正常停止旧驱动。")
    for package, executable in (("skye_robot_driver", "skye_robot_driver"),
                                ("skye_factr_teleop", "skye_factr_teleop_driver")):
        subprocess.run(["ros2", "pkg", "prefix", package], env=env,
                       stdout=subprocess.DEVNULL, check=True)
        # Both packages use isolated installs in this workspace.
        if not (WS / "install" / package / "lib" / package / executable).is_file():
            raise RuntimeError(f"缺少 {executable}，请执行 build。")
    if args.action == "check":
        print("检查通过（配置、串口、延迟、网络及安装入口）。")
        return 0
    print("启动大小臂：大臂将连接控制器；小臂初始 IDLE。同步和遥操需手动触发。", flush=True)
    print("Ctrl+C 统一退出；试验结束请先按 checklist 完成 stop / hold。", flush=True)
    os.chdir(WS)
    os.execvpe(command[0], command, env)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(130)
