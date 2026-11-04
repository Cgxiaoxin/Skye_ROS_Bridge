#!/usr/bin/env python3
"""Scan Dynamixel FTDI buses, bind left/right, write host-package yaml.

Usage (Orin / Thor host, no Docker):
  export ROBOT_PROFILE=orin   # or thor
  python3 scripts/bind_leader_arms.py
  # exactly 2 buses: unplug LEFT USB when prompted
  # >2 buses: pick left/right index

  python3 scripts/bind_leader_arms.py --dry-run
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path

BAUD = 4_000_000
BY_ID = Path("/dev/serial/by-id")
PORT_LINE = re.compile(
    r'^(?P<prefix>\s*dynamixel_port:\s*)'
    r'(?P<value>"[^"]+"|\S+)'
    r'(?P<suffix>\s*(?:#.*)?)?$',
    re.MULTILINE,
)


def repo_root() -> Path:
    here = Path(__file__).resolve().parent
    root = here.parent
    if (root / "skye_ros2_ws").is_dir() or (root / "marvin_ws").is_dir():
        return root
    if Path("/marvin_ws").is_dir():
        return Path("/")
    sys.exit("找不到仓库根目录（需要 skye_ros2_ws 或 marvin_ws）")


def resolve_profile() -> str:
    profile = os.environ.get(
        "ROBOT_PROFILE", os.environ.get("MARVIN_PROFILE", "thor")
    ).strip().lower()
    if profile not in ("thor", "orin"):
        profile = "thor"
    return profile


def host_config_paths(root: Path, profile: str) -> tuple[Path, Path] | None:
    pkg = root / "skye_ros2_ws" / "src" / "skye_factr_teleop" / "config"
    if not pkg.is_dir():
        return None
    if profile == "orin":
        left = pkg / "orin" / "grav_comp_m6_left.yaml"
        right = pkg / "orin" / "grav_comp_m6_right.yaml"
    else:
        left = pkg / "grav_comp_m6_left.yaml"
        right = pkg / "grav_comp_m6_right.yaml"
    return left, right


def host_install_paths(root: Path, profile: str) -> tuple[Path, Path] | None:
    share = (
        root
        / "skye_ros2_ws"
        / "install"
        / "skye_factr_teleop"
        / "share"
        / "skye_factr_teleop"
        / "config"
    )
    if not share.is_dir():
        return None
    if profile == "orin":
        left = share / "orin" / "grav_comp_m6_left.yaml"
        right = share / "orin" / "grav_comp_m6_right.yaml"
    else:
        left = share / "grav_comp_m6_left.yaml"
        right = share / "grav_comp_m6_right.yaml"
    return left, right


def marvin_config_paths(root: Path, profile: str) -> tuple[Path, Path] | None:
    marvin = root / "marvin_ws" if (root / "marvin_ws").is_dir() else Path("/marvin_ws")
    if not marvin.is_dir():
        return None
    cfg = marvin / "configs" / profile
    left = cfg / "grav_comp_m6_left.yaml"
    right = cfg / "grav_comp_m6_right.yaml"
    return left, right


def list_ftdi() -> list[str]:
    if not BY_ID.is_dir():
        return []
    return sorted(p.name for p in BY_ID.iterdir() if "FTDI" in p.name)


def probe(name: str) -> bool:
    from dynamixel_sdk import PacketHandler, PortHandler

    port = str(BY_ID / name)
    pk = PortHandler(port)
    try:
        if not pk.openPort() or not pk.setBaudRate(BAUD):
            return False
        ph = PacketHandler(2.0)
        for i in range(1, 9):
            _, res, _ = ph.ping(pk, i)
            if res == 0:
                return True
        return False
    except Exception:
        return False
    finally:
        try:
            pk.closePort()
        except Exception:
            pass


def scan() -> list[str]:
    names = list_ftdi()
    print(f"发现 FTDI {len(names)} 个: {names or '(无)'}")
    ok = []
    for n in names:
        good = probe(n)
        print(f"  {'OK' if good else '--'}  {n}")
        if good:
            ok.append(n)
    return ok


def pick_two(cands: list[str]) -> tuple[str, str]:
    if len(cands) < 2:
        sys.exit(f"需要至少 2 路 Dynamixel 总线，当前 {len(cands)}")
    if len(cands) > 2:
        print("\n多套并存，请选择:")
        for i, n in enumerate(cands):
            print(f"  [{i}] {n}")
        li = int(input("左臂 index: ").strip())
        ri = int(input("右臂 index: ").strip())
        if li == ri or not (0 <= li < len(cands) and 0 <= ri < len(cands)):
            sys.exit("左右 index 非法")
        return cands[li], cands[ri]

    print("\n请拔掉【左臂】USB，拔完后回车…")
    input()
    for _ in range(20):
        now = set(list_ftdi())
        gone = [n for n in cands if n not in now]
        if len(gone) == 1:
            left = gone[0]
            right = cands[0] if cands[1] == left else cands[1]
            print(f"左={left}\n右={right}")
            print("请插回左臂 USB，插完后回车…")
            input()
            return left, right
        time.sleep(0.25)
    sys.exit("未检测到拔出变化，请重试")


def patch_dynamixel_port(path: Path, name: str) -> bool:
    if not path.is_file():
        return False
    text = path.read_text()

    def repl(match: re.Match[str]) -> str:
        prefix = match.group("prefix")
        value = match.group("value")
        suffix = match.group("suffix") or ""
        quoted = value.startswith('"')
        new_value = f'"{name}"' if quoted else name
        return f"{prefix}{new_value}{suffix}"

    new, n = PORT_LINE.subn(repl, text, count=1)
    if n:
        path.write_text(new)
    return n > 0


def write_env(path: Path, left: str, right: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "# auto-generated by bind_leader_arms.py\n"
        f'export ROBOT_LEADER_DYNAMIXEL_PORT_LEFT="{left}"\n'
        f'export ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT="{right}"\n'
    )


def write_yaml_map(path: Path, left: str, right: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"left_by_id: {left}\nright_by_id: {right}\n")


def set_latency(names: list[str]) -> None:
    """拔插 USB 后 latency 会恢复默认，必须重新写成 1。"""
    for name in names:
        link = BY_ID / name
        if not link.exists():
            print(f"  skip latency: {name} 不存在")
            continue
        tty = os.path.basename(os.path.realpath(link))
        path = f"/sys/bus/usb-serial/devices/{tty}/latency_timer"
        try:
            with open(path, "w", encoding="ascii") as f:
                f.write("1\n")
            with open(path, encoding="ascii") as f:
                val = f.read().strip()
            print(f"  latency {tty}={val}")
        except OSError as e:
            print(f"  latency 失败 {tty}: {e}（可手动: echo 1 | sudo tee {path}）")


def collect_targets(
    root: Path, profile: str
) -> list[tuple[str, Path, Path]]:
    """Return labeled (left_yaml, right_yaml) pairs to patch."""
    targets: list[tuple[str, Path, Path]] = []
    host = host_config_paths(root, profile)
    if host and host[0].is_file() and host[1].is_file():
        targets.append(("skye_factr_teleop src", host[0], host[1]))
    install = host_install_paths(root, profile)
    if install and install[0].is_file() and install[1].is_file():
        # Avoid double-writing when install is a symlink into src.
        if not host or (
            install[0].resolve() != host[0].resolve()
            or install[1].resolve() != host[1].resolve()
        ):
            targets.append(("skye_factr_teleop install", install[0], install[1]))
    marvin = marvin_config_paths(root, profile)
    if marvin and (marvin[0].is_file() or marvin[1].is_file()):
        targets.append(("marvin_ws configs", marvin[0], marvin[1]))
    return targets


def apply(root: Path, left: str, right: str, dry: bool) -> None:
    profile = resolve_profile()
    targets = collect_targets(root, profile)
    marvin = root / "marvin_ws" if (root / "marvin_ws").is_dir() else Path("/marvin_ws")
    env = marvin / ".skye" / "leader_arms.env"
    yml = marvin / ".skye" / "leader_arms.yaml"

    print("\n将写入:")
    print(f"  profile={profile}")
    if marvin.is_dir():
        print(f"  {env}")
        print(f"  {yml}")
    if not targets:
        sys.exit(
            f"未找到可写配置（profile={profile}）。"
            "确认 skye_ros2_ws/src/skye_factr_teleop/config 存在。"
        )
    for label, left_cfg, right_cfg in targets:
        print(f"  [{label}] {left_cfg}")
        print(f"  [{label}] {right_cfg}")
    if dry:
        print("[dry-run] 未写盘")
        return

    if marvin.is_dir():
        write_env(env, left, right)
        write_yaml_map(yml, left, right)

    for label, left_cfg, right_cfg in targets:
        for path, port in ((left_cfg, left), (right_cfg, right)):
            ok = patch_dynamixel_port(path, port)
            print(f"  {'OK' if ok else 'SKIP'}  [{label}] {path.name} -> {port}")

    print("设置 latency_timer=1 …")
    set_latency([left, right])

    print("\n验证 dynamixel_port:")
    for label, left_cfg, right_cfg in targets:
        for path in (left_cfg, right_cfg):
            if not path.is_file():
                continue
            for line in path.read_text().splitlines():
                if "dynamixel_port" in line:
                    print(f"  [{label}] {path.name}: {line.strip()}")
                    break

    print("\n已写入。重启小臂 launch 即可（robot_profile 须与绑定一致）:")
    print(f"  export ROBOT_PROFILE={profile}")
    print(
        "  ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py "
        f"robot_profile:={profile}"
    )
    if marvin.is_dir():
        print(f"  # 旧 Docker 流程另: source {env}")


def main() -> None:
    ap = argparse.ArgumentParser(description="绑定左右主臂 FTDI 串口")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    root = repo_root()
    print(f"ROBOT_PROFILE={resolve_profile()}")
    cands = scan()
    left, right = pick_two(cands)
    apply(root, left, right, args.dry_run)


if __name__ == "__main__":
    main()
