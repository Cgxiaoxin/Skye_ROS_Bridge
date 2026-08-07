#!/usr/bin/env python3
"""夹爪桥接实机测试 — 连接控制柜 + GripperBridge + ROS2 topic.

优先读取 config.yaml, CLI 参数可覆盖.

用法:
  python3 test/test_gripper_bridge_live.py
  python3 test/test_gripper_bridge_live.py --arm A --rate 50
"""

import signal
import sys
import os
import argparse

import yaml

_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.join(_current_dir, "..", "..")
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

import rclpy
from PYTHON_SDK.GentoRobot import GentoRobot
from robot.gripper_bridge import GripperBridge
from robot.ros_adapter import GripperROSBridge, _get_node

DEFAULT_CONFIG = os.path.join(_parent_dir, "config", "config.yaml")
LEFT_CMD = "/control/gripperValueL"
RIGHT_CMD = "/control/gripperValueR"


def main():
    # ── 加载 config ──
    cfg = {}
    if os.path.exists(DEFAULT_CONFIG):
        with open(DEFAULT_CONFIG) as f:
            cfg = yaml.safe_load(f) or {}

    robot_cfg = cfg.get("robot", {})
    gripper_cfg = cfg.get("gripper", {})
    ctrl_cfg = cfg.get("gripper_control", {})

    # ── CLI (覆盖 config) ──
    p = argparse.ArgumentParser(description="夹爪桥接实机测试")
    p.add_argument("--ip", default=robot_cfg.get("ip", "6.6.7.190"))
    p.add_argument("--arm", default=gripper_cfg.get("arm", "AB"),
                   choices=["A", "B", "AB"])
    p.add_argument("--rate", type=float,
                   default=float(ctrl_cfg.get("rate_hz", 100.0)))
    p.add_argument("--kp", type=float,
                   default=float(ctrl_cfg.get("kp", 3.0)))
    p.add_argument("--kd", type=float,
                   default=float(ctrl_cfg.get("kd", 0.12)))
    args = p.parse_args()

    # ── SDK 连接 ──
    robot = GentoRobot()
    try:
        addr = args.ip.split(".")
        ret = robot.link(*[int(b) for b in addr])
        if ret < 0:
            raise ConnectionError(f"link() 失败: {ret}")
        print(f"已连接控制柜 {args.ip}")
    except Exception as e:
        print(f"连接失败: {e}")
        return 1

    # ── 桥接 ──
    bridge = GripperBridge(
        robot, kp=args.kp, kd=args.kd, rate_hz=args.rate)
    bridge.start()
    print(f"GripperBridge 已启动 (kp={args.kp}, kd={args.kd}, rate={args.rate}Hz)")

    # ── 诊断: 等待并打印左右臂状态 ──
    import time
    print("等待夹爪数据 (2s)...")
    time.sleep(2.0)
    fb_l = bridge.feedback_left
    fb_r = bridge.feedback_right
    print(f"左臂: pos={fb_l.get('pos', float('nan')):.4f}  "
          f"vel={fb_l.get('vel', float('nan')):.4f}  "
          f"torque={fb_l.get('torque', float('nan')):.4f}  "
          f"mos={fb_l.get('temp_mos', 0)}°C  motor={fb_l.get('temp_motor', 0)}°C" +
          (" [离线]" if not fb_l else ""))
    print(f"右臂: pos={fb_r.get('pos', float('nan')):.4f}  "
          f"vel={fb_r.get('vel', float('nan')):.4f}  "
          f"torque={fb_r.get('torque', float('nan')):.4f}  "
          f"mos={fb_r.get('temp_mos', 0)}°C  motor={fb_r.get('temp_motor', 0)}°C" +
          (" [离线]" if not fb_r else ""))
    print()

    # ── 自检: 开合两次 (临时注释) ──
    # import time
    # print("\n=== 夹爪自检: 开合两次 ===")
    # for i in range(2):
    #     print(f"[{i+1}/2] 开爪 ...")
    #     bridge.set_both(0.0)
    #     time.sleep(2.0)
    #     fb_l = bridge.feedback_left.get("pos", float("nan"))
    #     fb_r = bridge.feedback_right.get("pos", float("nan"))
    #     print(f"[{i+1}/2] 反馈: L={fb_l:.4f} R={fb_r:.4f}")
    #     print(f"[{i+1}/2] 闭爪 ...")
    #     bridge.set_both(1.0)
    #     time.sleep(2.0)
    #     fb_l = bridge.feedback_left.get("pos", float("nan"))
    #     fb_r = bridge.feedback_right.get("pos", float("nan"))
    #     print(f"[{i+1}/2] 反馈: L={fb_l:.4f} R={fb_r:.4f}")
    # bridge.set_both(0.0)
    # print("=== 自检完成, 等待 ROS2 topic ===\n")

    # ── ROS2 topic ──
    rclpy.init(args=sys.argv)
    ros_bridge = GripperROSBridge(bridge, arm=args.arm)

    # ── 自检: 开合两次 ──
    import time
    print("\n=== 夹爪自检: 开合两次 ===")
    for i in range(2):
        print(f"[{i+1}/2] 开爪 ...")
        bridge.set_both(0.0)
        time.sleep(2.0)
        fl = bridge.feedback_left; fr = bridge.feedback_right
        print(f"[{i+1}/2] 反馈: L pos={fl.get('pos', float('nan')):.4f} R pos={fr.get('pos', float('nan')):.4f}")
        print(f"[{i+1}/2] 闭爪 ...")
        bridge.set_both(1.0)
        time.sleep(2.0)
        fl = bridge.feedback_left; fr = bridge.feedback_right
        print(f"[{i+1}/2] 反馈: L pos={fl.get('pos', float('nan')):.4f} R pos={fr.get('pos', float('nan')):.4f}")
    bridge.set_both(0.0)
    print("=== 自检完成, 等待 ROS2 topic ===\n")

    print(f"""
╔═════════════════════════════════════════════╗
║  夹爪桥接已就绪 (arm={args.arm})               ║
╠═════════════════════════════════════════════╣
║  订阅: /control/gripperValueL/R             ║
║  发布: /info/gripper_feedback_L/R           ║
║                                             ║
║  开爪:                                       ║
║    ros2 topic pub -1 /control/gripperValueL std_msgs/msg/Float32 "data: 0.0"
║  闭爪:                                       ║
║    ros2 topic pub -1 /control/gripperValueL std_msgs/msg/Float32 "data: 1.0"
║  半开:                                       ║
║    ros2 topic pub -1 /control/gripperValueL std_msgs/msg/Float32 "data: 0.5"
║  右爪:                                       ║
║    ros2 topic pub -1 /control/gripperValueR std_msgs/msg/Float32 "data: 0.5"
║  看反馈:                                     ║
║    ros2 topic echo /info/gripper_feedback_L
╚═════════════════════════════════════════════╝
""")

    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    try:
        rclpy.spin(_get_node())
    except KeyboardInterrupt:
        pass
    finally:
        ros_bridge.shutdown()
        bridge.stop()
        try:
            robot.unlink()
        except Exception:
            pass
        print("夹爪桥接已停止")


if __name__ == "__main__":
    main()
