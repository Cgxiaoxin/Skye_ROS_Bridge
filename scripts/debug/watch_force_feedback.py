#!/usr/bin/env python3
"""Subscribe only: inspect force feedback and save a JSON summary (no SDK calls)."""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time


@dataclass
class Stream:
    size: int
    names: list[str] | None = None
    count: int = 0
    invalid: int = 0
    unchanged: int = 0
    first_rx: float | None = None
    last_rx: float | None = None
    last_valid_rx: float | None = None
    latest: list[float] | None = None
    minimum: list[float] = field(default_factory=list)
    maximum: list[float] = field(default_factory=list)
    stamp: dict | None = None
    frame_id: str = ''
    error: str = ''

    def observe(self, values, names, stamp, frame_id, now):
        self.count += 1
        if self.first_rx is None:
            self.first_rx = now
        self.last_rx = now
        values = list(values)
        if len(values) != self.size or not all(math.isfinite(v) for v in values):
            self.invalid += 1
            self.error = f'expected {self.size} finite values, got {len(values)}'
            return
        if self.names is not None and list(names) != self.names:
            self.invalid += 1
            self.error = 'unexpected joint names/order'
            return
        self.unchanged += int(self.latest == values)
        self.latest = values
        self.last_valid_rx = now
        self.error = ''
        self.stamp = {'sec': stamp.sec, 'nanosec': stamp.nanosec}
        self.frame_id = frame_id
        if not self.minimum:
            self.minimum = values.copy()
            self.maximum = values.copy()
        else:
            self.minimum = [min(a, b) for a, b in zip(self.minimum, values)]
            self.maximum = [max(a, b) for a, b in zip(self.maximum, values)]

    def summary(self, now, stale_after):
        age = None if self.last_valid_rx is None else now - self.last_valid_rx
        interval = 0 if self.first_rx is None else self.last_rx - self.first_rx
        hz = (self.count - 1) / interval if interval > 0 else 0.0
        if self.count == 0:
            status = 'WAIT'
        elif self.error:
            status = 'INVALID'
        elif age is None or age > stale_after:
            status = 'STALE'
        elif all(value == 0 for value in self.latest):
            status = 'ALL_ZERO'
        else:
            status = 'RECEIVING'
        return dict(status=status, received=self.count, invalid=self.invalid,
                    unchanged=self.unchanged, receive_hz=hz, valid_age_s=age,
                    latest=self.latest, minimum=self.minimum, maximum=self.maximum,
                    names=self.names, stamp=self.stamp, frame_id=self.frame_id,
                    last_error=self.error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30.0,
                        help='seconds to observe; 0 = until Ctrl+C (default: 30)')
    parser.add_argument('--stale-after', type=float, default=0.5)
    parser.add_argument('--output', type=Path,
                        help='JSON summary path (default: timestamped file in /tmp)')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and >= 0')
    if not math.isfinite(args.stale_after) or args.stale_after <= 0:
        parser.error('--stale-after must be finite and > 0')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import WrenchStamped
    from std_msgs.msg import Int16MultiArray

    started = datetime.now(timezone.utc)
    output = args.output or Path('/tmp') / f'skye_force_feedback_{started:%Y%m%dT%H%M%S%fZ}.json'
    streams = {}
    kinds = {}
    for side, prefix in [('left', 'l'), ('right', 'r')]:
        names = [f'{prefix}_j{i}' for i in range(1, 8)]
        for suffix in ('joint_states', 'joint_external_torque'):
            topic = f'/gento/{side}_{suffix}'
            streams[topic] = Stream(7, names)
            kinds[topic] = JointState
        topic = f'/gento/{side}_flange_wrench'
        streams[topic] = Stream(6)
        kinds[topic] = WrenchStamped
    streams['/gento/joint_states'] = Stream(
        14, [f'{side}_j{i}' for side in ('l', 'r') for i in range(1, 8)])
    kinds['/gento/joint_states'] = JointState
    modes = {'value': None, 'received_at': None}
    rclpy.init()
    node = Node('watch_force_feedback')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    def callback_for(topic):
        def receive(msg):
            if kinds[topic] is WrenchStamped:
                f, t = msg.wrench.force, msg.wrench.torque
                values, names = [f.x, f.y, f.z, t.x, t.y, t.z], None
            else:
                values, names = msg.effort, msg.name
            streams[topic].observe(values, names, msg.header.stamp,
                                   msg.header.frame_id, time.monotonic())
        return receive

    def on_mode(msg):
        modes.update(value=list(msg.data), received_at=time.monotonic())

    subscriptions = [node.create_subscription(kinds[t], t, callback_for(t), qos)
                     for t in streams]
    subscriptions.append(node.create_subscription(
        Int16MultiArray, '/gento/robot_state', on_mode, qos))
    start = time.monotonic()
    next_print = start
    print('Subscribe only. Values are SDK passthrough; units/axes need hardware confirmation.')
    print('Wrench order: Fx Fy Fz Tx Ty Tz. ALL_ZERO/unchanged do not establish sensor validity.')
    print(f'JSON summary: {output}')
    try:
        while rclpy.ok() and (args.duration == 0 or time.monotonic() - start < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now < next_print:
                continue
            next_print = now + 1.0
            mode_age = None if modes['received_at'] is None else round(now - modes['received_at'], 3)
            print(f'\n{now - start:.1f}s mode[L,R]={modes["value"]} mode_age={mode_age}s (Idle=0)')
            for topic, stream in streams.items():
                data = stream.summary(now, args.stale_after)
                values = '---' if stream.latest is None else ' '.join(f'{v:.4f}' for v in stream.latest)
                age = '---' if data['valid_age_s'] is None else f'{data["valid_age_s"]:.3f}s'
                print(f'{topic}: {data["status"]} rx={data["receive_hz"]:.1f}Hz '
                      f'age={age} bad={stream.invalid} repeated={stream.unchanged} [{values}]')
    except KeyboardInterrupt:
        pass
    finally:
        now = time.monotonic()
        report = dict(started_utc=started.isoformat(), duration_s=now - start,
                      robot_state=modes['value'],
                      robot_state_age_s=None if modes['received_at'] is None else now - modes['received_at'],
                      topics={t: s.summary(now, args.stale_after) for t, s in streams.items()})
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(f'Saved: {output}')


if __name__ == '__main__':
    main()
