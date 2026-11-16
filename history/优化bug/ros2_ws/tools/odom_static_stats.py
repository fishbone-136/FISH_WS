#!/usr/bin/env python3
"""Collect a short stationary odometry window and report drift statistics."""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quaternion_angle(a, b):
    dot = abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


class OdomStaticStats(Node):
    def __init__(self, topic):
        super().__init__("odom_static_stats")
        self.samples = []
        self.create_subscription(Odometry, topic, self.callback, 100)

    def callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self.samples.append(
            (
                time.monotonic(),
                (p.x, p.y, p.z),
                (q.x, q.y, q.z, q.w),
                math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z),
                math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z),
            )
        )


def norm_delta(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/Odometry")
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = OdomStaticStats(args.topic)
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    samples = node.samples
    node.destroy_node()
    rclpy.shutdown()
    if len(samples) < 2:
        raise SystemExit(f"insufficient samples: {len(samples)}")

    p0, q0 = samples[0][1], samples[0][2]
    pn, qn = samples[-1][1], samples[-1][2]
    excursions = [norm_delta(p0, sample[1]) for sample in samples]
    angles = [quaternion_angle(q0, sample[2]) for sample in samples]
    linear_speeds = [sample[3] for sample in samples]
    angular_speeds = [sample[4] for sample in samples]
    observed = samples[-1][0] - samples[0][0]

    print(f"samples={len(samples)} duration={observed:.3f}s rate={len(samples) / observed:.2f}Hz")
    print(f"start_position={p0}")
    print(f"end_position={pn}")
    print(f"position_delta={norm_delta(p0, pn):.6f}m max_excursion={max(excursions):.6f}m")
    print(
        f"orientation_delta={math.degrees(quaternion_angle(q0, qn)):.6f}deg "
        f"max_excursion={math.degrees(max(angles)):.6f}deg"
    )
    print(
        f"linear_speed_rms={math.sqrt(sum(v * v for v in linear_speeds) / len(linear_speeds)):.6f}m/s "
        f"max={max(linear_speeds):.6f}m/s"
    )
    print(
        f"angular_speed_rms={math.sqrt(sum(v * v for v in angular_speeds) / len(angular_speeds)):.6f}rad/s "
        f"max={max(angular_speeds):.6f}rad/s"
    )


if __name__ == "__main__":
    main()
