#!/usr/bin/env python3
"""Print one-frame range/height statistics for the live HAP obstacle pipeline."""

import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def percentile(values, fraction):
    if not values:
        return math.nan
    index = min(len(values) - 1, int(round(fraction * (len(values) - 1))))
    return values[index]


class CloudInspector(Node):
    def __init__(self, topic):
        super().__init__("obstacle_cloud_inspector")
        self.done = False
        self.create_subscription(
            PointCloud2, topic, self.on_cloud,
            qos_profile_sensor_data)

    def on_cloud(self, message):
        points = []
        for x, y, z in point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True):
            values = (float(x), float(y), float(z))
            if all(math.isfinite(value) for value in values):
                points.append(values)
        if not points:
            print("No finite points received")
            self.done = True
            return

        ranges = sorted(math.hypot(x, y) for x, y, _ in points)
        heights = sorted(z for _, _, z in points)
        obstacle_slice = [
            math.hypot(x, y) for x, y, z in points if -0.33 <= z <= 0.37]
        print(
            f"frame={message.header.frame_id} points={len(points)} "
            f"range[m] p50={percentile(ranges, 0.50):.2f} "
            f"p90={percentile(ranges, 0.90):.2f} "
            f"p99={percentile(ranges, 0.99):.2f} max={ranges[-1]:.2f}")
        print(
            f"z[m] min={heights[0]:.2f} p10={percentile(heights, 0.10):.2f} "
            f"p50={statistics.median(heights):.2f} "
            f"p90={percentile(heights, 0.90):.2f} max={heights[-1]:.2f}")
        print(
            f"current_slice_points={len(obstacle_slice)} "
            f"within_5.5m={sum(value <= 5.5 for value in obstacle_slice)}")
        bins = [(-0.50, -0.35), (-0.35, -0.30), (-0.30, -0.20),
                (-0.20, 0.00), (0.00, 0.20), (0.20, 0.40), (0.40, 0.80)]
        print("z_bins=" + " ".join(
            f"[{lower:.2f},{upper:.2f})="
            f"{sum(lower <= z < upper and math.hypot(x, y) <= 5.5 for x, y, z in points)}"
            for lower, upper in bins))
        self.done = True


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/cloud_registered_body"
    rclpy.init()
    node = CloudInspector(topic)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
