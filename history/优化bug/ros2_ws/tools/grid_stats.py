#!/usr/bin/env python3
"""Print compact content statistics for one ROS 2 OccupancyGrid message."""

import argparse
import os

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class GridStats(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("grid_stats")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            OccupancyGrid, topic, self.on_grid, qos)

    def on_grid(self, message: OccupancyGrid) -> None:
        data = np.asarray(message.data, dtype=np.int16)
        known = data[data >= 0]
        print(
            f"frame={message.header.frame_id} "
            f"size={message.info.width}x{message.info.height} "
            f"resolution={message.info.resolution:.3f} "
            f"origin=({message.info.origin.position.x:.3f},"
            f"{message.info.origin.position.y:.3f})"
        )
        print(
            f"cells={data.size} unknown={np.count_nonzero(data < 0)} "
            f"known={known.size} free={np.count_nonzero(data == 0)} "
            f"occupied={np.count_nonzero(data >= 90)}"
        )
        if known.size:
            bins = [0, 1, 10, 25, 50, 75, 90, 100, 101]
            histogram, _ = np.histogram(known, bins=bins)
            print(
                f"known_min={known.min()} known_max={known.max()} "
                f"known_mean={known.mean():.2f}"
            )
            print(
                "cost_bins="
                + ", ".join(
                    f"[{bins[index]},{bins[index + 1]}):{count}"
                    for index, count in enumerate(histogram)
                ),
                flush=True,
            )
        os._exit(0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("topic", nargs="?", default="/traversability/global")
    arguments = parser.parse_args()
    rclpy.init()
    node = GridStats(arguments.topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
