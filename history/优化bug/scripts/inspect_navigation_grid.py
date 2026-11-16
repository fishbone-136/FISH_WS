#!/usr/bin/env python3
"""Print compact occupancy statistics for one live navigation grid."""

import math
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class GridInspector(Node):
    def __init__(self, topic):
        super().__init__("navigation_grid_inspector")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.topic = topic
        self.done = False
        self.create_subscription(OccupancyGrid, topic, self.on_grid, qos)

    def on_grid(self, message):
        width = int(message.info.width)
        height = int(message.info.height)
        resolution = float(message.info.resolution)
        values = list(message.data)
        unknown = sum(value == -1 for value in values)
        free = sum(value == 0 for value in values)
        # nav_msgs/OccupancyGrid normalizes both Nav2 costmaps and terrain
        # grids to [-1, 100].
        lethal_threshold = 100
        lethal_cells = [index for index, value in enumerate(values)
                        if value >= lethal_threshold]
        cost = len(values) - unknown - free - len(lethal_cells)
        print(
            f"topic={self.topic} frame={message.header.frame_id} "
            f"size={width}x{height} resolution={resolution:.3f}m "
            f"free={free} cost={cost} lethal={len(lethal_cells)} unknown={unknown}")
        if lethal_cells:
            xs = [index % width for index in lethal_cells]
            ys = [index // width for index in lethal_cells]
            origin_x = message.info.origin.position.x
            origin_y = message.info.origin.position.y
            print(
                f"lethal_bounds x=[{origin_x + min(xs) * resolution:.2f},"
                f"{origin_x + (max(xs) + 1) * resolution:.2f}]m "
                f"y=[{origin_y + min(ys) * resolution:.2f},"
                f"{origin_y + (max(ys) + 1) * resolution:.2f}]m")
        self.done = True


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/global_costmap/costmap"
    rclpy.init()
    node = GridInspector(topic)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
