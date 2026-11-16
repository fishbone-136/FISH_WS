#!/usr/bin/env python3
import socket
import json
import struct
import time
import yaml
import os
import threading
import sys

# ROS2 依赖
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class RvizWaypointCollector(Node):
    def __init__(self):
        super().__init__('rviz_waypoint_collector')
        # 监听 Rviz 修改后的话题
        self.subscription = self.create_subscription(
            PoseStamped,
            '/rviz_waypoint',  # 请确保 Rviz 中 2D Nav Goal 的话题已改为这个
            self.goal_callback,
            10
        )
        self.waypoints = []
        self.get_logger().info('=== Rviz 选点模式就绪 ===')
        self.get_logger().info('1. 在 Rviz 中使用 "2D Nav Goal" 点击地图选点')
        self.get_logger().info('   (请确保 Rviz 工具话题已改为 /rviz_waypoint)')
        self.get_logger().info('2. 选完所有点后，在终端按 [Enter] 发送任务')

    def goal_callback(self, msg):
        # 提取坐标和方向
        point = {
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': 0.0
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
        }
        self.waypoints.append(point)
        print(f"\n[已捕获] 第 {len(self.waypoints)} 个目标点: "
              f"x={point['position']['x']:.2f}, y={point['position']['y']:.2f}")
        print("继续选点，或按 Enter 发送...", end='', flush=True)


class TcpClient:
    def __init__(self, config_path=None):
        if config_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(os.path.dirname(current_dir), 'config', 'tcp_config.yaml')

        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            self.host = config['nav_server']['host']
            self.port = config['nav_server']['port']
        except Exception:
            self.host = '127.0.0.1'
            self.port = 5432

        self.socket = None
        self.connected = False

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f'\n已连接到导航服务器 {self.host}:{self.port}')
        except Exception as e:
            print(f'连接失败: {e}')
            self.connected = False

    def send_goals(self, goals_list):
        if not self.connected: return False
        try:
            data = json.dumps(goals_list).encode()
            length = struct.pack('!I', len(data))
            self.socket.sendall(length + data)
            return True
        except Exception as e:
            print(f'发送失败: {e}')
            return False

    def close(self):
        if self.socket: self.socket.close()


def input_thread_func(ros_node, tcp_client):
    """处理用户键盘输入"""
    while rclpy.ok():
        # 阻塞等待用户按回车
        input()

        if not ros_node.waypoints:
            print("当前没有选定任何点，请先在 Rviz 上选点。")
            continue

        print(f"\n正在发送 {len(ros_node.waypoints)} 个目标点到机器人...")

        if tcp_client.connected:
            if tcp_client.send_goals(ros_node.waypoints):
                print(">>> 发送成功！机器人开始巡航。")
                ros_node.waypoints = []  # 清空列表，准备下一次任务
                print("任务队列已清空，可以开始新一轮选点。")
            else:
                print("发送失败，请检查连接。")
        else:
            # 尝试重连
            tcp_client.connect()
            if tcp_client.connected:
                tcp_client.send_goals(ros_node.waypoints)
                ros_node.waypoints = []


def main(args=None):
    rclpy.init(args=args)

    # 初始化 ROS 节点和 TCP 客户端
    node = RvizWaypointCollector()
    client = TcpClient()
    client.connect()

    # 启动键盘监听线程
    input_thread = threading.Thread(target=input_thread_func, args=(node, client))
    input_thread.daemon = True
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()