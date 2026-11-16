#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import socket
import json
import threading
import struct
import time
from action_msgs.msg import GoalStatus


class Nav2TcpBridge(Node):
    def __init__(self):
        super().__init__('nav2_tcp_bridge')

        self.host = self.declare_parameter('host', '0.0.0.0').value
        self.port = int(self.declare_parameter('port', 5432).value)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TCP Server
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)

        self.tcp_thread = threading.Thread(target=self.tcp_server_loop)
        self.tcp_thread.daemon = True
        self.tcp_thread.start()

        self.get_logger().info(f'Nav2 TCP Bridge 就绪，端口: {self.port}')

    def tcp_server_loop(self):
        while rclpy.ok():
            self.get_logger().info('等待客户端连接...')
            try:
                client_socket, addr = self.server_socket.accept()
                self.get_logger().info(f'客户端已连接: {addr}')
            except Exception:
                time.sleep(1)
                continue

            try:
                while True:
                    # 接收数据头
                    length_data = client_socket.recv(4)
                    if not length_data: break
                    length = struct.unpack('!I', length_data)[0]

                    # 接收数据体
                    data = b''
                    while len(data) < length:
                        chunk = client_socket.recv(length - len(data))
                        if not chunk: break
                        data += chunk
                    if not data: break

                    # 处理导航请求
                    try:
                        raw_data = json.loads(data.decode())
                        # 统一转为列表
                        goal_queue = raw_data if isinstance(raw_data, list) else [raw_data]

                        self.get_logger().info(f"收到任务序列，共 {len(goal_queue)} 个航点")

                        # === 顺序执行核心逻辑 ===
                        for i, goal_data in enumerate(goal_queue):
                            if not rclpy.ok(): break

                            self.get_logger().info(f"--- 执行第 {i + 1}/{len(goal_queue)} 个点 ---")

                            goal_msg = NavigateToPose.Goal()
                            goal_msg.pose.header.frame_id = "map"
                            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
                            goal_msg.pose.pose.position.x = float(goal_data['position']['x'])
                            goal_msg.pose.pose.position.y = float(goal_data['position']['y'])
                            goal_msg.pose.pose.position.z = float(goal_data['position']['z'])
                            goal_msg.pose.pose.orientation.x = float(goal_data['orientation']['x'])
                            goal_msg.pose.pose.orientation.y = float(goal_data['orientation']['y'])
                            goal_msg.pose.pose.orientation.z = float(goal_data['orientation']['z'])
                            goal_msg.pose.pose.orientation.w = float(goal_data['orientation']['w'])

                            # 同步等待直到到达
                            success = self.execute_goal_sync(goal_msg)

                            if success:
                                self.get_logger().info(f"第 {i + 1} 个点已到达")
                            else:
                                self.get_logger().error(f"第 {i + 1} 个点失败，终止后续任务")
                                break  # 失败则停止

                        self.get_logger().info("本轮任务序列结束")

                    except Exception as e:
                        self.get_logger().error(f'数据处理出错: {e}')

            finally:
                client_socket.close()

    def execute_goal_sync(self, goal_msg):
        """同步执行导航，阻塞直到完成"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server 未连接')
            return False

        send_future = self._action_client.send_goal_async(goal_msg)

        # 等待发送完成
        while not send_future.done():
            time.sleep(0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标点被拒绝')
            return False

        # 等待执行结果
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)  # 可以在此处添加反馈日志

        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED


def main(args=None):
    rclpy.init(args=args)
    node = Nav2TcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
