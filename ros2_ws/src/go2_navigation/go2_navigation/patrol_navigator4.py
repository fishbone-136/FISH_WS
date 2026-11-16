#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import json

class PatrolNavigator(Node):
    def __init__(self):
        super().__init__('patrol_navigator')
        
        # �� 核心修改：使用大写 F 的 /FollowWaypoints 匹配宇树的底层服务器
        self.action_client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')
        
        # 读取记录的路径点
        try:
            with open('saved_path.json', 'r') as f:
                self.waypoints = json.load(f)
        except FileNotFoundError:
            self.get_logger().error('当前目录下找不到 saved_path.json 文件！')
            return
            
        self.get_logger().info(f'成功加载 {len(self.waypoints)} 个路径点，准备开始批量循迹...')
        
        # 增加超时打印机制，防止程序默默死等卡住
        self.get_logger().info('正在等待 /FollowWaypoints 动作服务器上线...')
        while not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('仍在等待 /FollowWaypoints，请检查 Nav2 是否正常运行...')
            
        self.get_logger().info('✅ 动作服务器已成功连接！开始下发巡航任务...')
        self.send_route()

    def send_route(self):
        if not getattr(self, 'waypoints', None):
            return

        goal_msg = FollowWaypoints.Goal()
        poses = []
        
        # 将所有的路径点一次性打包成数组
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = wp['qx']
            pose.pose.orientation.y = wp['qy']
            pose.pose.orientation.z = wp['qz']
            pose.pose.orientation.w = wp['qw']
            poses.append(pose)
            
        goal_msg.poses = poses
        self.get_logger().info('已发送完整路线，交由 waypoint_follower 统一处理...')
        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 路线被底层服务器拒绝！请检查起点是否合法或有无障碍物。')
            return

        self.get_logger().info('✅ 路线已被接受，机器人开始移动...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # 移除了 time.sleep() 阻塞，实现走完一圈瞬间无缝衔接下一圈
        self.get_logger().info('�� 一轮巡航结束，立刻开始下一轮...')
        self.send_route()

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，程序退出。')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
