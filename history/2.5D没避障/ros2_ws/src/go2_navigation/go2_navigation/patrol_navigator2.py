#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json
import time

class PatrolNavigator(Node):
    def __init__(self):
        super().__init__('patrol_navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 读取记录的路径点
        with open('saved_path.json', 'r') as f:
            self.waypoints = json.load(f)
            
        self.current_wp_index = 0
        self.get_logger().info(f'成功加载 {len(self.waypoints)} 个路径点，准备开始循环循迹...')
        
        # 建立连接后开始循环
        self.action_client.wait_for_server()
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if not self.waypoints:
            return

        wp = self.waypoints[self.current_wp_index]
        goal_msg = NavigateToPose.Goal()
        
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
        
        goal_msg.pose = pose
        self.get_logger().info(f'前往目标点 {self.current_wp_index + 1}/{len(self.waypoints)}')
        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标点被拒绝！可能被障碍物完全卡死。')
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # 无论成功还是失败，稍微停顿后前往下一个点
        time.sleep(1.0) 
        
        # 循环递增索引
        self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
        self.send_next_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
