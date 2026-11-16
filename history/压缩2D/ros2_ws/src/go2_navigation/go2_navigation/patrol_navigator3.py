#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus  # 引入状态码枚举
import json
import time

class PatrolNavigator(Node):
    def __init__(self):
        super().__init__('patrol_navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        with open('saved_path.json', 'r') as f:
            self.waypoints = json.load(f)
            
        self.current_wp_index = 0
        self.get_logger().info(f'成功加载 {len(self.waypoints)} 个路径点，准备开始循环循迹...')
        
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
        self.get_logger().info(f'====== 前往目标点 {self.current_wp_index + 1}/{len(self.waypoints)} ======')
        
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标点被拒绝！等待 2 秒后重试当前点...')
            time.sleep(2.0)
            self.send_next_waypoint() # 拒绝时重试当前点
            return

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ 成功到达目标点！')
            time.sleep(0.5)  # 成功到达后停顿 0.5 秒，给机器狗一个平滑过渡
            self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
        else:
            self.get_logger().warn(f'⚠️ 导航失败或被中止 (状态码: {status})，等待 2 秒让 Nav2 恢复...')
            time.sleep(5.0)  # 如果失败，强行等待 2 秒，防止把 Nav2 节点塞死
            # 此处依然递增进入下一个点，如果你想死磕当前点，可以去掉下面这行
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
