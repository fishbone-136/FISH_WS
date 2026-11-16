#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
import json
import math

class WaypointPatrol(Node):
    def __init__(self):
        super().__init__('waypoint_patrol')
        
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # 加载路径点
        try:
            with open('saved_path.json', 'r') as f:
                raw_waypoints = json.load(f)
            self.get_logger().info(f'成功读取 {len(raw_waypoints)} 个原始路径点')
        except Exception as e:
            self.get_logger().error(f'无法加载 saved_path.json: {str(e)}')
            return

        # 过滤太密的点，避免连续航点距离过近
        self.waypoints = self.filter_waypoints(raw_waypoints, min_distance=0.8)
        if len(self.waypoints) < 1:
            self.get_logger().error('过滤后没有有效路径点！')
            return

        self.get_logger().info(f'过滤后剩余 {len(self.waypoints)} 个巡逻点 (最小间隔 0.8m)')

        # 等待 Action Server
        self.get_logger().info('正在等待 /follow_waypoints 服务...')
        if not self.action_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('等待 follow_waypoints server 超时！')
            return

        self.get_logger().info('✅ 已连接 follow_waypoints，开始巡逻...')
        self.send_all_waypoints()

    def filter_waypoints(self, raw_list, min_distance=0.8):
        """过滤距离太近的点，降低连续导航目标发送失败的概率。"""
        if not raw_list:
            return []
        filtered = [raw_list[0]]
        for wp in raw_list[1:]:
            last = filtered[-1]
            dist = math.hypot(wp['x'] - last['x'], wp['y'] - last['y'])
            if dist >= min_distance:
                filtered.append(wp)
        return filtered

    def create_pose_stamped(self, wp):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = wp.get('qx', 0.0)
        pose.pose.orientation.y = wp.get('qy', 0.0)
        pose.pose.orientation.z = wp.get('qz', 0.0)
        pose.pose.orientation.w = wp.get('qw', 1.0)
        return pose

    def send_all_waypoints(self):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self.create_pose_stamped(wp) for wp in self.waypoints]
        
        # 可选：设置循环次数（0 表示只走一次）
        # goal_msg.number_of_loops = 0   # 默认就是走一次

        self.get_logger().info(f'发送 {len(goal_msg.poses)} 个巡逻点给 follow_waypoints...')

        send_goal_future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('follow_waypoints goal 被拒绝！')
                return

            self.get_logger().info('follow_waypoints goal 已接受，开始执行...')
            self.get_result_future = goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f'发送 goal 异常: {str(e)}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_idx = feedback.current_waypoint_index
        total = len(self.waypoints)
        self.get_logger().info(f'正在执行第 {current_idx + 1}/{total} 个巡逻点...')

    def get_result_callback(self, future):
        try:
            result = future.result().result
            missed = result.missed_waypoints
            if missed:
                self.get_logger().warn(f'有 {len(missed)} 个点未能成功到达')
            else:
                self.get_logger().info('✅ 所有巡逻点已完成！')
        except Exception as e:
            self.get_logger().error(f'获取结果异常: {str(e)}')
        
        self.get_logger().info('本次巡逻任务结束。可按 Ctrl+C 退出，或修改代码实现无限循环。')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('巡逻节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
