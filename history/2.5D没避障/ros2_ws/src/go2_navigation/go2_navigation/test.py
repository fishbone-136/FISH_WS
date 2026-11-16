#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json
import math

class PatrolNavigator(Node):
    def __init__(self):
        super().__init__('patrol_navigator')
        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 延迟时间（秒）
        self.delay_time = 2.5
        self.delay_timer = None
        
        # 加载路径点
        try:
            with open('saved_path.json', 'r') as f:
                raw_waypoints = json.load(f)
            self.get_logger().info(f'成功读取 {len(raw_waypoints)} 个原始路径点')
        except Exception as e:
            self.get_logger().error(f'无法加载路径文件: {str(e)}')
            return

        # ====================== 关键优化：过滤密集点 ======================
        self.waypoints = self.filter_waypoints(raw_waypoints, min_distance=0.65)
        if not self.waypoints:
            self.get_logger().error('过滤后没有有效路径点！')
            return

        self.current_wp_index = 0
        self.get_logger().info(f'过滤后剩余 {len(self.waypoints)} 个巡逻点 (最小间隔 0.65m)')

        # 等待 Action Server 就绪
        self.get_logger().info('正在等待 navigate_to_pose 服务...')
        self.action_client.wait_for_server()
        self.get_logger().info('✅ 已连接 Nav2，启动巡逻...')
        
        self.send_next_waypoint()

    def filter_waypoints(self, waypoints, min_distance=0.65):
        """过滤距离太近的点"""
        if not waypoints:
            return []
        
        filtered = [waypoints[0]]
        for wp in waypoints[1:]:
            last = filtered[-1]
            dist = math.hypot(wp['x'] - last['x'], wp['y'] - last['y'])
            if dist >= min_distance:
                filtered.append(wp)
        return filtered

    def send_next_waypoint(self):
        if not self.waypoints:
            self.get_logger().info('所有巡逻点已完成！')
            return

        wp = self.waypoints[self.current_wp_index]
        goal_msg = NavigateToPose.Goal()
        
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
        
        goal_msg.pose = pose
        
        self.get_logger().info(
            f'→ 正在前往第 {self.current_wp_index + 1}/{len(self.waypoints)} 个巡逻点 '
            f'(x={wp["x"]:.2f}, y={wp["y"]:.2f})'
        )

        # 发送目标
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理目标是否被接受"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('目标被拒绝！2秒后重试...')
                self.create_timer(2.0, lambda: self.initiate_next_step())
                return

            # 等待结果
            self.get_result_future = goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.get_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'发送 goal 时异常: {str(e)}，2秒后重试')
            self.create_timer(2.0, lambda: self.initiate_next_step())

    def get_result_callback(self, future):
        """处理导航结果"""
        try:
            result = future.result()
            status = result.status
            
            if status == 4:  # SUCCEEDED
                self.get_logger().info(f'✅ 第 {self.current_wp_index + 1} 个点到达成功')
            else:
                self.get_logger().warn(
                    f'⚠️ 第 {self.current_wp_index + 1} 个点未成功 (Status: {status})，继续下一个'
                )
        except Exception as e:
            self.get_logger().error(f'获取结果时异常: {str(e)}')

        # 到达后进入延迟发送逻辑
        self.initiate_next_step()

    def initiate_next_step(self):
        """延迟5秒后发送下一个点"""

        # 更新索引
        self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)

        self.get_logger().info('⏳ 等待 5 秒后前往下一个点...')

        # 防止定时器叠加
        if self.delay_timer is not None:
            self.delay_timer.cancel()

        # 创建一次性定时器
        self.delay_timer = self.create_timer(self.delay_time, self.timer_callback)

    def timer_callback(self):
        """定时器触发：发送下一个目标"""
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.delay_timer = None

        self.send_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('巡逻节点被用户中断')
    except Exception as e:
        node.get_logger().error(f'巡逻节点异常退出: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
