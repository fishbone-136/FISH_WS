#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import threading
import time
import json 

class HumbleSmartWaypointNavigator(Node):
    def __init__(self):
        super().__init__('humble_smart_waypoint_navigator')
        self.waypoints = []
        
        self.point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
            
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.action_done_event = threading.Event()
        self.action_result = False
        
        self.get_logger().info('�� 智能多点导航节点已启动！')
        self.get_logger().info('指令说明: [start: 开始导航] [save: 保存航点] [clear: 清空] [exit: 退出]')

    def clicked_point_callback(self, msg: PointStamped):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.point.x
        goal_pose.pose.position.y = msg.point.y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0 # 默认朝向
        
        self.waypoints.append(goal_pose)
        self.get_logger().info(f'已记录第 {len(self.waypoints)} 个点: x={msg.point.x:.2f}, y={msg.point.y:.2f}')

    def save_waypoints_to_json(self):
        """将当前记录的点转换为 patrol_navigator 需要的格式并保存"""
        if not self.waypoints:
            self.get_logger().warn('当前列表为空，无法保存！')
            return

        save_data = []
        for wp in self.waypoints:
            save_data.append({
                "x": wp.pose.position.x,
                "y": wp.pose.position.y,
                "qx": wp.pose.orientation.x,
                "qy": wp.pose.orientation.y,
                "qz": wp.pose.orientation.z,
                "qw": wp.pose.orientation.w
            })
        
        try:
            with open('saved_path.json', 'w') as f:
                json.dump(save_data, f, indent=4)
            self.get_logger().info(f'✅ 成功将 {len(save_data)} 个点保存到 saved_path.json')
        except Exception as e:
            self.get_logger().error(f'保存失败: {str(e)}')

    def start_navigation(self):
        if not self.waypoints:
            self.get_logger().warn('没有记录任何航点！')
            return
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 服务器未响应！')
            return
        threading.Thread(target=self.execute_mission).start()

    def execute_mission(self):
        last_successful_waypoint = None
        total_points = len(self.waypoints)
        
        for i, target_waypoint in enumerate(self.waypoints):
            self.get_logger().info(f'>>> 目标 {i+1}/{total_points} <<<')
            success = False
            for attempt in range(1, 4):
                self.get_logger().info(f'尝试 {attempt}/3...')
                if self.send_pose_goal_blocking(target_waypoint):
                    last_successful_waypoint = target_waypoint
                    success = True
                    self.get_logger().info('✅ 到达目标点，系统缓冲 1.5 秒...')
                    time.sleep(1.5)  # �� 核心修改：等待资源释放，防止下一次发点崩溃
                    break
                else:
                    if last_successful_waypoint:
                        self.get_logger().info('♻️ 执行退回重试策略...')
                        self.send_pose_goal_blocking(last_successful_waypoint)
                        time.sleep(1.5)  # �� 核心修改：退回重试后也给足缓冲时间
            
            if not success:
                self.get_logger().error('任务失败，停止后续导航。')
                return

        self.get_logger().info('�� 导航任务圆满完成！')

    def send_pose_goal_blocking(self, pose_stamped):
        self.action_done_event.clear()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        self.action_done_event.wait()
        return self.action_result

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.action_result = False
            self.action_done_event.set()
            return
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_result = (future.result().status == GoalStatus.STATUS_SUCCEEDED)
        self.action_done_event.set()

def main(args=None):
    rclpy.init(args=args)
    node = HumbleSmartWaypointNavigator()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.daemon = True
    spin_thread.start()

    try:
        while rclpy.ok():
            cmd = input().strip().lower()
            if cmd == 'start':
                node.start_navigation()
            elif cmd == 'save':
                node.save_waypoints_to_json()
            elif cmd == 'clear':
                node.waypoints.clear()
                print("航点已清空。")
            elif cmd == 'exit':
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
