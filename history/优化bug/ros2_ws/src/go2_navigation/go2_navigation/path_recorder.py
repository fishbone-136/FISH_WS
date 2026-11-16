#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import json
import math
import os

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.waypoints = []
        self.last_x = None
        self.last_y = None
        self.record_distance = 0.5 # 记录间隔：每移动 0.5 米记录一个点
        
        # 提高检查频率至 10Hz，但依然只在满足距离条件时才记录
        self.timer = self.create_timer(0.1, self.record_pose)
        
        self.get_logger().info('✅ 路径记录节点已启动！')
        self.get_logger().info('⚠️ 请注意：按 Ctrl+C 结束程序时，才会将路径一次性保存到文件。')

    def record_pose(self):
        try:
            # 获取 map 到 base_link 的坐标变换
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w

            # 如果是第一个点，直接记录
            if self.last_x is None:
                self.add_waypoint(x, y, qx, qy, qz, qw)
            else:
                # 计算与上一个点的直线距离
                dist = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
                if dist >= self.record_distance:
                    self.add_waypoint(x, y, qx, qy, qz, qw)

        except (LookupException, ConnectivityException, ExtrapolationException):
            # 正常过滤：TF 树尚未建好，或偶尔的网络延迟丢包，直接跳过，不报 error 也不死机
            pass
        except Exception as e:
            # 捕获其他严重未知错误
            self.get_logger().error(f'发生未知错误: {e}')

    def add_waypoint(self, x, y, qx, qy, qz, qw):
        pose = {'x': x, 'y': y, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}
        self.waypoints.append(pose)
        self.last_x = x
        self.last_y = y
        self.get_logger().info(f'�� 记录路径点 {len(self.waypoints)}: [x: {x:.2f}, y: {y:.2f}]')

    def save_to_file(self):
        """在节点销毁前执行文件保存"""
        if not self.waypoints:
            self.get_logger().warn('没有记录到任何路径点，已取消保存。')
            return
            
        save_path = os.path.join(os.getcwd(), 'saved_path.json')
        try:
            with open(save_path, 'w') as f:
                json.dump(self.waypoints, f, indent=4)
            self.get_logger().info(f'�� 成功！共 {len(self.waypoints)} 个路径点已保存至: {save_path}')
        except Exception as e:
            self.get_logger().error(f'保存文件失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('�� 收到退出信号 (Ctrl+C)，准备保存文件并安全退出...')
    finally:
        # 无论程序如何退出，都会触发这里的保存逻辑
        node.save_to_file()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

