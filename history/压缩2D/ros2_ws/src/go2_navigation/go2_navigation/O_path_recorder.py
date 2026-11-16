#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import json
import math

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.waypoints = []
        self.last_x = None
        self.last_y = None
        self.record_distance = 0.22 # 每隔 0.5 米记录一个点
        
        # 使用定时器定期检查坐标
        self.timer = self.create_timer(1.0, self.record_pose)
        self.get_logger().info('开始记录路径，请遥控机器人移动...')

    def record_pose(self):
        try:
            # 获取 map 到 base_link 的坐标变换
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # 计算四元数以保存朝向
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w

            if self.last_x is None:
                self.save_waypoint(x, y, qx, qy, qz, qw)
            else:
                dist = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
                if dist >= self.record_distance:
                    self.save_waypoint(x, y, qx, qy, qz, qw)

        except Exception as e:
            self.get_logger().warn(f'无法获取坐标变换: {e}')

    def save_waypoint(self, x, y, qx, qy, qz, qw):
        pose = {'x': x, 'y': y, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}
        self.waypoints.append(pose)
        self.last_x = x
        self.last_y = y
        self.get_logger().info(f'记录路径点: [{x:.2f}, {y:.2f}]')
        
        # 实时写入文件
        with open('saved_path.json', 'w') as f:
            json.dump(self.waypoints, f, indent=4)

def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('路径记录已完成并保存。')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
