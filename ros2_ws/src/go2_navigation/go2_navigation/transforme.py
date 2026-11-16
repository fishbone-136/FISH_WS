#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
video_to_ros2.py
ROS2 Humble: 将 OpenCV VideoCapture 视频流发布为 /camera/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class VideoToRos2(Node):
    def __init__(self):
        super().__init__('video_to_ros2_publisher')

        # 声明参数（video_source 用字符串兼容摄像头序号和视频路径）
        self.declare_parameter('video_source', '1')
        self.declare_parameter('topic_name', '/camera/image_raw')
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('frame_id', 'camera_link')

        # 获取参数
        source_str = self.get_parameter('video_source').value
        self.topic_name = self.get_parameter('topic_name').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.frame_id = self.get_parameter('frame_id').value

        # 处理视频源：纯数字转为int（摄像头），否则保留为字符串（文件路径）
        try:
            self.video_source = int(source_str)
        except ValueError:
            self.video_source = source_str

        # 打开视频源
        self.cap = cv2.VideoCapture(self.video_source)
        if not self.cap.isOpened():
            self.get_logger().error(f"无法打开视频源: {self.video_source}")
            raise RuntimeError("Failed to open video source")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        # CvBridge
        self.bridge = CvBridge()

        # QoS：图像流通常用 BEST_EFFORT + KEEP_LAST 降低延迟
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建发布者
        self.image_pub = self.create_publisher(Image, self.topic_name, qos)

        # 定时器按设定帧率触发
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"视频源: {self.video_source}")
        self.get_logger().info(f"发布话题: {self.topic_name}")
        self.get_logger().info(f"分辨率: {self.frame_width}x{self.frame_height}")
        self.get_logger().info(f"帧率: {self.frame_rate} Hz")
        self.get_logger().info("=" * 50)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("读取视频帧失败")
            # 如果是视频文件，循环播放
            if isinstance(self.video_source, str):
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 2)
            return

        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = self.frame_id
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"转换或发布图像失败: {e}")

    def destroy_node(self):
        self.cap.release()
        self.get_logger().info("视频资源已释放")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VideoToRos2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().fatal(f"节点异常: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
