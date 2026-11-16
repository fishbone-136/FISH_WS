#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import time
import rclpy
import threading
import numpy as np
import pyrealsense2 as rs

from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# ================== 绑定 CPU 核心（0~7）==================
def set_cpu_affinity(cpu_id=7):   
    """将进程绑定到指定 CPU 核心"""
    try:
        if cpu_id < 0 or cpu_id > 7:
            cpu_id = 4
        os.sched_setaffinity(0, {cpu_id})
        print(f"✅ 已绑定到 CPU {cpu_id}")
    except Exception as e:
        print(f"⚠️ CPU 绑定失败: {e}（继续运行）")


class VisionTracker(Node):
    def __init__(self):
        super().__init__('vision_omni_tracker')

        set_cpu_affinity(7)   # 你可以改成 0~7 中的任意一个

        # ================== 参数配置 ==================
        self.declare_parameter('model_path', 'yolov8n.pt')
        
        # 仅保留用于画面 UI 颜色显示的距离参考参数
        self.target_distance = 0.8
        self.deadzone_dist_m = 0.1

        # ================== ROS ==================
        self.bridge = CvBridge()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.result_pub = self.create_publisher(Image, '/yolo/person_tracking', qos)

        # ================== 状态变量 ==================
        self.target_id = None
        self.candidate_id = None
        self.running = True
        self.latest_frame = None
        self.latest_depth_frame = None
        self.frame_lock = threading.Lock()
        self.window_name = "D435i_Tracker"

        # FPS
        self.fps = 0.0
        self.frame_count = 0
        self.start_time = time.time()

        # ================== 加载模型 ==================
        model_path = self.get_parameter('model_path').value
        self.get_logger().info(f"⏳ 正在加载模型: {model_path}")
        self.model = YOLO(model_path, task='detect')
        self.get_logger().info("✅ 模型加载成功")

        # ================== D435i 配置 ==================
        self.get_logger().info("�� 正在初始化 D435i ")
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # 统一使用最稳定的 640x480 分辨率，30 帧
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.get_logger().info("✅ 相机初始化完成")

        # 启动线程
        threading.Thread(target=self.camera_read_loop, daemon=True).start()
        self.inference_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.inference_thread.start()

        self.get_logger().info("⌨️ 交互：[Space]-锁定 | [B]-切换 | [Esc]-重置 | [Q]-退出")

    def camera_read_loop(self):
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not depth_frame or not color_frame:
                    continue
                    
                with self.frame_lock:
                    self.latest_frame = np.asanyarray(color_frame.get_data())
                    depth_frame.keep()
                    self.latest_depth_frame = depth_frame
            except Exception as e:
                time.sleep(0.01)

    def inference_loop(self):
        cv2.namedWindow(self.window_name)
        
        # 统一修改为 640x480
        img_w, img_h = 640, 480          
        center_x = img_w / 2.0

        target_frame_time = 1.0 / 30.0

        while self.running and rclpy.ok():
            loop_start = time.time()

            with self.frame_lock:
                if self.latest_frame is None or self.latest_depth_frame is None:
                    time.sleep(0.001)
                    continue
                frame = self.latest_frame.copy()
                depth_frame = self.latest_depth_frame
                self.latest_frame = None
                self.latest_depth_frame = None

            # YOLO 推理
            results = self.model.track(
                source=frame, persist=True, device=0, half=True,
                imgsz=640, conf=0.45, iou=0.5, classes=[0],
                tracker="bytetrack.yaml", verbose=False
            )

            annotated_frame = frame.copy()
            res = results[0]
            current_ids = []
            tracking_found = False
            id_to_box = {}

            if res.boxes is not None and res.boxes.id is not None:
                ids = res.boxes.id.int().cpu().tolist()
                boxes = res.boxes.xyxy.cpu().numpy()
                current_ids = ids
                id_to_box = {bid: box for bid, box in zip(ids, boxes)}

                if self.target_id not in current_ids:
                    self.target_id = None

                if self.target_id is None and current_ids:
                    if self.candidate_id not in current_ids:
                        min_dist = float('inf')
                        for bid, box in id_to_box.items():
                            dist = abs((box[0] + box[2])/2 - center_x)
                            if dist < min_dist:
                                min_dist = dist
                                self.candidate_id = bid
                else:
                    self.candidate_id = None
                    tracking_found = True

            # 获取深度信息（仅用于UI显示）
            target_dist_m = 0.0
            if tracking_found:
                box = id_to_box[self.target_id]
                bx1, by1, bx2, by2 = map(int, box)
                b_center_x = int(np.clip((bx1 + bx2) / 2, 0, img_w - 1))
                b_center_y = int(np.clip((by1 + by2) / 2, 0, img_h - 1))
                
                target_dist_m = depth_frame.get_distance(b_center_x, b_center_y)

            # FPS
            self.frame_count += 1
            if self.frame_count >= 30:
                self.fps = self.frame_count / (time.time() - self.start_time)
                self.frame_count = 0
                self.start_time = time.time()

            # UI
            cv2.putText(annotated_frame, f"FPS: {self.fps:.1f}", (10, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(annotated_frame, f"Target Dist: {self.target_distance:.1f}m (Desired)", 
                       (10, 55), 0, 0.65, (255, 255, 0), 2)

            if self.target_id is not None and target_dist_m > 0:
                color = (0, 255, 0)
                if target_dist_m < self.target_distance - self.deadzone_dist_m:
                    color = (0, 0, 255)
                elif target_dist_m > self.target_distance + self.deadzone_dist_m:
                    color = (255, 0, 0)
                cv2.putText(annotated_frame, f"Current Dist: {target_dist_m:.2f}m", 
                           (10, 85), 0, 0.65, color, 2)

            # 画框
            for bid, box in id_to_box.items():
                x1, y1, x2, y2 = map(int, box)
                if bid == self.target_id:
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                    cv2.putText(annotated_frame, f"LOCKED {bid}", (x1, y1-10), 0, 0.65, (0, 0, 255), 2)
                elif bid == self.candidate_id:
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(annotated_frame, f"CAND {bid}", (x1, y1-10), 0, 0.6, (0, 255, 255), 2)
                else:
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

            # 直接显示和发布 640x480 的原始推理画面
            cv2.imshow(self.window_name, annotated_frame)
            self.result_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))

            elapsed = time.time() - loop_start
            time.sleep(max(0, target_frame_time - elapsed))

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                if self.candidate_id is not None:
                    self.target_id = self.candidate_id
                    self.get_logger().info(f"�� 锁定目标: {self.target_id}")
            elif key in [ord('b'), ord('B')]:
                if current_ids:
                    curr = self.target_id if self.target_id else self.candidate_id
                    idx = current_ids.index(curr) if curr in current_ids else 0
                    self.target_id = current_ids[(idx + 1) % len(current_ids)]
            elif key == 27:
                self.target_id = None
                self.get_logger().info("�� 解除锁定")
            elif key == ord('q') or key == ord('Q'):
                self.running = False
                break

    def stop(self):
        self.running = False
        try:
            self.pipeline.stop()
        except:
            pass
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = VisionTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
