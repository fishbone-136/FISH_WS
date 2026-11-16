"""D435i 本地地面引导线、红绿灯与近障检测闭环。"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import threading
import time
from typing import Callable

import cv2
import numpy as np

from . import config


@dataclass(frozen=True)
class PerceptionResult:
    line_found: bool
    line_offset: float = 0.0
    traffic_light: str = "unknown"
    obstacle_distance_m: float | None = None


def detect_traffic_light(frame_bgr: np.ndarray) -> tuple[str, float]:
    """返回 red/green/unknown 与归一化面积置信值；红灯冲突时优先。"""
    if frame_bgr is None or frame_bgr.size == 0:
        return "unknown", 0.0
    height, width = frame_bgr.shape[:2]
    roi = frame_bgr[:max(1, int(height * 0.72)), :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    red = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 95, 80]), np.array([12, 255, 255])),
        cv2.inRange(hsv, np.array([168, 95, 80]), np.array([180, 255, 255])),
    )
    green = cv2.inRange(hsv, np.array([35, 70, 65]), np.array([92, 255, 255]))
    kernel = np.ones((3, 3), np.uint8)
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel)
    green = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernel)

    min_area = max(55.0, width * height * 0.00016)
    red_score = _best_light_blob(red, min_area) / (width * height)
    green_score = _best_light_blob(green, min_area) / (width * height)
    if red_score > 0 and red_score >= green_score * 0.65:
        return "red", red_score
    if green_score > 0:
        return "green", green_score
    return "unknown", 0.0


def _best_light_blob(mask: np.ndarray, min_area: float) -> float:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = 0.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if width <= 0 or height <= 0:
            continue
        aspect = width / height
        fill = area / (width * height)
        perimeter = cv2.arcLength(contour, True)
        circularity = (4.0 * np.pi * area / (perimeter * perimeter)) if perimeter > 0 else 0.0
        if 0.45 <= aspect <= 1.8 and fill >= 0.35 and circularity >= 0.42:
            best = max(best, area)
    return best


def detect_ground_guide(frame_bgr: np.ndarray) -> tuple[bool, float]:
    """检测底部 ROI 内白色或黄色地面导引线，返回中心横向偏差 [-1, 1]。"""
    if frame_bgr is None or frame_bgr.size == 0:
        return False, 0.0
    height, width = frame_bgr.shape[:2]
    y0 = int(height * 0.45)
    roi = frame_bgr[y0:, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    white = cv2.inRange(hsv, np.array([0, 0, 150]), np.array([180, 80, 255]))
    yellow = cv2.inRange(hsv, np.array([15, 70, 80]), np.array([42, 255, 255]))
    mask = cv2.bitwise_or(white, yellow)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = width * roi.shape[0] * 0.006
    candidates: list[tuple[float, np.ndarray]] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        _, _, box_w, box_h = cv2.boundingRect(contour)
        if box_w > width * 0.90 and box_h > roi.shape[0] * 0.75:
            continue
        # 更靠近画面底部的轮廓对当前转向更有意义。
        bottom = float(contour[:, 0, 1].max()) / max(1, roi.shape[0])
        candidates.append((area * (0.7 + bottom), contour))
    if not candidates:
        return False, 0.0

    contour = max(candidates, key=lambda item: item[0])[1]
    moments = cv2.moments(contour)
    if not moments["m00"]:
        return False, 0.0
    center_x = moments["m10"] / moments["m00"]
    offset = (center_x - width / 2.0) / (width / 2.0)
    return True, float(np.clip(offset, -1.0, 1.0))


def obstacle_distance(depth: np.ndarray | None) -> float | None:
    """从 D435i 对齐深度图中央通道取得鲁棒近障距离。"""
    if depth is None or not isinstance(depth, np.ndarray) or depth.size == 0:
        return None
    height, width = depth.shape[:2]
    roi = depth[int(height * 0.42):int(height * 0.86),
                int(width * 0.34):int(width * 0.66)].astype(np.float32)
    valid = roi[np.isfinite(roi) & (roi > 0)]
    if valid.size < 30:
        return None
    # RealSense z16 是毫米；也兼容调用方直接提供米制浮点深度图。
    if float(np.median(valid)) > 20.0:
        valid *= 0.001
    valid = valid[(valid >= 0.12) & (valid <= 8.0)]
    if valid.size < 30:
        return None
    # 取较近的 15 分位数，比最小值更能抵抗深度噪点。
    return float(np.percentile(valid, 15))


def analyze_frame(frame_bgr: np.ndarray, depth: np.ndarray | None = None) -> PerceptionResult:
    line_found, offset = detect_ground_guide(frame_bgr)
    light, _ = detect_traffic_light(frame_bgr)
    return PerceptionResult(line_found, offset, light, obstacle_distance(depth))


class _RealSenseSource:
    """仅在 UI 没有共享 D435i 帧时独占打开设备。"""

    def __init__(self):
        self.pipeline = None
        self.align = None

    def start(self) -> None:
        import pyrealsense2 as rs

        self._rs = rs
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 15)
        cfg.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 15)
        self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)

    def get(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        if self.pipeline is None:
            return None, None
        try:
            frames = self.align.process(self.pipeline.wait_for_frames(timeout_ms=1000))
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color:
                return None, None
            color_array = np.asanyarray(color.get_data()).copy()
            depth_array = np.asanyarray(depth.get_data()).copy() if depth else None
            return color_array, depth_array
        except Exception:
            return None, None

    def close(self) -> None:
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.pipeline = None


class VisualGuidanceController:
    """后台视觉闭环；丢线、红灯、近障、无帧和超时均停车。"""

    def __init__(self, robot, frame_provider: Callable[[], np.ndarray | None] | None = None,
                 depth_provider: Callable[[], np.ndarray | None] | None = None,
                 on_status: Callable[[str], None] | None = None):
        self.robot = robot
        self.frame_provider = frame_provider
        self.depth_provider = depth_provider
        self.on_status = on_status
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._source: _RealSenseSource | None = None
        self._last_status = ""

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, duration: float = 60.0, speed: float = 0.18) -> bool:
        self.stop()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(min(max(duration, 5.0), config.VISUAL_GUIDE_TIMEOUT),
                  min(max(speed, 0.10), 0.25)),
            daemon=True,
            name="bangbang-visual-guide",
        )
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self.robot is not None:
            self.robot.execute_action("stop")
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None
        if self._source is not None:
            self._source.close()
            self._source = None

    def _run(self, duration: float, speed: float) -> None:
        if self.frame_provider is None:
            try:
                self._source = _RealSenseSource()
                self._source.start()
            except Exception as exc:
                self._status(f"视觉引导无法启动 D435i：{exc}")
                self._safe_stop()
                return

        deadline = time.monotonic() + duration
        missing_frames = 0
        red_latched = False
        consecutive_green = 0
        self._status("D435i 地面与红绿灯引导已启动")
        try:
            while not self._stop.is_set() and time.monotonic() < deadline:
                frame, depth = self._get_frames()
                if frame is None or depth is None:
                    missing_frames += 1
                    self._safe_stop()
                    if missing_frames >= 5:
                        self._status("D435i 连续无完整 RGB-D 帧，视觉引导已安全停止")
                        return
                    self._stop.wait(0.2)
                    continue
                missing_frames = 0
                result = analyze_frame(frame, depth)

                if result.traffic_light == "red":
                    red_latched = True
                    consecutive_green = 0
                elif result.traffic_light == "green":
                    consecutive_green += 1
                    if consecutive_green >= 3:
                        red_latched = False
                else:
                    consecutive_green = 0

                if result.obstacle_distance_m is None:
                    self._safe_stop()
                    self._status("D435i 深度无有效数据，已停车")
                elif result.obstacle_distance_m < config.OBSTACLE_STOP_DISTANCE_M:
                    self._safe_stop()
                    self._status(f"前方 {result.obstacle_distance_m:.2f} 米有障碍，已停车")
                elif red_latched:
                    self._safe_stop()
                    self._status("红灯停车锁定，等待连续识别到绿灯")
                elif not result.line_found:
                    self._safe_stop()
                    self._status("未识别到地面引导线，已停车")
                elif result.line_offset < -0.16:
                    self.robot.execute_action("turn_left", speed=0.28, duration=0.35, angle=0.0)
                    self._status("沿引导线向左修正")
                elif result.line_offset > 0.16:
                    self.robot.execute_action("turn_right", speed=0.28, duration=0.35, angle=0.0)
                    self._status("沿引导线向右修正")
                else:
                    self.robot.execute_action("forward", speed=speed, duration=0.40, angle=0.0)
                    if result.traffic_light == "green":
                        self._status("识别到绿灯，沿地面引导线通行")
                    else:
                        self._status("沿地面引导线前进")
                self._stop.wait(0.22)
        except Exception as exc:
            logging.exception("视觉引导异常")
            self._status(f"视觉引导异常，已停车：{exc}")
        finally:
            self._safe_stop()
            if not self._stop.is_set() and time.monotonic() >= deadline:
                self._status("视觉引导达到时限，已停车")
            if self._source is not None:
                self._source.close()
                self._source = None

    def _get_frames(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        if self._source is not None:
            return self._source.get()
        frame = self.frame_provider() if self.frame_provider else None
        depth = self.depth_provider() if self.depth_provider else None
        return frame, depth

    def _safe_stop(self) -> None:
        if self.robot is not None:
            self.robot.execute_action("stop")

    def _status(self, message: str) -> None:
        if message == self._last_status:
            return
        self._last_status = message
        logging.info("[邦邦视觉引导] %s", message)
        if self.on_status:
            self.on_status(message)
