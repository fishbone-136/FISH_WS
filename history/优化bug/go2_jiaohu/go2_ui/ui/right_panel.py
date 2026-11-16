import threading
import time

import numpy as np
import cv2

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QTextEdit, QSizePolicy, QPushButton
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage

from go2_ui.ui.widgets.neon_frame import NeonFrame

try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False
    rs = None


class DepthWidget(QWidget):
    """Intel RealSense D435i 深度 + 彩色双流组件。
    SDK 回调 → 线程安全的最新帧缓冲 → 主线程 QTimer 取帧显示。"""

    def __init__(self, width=640, height=360, fps=30, parent=None):
        super().__init__(parent)
        self._width = width
        self._height = height
        self._fps = fps
        self._pipeline = None
        self._running = False
        self._poll_thread: threading.Thread | None = None
        self._startup_error = ""
        self._startup_attempts = 0
        self._shutdown_requested = False

        self._buf_lock = threading.Lock()
        self._buf_depth_bytes: bytes | None = None
        self._buf_depth_w: int = 0
        self._buf_depth_h: int = 0
        self._buf_depth_raw: np.ndarray | None = None
        self._buf_color_bgr: np.ndarray | None = None
        self._color_frame_time = 0.0
        self._frame_count = 0
        self._color_frame_count = 0
        self._last_depth_number = None
        self._last_color_number = None
        self._consecutive_timeouts = 0

        # Precompute the complete z16 -> RGB mapping once. The SDK callback
        # only copies frames and indexes this table, avoiding per-frame OpenCV
        # worker pools and several full-image temporary arrays.
        values = np.arange(65536, dtype=np.uint16)
        levels = np.rint(np.minimum(values, 4000) * (255.0 / 4000.0)).astype(np.uint8)
        self._depth_rgb_lut = cv2.applyColorMap(levels[:, None], cv2.COLORMAP_JET)[:, 0, ::-1].copy()
        self._depth_rgb_lut[(values == 0) | (values >= 10000)] = (24, 12, 6)

        self._depth_label_size = None
        self._depth_scaled = None
        self._depth_source_id = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.depth_label = QLabel("深度视频流\n\n(Depth Camera Stream)")
        self.depth_label.setAlignment(Qt.AlignCenter)
        self.depth_label.setMinimumHeight(180)
        self.depth_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.depth_label.setStyleSheet(
            "border: 1px solid #00E5FF; background: #060C18; color: #E0F0FF; font-size: 20px;"
        )
        layout.addWidget(self.depth_label)

        self._display_timer = QTimer(self)
        self._display_timer.setTimerType(Qt.PreciseTimer)
        self._display_timer.timeout.connect(self._display_latest_frame)

        if HAS_REALSENSE:
            self._start_realsense()

    def _start_realsense(self):
        if self._shutdown_requested:
            return
        self._startup_attempts += 1
        # 使用 SDK 回调模式：pipeline 的 start/stop 都在 Qt 主线程，帧由
        # librealsense 内部线程回调。这样可避开 Jetson 上跨线程 wait_for_frames
        # 持续超时、关闭时无法唤醒的问题。
        try:
            self._pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(
                rs.stream.depth, self._width, self._height, rs.format.z16, self._fps
            )
            # Keep both streams at the same rate so synchronization does not
            # limit either preview to 15 fps. Resolution stays at 640x360.
            config.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)
            self._running = True
            self._pipeline.start(config, self._on_frameset)
            self._startup_error = ""
            self.depth_label.setText("正在等待 RealSense 帧…")
            self._display_timer.start(max(1, 1000 // self._fps))
        except Exception as exc:
            self._startup_error = str(exc)
            self._running = False
            if self._pipeline is not None:
                try:
                    self._pipeline.stop()
                except Exception:
                    pass
                self._pipeline = None
            self.depth_label.setText(f"[!] RealSense 启动失败\n\n{exc}")
            # USB control transfers can time out briefly after the previous UI
            # releases the camera. Retry only during startup, without requiring
            # a second application or leaving a half-open pipeline behind.
            if self._startup_attempts < 3:
                QTimer.singleShot(1500, self._start_realsense)

    @property
    def provides_color_stream(self) -> bool:
        return bool(HAS_REALSENSE and self._running)

    def _on_frameset(self, frame):
        if not self._running:
            return
        try:
            frames = frame.as_frameset()
        except Exception:
            return

        depth_frame = frames.get_depth_frame()
        if depth_frame and depth_frame.get_frame_number() != self._last_depth_number:
            try:
                depth_data = np.asanyarray(depth_frame.get_data()).copy()
                rgb = self._depth_rgb_lut[depth_data]
                h, w, _ = rgb.shape
                depth_bytes = rgb.tobytes()
                self._last_depth_number = depth_frame.get_frame_number()
            except Exception:
                depth_bytes = None
                depth_data = None
        else:
            depth_bytes = None
            depth_data = None

        color_frame = frames.get_color_frame()
        if color_frame and color_frame.get_frame_number() != self._last_color_number:
            try:
                color_bgr = np.asanyarray(color_frame.get_data()).copy()
                self._last_color_number = color_frame.get_frame_number()
            except Exception:
                color_bgr = None
        else:
            color_bgr = None

        with self._buf_lock:
            if depth_bytes is not None:
                self._buf_depth_bytes = depth_bytes
                self._buf_depth_w = w
                self._buf_depth_h = h
                self._frame_count += 1
            if depth_data is not None:
                self._buf_depth_raw = depth_data
            if color_bgr is not None:
                self._buf_color_bgr = color_bgr
                self._color_frame_time = time.monotonic()
                self._color_frame_count += 1

    def _display_latest_frame(self):
        with self._buf_lock:
            depth_bytes = self._buf_depth_bytes
            w = self._buf_depth_w
            h = self._buf_depth_h
            buf_fc = self._frame_count

        if depth_bytes is not None and w > 0 and h > 0:
            cur_label_size = self.depth_label.size()
            cur_lk = (cur_label_size.width(), cur_label_size.height())
            if (self._depth_scaled is None or self._depth_label_size != cur_lk
                    or self._depth_source_id != buf_fc):
                self._depth_label_size = cur_lk
                self._depth_source_id = buf_fc
                qimg = QImage(depth_bytes, w, h, w * 3, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qimg)
                self._depth_scaled = pixmap.scaled(
                    cur_label_size, Qt.KeepAspectRatio, Qt.FastTransformation)
                self.depth_label.setPixmap(self._depth_scaled)
        elif self._startup_error:
            self.depth_label.setText(f"[!] RealSense 启动失败\n\n{self._startup_error}")

    def get_latest_color_frame(self) -> np.ndarray | None:
        _, bgr = self.get_latest_color_sample()
        return bgr.copy() if bgr is not None else None

    def get_latest_color_sample(self) -> tuple[int, np.ndarray | None]:
        """Preview sample: sequence plus shared array; callers must not mutate it."""
        with self._buf_lock:
            if not self._running or time.monotonic() - self._color_frame_time > 2.0:
                return self._color_frame_count, None
            return self._color_frame_count, self._buf_color_bgr

    def get_latest_depth_frame(self) -> np.ndarray | None:
        """返回 D435i 原始 z16 深度图副本（单位毫米），供语音视觉引导复用。"""
        with self._buf_lock:
            depth = self._buf_depth_raw
        if depth is not None:
            return depth.copy()
        return None

    def shutdown(self):
        self._shutdown_requested = True
        self._display_timer.stop()
        self._running = False
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self._pipeline = None

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)


class AnalysisWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        vlm_title = QLabel("当前画面内容")
        vlm_title.setStyleSheet(
            "color:#00E5FF; font-size:20px; font-weight:bold;"
            "qproperty-alignment: AlignCenter; background: transparent; border: none;"
        )
        self.analysis_box = QTextEdit()
        self.analysis_box.setReadOnly(True)
        self.analysis_box.setText("点击「视觉感知」开始分析...")
        self.analysis_box.setMinimumHeight(65)
        self.analysis_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(vlm_title)
        layout.addWidget(self.analysis_box)

        decision_title = QLabel("文字对话")
        decision_title.setStyleSheet(
            "color:#00E5FF; font-size:20px; font-weight:bold;"
            "qproperty-alignment: AlignCenter; background: transparent; border: none;"
        )
        self.result_box = QTextEdit()
        self.result_box.setReadOnly(True)
        self.result_box.setText("在交互日志输入框中提问，例如：左边的物体是什么？")
        self.result_box.setMinimumHeight(65)
        self.result_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(decision_title)
        layout.addWidget(self.result_box)

    def set_analysis(self, text: str):
        self.analysis_box.setPlainText(text)

    def set_decision(self, text: str):
        self.result_box.setPlainText(text)


class RightPanel(QWidget):
    analyze_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.depth_widget: DepthWidget = None
        self.analysis_widget: AnalysisWidget = None
        self.build_ui()

    def build_ui(self):
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(10)

        depth_frame = NeonFrame("深度信息显示")
        depth_frame.title_label.setAlignment(Qt.AlignCenter)
        depth_frame.title_label.setStyleSheet(
            "background: transparent; border: none; font-size: 22px;"
        )
        self.depth_widget = DepthWidget()
        depth_frame.layout.addWidget(self.depth_widget)
        root_layout.addWidget(depth_frame, 4)

        analysis_frame = NeonFrame("画面识别与对话")
        analysis_frame.title_label.setAlignment(Qt.AlignCenter)
        analysis_frame.title_label.setStyleSheet(
            "background: transparent; border: none; font-size: 22px;"
        )
        self.analysis_widget = AnalysisWidget()
        analysis_frame.layout.addWidget(self.analysis_widget)
        self.refresh_btn = QPushButton("重新识别当前画面")
        self.refresh_btn.clicked.connect(self.analyze_requested.emit)
        analysis_frame.layout.addWidget(self.refresh_btn)
        root_layout.addWidget(analysis_frame, 6)

    def set_depth_frame(self, pixmap: QPixmap):
        self.depth_widget.depth_label.setPixmap(pixmap)

    def set_analysis(self, text: str):
        self.analysis_widget.set_analysis(text)

    def set_decision(self, text: str):
        self.analysis_widget.set_decision(text)

    def shutdown(self):
        if self.depth_widget is not None:
            self.depth_widget.shutdown()
