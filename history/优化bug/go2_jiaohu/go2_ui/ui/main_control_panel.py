import os
import sys
import math
import time
import random
import threading
import logging
from collections import deque

import cv2
import numpy as np

# Windows: CAP_DSHOW 后端规避 MSMF 兼容性问题
# 摄像头后端优先级列表（Windows：DSHOW > MSMF > ANY）
_CAM_BACKENDS = []
for _attr in ('CAP_DSHOW', 'CAP_MSMF', 'CAP_ANY'):
    _val = getattr(cv2, _attr, None)
    if _val is not None:
        _CAM_BACKENDS.append((_attr, _val))
if not _CAM_BACKENDS:
    _CAM_BACKENDS = [('DEFAULT', 0)]

# ---- 检测 RealSense 摄像头名称，防止 VideoWidget 抢占 D435i RGB UVC 端点 ----
_RS_CAMERA_NAMES: set = set()
try:
    import pyrealsense2 as _rs_detect
    for _dev in _rs_detect.context().query_devices():
        _RS_CAMERA_NAMES.add(_dev.get_info(_rs_detect.camera_info.name))
except Exception:
    pass

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QFrame,
    QSizePolicy, QTextEdit, QPushButton
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QPointF, QDateTime
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QFont, QPen, QRadialGradient, QKeySequence
from PyQt5.QtWidgets import QShortcut

from go2_ui.ui.widgets.neon_frame import NeonFrame
from go2_ui.activity_manager import ExclusiveActivityManager
from hand.gesture import HandDetector

# 启动入口显式指定部署/离线模式，不能根据开发机是否碰巧安装 SDK 来猜测。
if os.environ.get("GO2_DEPLOY_MODE", "0").strip().lower() in {"1", "true", "yes", "on"}:
    from deploy.gesture_controller import GestureController
    from deploy.speech_controller import SpeechController
    from deploy.vlm_controller import VLMController
    _DEPLOY_MODE = True
else:
    from offline_test.gesture_controller import GestureController
    from offline_test.speech_controller import SpeechController
    from offline_test.vlm_controller import VLMController
    _DEPLOY_MODE = False

_ICON_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "icons")


class VideoWidget(QWidget):
    """实时视频组件。

    RealSense 可用时复用深度组件的彩色流；否则在后台线程连接 Go2 前置
    相机组播，再回退到本机 V4L2 摄像头。后台采集避免断流时阻塞 Qt 事件循环。
    """

    landmarks_updated = pyqtSignal(object)

    def __init__(self, camera_index: int = 0, fps: int = 30,
                 hand_detector: HandDetector | None = None):
        super().__init__()
        self.camera_index = camera_index
        self.fps = fps
        self.cap: cv2.VideoCapture | None = None
        self.hand_detector = hand_detector
        self._hand_detection_enabled = False
        self._pixmap: QPixmap | None = None
        self._last_bgr_frame: np.ndarray | None = None
        self._placeholder = True
        self._external_color_source: callable | None = None
        self._external_display_seq = -1
        self._external_empty_since: float | None = None
        self._status_text = "正在初始化视频源…"

        self._capture_lock = threading.Lock()
        self._capture_frame: np.ndarray | None = None
        self._capture_seq = 0
        self._display_seq = -1
        self._capture_stop = threading.Event()
        self._capture_thread: threading.Thread | None = None

        self.setMinimumHeight(260)
        self.setAttribute(Qt.WA_OpaquePaintEvent)
        self.setStyleSheet("border:2px solid #00F0FF; background:#080C14;")

        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.PreciseTimer)
        self._timer.timeout.connect(self._update_frame)

        self._last_size = None
        self._scaled_pixmap = None
        self._scaled_source_id = 0
        self._last_update_time = 0

        QTimer.singleShot(500, self._start_camera)

    def resizeEvent(self, event):
        self._scaled_pixmap = None
        self._last_size = None
        self._scaled_source_id = 0
        super().resizeEvent(event)

    def set_hand_detection(self, enabled: bool):
        self._hand_detection_enabled = enabled

    def set_external_color_source(self, source: callable):
        self._external_color_source = source
        self._external_display_seq = -1
        self._external_empty_since = time.monotonic()
        self._status_text = "正在等待 RealSense 彩色画面…"

    def _start_camera(self):
        self._timer.start(max(15, 1000 // self.fps))
        if self._external_color_source is not None:
            return

        self._start_capture_thread()

    def _start_capture_thread(self):
        if self._capture_thread is not None and self._capture_thread.is_alive():
            return
        self._capture_stop.clear()
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="go2-ui-camera",
            daemon=True,
        )
        self._capture_thread.start()

    @staticmethod
    def _go2_pipeline() -> str:
        interface = os.environ.get("GO2_NETWORK_INTERFACE", "eno1").strip() or "eno1"
        return (
            f"udpsrc address=230.1.1.1 port=1720 multicast-iface={interface} "
            "! application/x-rtp,media=video,encoding-name=H264,payload=96 "
            "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
            "! video/x-raw,format=BGR "
            "! appsink drop=true max-buffers=1 sync=false"
        )

    @staticmethod
    def _is_realsense_index(index: int) -> bool:
        name_path = f"/sys/class/video4linux/video{index}/name"
        try:
            with open(name_path, "r", encoding="utf-8") as stream:
                return "realsense" in stream.read().lower()
        except OSError:
            return False

    def _capture_loop(self):
        source_mode = os.environ.get("GO2_UI_CAMERA_SOURCE", "auto").strip().lower()
        if source_mode not in {"auto", "go2", "local"}:
            source_mode = "auto"

        if source_mode in {"auto", "go2"}:
            self._status_text = "正在连接 Go2 前置相机…"
            cap = cv2.VideoCapture(self._go2_pipeline(), cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self.cap = cap
                self._status_text = "等待 Go2 前置相机数据…"
                if self._read_capture(cap, "Go2 前置相机"):
                    return
            cap.release()
            self.cap = None
            if source_mode == "go2" or self._capture_stop.is_set():
                self._status_text = "[!] Go2 前置相机无数据"
                return

        self._status_text = "正在查找本机摄像头…"
        cap = self._open_local_camera()
        if cap is None:
            self._status_text = "[!] 未找到可用摄像头"
            return
        self.cap = cap
        self._read_capture(cap, f"本机摄像头 /dev/video{self.camera_index}")

    def _open_local_camera(self):
        indices = [self.camera_index]
        for extra in (0, 1, 2, 3, 4):
            if extra not in indices:
                indices.append(extra)

        for idx in indices:
            # 深度组件独占 D435i；普通摄像头回退不能再次打开它。
            if self._is_realsense_index(idx):
                continue
            for _name, backend in _CAM_BACKENDS:
                cap = cv2.VideoCapture(idx, backend)
                if cap.isOpened():
                    ret, _ = cap.read()
                    if ret:
                        self.camera_index = idx
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                        return cap
                    cap.release()
        return None

    def _read_capture(self, cap: cv2.VideoCapture, source_name: str) -> bool:
        received = False
        while not self._capture_stop.is_set() and cap.isOpened():
            ret, frame = cap.read()
            if not ret or frame is None:
                break
            received = True
            with self._capture_lock:
                self._capture_frame = frame
                self._capture_seq += 1
            self._status_text = source_name
        return received

    def _update_frame(self):
        if self._external_color_source is not None:
            sample = self._external_color_source()
            if isinstance(sample, tuple):
                sequence, frame = sample
                if frame is not None:
                    self._external_empty_since = None
                    if sequence == self._external_display_seq:
                        return
                    self._external_display_seq = sequence
            else:
                frame = sample
            if frame is None:
                # RealSense 初始化失败时不能永久绑在空帧源上。
                if (self._external_empty_since is not None
                        and time.monotonic() - self._external_empty_since > 8.0):
                    self._external_color_source = None
                    self._status_text = "RealSense 彩色流无数据，切换 Go2 前置相机…"
                    self._start_capture_thread()
                if self._placeholder:
                    self.update()
                return
            self._external_empty_since = None
            self._status_text = "RealSense 彩色相机"
        else:
            with self._capture_lock:
                if self._capture_frame is None or self._capture_seq == self._display_seq:
                    if self._placeholder:
                        self.update()
                    return
                frame = self._capture_frame
                self._display_seq = self._capture_seq
            if frame is None:
                return

        self._last_bgr_frame = frame

        if self._hand_detection_enabled and self.hand_detector is not None:
            frame, landmarks, handedness = self.hand_detector.detect(frame.copy())
            self.landmarks_updated.emit((landmarks, handedness, frame))

        now = time.time()
        if now - self._last_update_time < 0.016:
            return
        self._last_update_time = now

        h, w, ch = frame.shape
        qt_img = QImage(frame.data, w, h, frame.strides[0], QImage.Format_BGR888)
        self._pixmap = QPixmap.fromImage(qt_img)
        self._scaled_source_id += 1
        self._placeholder = False
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#080C14"))
        painter.setRenderHint(QPainter.Antialiasing)

        if self._pixmap and not self._placeholder:
            cur_size = self.size()
            cur_key = (cur_size.width(), cur_size.height())
            if (self._scaled_pixmap is None or self._last_size != cur_key
                    or self._scaled_source_id != getattr(self, '_paint_source_id', -1)):
                self._last_size = cur_key
                self._paint_source_id = self._scaled_source_id
                self._scaled_pixmap = self._pixmap.scaled(
                    cur_size, Qt.KeepAspectRatio, Qt.FastTransformation)

            x = (self.width() - self._scaled_pixmap.width()) // 2
            y = (self.height() - self._scaled_pixmap.height()) // 2
            painter.drawPixmap(x, y, self._scaled_pixmap)
        else:
            painter.setPen(QColor("#4A8098"))
            painter.setFont(QFont("Microsoft YaHei", 22))
            painter.drawText(self.rect(), Qt.AlignCenter, self._status_text)

        painter.end()

    def get_frame(self):
        if self._last_bgr_frame is not None:
            return self._last_bgr_frame.copy()
        return None

    def shutdown(self):
        self._timer.stop()
        self._capture_stop.set()
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self._capture_thread is not None and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
        self._capture_thread = None
        if self.hand_detector is not None and hasattr(self.hand_detector, "close"):
            self.hand_detector.close()

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)


class VoiceWaveWidget(QWidget):
    MAX_WAVES = 12
    WAVE_LIFETIME = 36
    BASE_RADIUS = 8
    EXPAND_SPEED = 2.0
    TEXT_Y = 14
    TEXT_H = 18
    WAVE_GAP = 8

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background: transparent;")
        self.setMinimumHeight(60)
        self._state = "off"
        self._status_text = ""
        self._audio_level = 0.0
        self._waves = deque()
        self._frame = 0
        self._spawn_cooldown = 0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)
        self._idle_centers = [(0.25, 0.5), (0.5, 0.5), (0.75, 0.5)]

    def set_state(self, state: str):
        self._state = state
        self._status_text = ""
        if state in {"idle", "off"}:
            self._audio_level = 0.0
        if state == "off":
            self._waves.clear()

    def set_status(self, text: str):
        self._status_text = text
        self.update()

    def set_audio_level(self, level: float):
        self._audio_level = max(0.0, min(1.0, level))
        if self._state != "listening" and self._audio_level > 0.05:
            self._state = "listening"
        elif self._audio_level < 0.02 and self._state == "listening":
            self._state = "idle"

    def _tick(self):
        if self._state == "off":
            return
        self._frame += 1
        if self._state == "listening" and self._audio_level < 0.02:
            t = self._frame * 0.15
            simulated = 0.3 + math.sin(t) * 0.25 + math.sin(t * 2.3) * 0.2 + math.sin(t * 5.1) * 0.15
            self._audio_level = max(0.02, min(0.9, simulated))
        if self._state == "listening" and self._audio_level > 0.05:
            spawn_rate = int(2 + (1.0 - self._audio_level) * 14)
            if self._spawn_cooldown <= 0 and len(self._waves) < self.MAX_WAVES:
                self._spawn_wave()
                self._spawn_cooldown = spawn_rate
            else:
                self._spawn_cooldown -= 1
        elif self._state == "idle":
            if self._frame % 60 == 0 and len(self._waves) < 3:
                self._spawn_idle_wave()
        for w in self._waves:
            w["radius"] += self.EXPAND_SPEED
            w["opacity"] = max(0.0, 1.0 - w["radius"] / w["max_radius"])
        while self._waves and self._waves[0]["opacity"] <= 0.0:
            self._waves.popleft()
        self.update()

    def _wave_area_bounds(self):
        h = self.height()
        y_start = self.TEXT_Y + self.TEXT_H + self.WAVE_GAP
        return y_start, h

    def _spawn_wave(self):
        w = self.width()
        h = self.height()
        if w < 10 or h < 10:
            return
        y_start, y_end = self._wave_area_bounds()
        area_h = y_end - y_start
        if area_h < 5:
            return
        cx = random.uniform(w * 0.2, w * 0.8)
        cy = random.uniform(y_start + area_h * 0.1, y_end - area_h * 0.1)
        intensity = max(0.2, self._audio_level)
        max_r = random.uniform(min(w, h) * 0.25, min(w, h) * 0.55) * intensity
        self._waves.append({
            "x": cx, "y": cy,
            "radius": self.BASE_RADIUS,
            "opacity": 0.75 * intensity,
            "max_radius": max_r,
            "intensity": intensity,
        })

    def _spawn_idle_wave(self):
        w = self.width()
        h = self.height()
        if w < 10 or h < 10:
            return
        y_start, y_end = self._wave_area_bounds()
        area_h = y_end - y_start
        if area_h < 5:
            return
        idx = random.randint(0, len(self._idle_centers) - 1)
        fx, fy = self._idle_centers[idx]
        cx = w * fx + random.uniform(-10, 10)
        cy = y_start + area_h * fy + random.uniform(-5, 5)
        self._waves.append({
            "x": cx, "y": cy,
            "radius": self.BASE_RADIUS,
            "opacity": 0.18,
            "max_radius": min(w, area_h) * 0.35,
            "intensity": 0.12,
        })

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        y_start, y_end = self._wave_area_bounds()

        font = QFont("Microsoft YaHei", 12)
        painter.setFont(font)
        if self._state == "off":
            text = "语音未启动"
            painter.setPen(QColor(96, 150, 175, 140))
        elif self._state in {"idle", "listening"}:
            text = "正在聆听"
            painter.setPen(QColor(96, 208, 232, 140))
        else:
            text = "正在回答" if self._state == "speaking" else "正在识别"
            painter.setPen(QColor(0, 240, 255, 220))
        if self._status_text and self._state != "off":
            text = self._status_text
        text = painter.fontMetrics().elidedText(text, Qt.ElideRight, max(0, w - 8))
        painter.drawText(0, self.TEXT_Y, w, self.TEXT_H, Qt.AlignCenter, text)
        if self._state == "off":
            painter.end()
            return

        painter.save()
        painter.setClipRect(0, y_start, w, y_end - y_start)
        for wave in self._waves:
            self._draw_wave_ring(painter, wave)
        painter.restore()

        base_y = y_start + (y_end - y_start) * 0.65
        painter.setPen(QPen(QColor(10, 110, 255, 70), 1))
        painter.drawLine(int(w * 0.12), int(base_y), int(w * 0.88), int(base_y))

        if self._state == "idle":
            self._draw_idle_dots(painter, w, base_y)
        else:
            self._draw_active_bars(painter, w, y_end, base_y)
        painter.end()

    def _draw_wave_ring(self, painter: QPainter, wave: dict):
        r = wave["radius"]
        if r < 0.5:
            return
        intensity = wave.get("intensity", 0.3)
        base_color = QColor(0, 240, 255)
        alpha = int(wave["opacity"] * 220)
        if alpha <= 0:
            return
        cx, cy = wave["x"], wave["y"]
        for i in range(3):
            ring_r = r - i * (r * 0.15)
            if ring_r < 0.5:
                continue
            ring_alpha = alpha - i * 55
            if ring_alpha <= 0:
                continue
            color = QColor(base_color.red(), base_color.green(), base_color.blue(), max(0, ring_alpha))
            pen = QPen(color, 1.2 - i * 0.3)
            pen.setCosmetic(True)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(cx, cy), ring_r, ring_r)
        inner_r = max(0, r * 0.35)
        if inner_r > 1:
            grad = QRadialGradient(cx, cy, r * 0.6)
            core_color = QColor(base_color.red(), base_color.green(), base_color.blue(), max(0, int(alpha * 0.35)))
            grad.setColorAt(0.0, core_color)
            grad.setColorAt(1.0, QColor(0, 240, 255, 0))
            painter.setPen(Qt.NoPen)
            painter.setBrush(grad)
            painter.drawEllipse(QPointF(cx, cy), r * 0.6, r * 0.6)

    def _draw_idle_dots(self, painter: QPainter, w: int, base_y: float):
        t = self._frame * 0.08
        n = 5
        total_w = w * 0.6
        start_x = (w - total_w) / 2
        step = total_w / (n - 1)
        for i in range(n):
            x = start_x + i * step
            amplitude = 1.5 + math.sin(t + i * 1.2) * 1.0
            alpha = 50 + int(math.sin(t + i * 0.8) * 20)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(0, 240, 255, max(0, min(255, alpha))))
            painter.drawEllipse(QPointF(x, base_y), amplitude, amplitude)

    def _draw_active_bars(self, painter: QPainter, w: int, h: int, base_y: float):
        n_bars = 12
        bar_w = (w * 0.7) / n_bars
        gap = bar_w * 0.22
        total_w = n_bars * bar_w + (n_bars - 1) * gap
        start_x = (w - total_w) / 2
        for i in range(n_bars):
            seed = math.sin(self._frame * 0.3 + i * 0.7) * 0.5 + 0.5
            seed += math.sin(self._frame * 0.15 + i * 1.3) * 0.3
            seed = max(0.0, min(1.0, seed))
            height_px = seed * self._audio_level * (h * 0.22) + 2
            x = start_x + i * (bar_w + gap)
            alpha = 80 + int(seed * 140)
            color = QColor(0, 240, 255, max(0, min(255, alpha)))
            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawRoundedRect(int(x), int(base_y - height_px), int(bar_w), int(height_px), 2, 2)

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)


class ModeButton(QFrame):
    clicked = pyqtSignal()

    _STYLE = (
        "QFrame#ModeButton {"
        "  background-color: #0C1E3A;"
        "  border: 1px solid #1A5080;"
        "  border-radius: 8px;"
        "}"
        "QFrame[hover=\"true\"] {"
        "  background-color: #163358;"
        "  border: 1px solid #2898D0;"
        "}"
        "QFrame[pressed=\"true\"] {"
        "  background-color: #081428;"
        "  border: 1px solid #0A5A80;"
        "}"
        "QFrame[active=\"true\"] {"
        "  background-color: #12345C;"
        "  border: 2px solid #00F0FF;"
        "}"
    )

    def __init__(self, icon_path: str, text: str, parent=None):
        super().__init__(parent)
        self._text = text
        self.setObjectName("ModeButton")
        self.setFixedHeight(42)
        self.setCursor(Qt.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setStyleSheet(self._STYLE)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addStretch()

        icon_label = QLabel()
        icon_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        icon_label.setFixedSize(28, 28)
        icon_label.setStyleSheet("background: transparent; border: none;")
        pix = QPixmap(icon_path).scaled(28, 28, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        icon_label.setPixmap(pix)
        layout.addWidget(icon_label)
        layout.addSpacing(14)

        self._text_label = QLabel(text)
        self._text_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        self._text_label.setStyleSheet(
            "color: #C8E8FF; font-size: 15px; font-weight: bold;"
            "background: transparent; border: none;"
        )
        layout.addWidget(self._text_label)
        layout.addStretch()

    def text(self) -> str:
        return self._text

    def _refresh_style(self):
        self.style().unpolish(self)
        self.style().polish(self)

    def set_active(self, active: bool):
        self.setProperty("active", active)
        self._refresh_style()

    def mousePressEvent(self, event):
        self.setProperty("pressed", event.button() == Qt.LeftButton)
        self._refresh_style()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        clicked = (event.button() == Qt.LeftButton
                   and self.property("pressed") and self.rect().contains(event.pos()))
        self.setProperty("pressed", False)
        self._refresh_style()
        if clicked:
            self.clicked.emit()
        super().mouseReleaseEvent(event)

    def enterEvent(self, event):
        self.setProperty("hover", True)
        self._refresh_style()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.setProperty("hover", False)
        self.setProperty("pressed", False)
        self._refresh_style()
        super().leaveEvent(event)


class InteractionLog(QTextEdit):
    def append(self, text):
        super().append(text)
        logging.getLogger("go2.ui").info("%s", text)


class MainControlPanel(QWidget):

    _sig_voice_state = pyqtSignal(str)
    _sig_voice_asr = pyqtSignal(str)
    _sig_voice_response = pyqtSignal(str)
    _sig_voice_error = pyqtSignal(str)
    _sig_voice_status = pyqtSignal(str)

    _sig_vlm_analysis = pyqtSignal(str)
    _sig_vlm_decision = pyqtSignal(str)
    _sig_vlm_error = pyqtSignal(str)

    def __init__(self, right_panel=None, parent=None):
        super().__init__(parent)
        self.voice_btn: ModeButton = None
        self.gesture_btn: ModeButton = None
        self.eye_btn: ModeButton = None
        self.vlm_btn: ModeButton = None
        self.mode_label: QLabel = None
        self.video_widget: VideoWidget = None
        self.log_top: QTextEdit = None
        self.log_bottom: QTextEdit = None
        self.send_btn: QPushButton = None
        self.top_left: NeonFrame = None
        self.top_right: NeonFrame = None
        self.voice_frame: NeonFrame = None
        self.gesture_frame: NeonFrame = None
        self.bottom_right: NeonFrame = None
        self.voice_wave: VoiceWaveWidget = None
        self.voice_asr_text: QTextEdit = None
        self.voice_cmd_text: QTextEdit = None
        self.gesture_cmd_label: QLabel = None
        self.gesture_hint: QLabel = None
        self._mode_manager = ExclusiveActivityManager()
        self._starting_mode = None
        self._visual_busy = False
        self._overlay = None
        self.hand_detector = HandDetector()
        self.gesture_controller: GestureController | None = None
        self._last_gesture_cmd: str | None = None
        self.speech_ctrl: SpeechController | None = None
        self.right_panel = right_panel
        self.vlm_ctrl: VLMController | None = None

        self._sig_voice_state.connect(self._on_voice_state)
        self._sig_voice_asr.connect(self._on_voice_asr)
        self._sig_voice_response.connect(self._on_voice_response)
        self._sig_voice_error.connect(self._on_voice_error)
        self._sig_voice_status.connect(self._on_voice_status)

        self._sig_vlm_analysis.connect(self._on_vlm_analysis)
        self._sig_vlm_decision.connect(self._on_vlm_decision)
        self._sig_vlm_error.connect(self._on_vlm_error)

        self._build()
        for mode_name in ("语音交互", "手势控制", "眼动控制", "视觉感知"):
            self._mode_manager.register(
                mode_name,
                lambda name=mode_name: self._activate_mode(name),
                lambda name=mode_name: self._deactivate_mode(name),
            )

    def set_right_panel(self, panel):
        self.right_panel = panel
        if panel is not None:
            panel.analyze_requested.connect(self._on_refresh_scene)
        if panel is not None and panel.depth_widget is not None:
            dw = panel.depth_widget
            if (getattr(dw, "provides_color_stream", False)
                    and hasattr(dw, 'get_latest_color_sample')):
                self.video_widget.set_external_color_source(dw.get_latest_color_sample)

    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)
        root.addWidget(self._build_top(), 7)
        root.addWidget(self._build_bottom(), 3)

    def _build_top(self):
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(15)

        self.top_left = NeonFrame("交互模式")
        self.top_left.title_label.setStyleSheet("background: transparent; border: none; font-size: 22px;")
        self.top_left.title_label.setAlignment(Qt.AlignCenter)
        self.top_left.layout.setSpacing(6)
        self.top_left.layout.setContentsMargins(8, 8, 8, 8)

        self.voice_btn = ModeButton(os.path.join(_ICON_DIR, "voice.svg"), "语音交互")
        self.gesture_btn = ModeButton(os.path.join(_ICON_DIR, "gesture.svg"), "手势控制")
        self.eye_btn = ModeButton(os.path.join(_ICON_DIR, "eye.svg"), "眼动控制")
        self.eye_btn.setEnabled(False)
        self.eye_btn.setToolTip("尚未接入眼动仪")
        self.eye_btn.setStyleSheet(self.eye_btn.styleSheet() +
                                  'QFrame#ModeButton:disabled { background: #101824; border-color: #263448; }')
        self.vlm_btn = ModeButton(os.path.join(_ICON_DIR, "vlm.svg"), "视觉感知")

        for btn in (self.voice_btn, self.gesture_btn, self.eye_btn, self.vlm_btn):
            btn.clicked.connect(self._on_mode_btn_clicked)
            self.top_left.layout.addWidget(btn)

        self.top_left.layout.addStretch()

        mode_container = QWidget()
        mode_container.setStyleSheet("background: transparent; border: none;")
        mode_layout = QHBoxLayout(mode_container)
        mode_layout.setContentsMargins(0, 0, 0, 0)
        mode_layout.setSpacing(0)

        mode_title = QLabel("当前模式：")
        mode_title.setStyleSheet("color: #7AA8C8; font-size: 14px; background: transparent; border: none;")
        self.mode_label = QLabel("待机")
        self.mode_label.setStyleSheet(
            "color: #00F0FF; font-size: 14px; font-weight: bold; background: transparent; border: none;"
        )
        mode_layout.addWidget(mode_title)
        mode_layout.addWidget(self.mode_label)
        mode_layout.addStretch()
        self.top_left.layout.addWidget(mode_container)
        self.stop_btn = QPushButton("停止当前功能")
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet(
            "QPushButton {min-height: 28px; padding: 4px; color: #FFB0BD;}"
            "QPushButton:disabled {color: #536477; background: #101824; border-color: #263448;}"
        )
        self.stop_btn.clicked.connect(self.stop_current_mode)
        self.top_left.layout.addWidget(self.stop_btn)

        self.top_right = NeonFrame("实时视图")
        self.top_right.title_label.setStyleSheet("background: transparent; border: none; font-size: 22px;")
        self.top_right.title_label.setAlignment(Qt.AlignCenter)
        self.top_right.layout.setContentsMargins(10, 5, 10, 10)
        self.video_widget = VideoWidget(hand_detector=self.hand_detector)
        self.video_widget.setMinimumHeight(160)
        self.video_widget.landmarks_updated.connect(self._on_landmarks_updated)

        overlay = QWidget()
        overlay.setAttribute(Qt.WA_TransparentForMouseEvents)
        overlay.setStyleSheet("background: transparent;")
        overlay_layout = QHBoxLayout(overlay)
        overlay_layout.setContentsMargins(10, 5, 10, 0)
        overlay_layout.setSpacing(0)

        left_items = [
            (os.path.join(_ICON_DIR, "power.svg"), "电量", "86%"),
            (os.path.join(_ICON_DIR, "wifi.svg"), "连接状态", "稳定"),
            (os.path.join(_ICON_DIR, "fuzai.svg"), "负载", "正常"),
        ]
        left_panel = self._build_overlay_panel(left_items, frame_w=150, frame_h=80, spacing=20, icon_size=40)
        left_wrapper = QWidget()
        left_wrapper.setFixedWidth(150)
        left_wrapper.setStyleSheet("background: transparent;")
        lw_layout = QVBoxLayout(left_wrapper)
        lw_layout.setContentsMargins(0, 0, 0, 20)
        lw_layout.setSpacing(0)
        lw_layout.addStretch()
        lw_layout.addWidget(left_panel)
        overlay_layout.addWidget(left_wrapper)
        overlay_layout.addStretch()

        right_items = [
            (os.path.join(_ICON_DIR, "ganzhi.svg"), "环境感知", "开启"),
            (os.path.join(_ICON_DIR, "chuangan.svg"), "传感器", "正常"),
            (os.path.join(_ICON_DIR, "gpu.svg"), "GPU", "67%"),
        ]
        right_panel = self._build_overlay_panel(right_items, frame_w=150, frame_h=80, spacing=20, icon_size=40)
        right_wrapper = QWidget()
        right_wrapper.setFixedWidth(150)
        right_wrapper.setStyleSheet("background: transparent;")
        rw_layout = QVBoxLayout(right_wrapper)
        rw_layout.setContentsMargins(0, 0, 0, 20)
        rw_layout.setSpacing(0)
        rw_layout.addStretch()
        rw_layout.addWidget(right_panel)
        overlay_layout.addWidget(right_wrapper)

        video_container = QWidget()
        vc_layout = QGridLayout(video_container)
        vc_layout.setContentsMargins(0, 0, 0, 0)
        vc_layout.setSpacing(0)
        vc_layout.addWidget(self.video_widget, 0, 0)
        vc_layout.addWidget(overlay, 0, 0)
        overlay.hide()
        self._overlay = overlay

        self.top_right.layout.addWidget(video_container, 1)

        layout.addWidget(self.top_left, 1)
        layout.addWidget(self.top_right, 3)
        return container

    def _build_bottom(self):
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        bottom_left = QWidget()
        bl_layout = QHBoxLayout(bottom_left)
        bl_layout.setContentsMargins(0, 0, 0, 0)
        bl_layout.setSpacing(10)

        self.voice_frame = NeonFrame("语音交互")
        self.voice_frame.title_label.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self.voice_frame.title_label.setStyleSheet("background: transparent; border: none; font-size: 22px;")
        self.voice_frame.title_label.setFixedHeight(26)
        self.voice_frame.layout.setContentsMargins(10, 10, 10, 10)
        self.voice_frame.layout.setSpacing(10)

        self.voice_wave = VoiceWaveWidget()

        voice_mid = QWidget()
        voice_mid.setStyleSheet("background: transparent; border: none;")
        mid_layout = QHBoxLayout(voice_mid)
        mid_layout.setContentsMargins(0, 0, 0, 0)
        mid_layout.setSpacing(8)
        mid_label = QLabel("识别文本")
        mid_label.setStyleSheet("color: #00F0FF; font-size: 20px; font-weight: bold; background: transparent; border: none;")
        self.voice_asr_text = QTextEdit()
        self.voice_asr_text.setReadOnly(True)
        self.voice_asr_text.setStyleSheet(
            "color: #C8E8FF; font-size: 14px;"
            "background-color: rgba(8, 18, 30, 200);"
            "border: 1px solid rgba(10, 110, 255, 100);"
            "border-radius: 6px;"
            "min-height: 50px; max-height: 50px;"
        )
        mid_layout.addWidget(mid_label)
        mid_layout.addWidget(self.voice_asr_text, 1)

        voice_bottom = QWidget()
        voice_bottom.setStyleSheet("background: transparent; border: none;")
        bot_layout = QHBoxLayout(voice_bottom)
        bot_layout.setContentsMargins(0, 0, 0, 0)
        bot_layout.setSpacing(8)
        bot_label = QLabel("指令响应")
        bot_label.setStyleSheet("color: #00F0FF; font-size: 20px; font-weight: bold; background: transparent; border: none;")
        self.voice_cmd_text = QTextEdit()
        self.voice_cmd_text.setReadOnly(True)
        self.voice_cmd_text.setStyleSheet(
            "color: #C8E8FF; font-size: 14px;"
            "background-color: rgba(8, 18, 30, 200);"
            "border: 1px solid rgba(10, 110, 255, 100);"
            "border-radius: 6px;"
            "min-height: 50px; max-height: 50px;"
        )
        bot_layout.addWidget(bot_label)
        bot_layout.addWidget(self.voice_cmd_text, 1)

        self.voice_frame.layout.addWidget(self.voice_wave)
        self.voice_frame.layout.addWidget(voice_mid)
        self.voice_frame.layout.addWidget(voice_bottom)

        self.gesture_frame = NeonFrame("手势控制")
        self.gesture_frame.title_label.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self.gesture_frame.title_label.setStyleSheet("background: transparent; border: none; font-size: 22px;")
        self.gesture_frame.title_label.setFixedHeight(26)
        self.gesture_frame.layout.setSpacing(10)
        self.gesture_frame.layout.setContentsMargins(10, 10, 10, 10)

        gesture_container = QWidget()
        gesture_container.setStyleSheet("background: transparent;")
        gc_layout = QGridLayout(gesture_container)
        gc_layout.setContentsMargins(0, 0, 0, 0)
        gc_layout.setSpacing(0)

        self.gesture_hint = QLabel("等待手势输入...")
        self.gesture_hint.setAlignment(Qt.AlignCenter)
        self.gesture_hint.setScaledContents(True)
        self.gesture_hint.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.gesture_hint.setStyleSheet(
            "color: #60D0E8; font-size: 16px;"
            "background-color: rgba(8, 18, 30, 200);"
            "border: 1px solid rgba(10, 110, 255, 100);"
            "border-radius: 6px;"
        )

        self.gesture_cmd_label = QLabel("识别指令：等待输入")
        self.gesture_cmd_label.setStyleSheet(
            "color: #00F0FF; font-size: 18px; font-weight: bold; background: transparent; border: none;"
        )

        cmd_wrapper = QWidget()
        cmd_wrapper.setStyleSheet("background: transparent;")
        cmd_wrapper_layout = QVBoxLayout(cmd_wrapper)
        cmd_wrapper_layout.setContentsMargins(8, 0, 0, 8)
        cmd_wrapper_layout.setSpacing(0)
        cmd_wrapper_layout.addWidget(self.gesture_cmd_label)

        gc_layout.addWidget(self.gesture_hint, 0, 0)
        gc_layout.addWidget(cmd_wrapper, 0, 0, Qt.AlignLeft | Qt.AlignBottom)
        self.gesture_frame.layout.addWidget(gesture_container)

        bl_layout.addWidget(self.voice_frame, 6)
        bl_layout.addWidget(self.gesture_frame, 4)

        self.bottom_right = NeonFrame("交互日志")
        self.bottom_right.title_label.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        self.bottom_right.title_label.setStyleSheet("background: transparent; border: none; font-size: 22px;")
        self.bottom_right.title_label.setFixedHeight(26)
        self.bottom_right.layout.setContentsMargins(10, 10, 10, 10)
        self.bottom_right.layout.setSpacing(10)

        log_top = InteractionLog()
        log_top.setReadOnly(True)
        log_top.setStyleSheet(
            "color: #C8E8FF; font-size: 13px;"
            "background-color: rgba(8, 18, 30, 200);"
            "border: 1px solid rgba(10, 110, 255, 100);"
            "border-radius: 6px;"
        )
        self.log_top = log_top
        self.log_top.document().setMaximumBlockCount(500)

        log_bottom_container = QWidget()
        log_bottom_container.setStyleSheet("background: transparent;")
        lb_layout = QHBoxLayout(log_bottom_container)
        lb_layout.setContentsMargins(0, 0, 0, 0)
        lb_layout.setSpacing(5)

        log_bottom = QTextEdit()
        log_bottom.setStyleSheet(
            "color: #C8E8FF; font-size: 13px;"
            "background-color: rgba(8, 18, 30, 200);"
            "border: 1px solid rgba(10, 110, 255, 100);"
            "border-radius: 6px;"
        )
        self.log_bottom = log_bottom
        self.log_bottom.setPlaceholderText("启用视觉感知后，可输入画面问题继续对话（Ctrl+Enter 发送）")
        self._send_shortcut = QShortcut(QKeySequence("Ctrl+Return"), self.log_bottom)
        self._send_shortcut.activated.connect(self._on_send_clicked)

        self.send_btn = QPushButton("发 送")
        self.send_btn.setFixedSize(60, 60)
        self.send_btn.setCursor(Qt.PointingHandCursor)
        self.send_btn.setStyleSheet(
            "QPushButton {"
            "  color: #00F0FF; font-size: 13px; font-weight: bold;"
            "  background-color: rgba(12, 30, 58, 220);"
            "  border: 1px solid #1A5080; border-radius: 4px;"
            "  min-height: 60px; max-height: 60px;"
            "}"
            "QPushButton:hover { background-color: #163358; border: 1px solid #2898D0; }"
            "QPushButton:pressed { background-color: #081428; border: 1px solid #0A5A80; }"
        )
        self.send_btn.clicked.connect(self._on_send_clicked)
        self.send_btn.setEnabled(False)
        lb_layout.addWidget(self.send_btn)
        lb_layout.addWidget(log_bottom, 1)

        self.bottom_right.layout.addWidget(log_top, 7)
        self.bottom_right.layout.addWidget(log_bottom_container, 3)

        layout.addWidget(bottom_left, 2)
        layout.addWidget(self.bottom_right, 1)
        return container

    def toggle_overlay(self):
        if self._overlay is not None:
            self._overlay.setVisible(not self._overlay.isVisible())

    def enter_safe_idle(self):
        """Stop interactive motion before handing control to autonomous Nav2."""
        self._mode_manager.deactivate()
        self._sync_mode_buttons()
        self.mode_label.setText("待机")

        # Stop microphone capture/ASR even if the mode button state was lost
        # during a rapid page switch.
        self._stop_speech()

        controllers = [self.gesture_controller, self.speech_ctrl, self.vlm_ctrl]
        robots = []
        for controller in controllers:
            robot = getattr(controller, "_robot", None) if controller is not None else None
            if robot is not None and getattr(robot, "client", None) is not None:
                try:
                    robot.execute_action("stop", speed=0.0, duration=0.0)
                except Exception:
                    pass
            if robot is not None and all(id(robot) != id(item) for item in robots):
                robots.append(robot)

        # Voice/gesture/VLM share an isolated ROS command bridge. Release it
        # when autonomous navigation takes control so no interactive lease remains.
        for robot in robots:
            try:
                robot.shutdown()
            except Exception:
                pass

    def shutdown(self):
        self.enter_safe_idle()
        self._stop_speech()
        if self.video_widget is not None:
            self.video_widget.shutdown()
        controllers = [self.gesture_controller, self.speech_ctrl, self.vlm_ctrl]
        seen = set()
        for controller in controllers:
            robot = getattr(controller, "_robot", None) if controller is not None else None
            if robot is None or id(robot) in seen:
                continue
            seen.add(id(robot))
            if hasattr(robot, "shutdown"):
                robot.shutdown()
        speech_shutdown = getattr(self.speech_ctrl, "shutdown", None)
        if callable(speech_shutdown):
            speech_shutdown()

    # MediaPipe 手部 21 点连接关系
    _HAND_CONNECTIONS = [
        (0, 1), (1, 2), (2, 3), (3, 4),        # 拇指
        (0, 5), (5, 6), (6, 7), (7, 8),        # 食指
        (0, 9), (9, 10), (10, 11), (11, 12),   # 中指
        (0, 13), (13, 14), (14, 15), (15, 16), # 无名指
        (0, 17), (17, 18), (18, 19), (19, 20), # 小指
        (5, 9), (9, 13), (13, 17),              # 掌心横连
    ]

    def _on_landmarks_updated(self, data: tuple):
        if self._mode_manager.active != "手势控制":
            return
        landmarks, handedness, _ = data
        if self.gesture_controller is None:
            return

        if not landmarks:
            self.gesture_hint.setText("未检测到手部")
            self.gesture_hint.setPixmap(QPixmap())
            stop_motion = getattr(self.gesture_controller, "stop_motion", None)
            if callable(stop_motion):
                stop_motion()
            self.gesture_controller.reset()
            self._last_gesture_cmd = None
            return

        gestures_dict = GestureController.classify_all(landmarks, handedness)
        landmark_counts = {h: len(lm) for h, lm in zip(handedness, landmarks)}
        confirmed_cmd, toggle_event = self.gesture_controller.update(gestures_dict, landmark_counts)

        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")

        if toggle_event:
            self.log_top.append(f"[{timestamp}] 手势：{toggle_event}")
            self.gesture_cmd_label.setText(f"状态：{toggle_event}")

        if confirmed_cmd and confirmed_cmd != self._last_gesture_cmd:
            self._last_gesture_cmd = confirmed_cmd
            self.gesture_cmd_label.setText(f"识别指令：{confirmed_cmd}")
            self.log_top.append(f"[{timestamp}] 手势指令识别：{confirmed_cmd}")

        # ---- 绘制骨架 ----
        pw, ph = 400, 300
        pixmap = QPixmap(pw, ph)
        pixmap.fill(QColor(8, 18, 30))

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        dot_color = QColor(0, 240, 255)
        line_color = QColor(0, 240, 255, 120)
        wrist_color = QColor(0, 255, 200)

        for hand_points in landmarks:
            points_xy = []
            for px, py, _ in hand_points:
                x = int(40 + px * (pw - 80))
                y = int(30 + py * (ph - 60))
                points_xy.append((x, y))

            painter.setPen(QPen(line_color, 2))
            for a, b in self._HAND_CONNECTIONS:
                painter.drawLine(points_xy[a][0], points_xy[a][1],
                                 points_xy[b][0], points_xy[b][1])

            for i, (x, y) in enumerate(points_xy):
                r = 4 if i == 0 else 3
                painter.setPen(Qt.NoPen)
                painter.setBrush(wrist_color if i == 0 else dot_color)
                painter.drawEllipse(QPointF(x, y), r, r)

        painter.end()
        self.gesture_hint.setPixmap(pixmap)
        self.gesture_hint.setText("")

    def _on_mode_btn_clicked(self):
        btn = self.sender()
        if not btn:
            return
        text = btn.text().split(" ", 1)[-1]

        self._starting_mode = text
        try:
            self._mode_manager.activate(text)
        except Exception as exc:
            self.log_top.append(f"功能启动失败：{exc}")
        finally:
            self._starting_mode = None
            self._sync_mode_buttons()

    def stop_current_mode(self):
        self._mode_manager.deactivate()
        self._sync_mode_buttons()

    def _sync_mode_buttons(self):
        active = self._mode_manager.active
        self.send_btn.setEnabled(active == "视觉感知" and not self._visual_busy)
        self.stop_btn.setEnabled(active is not None)
        for btn in (self.voice_btn, self.gesture_btn, self.eye_btn, self.vlm_btn):
            btn.set_active(btn.text() == active)
        if active is None:
            self.mode_label.setText("待机")
        else:
            self.mode_label.setText(active)

    def _activate_mode(self, text: str):
        if text == "语音交互":
            if self.voice_wave is not None:
                self.voice_wave.set_state("listening")
            started = self._start_speech()
            if not started and self.voice_wave is not None:
                self.voice_wave.set_state("off")
            return started
        elif text == "手势控制":
            if not self.hand_detector.available:
                self.gesture_cmd_label.setText("MediaPipe 未就绪，手势识别不可用")
                ts = QDateTime.currentDateTime().toString("hh:mm:ss")
                self.log_top.append(f"[{ts}] ⚠️ MediaPipe Hands 未加载")
                return False
            if self.gesture_controller is None:
                self.gesture_controller = GestureController()
            self._resume_robot(self.gesture_controller)
            if self.video_widget is not None:
                self.video_widget.set_hand_detection(True)
            self.gesture_controller.is_system_active = True
            self.gesture_controller.is_lying_down = False
            self.gesture_controller.reset()
            self._last_gesture_cmd = None
            self.gesture_cmd_label.setText("手势已启动 — 请做控制手势")
            return True
        elif text == "视觉感知":
            if self.voice_wave is not None:
                self.voice_wave.set_state("off")
            return self._vlm_analyze_scene()
        elif text == "眼动控制":
            self.mode_label.setText("眼动控制（未接入眼动仪）")
            ts = QDateTime.currentDateTime().toString("hh:mm:ss")
            self.log_top.append(f"[{ts}] ⚠️ 当前工程未配置眼动仪驱动/标定参数")
            return False
        else:
            if self.voice_wave is not None:
                self.voice_wave.set_state("off")
            return False

    def _deactivate_mode(self, text: str):
        if text == "语音交互":
            self._stop_speech()
            self._release_robot(self.speech_ctrl, send_stop=False)
            if self.voice_wave is not None:
                self.voice_wave.set_state("off")
            self.voice_asr_text.clear()
            self.voice_cmd_text.clear()
        elif text == "手势控制":
            stop_motion = getattr(self.gesture_controller, "stop_motion", None)
            if callable(stop_motion):
                stop_motion()
            if self.video_widget is not None:
                self.video_widget.set_hand_detection(False)
            self.gesture_hint.setText("等待手势输入...")
            self.gesture_cmd_label.setText("识别指令：等待输入")
            if self.gesture_controller is not None:
                self.gesture_controller.is_system_active = False
                self.gesture_controller.reset()
            self._last_gesture_cmd = None
            self._release_robot(self.gesture_controller, send_stop=False)
        elif text == "视觉感知":
            self._stop_vlm()
        else:
            if self.voice_wave is not None:
                self.voice_wave.set_state("off")

    def _start_speech(self):
        if self.speech_ctrl is None:
            frame_provider = None
            depth_provider = None
            if self.right_panel is not None and self.right_panel.depth_widget is not None:
                depth_widget = self.right_panel.depth_widget
                frame_provider = getattr(depth_widget, "get_latest_color_frame", None)
                depth_provider = getattr(depth_widget, "get_latest_depth_frame", None)
            self.speech_ctrl = SpeechController(
                frame_provider=frame_provider,
                depth_provider=depth_provider,
            )
            self.speech_ctrl.on_state = self._sig_voice_state.emit
            self.speech_ctrl.on_asr = self._sig_voice_asr.emit
            self.speech_ctrl.on_response = self._sig_voice_response.emit
            self.speech_ctrl.on_error = self._sig_voice_error.emit
            self.speech_ctrl.on_status = self._sig_voice_status.emit
        # Motion discovery is deferred until an actual voice action is requested.
        if not self.speech_ctrl.is_running:
            self.voice_asr_text.clear()
            self.voice_cmd_text.clear()
            started = self.speech_ctrl.start()
            if started is False:
                ts = QDateTime.currentDateTime().toString("hh:mm:ss")
                self.log_top.append(f"[{ts}] ⚠️ 语音功能未就绪，未重复启动")
                return False
        return self.speech_ctrl.is_running

    def _stop_speech(self):
        if self.speech_ctrl and self.speech_ctrl.is_running:
            self.speech_ctrl.stop()

    def _on_voice_state(self, state: str):
        if (self._mode_manager.active != "语音交互"
                and self._starting_mode != "语音交互"):
            return
        if self.voice_wave is not None:
            self.voice_wave.set_state(state)
        if (state == "idle" and self.speech_ctrl is not None
                and not self.speech_ctrl.is_running
                and self._mode_manager.active == "语音交互"):
            self._mode_manager.deactivate()
            self._sync_mode_buttons()

    def _on_voice_asr(self, text: str):
        if self._mode_manager.active != "语音交互" and self._starting_mode != "语音交互":
            return
        self.voice_asr_text.setPlainText(text)
        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_top.append(f"[{ts}] 🎤 语音识别：{text}")

    def _on_voice_response(self, reply: str):
        if self._mode_manager.active != "语音交互" and self._starting_mode != "语音交互":
            return
        self.voice_cmd_text.setPlainText(reply)
        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_top.append(f"[{ts}] 🤖 邦邦：{reply}")

    def _on_voice_error(self, msg: str):
        if self._mode_manager.active != "语音交互" and self._starting_mode != "语音交互":
            return
        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_top.append(f"[{ts}] ⚠️ 语音错误：{msg}")

    def _on_voice_status(self, message: str):
        if self._mode_manager.active != "语音交互" and self._starting_mode != "语音交互":
            return
        self.voice_wave.set_status(message)
        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_top.append(f"[{ts}] 语音状态：{message}")

    def _ensure_vlm(self):
        if self.vlm_ctrl is None:
            self.vlm_ctrl = VLMController()
            self.vlm_ctrl.on_error = self._sig_vlm_error.emit
        return self.vlm_ctrl.is_ready

    @staticmethod
    def _resume_robot(controller):
        robot = getattr(controller, "_robot", None) if controller is not None else None
        reconnect = getattr(robot, "reconnect", None)
        if callable(reconnect):
            reconnect()

    @staticmethod
    def _release_robot(controller, send_stop: bool = True):
        robot = getattr(controller, "_robot", None) if controller is not None else None
        if robot is None:
            return
        if send_stop and getattr(robot, "client", None) is not None:
            try:
                robot.execute_action("stop", speed=0.0, duration=0.0)
            except Exception:
                pass
        try:
            robot.shutdown()
        except Exception:
            pass

    def _stop_vlm(self):
        self._set_visual_busy(False)
        if self.vlm_ctrl is None:
            return
        cancel = getattr(self.vlm_ctrl, "cancel_pending", None)
        if callable(cancel):
            cancel()

    def _set_visual_busy(self, busy):
        self._visual_busy = busy
        self.send_btn.setEnabled(not busy and self._mode_manager.active == "视觉感知")
        if self.right_panel is not None:
            self.right_panel.refresh_btn.setEnabled(not busy)

    def _get_scene_frame(self):
        # Recognition must use the D435i source itself, never the Go2/local
        # fallback preview, and never a stale frame left after disconnection.
        depth_widget = getattr(self.right_panel, "depth_widget", None)
        if depth_widget is not None:
            frame = depth_widget.get_latest_color_frame()
            if frame is not None:
                return frame
        message = "当前彩色画面不可用，请检查相机连接并等待出图。"
        self.log_top.append(message)
        if self.right_panel is not None:
            self.right_panel.set_analysis(message)
        return None

    def _on_refresh_scene(self):
        if self._mode_manager.active == "视觉感知":
            self._vlm_analyze_scene()
        else:
            self._starting_mode = "视觉感知"
            try:
                self._mode_manager.activate("视觉感知")
            except Exception as exc:
                self.log_top.append(f"画面识别启动失败：{exc}")
            finally:
                self._starting_mode = None
                self._sync_mode_buttons()

    def _vlm_analyze_scene(self):
        if self._visual_busy:
            return False
        frame = self._get_scene_frame()
        if frame is None:
            ts = QDateTime.currentDateTime().toString("hh:mm:ss")
            self.log_top.append(f"[{ts}] ⚠️ 本次未识别画面")
            return False

        if not self._ensure_vlm():
            ts = QDateTime.currentDateTime().toString("hh:mm:ss")
            self.log_top.append(f"[{ts}] ⚠️ VLM 客户端初始化失败")
            return False

        if self.right_panel:
            self.right_panel.set_analysis("正在读取当前画面，请稍候...")

        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_top.append(f"[{ts}] 🔍 已抓取当前画面，正在分析场景...")

        self._set_visual_busy(True)
        started = self.vlm_ctrl.analyze_scene_async(
            frame,
            on_result=self._sig_vlm_analysis.emit,
        )
        if started is False:
            self._set_visual_busy(False)
            self.log_top.append(f"[{ts}] ⚠️ 已有视觉任务运行，未重复启动")
            return False
        return True

    def _on_send_clicked(self):
        user_text = self.log_bottom.toPlainText().strip()
        if not user_text:
            return

        if self._mode_manager.active != "视觉感知":
            ts = QDateTime.currentDateTime().toString("hh:mm:ss")
            self.log_top.append(f"[{ts}] ⚠️ 请先启用视觉感知，未启动后台任务")
            return

        if self._visual_busy:
            return

        if not self._ensure_vlm():
            ts = QDateTime.currentDateTime().toString("hh:mm:ss")
            self.log_top.append(f"[{ts}] ⚠️ VLM 客户端未就绪")
            return

        frame = self._get_scene_frame()
        if frame is None:
            ts = QDateTime.currentDateTime().toString("hh:mm:ss")
            self.log_top.append(f"[{ts}] ⚠️ 无法获取摄像头画面")
            return

        if self.right_panel:
            self.right_panel.set_decision("正在结合当前画面回答...")

        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self._set_visual_busy(True)
        started = self.vlm_ctrl.query_scene_async(
            frame, user_text,
            on_result=self._sig_vlm_decision.emit,
        )
        if started is False:
            self._set_visual_busy(False)
            self.log_top.append(f"[{ts}] ⚠️ 已有视觉任务运行，未重复启动")
        else:
            self.log_top.append(f"[{ts}] 你：{user_text}")
            self.log_bottom.clear()

    def _on_vlm_analysis(self, text: str):
        if self._mode_manager.active != "视觉感知":
            return
        self._set_visual_busy(False)
        if self.right_panel:
            self.right_panel.set_analysis(text)
        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_top.append(f"[{ts}] 🔍 当前画面：{text}")

    def _on_vlm_decision(self, text: str):
        if self._mode_manager.active != "视觉感知":
            return
        self._set_visual_busy(False)
        if self.right_panel:
            self.right_panel.set_decision(text)
        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_top.append(f"[{ts}] 邦邦：{text}")

    def _on_vlm_error(self, msg: str):
        if self._mode_manager.active != "视觉感知":
            return
        self._set_visual_busy(False)
        if self.right_panel:
            self.right_panel.set_decision(msg)
        ts = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_top.append(f"[{ts}] ⚠️ VLM 错误：{msg}")

    def _build_overlay_panel(self, items, frame_w=140, frame_h=58, spacing=6, icon_size=28):
        panel = QWidget()
        panel.setStyleSheet("background: transparent;")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(spacing)

        for icon_path, title, value in items:
            frame = QFrame()
            frame.setFixedSize(frame_w, frame_h)
            frame.setStyleSheet(
                "QFrame {"
                "  background-color: rgba(10, 18, 32, 180);"
                "  border: 1px solid rgba(10, 110, 255, 130);"
                "  border-radius: 6px;"
                "}"
            )
            fl = QHBoxLayout(frame)
            fl.setContentsMargins(0, 0, 0, 0)
            fl.setSpacing(30)
            fl.setAlignment(Qt.AlignVCenter)
            fl.addStretch(1)

            icon_label = QLabel()
            icon_label.setFixedSize(icon_size, icon_size)
            icon_label.setStyleSheet("background: transparent; border: none;")
            pix = QPixmap(icon_path).scaled(icon_size, icon_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            icon_label.setPixmap(pix)
            fl.addWidget(icon_label)

            text_col = QWidget()
            text_col.setFixedWidth(55)
            text_col.setStyleSheet("background: transparent; border: none;")
            tc = QVBoxLayout(text_col)
            tc.setContentsMargins(0, 0, 0, 0)
            tc.setSpacing(0)
            tc.setAlignment(Qt.AlignVCenter)

            tl = QLabel(title)
            tl.setAlignment(Qt.AlignCenter)
            tl.setStyleSheet("color: #7AA8C8; font-size: 12px; background: transparent; border: none;")
            vl = QLabel(value)
            vl.setAlignment(Qt.AlignCenter)
            vl.setStyleSheet("color: #00F0FF; font-size: 16px; font-weight: bold; background: transparent; border: none;")
            tc.addWidget(tl)
            tc.addWidget(vl)

            fl.addWidget(text_col)
            fl.addStretch(1)
            layout.addWidget(frame)

        layout.addStretch()
        return panel
