"""Current 2.5D terrain navigation, displayed through the deployed RViz profile."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QPointF, QProcess, QProcessEnvironment, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from go2_ui.navigation.ros_bridge import RosNavigationWorker
from go2_ui.navigation.rviz_view import RvizView, PROJECT_ROOT
from go2_ui.ui.widgets.neon_frame import NeonFrame


class MapCanvas(QWidget):
    goal_selected = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 320)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setCursor(Qt.CrossCursor)
        self._image = None
        self._scaled_map = None
        self._scaled_map_size = None
        self._map_revision = 0
        self._scaled_map_revision = -1
        self._pending_map = None
        self._map_color_table = self._build_map_color_table()
        self._meta = None
        self._image_rect = QRectF()
        self._robot = None
        self._goal = None
        self._path = []
        self._paint_pending = True
        self._paint_timer = QTimer(self)
        self._paint_timer.setInterval(50)  # 20 FPS: smooth pose, bounded GUI load.
        self._paint_timer.timeout.connect(self._flush_repaint)
        self._paint_timer.start()

    def _request_repaint(self) -> None:
        self._paint_pending = True

    def set_active(self, active: bool) -> None:
        if active:
            if not self._paint_timer.isActive():
                self._paint_timer.start()
            self._flush_repaint()
        else:
            self._paint_timer.stop()

    def _flush_repaint(self) -> None:
        if self._pending_map is not None:
            payload = self._pending_map
            self._pending_map = None
            self._apply_map(payload)
        if self._paint_pending:
            self._paint_pending = False
            self.update()

    def set_map(self, payload: dict) -> None:
        # Keep only the newest map. Occupancy-grid conversion is deliberately
        # deferred to the 20 FPS paint tick so queued ROS messages cannot make
        # the GUI process stale maps while navigation is active.
        self._pending_map = payload
        self._request_repaint()

    @staticmethod
    def _build_map_color_table():
        unknown = QColor(8, 20, 34).rgb()
        colors = [unknown] * 256
        for occupancy in range(101):
            intensity = max(28, min(238, 238 - occupancy * 2))
            colors[occupancy] = QColor(intensity, intensity, intensity).rgb()
        return colors

    def _apply_map(self, payload: dict) -> None:
        width = payload["width"]
        height = payload["height"]
        raw = np.frombuffer(payload["data"], dtype=np.uint8)
        if width <= 0 or height <= 0 or raw.size != width * height:
            return
        # ROS OccupancyGrid rows run bottom-to-top; QImage rows run top-to-bottom.
        grid = np.ascontiguousarray(np.flipud(raw.reshape((height, width))))
        image = QImage(grid.data, width, height, width, QImage.Format_Indexed8).copy()
        image.setColorTable(self._map_color_table)
        self._image = image
        self._meta = payload
        self._map_revision += 1
        self._scaled_map = None
        self._scaled_map_size = None
        self._request_repaint()

    def set_robot_pose(self, x: float, y: float, yaw: float) -> None:
        self._robot = (x, y, yaw)
        self._request_repaint()

    def set_goal(self, x: float, y: float, yaw: float | None) -> None:
        self._goal = (x, y, yaw)
        self._request_repaint()

    def set_path(self, points) -> None:
        self._path = points or []
        self._request_repaint()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#060C18"))

        if self._image is None:
            painter.setPen(QColor("#4A8098"))
            painter.setFont(QFont("Microsoft YaHei", 20))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待当前导航地图数据…")
            painter.end()
            return

        margin = 14.0
        available = QRectF(self.rect()).adjusted(margin, margin, -margin, -margin)
        scale = min(
            available.width() / self._image.width(),
            available.height() / self._image.height(),
        )
        draw_w = max(1, int(self._image.width() * scale))
        draw_h = max(1, int(self._image.height() * scale))
        self._image_rect = QRectF(
            available.center().x() - draw_w / 2,
            available.center().y() - draw_h / 2,
            draw_w,
            draw_h,
        )
        scaled_size = (draw_w, draw_h)
        if (
            self._scaled_map is None
            or self._scaled_map_size != scaled_size
            or self._scaled_map_revision != self._map_revision
        ):
            self._scaled_map = QPixmap.fromImage(self._image).scaled(
                draw_w, draw_h, Qt.IgnoreAspectRatio, Qt.FastTransformation
            )
            self._scaled_map_size = scaled_size
            self._scaled_map_revision = self._map_revision
        painter.drawPixmap(self._image_rect.topLeft(), self._scaled_map)
        painter.setPen(QPen(QColor(0, 240, 255, 120), 1.5))
        painter.drawRect(self._image_rect)

        if len(self._path) >= 2:
            points = [self._world_to_screen(x, y) for x, y in self._path]
            points = [p for p in points if p is not None]
            if len(points) >= 2:
                painter.setPen(QPen(QColor(168, 85, 247, 210), 3.0))
                painter.drawPolyline(QPolygonF(points))

        if self._goal is not None:
            self._draw_pose(painter, self._goal, QColor("#FFB020"), 13.0)
        if self._robot is not None:
            self._draw_pose(painter, self._robot, QColor("#00F0FF"), 11.0)

        painter.end()

    def _draw_pose(self, painter: QPainter, pose, color: QColor, radius: float) -> None:
        point = self._world_to_screen(pose[0], pose[1])
        if point is None:
            return
        painter.setBrush(QColor(color.red(), color.green(), color.blue(), 90))
        painter.setPen(QPen(color, 2.2))
        painter.drawEllipse(point, radius, radius)
        if pose[2] is None:
            painter.drawLine(
                QPointF(point.x() - radius * 0.55, point.y()),
                QPointF(point.x() + radius * 0.55, point.y()),
            )
            painter.drawLine(
                QPointF(point.x(), point.y() - radius * 0.55),
                QPointF(point.x(), point.y() + radius * 0.55),
            )
            return
        length = radius * 2.1
        painter.drawLine(
            point,
            QPointF(
                point.x() + math.cos(pose[2]) * length,
                point.y() - math.sin(pose[2]) * length,
            ),
        )

    def resizeEvent(self, event) -> None:
        self._scaled_map = None
        self._scaled_map_size = None
        super().resizeEvent(event)

    def _world_to_screen(self, x: float, y: float):
        if not self._meta or self._image_rect.isEmpty():
            return None
        dx = x - self._meta["origin_x"]
        dy = y - self._meta["origin_y"]
        yaw = self._meta["origin_yaw"]
        mx = (math.cos(yaw) * dx + math.sin(yaw) * dy) / self._meta["resolution"]
        my = (-math.sin(yaw) * dx + math.cos(yaw) * dy) / self._meta["resolution"]
        px = self._image_rect.left() + mx / self._meta["width"] * self._image_rect.width()
        py = self._image_rect.bottom() - my / self._meta["height"] * self._image_rect.height()
        return QPointF(px, py)

    def _screen_to_world(self, point: QPointF):
        if not self._meta or not self._image_rect.contains(point):
            return None
        mx = (point.x() - self._image_rect.left()) / self._image_rect.width()
        my = (self._image_rect.bottom() - point.y()) / self._image_rect.height()
        local_x = mx * self._meta["width"] * self._meta["resolution"]
        local_y = my * self._meta["height"] * self._meta["resolution"]
        yaw = self._meta["origin_yaw"]
        x = self._meta["origin_x"] + math.cos(yaw) * local_x - math.sin(yaw) * local_y
        y = self._meta["origin_y"] + math.sin(yaw) * local_x + math.cos(yaw) * local_y
        return x, y

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            world = self._screen_to_world(QPointF(event.pos()))
            if world is not None:
                self.goal_selected.emit(*world)
        super().mousePressEvent(event)


class NavigationPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background: transparent;")
        self._launch_process = QProcess(self)
        self._launch_process.setProcessChannelMode(QProcess.MergedChannels)
        self._launch_process.readyReadStandardOutput.connect(self._read_launch_output)
        self._launch_process.started.connect(lambda: self._append_log("导航启动入口已运行，等待系统就绪…"))
        self._launch_process.finished.connect(self._on_process_finished)
        self._launch_process.errorOccurred.connect(self._on_launch_error)
        self._close_process = QProcess(self)
        self._close_process.setProcessChannelMode(QProcess.MergedChannels)
        self._close_process.finished.connect(self._on_close_finished)
        self._close_process.errorOccurred.connect(self._on_close_error)
        self._navigation_closed = False
        self._close_failed = False
        self._active = False
        self._shutting_down = False
        self._autostart_attempted = False
        self._stop_requested = False
        self._startup_error = ''
        self._navigation_state = 'waiting'
        self._last_navigation_message = '等待 ROS 2'
        self._launch_output_tail = []
        self._control_pending = ''
        self._goal_submitting = False
        self._goal_active = False
        self._saving_map = False
        self._capabilities = {}
        self._autostart_timer = QTimer(self)
        self._autostart_timer.setSingleShot(True)
        self._autostart_timer.setInterval(500)
        self._autostart_timer.timeout.connect(self._autostart)
        self._ready_timer = QTimer(self)
        self._ready_timer.setSingleShot(True)
        self._ready_timer.setInterval(90000)
        self._ready_timer.timeout.connect(self._on_startup_timeout)
        self._latest_pose_yaw = 0.0
        self._terrain_display_process = QProcess(self)
        self._terrain_display_process.setProcessChannelMode(QProcess.MergedChannels)
        self._terrain_display_process.readyReadStandardOutput.connect(
            lambda: self._terrain_display_process.readAllStandardOutput())
        self._build()
        self._wire_ros()

    def _build(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(15)

        map_frame = NeonFrame("2.5D 地形导航")
        map_frame.title_label.setAlignment(Qt.AlignCenter)
        map_frame.title_label.setStyleSheet(
            "background: transparent; border: none; font-size: 20px;"
        )
        self.rviz_view = RvizView()
        self.rviz_view.log_message.connect(self._append_log)
        self.rviz_view.expand_requested.connect(self._toggle_map_expanded)
        map_frame.layout.addWidget(self.rviz_view, 1)
        root.addWidget(map_frame, 1)

        side = QWidget()
        side.setStyleSheet("background: transparent;")
        side.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.setSpacing(6)

        status_frame = NeonFrame("导航状态")
        self.status_label = QLabel("● 等待 ROS 2")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(42)
        self._set_status("waiting", "等待 ROS 2")
        self.safety_label = QLabel("等待实时安全检测数据")
        self.safety_label.setAlignment(Qt.AlignCenter)
        self.pose_label = QLabel("位姿: x=--  y=--  yaw=--")
        self.pose_label.setAlignment(Qt.AlignCenter)
        self.feedback_label = QLabel("距离目标: --")
        self.feedback_label.setAlignment(Qt.AlignCenter)
        for label in (self.status_label, self.safety_label, self.pose_label, self.feedback_label):
            label.setWordWrap(True)
            label.setMinimumWidth(0)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        for label in (self.safety_label, self.pose_label, self.feedback_label):
            label.setProperty('role', 'metric')
        status_frame.layout.addWidget(self.status_label)
        status_frame.layout.addWidget(self.safety_label)
        status_frame.layout.addWidget(self.pose_label)
        status_frame.layout.addWidget(self.feedback_label)
        side_layout.addWidget(status_frame)

        launch_frame = NeonFrame("运行模式")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "本体雷达 2.5D / 接入已有导航",
            "仅连接已有导航",
        ])
        self.mode_combo.setToolTip('进入本页时优先接入已有导航，否则用 Go2 本体雷达启动 2.5D 导航')
        launch_buttons = QHBoxLayout()
        self.start_button = QPushButton("启动 / 重试")
        self.start_button.clicked.connect(self._start_stack)
        self.stop_button = QPushButton("关闭导航")
        self.stop_button.setToolTip('取消目标并退出本机导航、定位和底盘控制进程，完成后可使用语音 / 手势')
        self.stop_button.clicked.connect(self._close_navigation)
        launch_buttons.addWidget(self.start_button)
        launch_buttons.addWidget(self.stop_button)
        launch_frame.layout.addWidget(self.mode_combo)
        help_text = QLabel('关闭导航将退出本机导航和底盘控制，完成后可使用语音 / 手势。再次使用导航需点击启动；仅连接模式也可关闭本机已有导航。')
        help_text.setWordWrap(True)
        help_text.setProperty('role', 'hint')
        launch_frame.layout.addWidget(help_text)
        launch_frame.layout.addLayout(launch_buttons)
        side_layout.addWidget(launch_frame)

        pose_frame = NeonFrame("目标坐标")
        grid = QGridLayout()
        self.goal_x = self._spin(-1000.0, 1000.0, 0.0, " m")
        self.goal_y = self._spin(-1000.0, 1000.0, 0.0, " m")
        for row, (label, widget) in enumerate((
            ("X", self.goal_x), ("Y", self.goal_y)
        )):
            axis = QLabel(label)
            axis.setFixedWidth(20)
            axis.setAlignment(Qt.AlignCenter)
            grid.addWidget(axis, row, 0)
            grid.addWidget(widget, row, 1)
        grid.setColumnStretch(1, 1)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        pose_frame.layout.addLayout(grid)
        action_row = QHBoxLayout()
        send = QPushButton("开始导航")
        self.send_goal_button = send
        send.clicked.connect(self._send_goal)
        cancel = QPushButton("取消目标")
        self.cancel_goal_button = cancel
        cancel.clicked.connect(self._cancel_goal)
        action_row.addWidget(send)
        action_row.addWidget(cancel)
        pose_frame.layout.addLayout(action_row)
        aux_row = QHBoxLayout()
        hint = QLabel('可用地形视图中的 Nav2 Goal 选点。手动坐标使用当前地图坐标系。')
        hint.setWordWrap(True)
        hint.setProperty('role', 'hint')
        aux_row.addWidget(hint)
        pose_frame.layout.addLayout(aux_row)
        side_layout.addWidget(pose_frame)

        save_frame = NeonFrame("建图保存")
        save_btn = QPushButton("保存当前环境地图")
        self.save_map_button = save_btn
        save_btn.clicked.connect(self._save_map)
        save_frame.layout.addWidget(save_btn)
        side_layout.addWidget(save_frame)

        stop = QPushButton("急  停")
        stop.setMinimumHeight(48)
        stop.setStyleSheet(
            "QPushButton {color:#FFFFFF; background:#5A1020; border:2px solid #FF4466;"
            "font-size:20px; font-weight:bold;}"
            "QPushButton:hover {background:#8A1830;}"
        )
        stop.clicked.connect(lambda: self.worker.enqueue("emergency_stop"))
        self.emergency_stop_button = stop

        log_frame = NeonFrame("运行日志")
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMinimumHeight(100)
        self.log_box.document().setMaximumBlockCount(300)
        log_frame.layout.addWidget(self.log_box)
        side_layout.addWidget(log_frame, 1)

        side_scroll = QScrollArea()
        side_scroll.setWidgetResizable(True)
        side_scroll.setFrameShape(QScrollArea.NoFrame)
        side_scroll.setWidget(side)
        self.side_column = QWidget()
        self.side_column.setFixedWidth(330)
        column_layout = QVBoxLayout(self.side_column)
        column_layout.setContentsMargins(0, 0, 0, 0)
        column_layout.setSpacing(8)
        column_layout.addWidget(side_scroll, 1)
        column_layout.addWidget(stop)
        self._map_expanded = False
        root.addWidget(self.side_column)
        self._sync_controls()

    def _toggle_map_expanded(self) -> None:
        self._map_expanded = not self._map_expanded
        stop = self.emergency_stop_button
        if self._map_expanded:
            self.side_column.layout().removeWidget(stop)
            self.rviz_view.toolbar.addWidget(stop)
            stop.setMinimumHeight(28)
            stop.setMaximumWidth(120)
        else:
            self.rviz_view.toolbar.removeWidget(stop)
            self.side_column.layout().addWidget(stop)
            stop.setMinimumHeight(48)
            stop.setMaximumWidth(16777215)
        self.side_column.setVisible(not self._map_expanded)
        self.rviz_view.set_expanded(self._map_expanded)

    def resizeEvent(self, event) -> None:
        if hasattr(self, 'side_column'):
            self.side_column.setFixedWidth(max(280, min(340, int(self.width() * .25))))
        super().resizeEvent(event)

    @staticmethod
    def _spin(low: float, high: float, value: float, suffix: str) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(2)
        spin.setSingleStep(0.1)
        spin.setValue(value)
        spin.setSuffix(suffix)
        return spin

    def _wire_ros(self) -> None:
        self.worker = RosNavigationWorker(self)
        self.worker.pose_updated.connect(self._on_pose)
        self.worker.status_updated.connect(self._on_navigation_status)
        self.worker.feedback_updated.connect(self._on_feedback)
        self.worker.goal_finished.connect(self._on_goal_finished)
        self.worker.map_saved.connect(self._on_map_saved)
        self.worker.log_message.connect(self._append_log)
        self.worker.terrain_publishers_updated.connect(self._ensure_terrain_display)
        self.worker.capabilities_updated.connect(self._on_capabilities)
        self.worker.control_finished.connect(self._on_control_finished)
        self.worker.start()

    def _ensure_terrain_display(self, publishers):
        if self._navigation_closed or self._control_pending == 'close' or self._shutting_down:
            return
        process = self._terrain_display_process
        if publishers > 1 and process.state() != QProcess.NotRunning:
            process.terminate()  # Another display publisher has taken over.
        elif publishers == 0 and process.state() == QProcess.NotRunning:
            script = PROJECT_ROOT / 'ros2_ws/install/go2_lio_navigation/lib/go2_lio_navigation/environment_terrain_display.py'
            process.start('/usr/bin/python3', [str(script), '--ros-args', '-r',
                                              '__node:=fishbone_ui_terrain_display'])
            self._append_log('补充启动历史地形显示节点（仅显示，不改变导航输入）')

    def _start_stack(self, _checked=False, automatic=False, background=False) -> None:
        if self._shutting_down:
            return
        if self._control_pending:
            return
        if not self.prepare_visual_bridge():
            return
        if self._navigation_state != 'fault' and (self._navigation_state == 'paused' or
                                                   self._capabilities.get('paused')):
            if not automatic:
                self._request_control('resume')
            return
        if self._navigation_state == 'fault':
            self._append_log('定位保护已触发，请先重启导航进程；恢复按钮不能解除定位故障。')
            return
        try:
            from go2_navigation.runtime_guard import navigation_network_error
            network_error = navigation_network_error()
        except ImportError as error:
            network_error = f'导航环境检查失败: {error}；请使用 scripts/run_ui.sh 启动'
        if network_error:
            self._autostart_timer.stop()
            self._autostart_attempted = True
            self._startup_failed(network_error)
            if not background:
                self.rviz_view._show_error(network_error)
            return
        self._autostart_timer.stop()
        self._autostart_attempted = True
        self._startup_error = ''
        self._stop_requested = False
        if not background:
            self.rviz_view.start()
        if self.mode_combo.currentIndex() == 1:
            self._ready_timer.stop()
            self._append_log("连接模式：不会自动启动导航；可手动关闭本机已有导航")
            self._set_status(self._navigation_state, self._last_navigation_message)
            return
        if self._navigation_state == 'ready':
            self._append_log('当前导航已就绪，已接入。')
            self._set_status('ready', self._last_navigation_message)
            return
        if self._launch_process.state() != QProcess.NotRunning:
            self._append_log("导航进程已在运行")
            self._ready_timer.start()
            self._set_status('waiting', '正在检查导航就绪状态 · ' + self._last_navigation_message)
            return

        try:
            from go2_navigation.runtime_guard import conflicting_navigation_processes
            existing = conflicting_navigation_processes(include_launch=True)
        except Exception as error:
            self._startup_failed(f'导航环境检查失败: {error}；请使用 scripts/run_ui.sh 启动')
            return
        self._ready_timer.start()
        if existing:
            self._append_log('检测到已有导航，已接入；不会重复启动。')
            self._set_status('waiting', '已接入已有导航，等待系统就绪…')
            return
        args = [str(Path(__file__).with_name('stack_launcher.py'))]
        self._launch_output_tail.clear()
        self._append_log('正在以 Go2 本体雷达启动 2.5D 导航…')
        self._set_status('waiting', '正在检查本体雷达连接与站姿…')
        self._launch_process.setProcessEnvironment(QProcessEnvironment.systemEnvironment())
        self._launch_process.start(sys.executable, args)
        self._sync_controls()

    def prepare_visual_bridge(self):
        """Connect display/TF only; do not start any navigation or base process."""
        if self._shutting_down or self._control_pending:
            return False
        if self._navigation_closed:
            # Start with fresh ROS observations, not the previous lifecycle/map cache.
            if not self.worker.wait(2000):
                self._append_log('显示连接仍在退出，请稍后重试启动导航。')
                return False
            self._navigation_closed = False
            self._stop_requested = False
        self._ensure_bridge()
        return True

    def _autostart(self) -> None:
        if self._active and not self._autostart_attempted and not self._shutting_down:
            self._start_stack(automatic=True)

    def _ensure_bridge(self):
        if getattr(self.worker, 'isRunning', lambda: True)():
            return
        self.worker.deleteLater()
        self._control_pending = ''
        self._saving_map = self._goal_submitting = self._goal_active = False
        self._capabilities = {}
        self._navigation_state = 'waiting'
        self._wire_ros()
        self._sync_controls()

    def _on_navigation_status(self, state: str, message: str) -> None:
        if self._navigation_closed or self._control_pending == 'close':
            return
        self._navigation_state, self._last_navigation_message = state, message
        if state in ('ready', 'paused', 'fault', 'preview'):
            if self._ready_timer.isActive():
                if state == 'ready':
                    self._append_log('2.5D 导航已就绪，可以选择目标点。')
            self._ready_timer.stop()
            self._startup_error = ''
        if self._startup_error:
            self._set_status('error', self._startup_error)
        elif self._ready_timer.isActive() and state == 'waiting':
            self._set_status('waiting', '导航初始化中 · ' + message)
        else:
            self._set_status(state, message)
        self._sync_controls()

    def _on_capabilities(self, capabilities):
        if self._navigation_closed or self._control_pending == 'close':
            return
        self._capabilities = capabilities
        if capabilities.get('online') is False:
            self._control_pending = ''
            self._saving_map = self._goal_submitting = False
            self._navigation_state = 'offline'
            self._set_status('offline', 'ROS 连接已退出，请重新打开导航界面')
        self._goal_active = bool(capabilities.get('goal_active'))
        if self._goal_active:
            self._goal_submitting = False
        self._sync_controls()

    def _sync_controls(self):
        ready = self._navigation_state == 'ready'
        paused = self._navigation_state == 'paused'
        preview = self._navigation_state == 'preview'
        pending = bool(self._control_pending)
        warning = self._capabilities.get('motion_safety_warning', '等待实时安全检测数据')
        lidar = {'unitree': '本体雷达', 'hap': 'HAP 外接雷达'}.get(self._capabilities.get('lidar_source'))
        safety_text = '导航已关闭' if self._navigation_closed else warning or '保护区净空'
        self.safety_label.setText(f'{lidar} · {safety_text}' if lidar else safety_text)
        self.start_button.setEnabled(not pending and not ready and not preview and self._navigation_state != 'fault'
                                    and (not self._navigation_closed or self._launch_process.state() == QProcess.NotRunning))
        self.start_button.setText('切换中…' if pending else '恢复导航' if paused else
                                  '已启用' if ready else '地图预览' if preview else '启动 / 重试')
        self.start_button.setToolTip('定位故障需重启导航进程' if self._navigation_state == 'fault' else
                                    '启动本机导航或恢复已停止的 Nav2；不会重复创建进程')
        self.stop_button.setEnabled(not pending and not self._navigation_closed and
            (ready or self._capabilities.get('can_control', False) or
             self._stop_requested or self._navigation_state == 'fault' or
             self._launch_process.state() != QProcess.NotRunning))
        goal_active = self._goal_active or self._goal_submitting
        self.send_goal_button.setEnabled(ready and not warning and not pending and not goal_active)
        self.send_goal_button.setToolTip('请先启用导航并等待地图、有效位姿就绪' if not ready else
                                        '当前目标执行中，请先取消或等待完成' if goal_active else
                                        warning or '发送目标坐标')
        self.cancel_goal_button.setEnabled(goal_active and not pending and not self._navigation_closed)
        self.cancel_goal_button.setToolTip('取消本页或 RViz 中的目标' if goal_active else '当前没有执行中的目标')
        self.save_map_button.setEnabled(bool(self._capabilities.get('can_save')) and not self._saving_map
                                       and self._control_pending != 'close')
        self.save_map_button.setText('正在保存…' if self._saving_map else '保存当前环境地图')
        self.save_map_button.setToolTip('正在等待保存结果' if self._saving_map else
            '保存当前地图' if self._capabilities.get('can_save') else '等待地图与 /mapping/save 保存服务就绪')

    def _close_navigation(self):
        if self._control_pending:
            return
        self._autostart_timer.stop()
        self._autostart_attempted = True
        self._ready_timer.stop()
        self._stop_requested = True
        self._startup_error = ''
        self._control_pending = 'close'
        self.worker.enqueue('emergency_stop')
        self._set_status('waiting', '正在关闭导航和底盘控制，请等待进程退出…')
        self._sync_controls()
        self._close_process.setProcessEnvironment(QProcessEnvironment.systemEnvironment())
        self._close_process.start(sys.executable, [str(Path(__file__).with_name('stack_stopper.py'))])

    def _on_close_error(self, error):
        if error == QProcess.FailedToStart:
            self._finish_close(False, '关闭程序启动失败：' + self._close_process.errorString())

    def _on_close_finished(self, code, status):
        message = bytes(self._close_process.readAllStandardOutput()).decode('utf-8', errors='replace').strip()
        self._finish_close(code == 0 and status == QProcess.NormalExit,
                           message or '导航关闭失败，请重试关闭')

    def _finish_close(self, success, message):
        if self._control_pending != 'close':
            return
        self._control_pending = ''
        self._navigation_closed = success
        self._close_failed = not success
        self._navigation_state = 'closed' if success else 'error'
        self._last_navigation_message = message
        self._startup_error = '' if success else message
        if success:
            self._capabilities = {}
            self._goal_active = self._goal_submitting = self._saving_map = False
            self.worker.stop()
            self.rviz_view.shutdown()
            self._terrain_display_process.terminate()
        self._set_status(self._navigation_state, message)
        self._append_log(message)
        self._sync_controls()

    def _request_control(self, operation):
        self._control_pending = operation
        self._startup_error = ''
        self._ready_timer.stop()
        self.worker.enqueue(operation)
        self._set_status('waiting', '正在停止导航…' if operation == 'pause' else '正在恢复导航…')
        self._sync_controls()

    def _on_control_finished(self, operation, success, message):
        if self._control_pending != operation:
            return
        self._control_pending = ''
        if success:
            self._navigation_state = 'paused' if operation == 'pause' else 'ready'
            self._last_navigation_message = message
        self._set_status(self._navigation_state if success else 'error', message)
        self._append_log(('完成：' if success else '失败：') + message)
        self._sync_controls()

    def _cancel_goal(self):
        self.worker.enqueue('cancel')
        self.feedback_label.setText('正在取消目标…')

    def _startup_failed(self, message: str) -> None:
        self._ready_timer.stop()
        self._navigation_state = 'error'
        self._startup_error = message
        self._set_status('error', message)
        self._append_log(message)
        self._sync_controls()

    def _on_startup_timeout(self) -> None:
        self._startup_failed('导航尚未就绪：' + self._last_navigation_message + '。请检查连接与下方日志，再点击启动 / 重试。')

    def _on_launch_error(self, error) -> None:
        if not self._stop_requested and not self._shutting_down:
            self._startup_failed('导航启动进程异常：' + self._launch_process.errorString())

    def _stop_stack(self, quiet: bool = False, wait: bool = False) -> None:
        self._autostart_timer.stop()
        self._autostart_attempted = True
        self._ready_timer.stop()
        self._stop_requested = True
        if self._launch_process.state() == QProcess.NotRunning:
            if not quiet:
                self._append_log("没有由本页面启动的 ROS 2 进程")
            return
        self._launch_process.terminate()
        if wait:
            if not self._launch_process.waitForFinished(18000):
                self._launch_process.kill()
                self._launch_process.waitForFinished(1000)
        else:
            QTimer.singleShot(18000, self._kill_process_if_needed)

    def _kill_process_if_needed(self) -> None:
        if self._launch_process.state() != QProcess.NotRunning:
            self._launch_process.kill()

    def _read_launch_output(self) -> None:
        raw = bytes(self._launch_process.readAllStandardOutput()).decode("utf-8", errors="replace")
        for line in raw.splitlines():
            if line.strip():
                self._append_log("[ROS] " + line.strip())
                self._launch_output_tail.append(line.strip())
                self._launch_output_tail = self._launch_output_tail[-8:]

    def _on_process_finished(self, code: int, _status) -> None:
        self._read_launch_output()
        self._append_log(f"ROS 2 导航进程已退出，代码 {code}")
        if self._shutting_down:
            return
        if self._control_pending == 'close' or self._navigation_closed:
            self._sync_controls()
            return
        if self._stop_requested:
            self._navigation_state = 'waiting'
            self._startup_error = '本页导航已停止；点击启动 / 重试可重新启动'
            self._set_status('waiting', self._startup_error)
            self._sync_controls()
        else:
            reason = self._launch_output_tail[-1] if self._launch_output_tail else f'退出代码 {code}'
            self._startup_failed('导航启动失败 / 已退出：' + reason)

    def _send_goal(self) -> None:
        if not self.send_goal_button.isEnabled():
            self._append_log('导航未就绪或已有目标，未重复发送。')
            return
        self._goal_submitting = True
        self._sync_controls()
        x, y = self.goal_x.value(), self.goal_y.value()
        # Nav2 still requires a valid quaternion in the action message. Send the
        # robot's current yaw, while SimpleGoalChecker accepts any final heading.
        yaw = self._latest_pose_yaw
        # Current LIO navigation may plan beyond the rolling map's bounds.
        self.worker.enqueue("goal", x, y, yaw, False)
        self.feedback_label.setText("距离目标: 计算中…")

    def _save_map(self) -> None:
        if self._saving_map or not self._capabilities.get('can_save'):
            return
        self._saving_map = True
        self._sync_controls()
        self.worker.enqueue("save_environment")

    def _on_pose(self, x: float, y: float, yaw: float) -> None:
        self._latest_pose_yaw = yaw
        self.pose_label.setText(f"位姿: x={x:.2f}  y={y:.2f}  yaw={math.degrees(yaw):.1f}°")

    def _on_feedback(self, distance: float, eta: str) -> None:
        self.feedback_label.setText(f"距离目标: {distance:.2f} m · {eta}")

    def _on_goal_finished(self, success: bool, message: str) -> None:
        self._goal_active = self._goal_submitting = False
        self._sync_controls()
        self.feedback_label.setText(message)
        self._append_log(("完成: " if success else "提示: ") + message)

    def _on_map_saved(self, success: bool, message: str) -> None:
        self._saving_map = False
        self._sync_controls()
        self._append_log(("完成: " if success else "失败: ") + message)

    def _set_status(self, state: str, message: str) -> None:
        colors = {
            "ready": ("#00FF88", "rgba(0, 255, 136, 25)"),
            "waiting": ("#FFB020", "rgba(255, 176, 32, 22)"),
            "offline": ("#7AA8C8", "rgba(122, 168, 200, 20)"),
            "error": ("#FF4466", "rgba(255, 68, 102, 25)"),
            "fault": ("#FF4466", "rgba(255, 68, 102, 25)"),
            "paused": ("#FFB020", "rgba(255, 176, 32, 22)"),
            "closed": ("#7AA8C8", "rgba(122, 168, 200, 20)"),
        }
        fg, bg = colors.get(state, colors["waiting"])
        self.status_label.setToolTip(message)
        self.status_label.setText(f"● {message}")
        self.status_label.setStyleSheet(
            f"QLabel {{color:{fg}; background:{bg}; border:1px solid {fg}; border-radius:6px;"
            "padding:8px; font-size:14px; font-weight:bold;}"
        )

    def _append_log(self, message: str) -> None:
        self.log_box.appendPlainText(message)

    def shutdown(self) -> None:
        if self._shutting_down:
            return
        self._shutting_down = True
        if self._close_process.state() != QProcess.NotRunning:
            # Let the already requested shutdown finish even when the window closes.
            self._close_process.waitForFinished(22000)
        self._stop_stack(quiet=True, wait=True)
        self.rviz_view.shutdown()
        if self._terrain_display_process.state() != QProcess.NotRunning:
            self._terrain_display_process.terminate()
            if not self._terrain_display_process.waitForFinished(2000):
                self._terrain_display_process.kill()
                self._terrain_display_process.waitForFinished(1000)
        self.worker.stop()
        if not self.worker.wait(5000):
            self._append_log('等待 ROS 显示桥接器退出…')
            self.worker.wait()

    def activate(self) -> bool:
        self._active = True
        if self._navigation_closed or self._control_pending == 'close':
            return True
        self._ensure_bridge()
        self.rviz_view.start()
        if not self._autostart_attempted:
            self._autostart_timer.start()
        return True

    def deactivate(self) -> None:
        """Hide the display without cancelling or stopping external navigation."""
        self._active = False
        self._autostart_timer.stop()

    def cancel_navigation(self) -> None:
        """Backward-compatible alias for the exclusive page lifecycle."""
        self.deactivate()
