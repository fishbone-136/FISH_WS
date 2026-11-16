"""Current LIO navigation and its existing RViz profile in the dashboard."""

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
    QTabWidget,
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
        self._launch_process.started.connect(lambda: self._append_log("ROS 2 导航进程已启动"))
        self._launch_process.finished.connect(self._on_process_finished)
        self._launch_process.errorOccurred.connect(
            lambda error: self._append_log(f"启动进程错误: {error}")
        )
        self._latest_pose_yaw = 0.0
        self._terrain_display_process = QProcess(self)
        self._terrain_display_process.setProcessChannelMode(QProcess.MergedChannels)
        self._terrain_display_process.readyReadStandardOutput.connect(
            lambda: self._terrain_display_process.readAllStandardOutput())
        self._build()
        self._wire_ros()
        self.map_canvas.set_active(False)

    def _build(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(15)

        map_frame = NeonFrame("当前 LIO 导航 · RViz 实时显示")
        map_frame.title_label.setAlignment(Qt.AlignCenter)
        map_frame.title_label.setStyleSheet(
            "background: transparent; border: none; font-size: 22px;"
        )
        self.map_canvas = MapCanvas()
        self.rviz_view = RvizView()
        self.rviz_view.log_message.connect(self._append_log)
        self.view_tabs = QTabWidget()
        self.view_tabs.addTab(self.rviz_view, 'RViz 完整视图')
        self.view_tabs.addTab(self.map_canvas, '二维选点 · odom_lio')
        map_frame.layout.addWidget(self.view_tabs)
        root.addWidget(map_frame, 7)

        side = QWidget()
        side.setStyleSheet("background: transparent;")
        side.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.setSpacing(6)

        status_frame = NeonFrame("导航状态")
        self.status_label = QLabel("● 等待 ROS 2")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setMinimumHeight(42)
        self._set_status("waiting", "等待 ROS 2")
        self.pose_label = QLabel("位姿: x=--  y=--  yaw=--")
        self.pose_label.setAlignment(Qt.AlignCenter)
        self.feedback_label = QLabel("距离目标: --")
        self.feedback_label.setAlignment(Qt.AlignCenter)
        status_frame.layout.addWidget(self.status_label)
        status_frame.layout.addWidget(self.pose_label)
        status_frame.layout.addWidget(self.feedback_label)
        side_layout.addWidget(status_frame)

        launch_frame = NeonFrame("运行模式")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "接入已运行的当前导航（默认）",
            "启动当前 LIO 导航",
        ])
        launch_buttons = QHBoxLayout()
        start_btn = QPushButton("连接 / 启动")
        start_btn.clicked.connect(self._start_stack)
        stop_btn = QPushButton("停止本页启动的导航")
        stop_btn.clicked.connect(self._stop_stack)
        launch_buttons.addWidget(start_btn)
        launch_buttons.addWidget(stop_btn)
        launch_frame.layout.addWidget(self.mode_combo)
        help_text = QLabel('使用现有 LIO 导航及 RViz 配置。外部导航不会被重复启动或随页面关闭。')
        help_text.setWordWrap(True)
        launch_frame.layout.addWidget(help_text)
        launch_frame.layout.addLayout(launch_buttons)
        side_layout.addWidget(launch_frame)

        pose_frame = NeonFrame("目标点 · odom_lio（忽略终点朝向）")
        grid = QGridLayout()
        self.goal_x = self._spin(-1000.0, 1000.0, 0.0, " m")
        self.goal_y = self._spin(-1000.0, 1000.0, 0.0, " m")
        for row, (label, widget) in enumerate((
            ("X", self.goal_x), ("Y", self.goal_y)
        )):
            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(widget, row, 1)
        pose_frame.layout.addLayout(grid)
        action_row = QHBoxLayout()
        send = QPushButton("开始导航")
        send.clicked.connect(self._send_goal)
        cancel = QPushButton("取消目标")
        cancel.clicked.connect(lambda: self.worker.enqueue("cancel"))
        action_row.addWidget(send)
        action_row.addWidget(cancel)
        pose_frame.layout.addLayout(action_row)
        aux_row = QHBoxLayout()
        hint = QLabel('可直接使用 RViz 的 Nav2 Goal；数值坐标和二维选点使用 odom_lio。')
        hint.setWordWrap(True)
        aux_row.addWidget(hint)
        pose_frame.layout.addLayout(aux_row)
        side_layout.addWidget(pose_frame)

        save_frame = NeonFrame("建图保存")
        save_btn = QPushButton("保存当前环境地图")
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
        side_layout.addWidget(stop)

        log_frame = NeonFrame("运行日志")
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.document().setMaximumBlockCount(300)
        log_frame.layout.addWidget(self.log_box)
        side_layout.addWidget(log_frame, 1)

        side_scroll = QScrollArea()
        side_scroll.setWidgetResizable(True)
        side_scroll.setFrameShape(QScrollArea.NoFrame)
        side_scroll.setMinimumWidth(330)
        side_scroll.setWidget(side)
        root.addWidget(side_scroll, 3)
        self.map_canvas.goal_selected.connect(self._on_map_goal_selected)

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
        self.worker.map_updated.connect(self.map_canvas.set_map)
        self.worker.pose_updated.connect(self._on_pose)
        self.worker.path_updated.connect(self.map_canvas.set_path)
        self.worker.status_updated.connect(self._set_status)
        self.worker.feedback_updated.connect(self._on_feedback)
        self.worker.goal_finished.connect(self._on_goal_finished)
        self.worker.map_saved.connect(self._on_map_saved)
        self.worker.log_message.connect(self._append_log)
        self.worker.terrain_publishers_updated.connect(self._ensure_terrain_display)
        self.worker.start()

    def _ensure_terrain_display(self, publishers):
        process = self._terrain_display_process
        if publishers > 1 and process.state() != QProcess.NotRunning:
            process.terminate()  # Another display publisher has taken over.
        elif publishers == 0 and process.state() == QProcess.NotRunning:
            script = PROJECT_ROOT / 'ros2_ws/install/go2_lio_navigation/lib/go2_lio_navigation/environment_terrain_display.py'
            process.start('/usr/bin/python3', [str(script), '--ros-args', '-r',
                                              '__node:=fishbone_ui_terrain_display'])
            self._append_log('补充启动历史地形显示节点（仅显示，不改变导航输入）')

    def _start_stack(self) -> None:
        mode = self.mode_combo.currentIndex()
        self.rviz_view.start()
        if mode == 0:
            self._append_log("连接模式：不会启动或停止外部 ROS 2 节点")
            return
        if self._launch_process.state() != QProcess.NotRunning:
            self._append_log("导航进程已在运行")
            return

        from go2_navigation.runtime_guard import conflicting_navigation_processes
        if conflicting_navigation_processes():
            self._append_log('检测到已有导航，已接入；不会重复启动。')
            return
        args = [str(Path(__file__).with_name('stack_launcher.py'))]
        self._append_log('使用 scripts/run_lio_navigation.sh 启动当前导航；参数保持现有设置。')
        self._launch_process.setProcessEnvironment(QProcessEnvironment.systemEnvironment())
        self._launch_process.start(sys.executable, args)

    def _stop_stack(self, quiet: bool = False, wait: bool = False) -> None:
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

    def _on_process_finished(self, code: int, _status) -> None:
        self._append_log(f"ROS 2 导航进程已退出，代码 {code}")

    def _send_goal(self) -> None:
        x, y = self.goal_x.value(), self.goal_y.value()
        # Nav2 still requires a valid quaternion in the action message. Send the
        # robot's current yaw, while SimpleGoalChecker accepts any final heading.
        yaw = self._latest_pose_yaw
        self.map_canvas.set_goal(x, y, None)
        # Current LIO navigation may plan beyond the rolling map's bounds.
        self.worker.enqueue("goal", x, y, yaw, False)
        self.feedback_label.setText("距离目标: 计算中…")

    def _save_map(self) -> None:
        self.worker.enqueue("save_environment")

    def _on_map_goal_selected(self, x: float, y: float) -> None:
        self.goal_x.setValue(x)
        self.goal_y.setValue(y)
        self.map_canvas.set_goal(x, y, None)
        self._append_log(
            f"已从地图选择目标: x={x:.2f}, y={y:.2f}；目标朝向不作限制，点击“开始导航”发送"
        )

    def _on_pose(self, x: float, y: float, yaw: float) -> None:
        self._latest_pose_yaw = yaw
        self.pose_label.setText(f"位姿: x={x:.2f}  y={y:.2f}  yaw={math.degrees(yaw):.1f}°")
        self.map_canvas.set_robot_pose(x, y, yaw)

    def _on_feedback(self, distance: float, eta: str) -> None:
        self.feedback_label.setText(f"距离目标: {distance:.2f} m · {eta}")

    def _on_goal_finished(self, success: bool, message: str) -> None:
        self.feedback_label.setText(message)
        self._append_log(("完成: " if success else "提示: ") + message)

    def _on_map_saved(self, success: bool, message: str) -> None:
        self._append_log(("完成: " if success else "失败: ") + message)

    def _set_status(self, state: str, message: str) -> None:
        colors = {
            "ready": ("#00FF88", "rgba(0, 255, 136, 25)"),
            "waiting": ("#FFB020", "rgba(255, 176, 32, 22)"),
            "offline": ("#7AA8C8", "rgba(122, 168, 200, 20)"),
            "error": ("#FF4466", "rgba(255, 68, 102, 25)"),
        }
        fg, bg = colors.get(state, colors["waiting"])
        self.status_label.setText(f"● {message}")
        self.status_label.setStyleSheet(
            f"color:{fg}; background:{bg}; border:1px solid {fg}; border-radius:6px;"
            "font-size:15px; font-weight:bold;"
        )

    def _append_log(self, message: str) -> None:
        self.log_box.appendPlainText(message)

    def shutdown(self) -> None:
        self.worker.terrain_publishers_updated.disconnect(self._ensure_terrain_display)
        self._stop_stack(quiet=True, wait=True)
        self.rviz_view.shutdown()
        if self._terrain_display_process.state() != QProcess.NotRunning:
            self._terrain_display_process.terminate()
            if not self._terrain_display_process.waitForFinished(2000):
                self._terrain_display_process.kill()
                self._terrain_display_process.waitForFinished(1000)
        self.map_canvas.set_active(False)
        self.worker.stop()
        if not self.worker.wait(5000):
            self._append_log('等待 ROS 显示桥接器退出…')
            self.worker.wait()

    def activate(self) -> bool:
        self.map_canvas.set_active(True)
        self.rviz_view.start()
        return True

    def deactivate(self) -> None:
        """Hide the display without cancelling or stopping external navigation."""
        self.map_canvas.set_active(False)

    def cancel_navigation(self) -> None:
        """Backward-compatible alias for the exclusive page lifecycle."""
        self.deactivate()
