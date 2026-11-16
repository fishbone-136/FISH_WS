import os

from PyQt5.QtWidgets import QWidget, QLabel, QPushButton, QHBoxLayout, QGraphicsDropShadowEffect, QSizePolicy, QStyle
from PyQt5.QtCore import QTimer, QDateTime, Qt
from PyQt5.QtGui import QPainter, QLinearGradient, QColor, QIcon
from PyQt5 import QtCore
from PyQt5.QtSvg import QSvgRenderer

_ICON_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "icons")


class TopBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("TopBar")
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self.setFixedHeight(56)
        self.init_ui()
        self._pulse_phase = 0
        self._drag_offset = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_time)
        self.timer.start(1000)
        self.update_time()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # ---- 主渐变 ----
        gradient = QLinearGradient(0, 0, w, 0)
        gradient.setColorAt(0.0, QColor("#0C1424"))
        gradient.setColorAt(0.25, QColor("#121E36"))
        gradient.setColorAt(0.5, QColor("#182A48"))
        gradient.setColorAt(0.75, QColor("#121E36"))
        gradient.setColorAt(1.0, QColor("#0C1424"))
        painter.fillRect(self.rect(), gradient)

        # ---- 底边辉光线 ----
        glow = QLinearGradient(0, h - 2, w, h - 2)
        glow.setColorAt(0.0, QColor(0, 240, 255, 0))
        glow.setColorAt(0.2, QColor(0, 240, 255, 25))
        glow.setColorAt(0.5, QColor(0, 240, 255, 160))
        glow.setColorAt(0.8, QColor(0, 240, 255, 25))
        glow.setColorAt(1.0, QColor(0, 240, 255, 0))
        painter.setPen(Qt.NoPen)
        painter.setBrush(glow)
        painter.drawRect(0, h - 2, w, 2)

    def init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setSpacing(0)

        left_group = QHBoxLayout()
        left_group.setSpacing(5)

        self.logo_label = QLabel("\U0001F415")
        self.logo_label.setStyleSheet(
            "QLabel { color: #00F0FF; font-size: 26px; background: transparent; border: none; padding: 0px; }"
        )
        left_group.addWidget(self.logo_label)

        self.sys_name = QLabel("FISHBONE")
        self.sys_name.setStyleSheet(
            "QLabel { color: #00F0FF; font-size: 18px; font-weight: bold; background: transparent; border: none; }"
        )
        left_group.addWidget(self.sys_name)

        sep1 = QLabel()
        self._brand_separator = sep1
        sep1.setFixedSize(3, 24)
        sep1.setStyleSheet("QLabel { background-color: rgba(0, 240, 255, 80); border: none; }")
        left_group.addWidget(sep1)

        self.status_dot = QLabel("  ● ")
        self.status_dot.setStyleSheet(
            "QLabel { color: #00FF88; font-size: 18px; background: transparent; border: none; }"
        )
        left_group.addWidget(self.status_dot)

        self.status_label = QLabel("系统运行中")
        self.status_label.setStyleSheet(
            "QLabel { color: #00FF88; font-size: 14px; background: transparent; border: none; }"
        )
        left_group.addWidget(self.status_label)

        layout.addLayout(left_group)
        layout.addSpacing(10)

        center_group = QHBoxLayout()
        center_group.setSpacing(0)

        self.title = QLabel("邦邦机器狗智能交互控制平台")
        self.title.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.title.setMinimumWidth(0)
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet(
            "QLabel { color: #C8E8FF; font-size: 18px; font-weight: bold; background: transparent; border: none; }"
        )
        center_group.addWidget(self.title)

        sep_title = QLabel()
        self._school_separator = sep_title
        sep_title.setFixedSize(3, 30)
        sep_title.setStyleSheet("QLabel { background-color: rgba(0, 240, 255, 100); border: none; margin: 0 14px; }")
        center_group.addWidget(sep_title)

        self.school_label = QLabel("  西安电子科技大学")
        self.school_label.setAlignment(Qt.AlignCenter)
        self.school_label.setStyleSheet(
            "QLabel { color: #C8E8FF; font-size: 20px; font-weight: bold; background: transparent; border: none; }"
        )
        center_group.addWidget(self.school_label)

        layout.addLayout(center_group, 1)
        layout.addSpacing(10)

        right_group = QHBoxLayout()
        right_group.setSpacing(8)

        self.time_label = QLabel()
        self.time_label.setStyleSheet(
            "QLabel { color: #9CBBD1; font-size: 12px; background: transparent; border: none; padding: 0 6px; }"
        )
        time_glow = QGraphicsDropShadowEffect(self)
        time_glow.setBlurRadius(12)
        time_glow.setColor(QColor(0, 240, 255, 50))
        time_glow.setOffset(0, 0)
        self.time_label.setGraphicsEffect(time_glow)
        right_group.addWidget(self.time_label)

        icons = [
            (os.path.join(_ICON_DIR, "wifi.svg"), "WiFi", "#00F0FF"),
            (os.path.join(_ICON_DIR, "bt.svg"), "蓝牙", "#00D4AA"),
            (os.path.join(_ICON_DIR, "power.svg"), "电池", "#A855F7"),
            (os.path.join(_ICON_DIR, "set.svg"), "设置", "#C8E8FF"),
        ]

        self._status_icons = []
        for icon_path, tooltip, color in icons:
            btn = QPushButton()
            btn.setToolTip(tooltip)
            btn.setFixedSize(32, 32)
            icon = QIcon(icon_path)
            btn.setIcon(icon)
            btn.setStyleSheet(
                f"QPushButton {{ color: {color}; font-size: 30px; font-weight: normal;"
                f"background: transparent; border: 1px solid transparent; border-radius: 6px;"
                f"padding: 0px; min-width: 0px; min-height: 0px; }}"
                f"QPushButton:hover {{ background: rgba(0, 240, 255, 12); border: 1px solid rgba(0, 240, 255, 50); }}"
            )
            right_group.addWidget(btn)
            if tooltip == "设置":
                self.settings_btn = btn
            else:
                self._status_icons.append(btn)

        sep3 = QLabel()
        sep3.setFixedSize(3, 30)
        sep3.setStyleSheet("QLabel { background-color: rgba(0, 240, 255, 100); border: none; }")
        right_group.addWidget(sep3)

        self.minimize_btn = self._window_button(QStyle.SP_TitleBarMinButton, "最小化")
        self.minimize_btn.clicked.connect(lambda: self.window().showMinimized())
        right_group.addWidget(self.minimize_btn)
        self.maximize_btn = self._window_button(QStyle.SP_TitleBarMaxButton, "最大化")
        self.maximize_btn.clicked.connect(self.toggle_maximized)
        right_group.addWidget(self.maximize_btn)

        self.close_btn = QPushButton()
        self.close_btn.setToolTip("关闭")
        self.close_btn.setFixedSize(40, 40)
        self.close_btn.setText("×")
        self.close_btn.setIconSize(QtCore.QSize(24, 24))
        self.close_btn.setStyleSheet(
            "QPushButton { background: transparent; border: 1px solid transparent;"
            "border-radius: 6px; padding: 0px; min-width: 0px; min-height: 0px;"
            "color: #C8E8FF; font-size: 24px; }"
            "QPushButton:hover { background: rgba(255, 68, 102, 40); border: 1px solid #FF4466; }"
            "QPushButton:pressed { background: rgba(255, 68, 102, 80); }"
        )
        self.close_btn.clicked.connect(self.window_close)
        right_group.addWidget(self.close_btn)

        layout.addLayout(right_group)

    def _window_button(self, icon, tooltip):
        button = QPushButton(self)
        button.setToolTip(tooltip)
        button.setFixedSize(36, 36)
        button.setText("—" if icon == QStyle.SP_TitleBarMinButton else "□")
        button.setStyleSheet(
            "QPushButton {background: transparent; border: none; padding: 0; min-height: 0;"
            "color: #C8E8FF; font-size: 20px;}"
            "QPushButton:hover {background: #163358;}"
        )
        return button

    def toggle_maximized(self):
        window = self.window()
        if window.isMaximized():
            window.showNormal()
        else:
            window.showMaximized()
        self.update_window_buttons()

    def update_window_buttons(self):
        maximized = self.window().isMaximized()
        self.maximize_btn.setToolTip("还原" if maximized else "最大化")
        self.maximize_btn.setText("❐" if maximized else "□")

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.toggle_maximized()
            event.accept()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and not self.window().isMaximized():
            self._drag_offset = event.globalPos() - self.window().frameGeometry().topLeft()
            event.accept()

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton and self._drag_offset is not None:
            self.window().move(event.globalPos() - self._drag_offset)
            event.accept()

    def mouseReleaseEvent(self, event):
        self._drag_offset = None

    def resizeEvent(self, event):
        width = event.size().width()
        for widget in (self.school_label, self._school_separator):
            widget.setVisible(width >= 1600)
        for widget in (self.sys_name, self._brand_separator):
            widget.setVisible(width >= 1050)
        for widget in (self.status_dot, self.status_label):
            widget.setVisible(width >= 1420)
        for widget in self._status_icons:
            widget.setVisible(width >= 1200)
        self.update_time()
        super().resizeEvent(event)

    def update_time(self):
        current = QDateTime.currentDateTime()
        self.time_label.setText(current.toString("yyyy-MM-dd hh:mm:ss" if self.width() >= 1200 else "hh:mm:ss"))
        self.time_label.setToolTip(current.toString("yyyy-MM-dd hh:mm:ss"))

        # 状态灯脉冲
        self._pulse_phase = (self._pulse_phase + 1) % 60
        if self._pulse_phase < 30:
            a = 180 + int(self._pulse_phase * 75 / 30)
        else:
            a = 255 - int((self._pulse_phase - 30) * 75 / 30)
        self.status_dot.setStyleSheet(
            f"QLabel {{ color: rgba(0, 255, 136, {a}); font-size: 18px; background: transparent; border: none; }}"
        )

    def window_close(self):
        self.window().close()
