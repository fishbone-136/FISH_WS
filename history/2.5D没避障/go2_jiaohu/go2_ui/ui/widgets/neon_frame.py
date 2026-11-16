from PyQt5.QtWidgets import QFrame, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QLinearGradient, QColor


class NeonFrame(QFrame):
    def __init__(self, title="", parent=None):
        super().__init__(parent)
        self.setObjectName("NeonFrame")
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(10, 5, 10, 12)
        self.layout.setSpacing(5)

        if title:
            self.title_label = QLabel(title)
            self.title_label.setObjectName("PanelTitle")
            self.title_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.layout.addWidget(self.title_label)

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # 底部渐变辉光线 (中间亮 -> 两端暗)
        bot_glow = QLinearGradient(0, 0, w, 0)
        bot_glow.setColorAt(0.0, QColor(0, 240, 255, 0))
        bot_glow.setColorAt(0.2, QColor(0, 240, 255, 15))
        bot_glow.setColorAt(0.5, QColor(0, 240, 255, 100))
        bot_glow.setColorAt(0.8, QColor(0, 240, 255, 15))
        bot_glow.setColorAt(1.0, QColor(0, 240, 255, 0))
        painter.setPen(Qt.NoPen)
        painter.setBrush(bot_glow)
        painter.drawRect(int(w * 0.1), h - 2, int(w * 0.8), 2)

        painter.end()
