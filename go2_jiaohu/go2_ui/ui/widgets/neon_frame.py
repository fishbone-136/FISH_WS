from PyQt5.QtWidgets import QFrame, QLabel, QVBoxLayout, QSizePolicy
from pathlib import Path
from PyQt5.QtCore import Qt, QDir
from PyQt5.QtGui import QPainter, QLinearGradient, QColor

QDir.addSearchPath('uiicons', str(Path(__file__).resolve().parents[2] / 'resources/icons'))


class NeonFrame(QFrame):
    def __init__(self, title="", parent=None):
        super().__init__(parent)
        self.setObjectName("NeonFrame")
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(12, 10, 12, 12)
        self.layout.setSpacing(8)

        if title:
            self.title_label = QLabel(title)
            self.title_label.setObjectName("PanelTitle")
            self.title_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            self.title_label.setMinimumHeight(28)
            self.title_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
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
