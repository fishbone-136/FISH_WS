"""
机器狗部署版启动器 —— Unitree Go2 实机运行入口
================================================
在机器狗端运行完整 PyQt5 UI 交互面板。
部署入口显式启用 ROS 2 实机控制器，不会静默降级为离线模拟。

运行方式:
    python main.py

离线测试 (明确指定):
    python offline_test/main.py
"""
import os
import sys

# 部署入口显式启用实机控制器，避免再用“是否安装 SDK”猜测运行模式。
os.environ["GO2_DEPLOY_MODE"] = "1"

# 将项目根目录加入 sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from go2_ui.runtime_logging import configure_logging
configure_logging()

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QPixmap, QPainter, QColor, QFont, QPen, QLinearGradient
from PyQt5.QtCore import QTimer, Qt, QPointF

from go2_ui.ui.QStackedWidget import Dashboard
from go2_ui.ui.splash_screen import SplashScreen


def create_splash_pixmap():
    w, h = 640, 420
    pixmap = QPixmap(w, h)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.Antialiasing)
    painter.setRenderHint(QPainter.TextAntialiasing)

    #  背景
    bg = QLinearGradient(0, 0, 0, h)
    bg.setColorAt(0.0, QColor('#080C16'))
    bg.setColorAt(0.5, QColor('#060A12'))
    bg.setColorAt(1.0, QColor('#04080C'))
    painter.fillRect(pixmap.rect(), bg)

    #  暗点网格
    painter.setPen(Qt.NoPen)
    for gx in range(0, w + 30, 30):
        for gy in range(0, h + 30, 30):
            painter.setBrush(QColor(0, 220, 255, 8))
            painter.drawEllipse(QPointF(gx + 2, gy + 2), 0.8, 0.8)

    #  四角 HUD 角标
    gap, arm = 18, 40
    corners = [
        [(gap, gap, gap, gap + arm), (gap, gap, gap + arm, gap)],
        [(w - gap, gap, w - gap, gap + arm), (w - gap, gap, w - gap - arm, gap)],
        [(gap, h - gap, gap, h - gap - arm), (gap, h - gap, gap + arm, h - gap)],
        [(w - gap, h - gap, w - gap, h - gap - arm), (w - gap, h - gap, w - gap - arm, h - gap)],
    ]
    for corner in corners:
        for x1, y1, x2, y2 in corner:
            painter.setPen(QPen(QColor(0, 240, 255, 45), 2.5, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(x1, y1, x2, y2)
            painter.setPen(QPen(QColor(0, 240, 255, 150), 1.0, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(x1, y1, x2, y2)

    #  顶部装饰线
    top_y = 80
    top_glow = QLinearGradient(0, top_y, w, top_y)
    top_glow.setColorAt(0.0, QColor(0, 240, 255, 0))
    top_glow.setColorAt(0.3, QColor(0, 240, 255, 20))
    top_glow.setColorAt(0.5, QColor(0, 240, 255, 80))
    top_glow.setColorAt(0.7, QColor(0, 240, 255, 20))
    top_glow.setColorAt(1.0, QColor(0, 240, 255, 0))
    painter.setPen(Qt.NoPen)
    painter.setBrush(top_glow)
    painter.drawRect(int(w * 0.1), top_y, int(w * 0.8), 2)

    #  标题
    font_title = QFont("Microsoft YaHei", 34, QFont.Bold)
    painter.setFont(font_title)
    painter.setPen(QColor("#00F0FF"))
    painter.drawText(pixmap.rect().adjusted(0, 20, 0, 0),
                     Qt.AlignHCenter | Qt.AlignVCenter, "邦邦智能交互控制平台")

    #  副标题
    font_sub = QFont("Microsoft YaHei", 13)
    painter.setFont(font_sub)
    painter.setPen(QColor("#4A8098"))
    painter.drawText(pixmap.rect().adjusted(0, 0, 0, -50),
                     Qt.AlignHCenter | Qt.AlignBottom, "邦邦机器狗智能交互系统")

    #  底部加载指示点
    dot_y = h - 40
    dot_count = 5
    dot_spacing = 18
    total_w = (dot_count - 1) * dot_spacing
    start_x = (w - total_w) / 2
    for i in range(dot_count):
        dx = start_x + i * dot_spacing
        alpha = 40 + i * 25
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 240, 255, alpha))
        painter.drawEllipse(QPointF(dx, dot_y), 3.5, 3.5)

    #  底部辉光线
    bot_y = dot_y + 24
    bot_glow = QLinearGradient(0, bot_y, w, bot_y)
    bot_glow.setColorAt(0.0, QColor(0, 240, 255, 0))
    bot_glow.setColorAt(0.5, QColor(0, 240, 255, 50))
    bot_glow.setColorAt(1.0, QColor(0, 240, 255, 0))
    painter.setPen(Qt.NoPen)
    painter.setBrush(bot_glow)
    painter.drawRect(int(w * 0.25), bot_y, int(w * 0.5), 1)

    painter.end()
    return pixmap


def main():
    app = QApplication(sys.argv)

    qss_path = os.path.join(os.path.dirname(__file__), "go2_ui", "resources", "qss", "cyber_blue.qss")
    with open(qss_path, "r", encoding="utf-8") as f:
        app.setStyleSheet(f.read())

    splash_pix = create_splash_pixmap()
    splash = SplashScreen(splash_pix)
    splash.show()
    app.processEvents()

    window = Dashboard()

    def show_main():
        splash.finish(window)
        window.showMaximized()

    QTimer.singleShot(0, show_main)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
