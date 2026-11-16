from PyQt5.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QScrollArea,
    QSizeGrip,
)
from PyQt5.QtCore import Qt, QEvent
from go2_ui.ui.top_bar import TopBar
from go2_ui.ui.main_control_panel import MainControlPanel
from go2_ui.ui.right_panel import RightPanel
from go2_ui.ui.widgets.cyber_background import CyberBackground


class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("邦邦机器狗智能交互控制平台")
        self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint)
        screen = self.screen()
        if screen is not None:
            area = screen.availableGeometry()
            self.setGeometry(area.adjusted(area.width() // 10, area.height() // 10,
                                           -area.width() // 10, -area.height() // 10))
        self.init_ui()

    def init_ui(self):
        central = CyberBackground()
        self.setCentralWidget(central)

        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(10, 10, 10, 10)
        root_layout.setSpacing(10)

        self.top_bar = TopBar()
        root_layout.addWidget(self.top_bar, 0)

        main_page = QWidget()
        content_layout = QHBoxLayout(main_page)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(10)

        self.main_panel = MainControlPanel(right_panel=None)  # 先创建，后面注入
        self.right_panel = RightPanel()
        self.main_panel.set_right_panel(self.right_panel)
        self.top_bar.settings_btn.clicked.connect(self.main_panel.toggle_overlay)

        content_layout.addWidget(self.main_panel, 7)
        content_layout.addWidget(self.right_panel, 3)
        # Keep all controls reachable without forcing the window past the screen.
        main_page.setMinimumSize(960, 540)
        home_scroll = QScrollArea()
        home_scroll.setWidgetResizable(True)
        home_scroll.setFrameShape(QScrollArea.NoFrame)
        home_scroll.setWidget(main_page)
        self.home_scroll = home_scroll
        root_layout.addWidget(home_scroll, 1)
        self.size_grip = QSizeGrip(central)
        self.size_grip.setFixedSize(16, 16)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "size_grip"):
            self.size_grip.move(self.centralWidget().width() - 16,
                                self.centralWidget().height() - 16)

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() == QEvent.WindowStateChange and hasattr(self, "top_bar"):
            self.top_bar.update_window_buttons()
            self.size_grip.setVisible(not self.isMaximized())

    def closeEvent(self, event):
        if hasattr(self, "main_panel"):
            self.main_panel.shutdown()
        if hasattr(self, "right_panel"):
            self.right_panel.shutdown()
        super().closeEvent(event)
