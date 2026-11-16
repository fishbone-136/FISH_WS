from PyQt5.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QScrollArea,
    QSizeGrip,
    QStackedWidget,
)
from PyQt5.QtCore import Qt, QEvent
from go2_ui.ui.top_bar import TopBar
from go2_ui.ui.main_control_panel import MainControlPanel
from go2_ui.ui.right_panel import RightPanel
from go2_ui.ui.widgets.cyber_background import CyberBackground
from go2_ui.ui.bottom_nav import BottomNav


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
        self.main_panel.motion_start_allowed = self._motion_start_allowed
        self.main_panel.relative_motion_handoff = self._request_relative_handoff
        self.main_panel.visual_move_requested.connect(self._request_visual_move)
        self.main_panel.visual_target_located.connect(self._visual_target_ready)
        self.main_panel.visual_cancel_requested.connect(self._cancel_visual_move)
        self.main_panel.visual_grounding_failed.connect(self._visual_grounding_failed)
        self._visual_session = None
        self._near_session = None
        self._relative_handoff = None
        self._relative_session = None
        import logging
        logging.getLogger('go2.ui').info('视觉对话动作路由已加载：直接文字动作 / RGB-D 目标导航（20261111）')
        logging.getLogger('go2.ui').info('视觉移动优化已加载：CUDA 人物检测 / 近距 SDK 闭环 / 复用导航（20261111）')
        logging.getLogger('go2.ui').info('连续文字动作已加载：转身 / 直行 / 导航与 SDK 独占执行（20261111）')
        logging.getLogger('go2.ui').info('导航连续动作优化已加载：复用 Spin / DriveOnHeading，保持定位（20261111）')
        self.top_bar.settings_btn.clicked.connect(self.main_panel.toggle_overlay)

        # Equal camera columns (mode : color : depth = 1 : 2 : 2).
        # Matching lower-row minima keeps both camera cards aligned even on
        # short screens, where the controls impose a minimum row height.
        lower_height = max(self.main_panel.bottom_container.minimumSizeHint().height(),
                           self.right_panel.analysis_frame.minimumSizeHint().height())
        self.main_panel.bottom_container.setMinimumHeight(lower_height)
        self.right_panel.analysis_frame.setMinimumHeight(lower_height)
        content_layout.addWidget(self.main_panel, 3)
        content_layout.addWidget(self.right_panel, 2)
        # Keep all controls reachable without forcing the window past the screen.
        main_page.setMinimumSize(960, 540)
        home_scroll = QScrollArea()
        home_scroll.setWidgetResizable(True)
        home_scroll.setFrameShape(QScrollArea.NoFrame)
        home_scroll.setWidget(main_page)
        self.home_scroll = home_scroll
        self.pages = QStackedWidget()
        self.pages.addWidget(home_scroll)
        root_layout.addWidget(self.pages, 1)
        self.bottom_nav = BottomNav(items=[('首页', 0, 'home.svg'), ('2.5D 导航', 1, 'daoh.svg')])
        self.bottom_nav.switch_page.connect(self.switch_page)
        self.bottom_nav.set_active(0)
        root_layout.addWidget(self.bottom_nav)
        self.size_grip = QSizeGrip(central)
        self.size_grip.setFixedSize(16, 16)

    def switch_page(self, index):
        if index == self.pages.currentIndex():
            return
        if index == 1:
            self.main_panel.enter_safe_idle()
            if self._near_session is not None and self._near_session.active:
                self.main_panel.on_visual_navigation_update('notice', 'SDK 视觉任务仍在退出，请稍后再进入导航。')
                return
            self._get_navigation_page()
            self.navigation_page.activate()
        elif hasattr(self, 'navigation_page'):
            self.navigation_page.deactivate()
        self.pages.setCurrentIndex(index)
        self.bottom_nav.set_active(index)

    def _get_navigation_page(self):
        if not hasattr(self, 'navigation_page'):
            from go2_ui.navigation.navigation_page import NavigationPage
            self.navigation_page = NavigationPage(self)
            self.navigation_page.setMinimumSize(940, 540)
            self.navigation_scroll = QScrollArea()
            self.navigation_scroll.setWidgetResizable(True)
            self.navigation_scroll.setFrameShape(QScrollArea.NoFrame)
            self.navigation_scroll.setWidget(self.navigation_page)
            self.pages.addWidget(self.navigation_scroll)
        return self.navigation_page

    def _request_visual_move(self, intent):
        if self._relative_session is not None and self._relative_session.active:
            self.main_panel.on_visual_navigation_update('notice', '正在执行导航相对动作，请先停止或等待完成。')
            return
        if self._relative_handoff is not None and self._relative_handoff.active:
            self.main_panel.on_visual_navigation_update('notice', '正在关闭导航并准备文字动作，请先停止或等待完成。')
            return
        if (self._visual_session is not None and self._visual_session.active) or (
                self._near_session is not None and self._near_session.active):
            self.main_panel.on_visual_navigation_update('notice', '已有视觉移动任务，请先停止或等待完成。')
            return
        mode = self.right_panel.analysis_widget.motion_mode.currentData()
        from voice.person_detector import person_side
        if (mode == 'nav' or (mode == 'auto' and (person_side(intent.target) is None
                                                or not self._motion_start_allowed()))):
            self._request_navigation_move(intent)
            return
        if self._near_session is None:
            from go2_ui.navigation.near_session import NearApproachSession
            self._near_session = NearApproachSession(self.right_panel.depth_widget.get_rgbd_snapshot,
                self.main_panel.on_visual_navigation_update, self._request_navigation_move,
                lambda: self.main_panel._mode_manager.active == '视觉感知', self)
        self._near_session.request(intent, allow_fallback=mode == 'auto')

    def _request_navigation_move(self, intent):
        if self._visual_session is None:
            from go2_ui.navigation.visual_session import VisualNavigationSession
            self._visual_session = VisualNavigationSession(self._get_navigation_page,
                self.right_panel.depth_widget.get_rgbd_snapshot,
                self.main_panel.locate_visual_target,
                self.main_panel.on_visual_navigation_update,
                lambda: self.main_panel._mode_manager.active == '视觉感知', self)
        self._visual_session.request(intent)

    def _request_relative_handoff(self, actions, warnings):
        from deploy.text_motion_controller import navigation_is_idle
        page = getattr(self, 'navigation_page', None)
        if page is None:
            return None  # External navigation must be inspected/closed from its page first.
        from voice.navigation_actions import supports_navigation
        if supports_navigation(actions):
            if self._relative_session is None:
                from go2_ui.navigation.relative_session import RelativeNavigationSession
                self._relative_session = RelativeNavigationSession(self._get_navigation_page,
                    self.main_panel.on_visual_navigation_update,
                    lambda: self.main_panel._mode_manager.active == '视觉感知', self)
            return self._relative_session.request(actions, warnings)
        if navigation_is_idle() and not (page is not None and
                (page._control_pending or page._close_failed or page._launch_process.state())):
            return None  # Another interactive source is blocking; do not close a display.
        if self._relative_handoff is None:
            from go2_ui.navigation.relative_handoff import RelativeMotionHandoff
            self._relative_handoff = RelativeMotionHandoff(self._get_navigation_page,
                self.main_panel._start_text_motion, self.main_panel.on_relative_handoff_update,
                lambda: self.main_panel._mode_manager.active == '视觉感知', self)
        return self._relative_handoff.request(actions, warnings)

    def _visual_target_ready(self, target):
        if self._visual_session is not None:
            self._visual_session.target_ready(target)

    def _visual_grounding_failed(self, request_id, message):
        if self._visual_session is not None:
            self._visual_session.grounding_failed(request_id, message)

    def _cancel_visual_move(self):
        if self._relative_session is not None:
            self._relative_session.cancel()
        if self._relative_handoff is not None:
            self._relative_handoff.cancel()
        if self._near_session is not None:
            self._near_session.cancel()
        if self._visual_session is not None:
            self._visual_session.cancel()

    def _motion_start_allowed(self):
        # Interactive SDK control must not compete with the unchanged Nav2
        # stack. Passive camera/scene viewing remains available on the home.
        if self.main_panel._text_motion_active:
            return False
        if self._relative_session is not None and self._relative_session.active:
            return False
        if self._relative_handoff is not None and self._relative_handoff.active:
            return False
        if self._visual_session is not None and self._visual_session.active:
            return False
        if self._near_session is not None and self._near_session.active:
            return False
        page = getattr(self, 'navigation_page', None)
        if page is not None and (page._control_pending or page._close_failed or page._launch_process.state()):
            return False
        if page is not None and not page._navigation_closed and page._capabilities.get('can_control'):
            return False
        try:
            from go2_navigation.runtime_guard import conflicting_navigation_processes
            return not conflicting_navigation_processes(include_launch=True)
        except ImportError:
            return True  # Explicit offline UI deployment has no ROS stack.

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
        self._cancel_visual_move()
        if self._near_session is not None and self._near_session.active:
            event.ignore()
            self.main_panel.on_visual_navigation_update('notice', '正在等待 SDK 视觉任务停止，请稍后关闭界面。')
            return
        if hasattr(self, 'navigation_page'):
            self.navigation_page.shutdown()
        if hasattr(self, "main_panel"):
            self.main_panel.shutdown()
        if hasattr(self, "right_panel"):
            self.right_panel.shutdown()
        super().closeEvent(event)
