"""UI integration checks: no ROS context, devices, goals or robot commands."""
import os
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtTest import QTest
from go2_ui.navigation import navigation_page as page_module
from go2_ui.navigation.ros_bridge import RosNavigationWorker
from go2_ui.navigation.rviz_view import rviz_config_path
from tests.test_dashboard_interaction import dashboard


class DisplayWorker(QObject):
    map_updated = pyqtSignal(object)
    pose_updated = pyqtSignal(float, float, float)
    path_updated = pyqtSignal(object)
    status_updated = pyqtSignal(str, str)
    feedback_updated = pyqtSignal(float, str)
    goal_finished = pyqtSignal(bool, str)
    map_saved = pyqtSignal(bool, str)
    log_message = pyqtSignal(str)
    terrain_publishers_updated = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.commands = []

    def start(self):
        pass

    def enqueue(self, *args):
        self.commands.append(args)

    def stop(self):
        self.enqueue('shutdown')

    def wait(self, *_args):
        return True


def test_page_switches_keep_external_navigation_running(dashboard, monkeypatch):
    app, window = dashboard
    calls = []
    monkeypatch.setattr(page_module, 'RosNavigationWorker', DisplayWorker)
    monkeypatch.setattr(page_module.RvizView, 'start', lambda self: calls.append('rviz view'))
    monkeypatch.setattr(page_module.RvizView, 'shutdown', lambda self: calls.append('rviz close'))
    window.switch_page(1)
    app.processEvents()
    page = window.navigation_page
    assert window.pages.currentWidget() is window.navigation_scroll
    assert window.navigation_scroll.widget() is page
    assert page.mode_combo.currentIndex() == 0
    assert page.view_tabs.currentIndex() == 0
    assert 'RViz' in page.view_tabs.tabText(0)
    assert 'SLAM' not in ''.join(page.mode_combo.itemText(i) for i in range(page.mode_combo.count()))
    window.resize(800, 600)
    app.processEvents()
    assert window.width() == 800
    assert window.navigation_scroll.horizontalScrollBar().maximum() > 0
    worker = page.worker
    page._start_stack()  # Default connect only: no navigation launch.
    page._stop_stack()   # There is no UI-owned launch: do not cancel anything.
    window.switch_page(0)
    app.processEvents()
    assert worker.commands == []
    window.close()
    assert worker.commands == [('shutdown',)]
    assert calls[-1] == 'rviz close'


def test_worker_close_is_display_only_and_soft_cost_is_not_rejected():
    worker = RosNavigationWorker()
    worker.stop()
    assert worker._commands.get_nowait() == ('shutdown', ())
    assert worker._commands.empty()
    worker._map_payload = dict(width=2, height=1, resolution=1., origin_x=0.,
                               origin_y=0., origin_yaw=0., data=bytes([95,100]))
    assert worker._validate_goal(.5,.5,True) == ''
    assert '占用' in worker._validate_goal(1.5,.5,True)


def test_rviz_reads_current_deployed_profile():
    path = rviz_config_path()
    assert path.is_file()
    text = path.read_text()
    assert '/mapping/environment_map' in text
    assert '/visualization/environment_terrain' in text
    assert '/local_costmap/costmap' in text
    assert 'Environment Structure' in text


def test_path_is_transformed_to_navigation_map_frame(monkeypatch):
    from go2_ui.navigation import ros_bridge
    monkeypatch.setattr(ros_bridge, 'Time', lambda: None, raising=False)
    worker = RosNavigationWorker()
    worker._display_frame = 'odom_lio'
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=10., y=20.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    worker._tf_buffer = SimpleNamespace(lookup_transform=lambda target, source, when: transform)
    results = []
    worker.path_updated.connect(results.append)
    pose = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1., y=2.)))
    worker._on_path(SimpleNamespace(header=SimpleNamespace(frame_id='map'), poses=[pose]))
    assert results == [[(11.,22.)]]


def test_interactive_control_does_not_start_over_navigation(dashboard, monkeypatch):
    from go2_ui.ui import main_control_panel as controls
    _, window = dashboard
    monkeypatch.setattr(controls, '_DEPLOY_MODE', True)
    window.main_panel.motion_start_allowed = lambda: False
    QTest.mouseClick(window.main_panel.voice_btn, Qt.LeftButton)
    assert window.main_panel.speech_ctrl is None
    assert window.main_panel._mode_manager.active is None
    assert '导航正在运行' in window.main_panel.log_top.toPlainText()
