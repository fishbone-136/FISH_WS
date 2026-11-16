"""UI integration checks: no ROS context, devices, goals or robot commands."""
import os
from types import SimpleNamespace
import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QTabWidget, QWidget
from go2_ui.navigation import navigation_page as page_module
from go2_ui.navigation.ros_bridge import RosNavigationWorker
from go2_ui.navigation.rviz_view import rviz_config_path
from tests.test_dashboard_interaction import dashboard


@pytest.fixture(autouse=True)
def existing_stack_by_default(monkeypatch):
    # UI tests must never discover or start the physical robot's stack.
    from go2_navigation import runtime_guard
    monkeypatch.setattr(runtime_guard, 'conflicting_navigation_processes', lambda **kwargs: [9000])


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
    capabilities_updated = pyqtSignal(object)
    control_finished = pyqtSignal(str, bool, str)

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
    assert page.rviz_view.parentWidget() is not None
    assert page.findChildren(QTabWidget) == []
    assert not hasattr(page, 'map_canvas')
    assert '2.5D' in page.mode_combo.currentText()
    assert 'SLAM' not in ''.join(page.mode_combo.itemText(i) for i in range(page.mode_combo.count()))
    window.resize(800, 600)
    app.processEvents()
    assert window.width() == 800
    assert window.navigation_scroll.horizontalScrollBar().maximum() > 0
    worker = page.worker
    page._start_stack()  # Existing stack: connect without a duplicate launch.
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


def test_embedded_profile_preserves_goal_dispatch_and_map_topics():
    import yaml
    from go2_ui.navigation.rviz_view import PROJECT_ROOT, build_ui_rviz_profile, rviz_style_path
    from copy import deepcopy
    source = yaml.safe_load((PROJECT_ROOT / 'ros2_ws/src/go2_lio_navigation/config/lio_environment.rviz').read_text())
    original = deepcopy(source)
    adapted = build_ui_rviz_profile(source)
    assert source == original
    assert adapted['Visualization Manager']['Displays'] == source['Visualization Manager']['Displays']
    assert adapted['Visualization Manager']['Tools'] == source['Visualization Manager']['Tools']
    assert [p['Class'] for p in adapted['Panels']] == ['nav2_rviz_plugins/Navigation 2']
    assert adapted['Window Geometry']['Hide Left Dock'] is True
    assert adapted['Window Geometry']['Navigation 2']['collapsed'] is False
    assert adapted['Window Geometry']['QMainWindow State']
    assert adapted['Visualization Manager']['Views']['Current']['Target Frame'] == 'base_link'
    assert rviz_style_path().is_file()


def test_map_window_layout_hides_loaded_nav2_panel(dashboard):
    from PyQt5.QtCore import QByteArray
    from PyQt5.QtWidgets import QMainWindow, QDockWidget
    from go2_ui.navigation.rviz_view import EMBEDDED_WINDOW_STATE
    app, _window = dashboard
    frame = QMainWindow()
    frame.setCentralWidget(QWidget())
    toolbar = frame.addToolBar('Tools')
    toolbar.setObjectName('Tools')
    panel = QDockWidget('Navigation 2')
    panel.setObjectName('Navigation 2')
    panel.setWidget(QWidget())
    frame.addDockWidget(Qt.LeftDockWidgetArea, panel)
    assert frame.restoreState(QByteArray.fromHex(EMBEDDED_WINDOW_STATE.encode()))
    frame.show()
    app.processEvents()
    assert not panel.isVisible()
    assert toolbar.isVisible()
    assert frame.findChild(QDockWidget, 'Navigation 2') is panel
    frame.close()


def test_expanded_map_keeps_emergency_stop_and_sends_no_commands(dashboard, monkeypatch):
    app, window = dashboard
    monkeypatch.setattr(page_module, 'RosNavigationWorker', DisplayWorker)
    monkeypatch.setattr(page_module.RvizView, 'start', lambda self: None)
    monkeypatch.setattr(page_module.RvizView, 'shutdown', lambda self: None)
    window.switch_page(1)
    app.processEvents()
    page = window.navigation_page
    QTest.mouseClick(page.rviz_view.expand_button, Qt.LeftButton)
    app.processEvents()
    assert page.side_column.isHidden()
    assert page.emergency_stop_button.isVisible()
    assert page.rviz_view.expand_button.text() == '恢复侧栏'
    QTest.mouseClick(page.rviz_view.expand_button, Qt.LeftButton)
    app.processEvents()
    assert not page.side_column.isHidden()
    assert page.emergency_stop_button.isVisible()
    assert page.worker.commands == []


def test_rviz_rebinds_when_profile_recreates_native_window(dashboard, monkeypatch):
    from go2_ui.navigation import rviz_view
    app, _ = dashboard
    view = rviz_view.RvizView()
    view._process = SimpleNamespace(state=lambda: rviz_view.QProcess.Running, processId=lambda: 42)
    current = [100]
    handles = []
    view._x11 = SimpleNamespace(find=lambda pid: current[0], close=lambda: None,
        embed=lambda window,parent:handles.append(window),fit=lambda *args:None)
    view._started_at = rviz_view.time.monotonic() - 3.
    view._timer.start()
    view._attach()
    assert view._container is None  # Wait for a stable native window.
    view._candidate_since -= 2.
    view._attach()
    first = view._container
    assert view._window_id == 100
    current[0] = 200
    view._attach()
    view._candidate_since -= 2.
    view._attach()
    assert view._window_id == 200
    assert view._container is not first
    assert handles == [100, 200]
    assert view._timer.isActive()
    view._timer.stop()
    view.deleteLater()
    app.processEvents()


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
