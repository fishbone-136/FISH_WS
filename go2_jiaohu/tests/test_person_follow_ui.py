"""UI follower integration with fake devices/SDK; never sends robot commands."""
import os
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication
from go2_ui.person_follow import PersonFollowSession
from tests.test_person_follow import scene, project, BOX
from tests.test_dashboard_interaction import dashboard


def wait_until(condition,timeout=2500):
    end = time.monotonic()+timeout/1000
    while not condition() and time.monotonic()<end:
        QApplication.processEvents()
        QTest.qWait(10)
    assert condition()


@pytest.fixture
def session_fixture():
    app = QApplication.instance() or QApplication([])
    events = []
    camera = SimpleNamespace(start=lambda:events.append('IR start'),snapshot=scene,
                             close=lambda:events.append('IR stop'))
    depth = SimpleNamespace(pause_capture=lambda:events.append('RGB stop'),
        resume_capture=lambda:events.append('RGB resume'),
        get_latest_color_sample=lambda:(1,scene().color),show_external_snapshot=Mock())
    people = SimpleNamespace(detect=Mock(return_value=[BOX]),close=lambda:events.append('detector close'))
    controller = SimpleNamespace(active=False,cancel=Mock(),start=Mock())
    def factory(snapshot,update,**kwargs):
        controller.update = update
        def start(**kwargs):
            controller.active = True
            return True
        controller.start.side_effect = start
        return controller
    session = PersonFollowSession(depth,camera_factory=lambda:camera,people=people,
                                  controller_factory=factory,project=project)
    yield SimpleNamespace(app=app,session=session,camera=camera,depth=depth,people=people,
                          controller=controller,events=events)
    controller.active = False
    session.close(restore_camera=False)
    wait_until(lambda:not session.active)
    session.deleteLater()


def test_open_is_preview_only_and_releases_rgb_before_ir(session_fixture):
    f = session_fixture
    assert f.session.open()
    wait_until(lambda:f.session._ready)
    assert f.events[:2] == ['RGB stop','IR start']
    f.controller.start.assert_not_called()
    sequence,image = f.session.frame()
    assert sequence > 0 and image.shape == (240,320,3)
    assert f.depth.show_external_snapshot.called


def test_motion_requires_explicit_start_and_keeps_tested_defaults(session_fixture):
    f = session_fixture
    f.session.open()
    assert not f.session.start_motion()
    wait_until(lambda:f.session._ready)
    assert f.session.start_motion()
    f.controller.start.assert_called_once_with(stand_off=1.5,max_speed=.35,duration=120.)
    f.session.stop_motion()
    assert f.controller.cancel.called
    assert f.session.active


def test_close_waits_for_motion_before_restoring_rgb(session_fixture):
    f = session_fixture
    f.session.open()
    wait_until(lambda:f.session._ready)
    f.session.start_motion()
    assert f.session.close() is False
    QTest.qWait(80)
    assert f.session.closing
    assert 'IR stop' not in f.events and 'RGB resume' not in f.events
    f.controller.active = False
    wait_until(lambda:not f.session.active)
    assert f.events.index('IR stop') < f.events.index('RGB resume')


def test_stop_during_model_loading_keeps_gui_responsive_and_no_late_start(session_fixture):
    f = session_fixture
    entered,release = threading.Event(),threading.Event()
    def load(_):
        entered.set()
        release.wait(2.)
        return [BOX]
    f.people.detect.side_effect = load
    f.session.open()
    wait_until(entered.is_set)
    begin = time.monotonic()
    assert f.session.close(restore_camera=False) is False
    assert time.monotonic()-begin < .1
    QTest.qWait(60)
    assert f.session.closing
    assert not f.session.start_motion()
    release.set()
    wait_until(lambda:not f.session.active)
    assert 'RGB resume' not in f.events
    f.controller.start.assert_not_called()


def test_camera_start_failure_restores_rgb_without_motion(session_fixture):
    f = session_fixture
    f.camera.start = Mock(side_effect=RuntimeError('camera busy'))
    with pytest.raises(RuntimeError,match='camera busy'):
        f.session.open()
    wait_until(lambda:not f.session.active)
    assert 'RGB resume' in f.events
    f.controller.start.assert_not_called()


def test_follow_error_keeps_preview_and_allows_retry(session_fixture):
    f = session_fixture
    f.session.open()
    wait_until(lambda:f.session._ready)
    assert f.session.start_motion()
    f.controller.update('目标不确定，已停止',stopped=True)
    f.controller.active = False
    wait_until(lambda:not f.session.motion_active)
    assert f.session.active
    assert f.session._status == '目标不确定，已停止'
    assert f.session.start_motion()


class FakeSession(QObject):
    status_changed = pyqtSignal(str)
    state_changed = pyqtSignal(bool,bool,bool)
    metrics_changed = pyqtSignal(str)
    closed = pyqtSignal()
    instances = []

    def __init__(self,depth,parent=None):
        super().__init__(parent)
        self.active = self.closing = self.motion_active = False
        self.starts = []
        self.closes = []
        self.stops = 0
        self.instances.append(self)

    def open(self):
        self.active = True
        self.state_changed.emit(True,False,False)
        self.status_changed.emit('预览就绪')
        return True

    def frame(self):
        return 0,None

    def start_motion(self,speed,distance):
        self.starts.append((speed,distance))
        self.motion_active = True
        self.state_changed.emit(True,True,False)

    def stop_motion(self):
        self.stops += 1
        self.motion_active = False
        self.state_changed.emit(True,False,False)

    def close(self,restore_camera=True):
        if not self.active:
            return True
        self.closes.append(restore_camera)
        self.closing = True
        self.state_changed.emit(False,self.motion_active,True)
        return False

    def finish(self):
        self.active = self.closing = self.motion_active = False
        self.closed.emit()


@pytest.fixture
def follow_dashboard(dashboard,monkeypatch):
    from go2_ui import person_follow
    from go2_ui.ui import main_control_panel as controls
    FakeSession.instances = []
    monkeypatch.setattr(person_follow,'PersonFollowSession',FakeSession)
    monkeypatch.setattr(controls,'_DEPLOY_MODE',True)
    app,window = dashboard
    window.main_panel.motion_start_allowed = lambda:True
    yield app,window
    for session in FakeSession.instances:
        if session.active:
            session.finish()
    app.processEvents()


def test_follow_button_embeds_controls_and_explicit_motion_start(follow_dashboard):
    app,window = follow_dashboard
    panel = window.main_panel
    QTest.mouseClick(panel.follow_btn,Qt.LeftButton)
    session = panel._follow_session
    assert panel._mode_manager.active == '人体跟随'
    assert panel.follow_btn.property('active') is True
    assert panel._follow_controls.isVisible()
    assert not window.right_panel.analysis_widget.isVisible()
    assert not session.starts
    panel.follow_btn.clicked.emit()
    assert len(FakeSession.instances) == 1
    panel._follow_controls.start_btn.click()
    assert session.starts == [(.35,1.5)]
    panel._follow_controls.stop_btn.click()
    assert session.stops == 1
    assert panel._mode_manager.active == '人体跟随'
    panel.stop_btn.click()
    assert panel._follow_session.closing
    session.finish()
    app.processEvents()
    assert panel._mode_manager.active is None
    assert window.right_panel.analysis_widget.isVisible()
    assert not panel._follow_controls.isVisible()


def test_mode_switch_waits_for_follow_stop_and_camera_release(follow_dashboard,monkeypatch):
    app,window = follow_dashboard
    panel = window.main_panel
    starts = []
    monkeypatch.setattr(panel,'_start_speech',lambda:starts.append('voice') or True)
    panel.follow_btn.clicked.emit()
    session = panel._follow_session
    panel.voice_btn.clicked.emit()
    assert session.closing and starts == []
    assert panel._mode_manager.active == '人体跟随'
    session.finish()
    app.processEvents()
    assert starts == ['voice']
    assert panel._mode_manager.active == '语音交互'


def test_stop_cancels_a_queued_mode_switch(follow_dashboard,monkeypatch):
    app,window = follow_dashboard
    panel = window.main_panel
    start = Mock(return_value=True)
    monkeypatch.setattr(panel,'_start_speech',start)
    panel.follow_btn.clicked.emit()
    session = panel._follow_session
    panel.voice_btn.clicked.emit()
    panel.stop_btn.click()
    session.finish()
    app.processEvents()
    start.assert_not_called()
    assert panel._mode_manager.active is None


def test_stop_also_cancels_switch_already_posted_to_gui_queue(follow_dashboard,monkeypatch):
    app,window = follow_dashboard
    panel = window.main_panel
    start = Mock(return_value=True)
    monkeypatch.setattr(panel,'_start_speech',start)
    panel.follow_btn.clicked.emit()
    session = panel._follow_session
    panel.voice_btn.clicked.emit()
    session.finish()
    panel.stop_current_mode()
    app.processEvents()
    start.assert_not_called()
    assert panel._mode_manager.active is None


def test_conflicting_motion_task_blocks_follow_start(follow_dashboard):
    app,window = follow_dashboard
    panel = window.main_panel
    panel.follow_btn.clicked.emit()
    panel.motion_start_allowed = lambda:False
    panel._follow_controls.start_btn.click()
    assert panel._follow_session.starts == []
    assert '其他运动任务' in panel._follow_controls.status.toPlainText()


def test_navigation_waits_until_follow_releases_control(follow_dashboard,monkeypatch):
    app,window = follow_dashboard
    panel = window.main_panel
    panel.follow_btn.clicked.emit()
    navigation = Mock()
    monkeypatch.setattr(window,'_get_navigation_page',navigation)
    window.switch_page(1)
    assert panel._follow_session.closing
    assert window.pages.currentIndex() == 0
    navigation.assert_not_called()


def test_window_close_waits_for_follow_and_does_not_reopen_camera(follow_dashboard):
    app,window = follow_dashboard
    panel = window.main_panel
    panel.follow_btn.clicked.emit()
    session = panel._follow_session
    window.close()
    assert window.isVisible()
    assert session.closing and session.closes == [False,False]
    session.finish()
    app.processEvents()
    assert not window.isVisible()


def test_depth_camera_handoff_clears_stale_frames_and_displays_native_depth(monkeypatch):
    from go2_ui.ui import right_panel
    app = QApplication.instance() or QApplication([])
    monkeypatch.setattr(right_panel,'HAS_REALSENSE',False)
    widget = right_panel.DepthWidget()
    pipeline = SimpleNamespace(stop=Mock())
    widget._pipeline = pipeline
    widget._running = True
    widget._buf_color_bgr = scene().color
    widget._rgbd_sample = ('old',)
    widget.pause_capture()
    pipeline.stop.assert_called_once()
    assert widget._paused and not widget._running
    assert widget.get_latest_color_frame() is None
    assert widget.get_rgbd_snapshot() is None
    sample = scene()
    widget.show_external_snapshot(sample)
    assert widget._buf_depth_raw is sample.depth_mm
    assert widget.depth_label.pixmap() is not None
    assert widget._pipeline is None
    start = Mock()
    monkeypatch.setattr(widget,'_start_realsense',start)
    widget.resume_capture()
    start.assert_called_once()
    assert not widget._paused
    assert widget._buf_depth_raw is None
    widget.close()


@pytest.mark.parametrize('size',[(1280,800),(1280,720),(1024,768),(800,600)])
def test_follow_controls_remain_reachable_on_small_screens(follow_dashboard,size):
    app,window = follow_dashboard
    window.resize(*size)
    panel = window.main_panel
    panel.follow_btn.clicked.emit()
    app.processEvents()
    if size == (1280,800):
        assert window.home_scroll.verticalScrollBar().maximum() == 0
    for button in (panel._follow_controls.start_btn,panel.stop_btn):
        window.home_scroll.ensureWidgetVisible(button)
        app.processEvents()
        assert window.home_scroll.viewport().rect().contains(
            button.mapTo(window.home_scroll.viewport(),button.rect().center()))
