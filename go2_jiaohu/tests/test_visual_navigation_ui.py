"""Visual text-box -> target grounding -> shared Nav2 lifecycle with fakes."""
import os
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

from go2_ui.navigation.visual_session import VisualNavigationSession
from tests.test_dashboard_interaction import dashboard
from voice.visual_navigation import ApproachIntent, GroundedTarget, RgbdSnapshot


class NavigationWorker(QObject):
    visual_goal_updated = pyqtSignal(str,str,str)

    def __init__(self):
        super().__init__()
        self.commands = []

    def enqueue(self, *command):
        self.commands.append(command)


@pytest.fixture
def fixture():
    app = QApplication.instance() or QApplication([])
    worker = NavigationWorker()
    page = SimpleNamespace(worker=worker,_navigation_state='ready',_last_navigation_message='',
        _navigation_closed=False,_control_pending='',_close_failed=False,_start_stack=Mock(),
        _capabilities={'camera_calibrated':True})
    sample = RgbdSnapshot(np.full((100,100,3),80,np.uint8),np.full((100,100),3000,np.uint16),
        time.time(),time.monotonic(),lambda *a:(0.,0.,3.))
    locate, update = Mock(return_value=True), Mock()
    mode = [True]
    session = VisualNavigationSession(lambda:page,lambda:sample,locate,update,lambda:mode[0])
    result = SimpleNamespace(app=app,worker=worker,page=page,snapshot=sample,session=session,
        locate=locate,update=update,mode=mode)
    yield result
    session.cancel()


def test_request_waits_for_navigation_and_depth_before_sending_goal(fixture):
    n = fixture
    n.page._navigation_state = 'waiting'
    assert n.session.request(ApproachIntent('黑色椅子'))
    assert not n.worker.commands
    n.page._navigation_state = 'ready'
    n.session._poll()
    operation, ticket = n.worker.commands[-1]
    assert operation == 'visual_preflight'
    n.worker.visual_goal_updated.emit(ticket,'prepared','ready')
    n.locate.assert_called_once_with(n.snapshot,ApproachIntent('黑色椅子'),ticket)
    target = GroundedTarget(ticket,ApproachIntent('黑色椅子'),n.snapshot,(.1,.1,.9,.9),(0,0,3))
    n.session.target_ready(target)
    assert n.worker.commands[-1] == ('visual_goal',target)
    n.worker.visual_goal_updated.emit(ticket,'accepted','accepted')
    assert n.session.active
    n.worker.visual_goal_updated.emit(ticket,'succeeded','done')
    assert not n.session.active
    assert '黑色椅子' in n.update.call_args.args[1]


def test_cancel_before_model_response_prevents_late_motion(fixture):
    n = fixture
    n.session.request(ApproachIntent('椅子'))
    ticket = n.worker.commands[-1][1]
    n.worker.visual_goal_updated.emit(ticket,'prepared','ready')
    n.session.cancel()
    n.session.target_ready(GroundedTarget(ticket,ApproachIntent('椅子'),n.snapshot,(.1,.1,.9,.9),(0,0,3)))
    assert not any(command[0] == 'visual_goal' for command in n.worker.commands)
    assert not any(command[0] in ('cancel','emergency_stop') for command in n.worker.commands)
    n.session.request(ApproachIntent('桌子'))
    assert n.worker.commands[-1][1] != ticket
    n.worker.visual_goal_updated.emit(ticket,'prepared','stale')
    assert n.locate.call_count == 1


def test_reuses_ready_navigation_without_relaunching_viewer_or_stack(fixture):
    n = fixture
    n.session.request(ApproachIntent('人'))
    n.page._start_stack.assert_not_called()
    assert n.worker.commands[-1][0] == 'visual_preflight'


def test_cold_visual_navigation_launch_does_not_start_rviz(fixture):
    n = fixture
    n.page._navigation_state = 'waiting'
    n.session.request(ApproachIntent('人'))
    n.page._start_stack.assert_called_once_with(background=True)


def test_old_grounding_error_does_not_abort_a_new_request(fixture):
    n=fixture
    n.session.request(ApproachIntent('椅子'))
    old_ticket=n.worker.commands[-1][1]
    n.worker.visual_goal_updated.emit(old_ticket,'prepared','ready')
    n.session.cancel()
    n.session.request(ApproachIntent('桌子'))
    ticket=n.worker.commands[-1][1]
    n.worker.visual_goal_updated.emit(ticket,'prepared','ready')
    n.session.grounding_failed(old_ticket,'stale error')
    assert n.session.active
    n.session.grounding_failed(ticket,'current failure')
    assert not n.session.active


def test_cancel_after_dispatch_cancels_only_the_owned_goal(fixture):
    n = fixture
    n.session.request(ApproachIntent('椅子'))
    ticket = n.worker.commands[-1][1]
    n.worker.visual_goal_updated.emit(ticket,'prepared','ready')
    n.session.target_ready(GroundedTarget(ticket,ApproachIntent('椅子'),n.snapshot,(.1,.1,.9,.9),(0,0,3)))
    n.session.cancel()
    assert n.worker.commands[-1] == ('cancel_visual',ticket)


def test_closed_navigation_and_changed_mode_cancel_pending_target(fixture):
    n = fixture
    n.session.request(ApproachIntent('椅子'))
    n.page._navigation_closed = True
    n.session._poll()
    assert not n.session.active
    n.page._navigation_closed = False
    n.session.request(ApproachIntent('椅子'))
    n.mode[0] = False
    n.session._poll()
    assert not n.session.active
    assert not any(command[0] == 'visual_goal' for command in n.worker.commands)


def test_text_box_submits_visual_target_and_stop_works_while_busy(dashboard, monkeypatch):
    from go2_ui.ui import main_control_panel as controls
    app, window = dashboard
    panel = window.main_panel
    monkeypatch.setattr(controls,'_DEPLOY_MODE',True)
    monkeypatch.setattr(panel,'_vlm_analyze_scene',lambda:True)
    snapshot = RgbdSnapshot(np.full((100,100,3),80,np.uint8),np.full((100,100),3000,np.uint16),
        time.time(),time.monotonic(),lambda *args:(0,0,3))
    monkeypatch.setattr(window.right_panel.depth_widget,'get_rgbd_snapshot',lambda:snapshot)
    worker = NavigationWorker()
    page = SimpleNamespace(worker=worker,_navigation_state='ready',_last_navigation_message='',
        _navigation_closed=False,_control_pending='',_close_failed=False,_start_stack=Mock(),
        _capabilities={'camera_calibrated':True})
    monkeypatch.setattr(window,'_get_navigation_page',lambda:page)
    callbacks = []
    class Vision:
        is_ready = True
        def locate_target_async(self,sample,intent,ticket,callback,**kwargs):
            callbacks.append((intent,ticket,callback))
            return True
        def cancel_pending(self):
            pass
    monkeypatch.setattr(controls,'VLMController',Vision)
    QTest.mouseClick(panel.vlm_btn, Qt.LeftButton)
    panel.log_bottom.setPlainText('走到黑色椅子一米前')
    panel.send_btn.click()
    assert worker.commands[0][0] == 'visual_preflight'
    ticket = worker.commands[0][1]
    worker.visual_goal_updated.emit(ticket,'prepared','正在定位')
    assert callbacks[0][0] == ApproachIntent('黑色椅子',1.)
    assert panel._visual_busy
    assert panel.send_btn.isEnabled()
    panel.log_bottom.setPlainText('停止')
    panel.send_btn.click()
    intent, _, callback = callbacks[0]
    callback(GroundedTarget(ticket,intent,snapshot,(.1,.1,.9,.9),(0,0,3)))
    app.processEvents()
    assert not window._visual_session.active
    assert not any(command[0] == 'visual_goal' for command in worker.commands)


def test_text_box_keeps_questions_on_readonly_path(dashboard, monkeypatch):
    _, window = dashboard
    panel = window.main_panel
    monkeypatch.setattr(panel,'_vlm_analyze_scene',lambda:True)
    monkeypatch.setattr(panel,'_get_scene_frame',lambda:np.zeros((50,50,3),np.uint8))
    calls=[]
    class Vision:
        is_ready=True
        def query_scene_async(self,frame,text,on_result):
            calls.append(text)
            on_result('左侧是一把黑色椅子。')
            return True
        def cancel_pending(self):
            pass
    from go2_ui.ui import main_control_panel as controls
    monkeypatch.setattr(controls,'VLMController',Vision)
    QTest.mouseClick(panel.vlm_btn, Qt.LeftButton)
    panel.log_bottom.setPlainText('黑色椅子在哪里？')
    panel.send_btn.click()
    assert calls == ['黑色椅子在哪里？']
    assert window._visual_session is None
    assert not hasattr(window,'navigation_page')


def test_aligned_rgbd_snapshot_keeps_pair_intrinsics_and_depth_units(dashboard, monkeypatch):
    from go2_ui.ui import right_panel
    _, window = dashboard
    camera = window.right_panel.depth_widget
    camera._running = True
    camera._depth_scale_m = .0005
    intrinsics = object()
    depth = SimpleNamespace(get_frame_number=lambda:1,get_timestamp=lambda:1000.,
        get_data=lambda:np.full((40,60),6000,np.uint16),
        profile=SimpleNamespace(as_video_stream_profile=lambda:SimpleNamespace(get_intrinsics=lambda:intrinsics)))
    color = SimpleNamespace(get_frame_number=lambda:1,get_timestamp=lambda:1000.,
        get_data=lambda:np.full((40,60,3),80,np.uint8))
    frames = SimpleNamespace(get_depth_frame=lambda:depth,get_color_frame=lambda:color)
    aligned = SimpleNamespace(as_frameset=lambda:frames)
    process = Mock(return_value=aligned)
    camera._align = SimpleNamespace(process=process)
    monkeypatch.setattr(right_panel,'rs',SimpleNamespace(rs2_deproject_pixel_to_point=lambda i,px,z:(px[0],px[1],z)))
    camera._on_frameset(SimpleNamespace(as_frameset=lambda:frames))
    snapshot = camera.get_rgbd_snapshot()
    assert snapshot is not None
    assert np.all(snapshot.depth_mm == 3000)
    assert snapshot.color is not camera._rgbd_sample[0]
    assert snapshot.deproject(20.,15.,3.) == (20.,15.,3.)
    process.assert_called_once()
    camera._running = False
    assert camera.get_rgbd_snapshot() is None


def test_rgbd_with_unsynchronized_timestamps_is_not_used_for_motion(dashboard):
    _, window = dashboard
    camera = window.right_panel.depth_widget
    camera._running = True
    depth = SimpleNamespace(get_frame_number=lambda:1,get_timestamp=lambda:1000.,
        get_data=lambda:np.full((40,60),3000,np.uint16))
    color = SimpleNamespace(get_frame_number=lambda:1,get_timestamp=lambda:1100.,
        get_data=lambda:np.full((40,60,3),80,np.uint8))
    frames = SimpleNamespace(get_depth_frame=lambda:depth,get_color_frame=lambda:color)
    camera._align = SimpleNamespace(process=lambda f:SimpleNamespace(as_frameset=lambda:f))
    camera._on_frameset(SimpleNamespace(as_frameset=lambda:frames))
    assert camera.get_rgbd_snapshot() is None


def test_missing_camera_calibration_is_reported_before_navigation_launch(fixture):
    n=fixture
    n.page._capabilities={'camera_calibrated':False,'camera_error':'缺少相机安装标定'}
    n.session.request(ApproachIntent('黑色设备',1.))
    n.page._start_stack.assert_not_called()
    n.session._deadline=time.monotonic()-1.
    n.session._poll()
    assert not n.session.active
    assert '标定' in n.update.call_args.args[1]
    n.page._start_stack.assert_not_called()


def test_relative_text_command_uses_motion_executor_without_camera_or_llm(dashboard,monkeypatch):
    from go2_ui.ui import main_control_panel as controls
    from deploy import text_motion_controller
    _,window=dashboard
    panel=window.main_panel
    monkeypatch.setattr(controls,'_DEPLOY_MODE',True)
    monkeypatch.setattr(panel,'_vlm_analyze_scene',lambda:True)
    panel.motion_start_allowed=lambda:True
    execute=Mock(return_value=True)
    fake=SimpleNamespace(start=execute,cancel=Mock(),close=Mock())
    monkeypatch.setattr(text_motion_controller,'TextMotionController',lambda callback:fake)
    monkeypatch.setattr(panel,'_get_scene_frame',lambda:pytest.fail('relative actions must not require an image'))
    QTest.mouseClick(panel.vlm_btn,Qt.LeftButton)
    panel.log_bottom.setPlainText('往前走半米')
    panel.send_btn.click()
    assert execute.call_count==1
    actions,ticket=execute.call_args.args
    assert actions[0].action=='forward'
    assert window._visual_session is None
    panel.log_bottom.setPlainText('停止')
    panel.send_btn.click()
    fake.cancel.assert_called_once()
    assert '等待控制接口确认' in panel.log_top.toPlainText()
    panel._on_text_motion_update(ticket,'finished','已停止')
    assert not panel._text_motion_active


def test_relative_actions_are_blocked_when_navigation_owns_control(dashboard,monkeypatch):
    from go2_ui.ui import main_control_panel as controls
    _,window=dashboard
    panel=window.main_panel
    monkeypatch.setattr(controls,'_DEPLOY_MODE',True)
    monkeypatch.setattr(panel,'_vlm_analyze_scene',lambda:True)
    panel.motion_start_allowed=lambda:False
    QTest.mouseClick(panel.vlm_btn,Qt.LeftButton)
    panel.log_bottom.setPlainText('往前走半米')
    panel.send_btn.click()
    assert panel._text_motion is None
    assert '关闭导航' in panel.log_top.toPlainText()
