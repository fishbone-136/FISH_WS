from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtTest import QTest

from tests.test_dashboard_interaction import dashboard


def setup_panel(window, monkeypatch):
    from go2_ui.ui import main_control_panel as controls
    from deploy import text_motion_controller
    panel=window.main_panel
    monkeypatch.setattr(controls,'_DEPLOY_MODE',True)
    monkeypatch.setattr(panel,'_vlm_analyze_scene',lambda:True)
    monkeypatch.setattr(panel,'_get_scene_frame',lambda:pytest.fail('motion must not invoke scene Q&A'))
    sdk=SimpleNamespace(start=Mock(return_value=True),cancel=Mock(),close=Mock())
    monkeypatch.setattr(text_motion_controller,'TextMotionController',lambda callback:sdk)
    QTest.mouseClick(panel.vlm_btn,Qt.LeftButton)
    return panel,sdk


def fake_navigation(window, monkeypatch):
    from deploy import text_motion_controller
    from go2_navigation import runtime_guard
    idle=[False]
    monkeypatch.setattr(text_motion_controller,'navigation_is_idle',lambda:idle[0])
    monkeypatch.setattr(runtime_guard,'conflicting_navigation_processes',lambda **kw:[] if idle[0] else [999999])
    class Worker(QObject):
        relative_action_updated=pyqtSignal(str,str,str)
        def __init__(self):
            super().__init__(); self.commands=[]
        def enqueue(self,*command): self.commands.append(command)
        def isRunning(self): return False
    page=SimpleNamespace(_control_pending='',_close_failed=False,_navigation_closed=False,
        _capabilities={'online':True,'goal_active':False,'can_control':True},_goal_active=False,_goal_submitting=False,
        _navigation_state='ready',_last_navigation_message='',
        _launch_process=SimpleNamespace(state=Mock(return_value=1)),
        worker=Worker(),shutdown=Mock())
    def close(): page._control_pending='close'
    page._close_navigation=Mock(side_effect=close)
    window.navigation_page=page
    def finish():
        page._control_pending=''
        page._navigation_closed=True
        page._launch_process.state.return_value=0
        idle[0]=True
    return page,finish


def test_observed_turnaround_sequence_bypasses_scene_model(dashboard,monkeypatch):
    _,window=dashboard
    panel,sdk=setup_panel(window,monkeypatch)
    panel.motion_start_allowed=lambda:True
    panel.log_bottom.setPlainText('转身，然后直行半米')
    panel.send_btn.click()
    actions,_=sdk.start.call_args.args
    assert sum(action.angle for action in actions if action.action=='turn_left')==pytest.approx(180.)
    assert actions[-1].action=='forward'
    assert '动作计划' in panel.log_top.toPlainText()
    assert '90°' in panel.log_top.toPlainText()


def test_completed_navigation_is_reused_for_relative_sequence_without_sdk(dashboard,monkeypatch):
    _,window=dashboard
    panel,sdk=setup_panel(window,monkeypatch)
    page,finish=fake_navigation(window,monkeypatch)
    panel.log_bottom.setPlainText('转身，然后直行半米')
    panel.send_btn.click()
    page._close_navigation.assert_not_called()
    sdk.start.assert_not_called()
    assert window._relative_session.active
    assert panel.send_btn.isEnabled()  # Text stop stays available while closing.
    assert not window._motion_start_allowed()
    operation,ticket,actions=page.worker.commands[-1]
    assert operation=='relative_sequence'
    page.worker.relative_action_updated.emit(ticket,'succeeded','完成，导航保持就绪')
    assert not window._relative_session.active
    assert page._navigation_state=='ready' and not page._navigation_closed
    sdk.start.assert_not_called()


def test_stop_cancels_only_navigation_relative_sequence_and_never_replays_sdk(dashboard,monkeypatch):
    _,window=dashboard
    panel,sdk=setup_panel(window,monkeypatch)
    page,finish=fake_navigation(window,monkeypatch)
    panel.log_bottom.setPlainText('转身，然后直行半米')
    panel.send_btn.click()
    ticket=page.worker.commands[-1][1]
    panel.log_bottom.setPlainText('停止')
    panel.send_btn.click()
    assert page.worker.commands[-1]==('cancel_relative',ticket)
    page.worker.relative_action_updated.emit(ticket,'accepted','late acceptance')
    page.worker.relative_action_updated.emit(ticket,'finished','已取消')
    sdk.start.assert_not_called()
    page._close_navigation.assert_not_called()
    assert not window._relative_session.active


def test_unsupported_mixed_search_command_is_explained_without_qwen_or_partial_motion(dashboard,monkeypatch):
    _,window=dashboard
    panel,sdk=setup_panel(window,monkeypatch)
    panel.log_bottom.setPlainText('现在转身，然后寻找黑衣服的人，走到他前一米')
    panel.send_btn.click()
    sdk.start.assert_not_called()
    assert '暂不支持' in panel.log_top.toPlainText()
    assert window._visual_session is None
