from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest

from tests.test_dashboard_interaction import dashboard
from tests.test_visual_navigation_ui import NavigationWorker
from voice.visual_navigation import ApproachIntent


@pytest.mark.parametrize('mode, fallback', [('auto',True),('sdk',False)])
def test_person_text_routes_to_near_control_without_starting_nav(dashboard, monkeypatch, mode, fallback):
    _, window = dashboard
    panel = window.main_panel
    from go2_ui.ui import main_control_panel as controls
    from go2_ui.navigation import near_session
    monkeypatch.setattr(controls,'_DEPLOY_MODE',True)
    monkeypatch.setattr(panel,'_vlm_analyze_scene',lambda:True)
    monkeypatch.setattr(window,'_motion_start_allowed',lambda:True)
    near = SimpleNamespace(active=False,request=Mock(),cancel=Mock())
    monkeypatch.setattr(near_session,'NearApproachSession',lambda *args:near)
    picker = window.right_panel.analysis_widget.motion_mode
    picker.setCurrentIndex(picker.findData(mode))
    QTest.mouseClick(panel.vlm_btn,Qt.LeftButton)
    panel.log_bottom.setPlainText('走到人前一米')
    panel.send_btn.click()
    near.request.assert_called_once_with(ApproachIntent('人'),allow_fallback=fallback)
    assert window._visual_session is None
    assert not hasattr(window,'navigation_page')


def test_nav_picker_uses_ready_stack_and_never_constructs_sdk_session(dashboard, monkeypatch):
    _, window = dashboard
    panel = window.main_panel
    from go2_ui.ui import main_control_panel as controls
    monkeypatch.setattr(controls,'_DEPLOY_MODE',True)
    monkeypatch.setattr(panel,'_vlm_analyze_scene',lambda:True)
    monkeypatch.setattr(window.right_panel.depth_widget,'get_rgbd_snapshot',lambda:object())
    worker = NavigationWorker()
    page = SimpleNamespace(worker=worker,_navigation_state='ready',_last_navigation_message='',
        _navigation_closed=False,_control_pending='',_close_failed=False,_start_stack=Mock(),
        _capabilities={'camera_calibrated':True})
    monkeypatch.setattr(window,'_get_navigation_page',lambda:page)
    picker = window.right_panel.analysis_widget.motion_mode
    picker.setCurrentIndex(picker.findData('nav'))
    QTest.mouseClick(panel.vlm_btn,Qt.LeftButton)
    panel.log_bottom.setPlainText('走到人前一米')
    panel.send_btn.click()
    assert worker.commands[-1][0] == 'visual_preflight'
    assert window._near_session is None
    page._start_stack.assert_not_called()
