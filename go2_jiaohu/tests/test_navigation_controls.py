"""Click the real controls without launching processes or sending robot goals."""
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from tests.test_navigation_autostart import navigation
from tests.test_navigation_autostart import LaunchProcess
from tests.test_dashboard_interaction import dashboard
from tests.test_current_navigation_ui import DisplayWorker


def test_external_navigation_is_fully_closed_before_interactive_control(navigation, monkeypatch):
    n = navigation
    from PyQt5.QtCore import QProcess
    n.page._close_process = closer = LaunchProcess()
    n.existing.append(999)
    n.page.worker.status_updated.emit('ready', '就绪')
    assert not n.page.start_button.isEnabled()
    assert n.page.stop_button.isEnabled()
    QTest.mouseClick(n.page.stop_button, Qt.LeftButton)
    QTest.mouseClick(n.page.stop_button, Qt.LeftButton)
    assert n.page.worker.commands == [('emergency_stop',)]
    assert len(closer.starts) == 1
    assert closer.starts[0][1][0].endswith('/stack_stopper.py')
    assert not n.window._motion_start_allowed()
    assert n.launch.starts == []
    assert not n.page.send_goal_button.isEnabled()
    # Stale ROS callbacks must not unlock controls while processes are closing.
    n.page.worker.status_updated.emit('ready', '就绪')
    n.page.worker.capabilities_updated.emit({'online': False})
    assert not n.page.start_button.isEnabled()
    assert not n.page.send_goal_button.isEnabled()
    assert n.page._control_pending == 'close'
    assert not n.window._motion_start_allowed()
    n.existing.clear()
    # Even after the last process exits, wait for the shutdown acknowledgement.
    assert not n.window._motion_start_allowed()
    closer.running = False
    closer.output = '导航已完全关闭，可以使用语音 / 手势控制'.encode()
    n.page._on_close_finished(0, QProcess.NormalExit)
    assert n.page.start_button.isEnabled()
    assert not n.page.stop_button.isEnabled()
    assert n.window._motion_start_allowed()
    n.page.worker.status_updated.emit('ready', '旧状态')
    n.page.worker.capabilities_updated.emit({'can_control': True, 'goal_active': True})
    assert n.page._navigation_state == 'closed'
    assert not n.page.send_goal_button.isEnabled()
    n.window.switch_page(0)
    n.window.switch_page(1)
    n.page._autostart()
    assert n.launch.starts == []
    assert '已完全关闭' in n.page.status_label.text()
    # Exercise the actual deploy-mode gesture button, with a fake robot controller.
    from types import SimpleNamespace
    from go2_ui.ui import main_control_panel as controls
    monkeypatch.setattr(controls, '_DEPLOY_MODE', True)
    panel = n.window.main_panel
    panel.hand_detector = SimpleNamespace(available=True)
    gesture = SimpleNamespace(reset=lambda: None, stop_motion=lambda: None)
    monkeypatch.setattr(controls, 'GestureController', lambda: gesture)
    n.window.switch_page(0)
    QTest.mouseClick(panel.gesture_btn, Qt.LeftButton)
    assert panel._mode_manager.active == '手势控制'
    assert gesture.is_system_active
    panel.stop_btn.click()


def test_close_failure_keeps_interlock_and_can_be_retried(navigation):
    n = navigation
    from PyQt5.QtCore import QProcess
    n.page._close_process = closer = LaunchProcess()
    n.existing.append(999)
    n.page.worker.status_updated.emit('fault', '定位故障')
    assert n.page.stop_button.isEnabled()
    n.page.stop_button.click()
    closer.running = False
    closer.output = '仍有底盘进程，请重试关闭'.encode()
    n.page._on_close_finished(1, QProcess.NormalExit)
    assert not n.window._motion_start_allowed()
    assert n.page.stop_button.isEnabled()
    assert not n.page._navigation_closed
    n.existing.clear()
    assert not n.window._motion_start_allowed()  # A failed helper cannot release ownership.
    n.page.stop_button.click()
    assert len(closer.starts) == 2
    # Complete the fake helper before the fixture closes the window.
    closer.running = False
    n.page._on_close_finished(1, QProcess.NormalExit)


def test_close_during_startup_and_explicit_restart(navigation, monkeypatch):
    from PyQt5.QtCore import QProcess
    from PyQt5.QtTest import QTest
    from tests.test_current_navigation_ui import DisplayWorker
    n = navigation
    QTest.qWait(30)
    assert len(n.launch.starts) == 1
    n.page._close_process = closer = LaunchProcess()
    n.page.stop_button.click()
    n.launch.running = False
    n.page._on_process_finished(0, QProcess.NormalExit)
    assert n.page._control_pending == 'close'
    assert not n.page.start_button.isEnabled()
    closer.running = False
    closer.output = '导航已完全关闭'.encode()
    n.page._on_close_finished(0, QProcess.NormalExit)
    old_worker = n.page.worker
    old_worker.isRunning = lambda: False
    monkeypatch.setattr(n.page, '_wire_ros', lambda: setattr(n.page, 'worker', DisplayWorker(n.page)))
    n.page.start_button.click()
    assert len(n.launch.starts) == 2
    assert n.page.worker is not old_worker
    assert not n.page._navigation_closed


def test_failed_control_and_save_requests_can_be_retried(navigation):
    p = navigation.page
    p.worker.status_updated.emit('paused', '已停止')
    p.start_button.click()
    p.worker.control_finished.emit('resume', False, '服务超时')
    assert p.start_button.isEnabled()
    p.worker.capabilities_updated.emit({'paused': True, 'can_control': True})
    p.worker.status_updated.emit('error', '恢复服务超时')
    p.start_button.click()
    assert p.worker.commands == [('resume',), ('resume',)]
    p.worker.capabilities_updated.emit({'can_save': True})
    p.save_map_button.click()
    p.save_map_button.click()
    assert p.worker.commands.count(('save_environment',)) == 1
    assert not p.save_map_button.isEnabled()
    p.worker.map_saved.emit(False, '保存服务超时')
    assert p.save_map_button.isEnabled()


def test_goal_buttons_follow_readiness_and_rviz_goal_activity(navigation):
    p = navigation.page
    assert not p.send_goal_button.isEnabled()
    p.worker.status_updated.emit('ready', '就绪')
    p.send_goal_button.click()
    p.send_goal_button.click()
    assert len(p.worker.commands) == 1
    assert p.cancel_goal_button.isEnabled()
    p.cancel_goal_button.click()
    assert p.worker.commands[-1] == ('cancel',)
    p.worker.goal_finished.emit(False, '已取消')
    assert p.send_goal_button.isEnabled()
    p.worker.capabilities_updated.emit({'goal_active': True})
    assert not p.send_goal_button.isEnabled()
    assert p.cancel_goal_button.isEnabled()
    p.worker.capabilities_updated.emit({'goal_active': False})
    assert p.send_goal_button.isEnabled()
    p.worker.status_updated.emit('fault', '定位已停止')
    assert not p.send_goal_button.isEnabled()
    assert not p.start_button.isEnabled()
    assert p.emergency_stop_button.isEnabled()


def test_page_entry_never_automatically_resumes_an_operator_stop(navigation):
    p = navigation.page
    p.worker.status_updated.emit('paused', '已停止')
    p._autostart()
    assert p.worker.commands == []
    assert navigation.launch.starts == []


def test_retry_reconnects_a_stopped_bridge_without_duplicate_stack(navigation, monkeypatch):
    p = navigation.page
    navigation.existing.append(999)
    p.worker.isRunning = lambda: False
    calls = []
    def reconnect():
        p.worker = DisplayWorker(p)
        calls.append('reconnected')
    monkeypatch.setattr(p, '_wire_ros', reconnect)
    p._start_stack()
    assert calls == ['reconnected']
    assert navigation.launch.starts == []
