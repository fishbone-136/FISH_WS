"""Exercise automatic navigation startup without starting ROS or robot nodes."""
from types import SimpleNamespace

import pytest
from PyQt5.QtCore import QProcess
from PyQt5.QtTest import QTest

from go2_ui.navigation import navigation_page as page_module
from go2_navigation import runtime_guard
from tests.test_current_navigation_ui import DisplayWorker
from tests.test_dashboard_interaction import dashboard


class LaunchProcess:
    def __init__(self):
        self.running = False
        self.starts = []
        self.output = b''

    def state(self):
        return QProcess.Running if self.running else QProcess.NotRunning

    def processId(self):
        return 12345 if self.running else 0

    def setProcessEnvironment(self, env):
        self.environment = env

    def start(self, executable, arguments):
        self.starts.append((executable, arguments))
        self.running = True

    def terminate(self):
        self.running = False

    kill = terminate

    def waitForFinished(self, _timeout):
        return True

    def readAllStandardOutput(self):
        output, self.output = self.output, b''
        return output

    def errorString(self):
        return 'test launch failure'


@pytest.fixture
def navigation(dashboard, monkeypatch):
    app, window = dashboard
    monkeypatch.setattr(page_module, 'RosNavigationWorker', DisplayWorker)
    monkeypatch.setattr(page_module.RvizView, 'start', lambda self: None)
    monkeypatch.setattr(page_module.RvizView, 'shutdown', lambda self: None)
    existing = []
    monkeypatch.setattr(runtime_guard, 'navigation_network_error', lambda: '')
    monkeypatch.setattr(runtime_guard, 'conflicting_navigation_processes', lambda **kw: existing)
    window.switch_page(1)
    page = window.navigation_page
    launch = LaunchProcess()
    page._launch_process = launch
    page._autostart_timer.setInterval(10)
    # activate() has already scheduled the initial startup; shorten its delay.
    page._autostart_timer.start()
    return SimpleNamespace(app=app, window=window, page=page, launch=launch, existing=existing)


def test_first_entry_starts_once_and_waits_for_actual_readiness(navigation):
    n = navigation
    QTest.qWait(30)
    assert len(n.launch.starts) == 1
    assert n.launch.starts[0][1][0].endswith('/stack_launcher.py')
    assert n.page._ready_timer.isActive()
    n.page._start_stack()
    n.window.switch_page(0)
    n.window.switch_page(1)
    QTest.qWait(30)
    assert len(n.launch.starts) == 1
    n.page.worker.status_updated.emit('ready', 'Nav2、地图、位姿均正常')
    assert not n.page._ready_timer.isActive()
    assert '均正常' in n.page.status_label.text()
    assert n.page.worker.commands == []


def test_existing_or_initializing_stack_is_connected_without_launch(navigation):
    n = navigation
    n.existing.append(1234)
    QTest.qWait(30)
    assert n.launch.starts == []
    assert n.page._ready_timer.isActive()
    assert '已接入' in n.page.status_label.text()


def test_remote_ready_stack_is_not_started_locally(navigation):
    n = navigation
    n.page.worker.status_updated.emit('ready', 'Nav2、地图、位姿均正常')
    QTest.qWait(30)
    assert n.launch.starts == []
    assert not n.page._ready_timer.isActive()


def test_leaving_page_before_start_cancels_pending_autostart(navigation):
    n = navigation
    n.window.switch_page(0)
    QTest.qWait(30)
    assert n.launch.starts == []
    n.window.switch_page(1)
    QTest.qWait(30)
    assert len(n.launch.starts) == 1


def test_manual_stop_stays_stopped_until_explicit_retry(navigation):
    n = navigation
    QTest.qWait(30)
    n.page._stop_stack()
    n.page._on_process_finished(0, QProcess.NormalExit)
    n.window.switch_page(0)
    n.window.switch_page(1)
    QTest.qWait(30)
    assert len(n.launch.starts) == 1
    n.page.start_button.click()
    assert len(n.launch.starts) == 2
    assert n.page.worker.commands == []


def test_connect_only_mode_never_launches(navigation):
    n = navigation
    n.page.mode_combo.setCurrentIndex(1)
    QTest.qWait(30)
    n.page.start_button.click()
    assert n.launch.starts == []
    assert not n.page._ready_timer.isActive()


def test_preflight_failure_is_visible_and_manual_retry_is_possible(navigation):
    n = navigation
    QTest.qWait(30)
    n.launch.running = False
    n.launch.output = '导航未启动：机器狗尚未站稳。\n'.encode()
    n.page._on_process_finished(2, QProcess.NormalExit)
    n.page.worker.status_updated.emit('waiting', '等待: Nav2 / 地图')
    assert '尚未站稳' in n.page.status_label.text()
    assert not n.page._ready_timer.isActive()
    QTest.qWait(30)
    assert len(n.launch.starts) == 1  # No endless restarts after a failure.
    n.page.start_button.click()
    assert len(n.launch.starts) == 2
    assert n.page._startup_error == ''


def test_timeout_and_process_error_remain_visible(navigation):
    n = navigation
    QTest.qWait(30)
    n.page.worker.status_updated.emit('waiting', '等待: 地图')
    n.page._ready_timer.setInterval(10)
    n.page._ready_timer.start()
    QTest.qWait(30)
    assert '导航尚未就绪' in n.page.status_label.text()
    n.page.start_button.click()
    assert n.page._ready_timer.isActive()
    assert len(n.launch.starts) == 1  # Retry checks the running stack in place.
    n.page._on_launch_error(QProcess.FailedToStart)
    n.page.worker.status_updated.emit('waiting', '等待: Nav2')
    assert 'test launch failure' in n.page.status_label.text()
    assert n.page.worker.commands == []
