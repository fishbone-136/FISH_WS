"""Network failures must be diagnosed before DDS or robot processes start."""
import json
from unittest.mock import Mock

import pytest
from PyQt5.QtTest import QTest

from go2_navigation import runtime_guard
from go2_ui.navigation import ros_bridge
from tests.test_navigation_autostart import navigation
from tests.test_dashboard_interaction import dashboard


@pytest.fixture
def network(tmp_path, monkeypatch):
    device = tmp_path / 'eno1'
    device.mkdir()
    (device / 'operstate').write_text('up')
    monkeypatch.setattr(runtime_guard, 'Path', lambda _: tmp_path)
    output = Mock(side_effect=[
        json.dumps([{'addr_info': [{'family': 'inet', 'local': '192.168.123.10'}]}]),
        json.dumps([{'dev': 'eno1'}]),
    ])
    monkeypatch.setattr(runtime_guard.subprocess, 'check_output', output)
    return device, output


def test_missing_interface(network):
    assert '不存在' in runtime_guard.navigation_network_error('missing')
    network[1].assert_not_called()


def test_disconnected_cable_is_actionable_without_ros(network):
    device, output = network
    (device / 'operstate').write_text('down')
    message = runtime_guard.navigation_network_error('eno1')
    assert '网线' in message and '无需重启 UI' in message
    output.assert_not_called()


def test_connected_interface_requires_ipv4(network):
    network[1].side_effect = ['[{"addr_info": []}]']
    assert 'IPv4' in runtime_guard.navigation_network_error('eno1')


def test_wifi_route_cannot_substitute_for_robot_network(network):
    network[1].side_effect = ['[{"addr_info": [{"family": "inet"}]}]',
                              '[{"dev": "wlan0"}]']
    assert '路由' in runtime_guard.navigation_network_error('eno1')


def test_connected_correct_route_passes(network):
    assert runtime_guard.navigation_network_error('eno1') == ''


def test_network_probe_failure_is_visible(network):
    network[1].side_effect = OSError('ip unavailable')
    assert '无法检查' in runtime_guard.navigation_network_error('eno1')


def test_retry_after_cable_reconnected_recreates_failed_bridge(navigation, monkeypatch):
    n = navigation
    monkeypatch.setattr(runtime_guard, 'navigation_network_error', lambda: '机器狗网口 eno1 未连接，请检查网线')
    QTest.qWait(30)
    assert n.launch.starts == []
    assert '网线' in n.page.status_label.text()
    assert not n.page._ready_timer.isActive()
    old_worker = n.page.worker
    monkeypatch.setattr(old_worker, 'isRunning', lambda: False, raising=False)
    monkeypatch.setattr(runtime_guard, 'navigation_network_error', lambda: '')
    n.page.start_button.click()
    assert n.page.worker is not old_worker
    assert len(n.launch.starts) == 1
    assert n.page._startup_error == ''
    n.page.worker.status_updated.emit('ready', '连接已恢复')
    assert '连接已恢复' in n.page.status_label.text()


def test_bridge_does_not_create_dds_on_disconnected_network(monkeypatch):
    monkeypatch.setattr(runtime_guard, 'navigation_network_error', lambda: '网口未连接')
    monkeypatch.setattr(ros_bridge, 'ROS_AVAILABLE', True)
    init = Mock()
    monkeypatch.setattr(ros_bridge.rclpy, 'init', init)
    worker = ros_bridge.RosNavigationWorker()
    statuses = []
    worker.status_updated.connect(lambda *args: statuses.append(args))
    worker.run()
    init.assert_not_called()
    assert statuses == [('error', '网口未连接')]
