"""Recognize startup before Nav2's composed container is created."""
from pathlib import Path

import pytest
from go2_navigation import runtime_guard


@pytest.mark.parametrize('command', [
    ['bash', '/workspace/scripts/run_lio_navigation.sh'],
    ['python3', '/workspace/scripts/check_navigation_stance.py'],
    ['python3', '/workspace/go2_ui/navigation/stack_launcher.py'],
    ['ros2', 'launch', 'go2_lio_navigation', 'go2_lio_navigation.launch.py'],
])
def test_discovery_recognizes_launch_stages_in_current_domain(tmp_path, monkeypatch, command):
    def process(pid, domain, state='S'):
        proc = tmp_path / str(pid)
        proc.mkdir()
        (proc/'stat').write_text(f'{pid} (test) {state} 1')
        (proc/'cmdline').write_bytes(b'\0'.join(arg.encode() for arg in command))
        (proc/'environ').write_text(f'ROS_DOMAIN_ID={domain}\0')
    process(900001, '86')
    process(900002, '87')
    process(900003, '86', 'Z')
    monkeypatch.setenv('ROS_DOMAIN_ID', '86')
    monkeypatch.setattr(runtime_guard, 'Path', lambda name: tmp_path if name == '/proc' else Path(name))
    assert runtime_guard.conflicting_navigation_processes() == []
    assert runtime_guard.conflicting_navigation_processes(include_launch=True) == [900001]
