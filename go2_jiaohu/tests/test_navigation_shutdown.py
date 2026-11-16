"""Real local child-process shutdown; no ROS, devices or robot commands."""
import os
import signal
import subprocess
import sys

from go2_navigation import runtime_guard
from go2_ui.navigation import stack_stopper


def test_shutdown_closes_launcher_descendants_and_orphan_base_only_in_its_domain(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '217')
    # Descendant has its own session and no recognizable navigation name.
    code = ("import subprocess,time; "
            "p=subprocess.Popen(['/bin/sleep','30'],start_new_session=True); "
            "print(p.pid,flush=True); time.sleep(30)")
    launcher = subprocess.Popen([sys.executable, '-c', code, 'go2_lio_navigation.launch.py'],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    child_pid = int(launcher.stdout.readline())
    base = subprocess.Popen(['go2_base', '30'], executable='/bin/sleep')
    other_env = dict(os.environ, ROS_DOMAIN_ID='218')
    other_base = subprocess.Popen(['go2_base', '30'], executable='/bin/sleep', env=other_env)
    unrelated = subprocess.Popen(['/bin/sleep', '30'])
    try:
        assert launcher.pid in runtime_guard.conflicting_navigation_processes(include_launch=True)
        assert base.pid in runtime_guard.conflicting_navigation_processes()
        assert other_base.pid not in runtime_guard.conflicting_navigation_processes()
        assert stack_stopper.stop_navigation()
        assert launcher.wait(timeout=2) != 0
        assert base.wait(timeout=2) == -signal.SIGINT
        assert child_pid not in stack_stopper.process_snapshot()
        assert other_base.poll() is None
        assert unrelated.poll() is None
        assert runtime_guard.conflicting_navigation_processes(include_launch=True) == []
        assert stack_stopper.stop_navigation()  # Repeated close is harmless.
    finally:
        for process in (launcher, base, other_base, unrelated):
            if process.poll() is None:
                process.kill()
            process.wait(timeout=2)
        if child_pid in stack_stopper.process_snapshot():
            os.kill(child_pid, signal.SIGKILL)
        launcher.stdout.close()


def test_failed_shutdown_never_reports_success(monkeypatch, capsys):
    monkeypatch.setattr(stack_stopper, 'process_snapshot', lambda: {900001: (1, '123')})
    monkeypatch.setattr(stack_stopper, 'conflicting_navigation_processes', lambda **kw: [900001])
    monkeypatch.setattr(stack_stopper.Path, 'read_text', lambda self: '900001 (base) S 1 ' + '0 ' * 17 + '123')
    now = iter(range(0, 1000, 10))
    monkeypatch.setattr(stack_stopper.time, 'monotonic', lambda: next(now))
    monkeypatch.setattr(stack_stopper.time, 'sleep', lambda _: None)
    signals = []
    monkeypatch.setattr(stack_stopper.os, 'kill', lambda pid, sig: signals.append((pid, sig)))
    assert not stack_stopper.stop_navigation()
    assert [sig for _, sig in signals] == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    assert '仍有控制进程' in capsys.readouterr().out
