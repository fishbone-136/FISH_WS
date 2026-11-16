"""Explicit operator shutdown of local navigation in the current ROS domain.

Use individual PID identities, never a terminal's process group. Launchers,
their descendants (including reload supervisors), and orphaned motion nodes
must all exit before the UI releases interactive control.
"""
import os
from pathlib import Path
import signal
import sys
import time

from go2_navigation.runtime_guard import conflicting_navigation_processes


def process_snapshot():
    processes = {}
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    for directory in Path('/proc').glob('[0-9]*'):
        try:
            pid = int(directory.name)
            if pid == os.getpid() or directory.stat().st_uid != os.getuid():
                continue
            fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                continue
            env = dict(v.split('=', 1) for v in
                       (directory / 'environ').read_text().split('\0') if '=' in v)
            if env.get('ROS_DOMAIN_ID', '0') == domain:
                processes[pid] = (int(fields[1]), fields[19])
        except (OSError, ValueError, IndexError):
            continue
    return processes


def stop_navigation():
    tracked = {}
    signalled = {}

    def remaining():
        snapshot = process_snapshot()
        roots = set(conflicting_navigation_processes(include_launch=True))
        roots.update(pid for pid, identity in tracked.items()
                     if pid in snapshot and snapshot[pid][1] == identity)
        while True:
            children = {pid for pid, (parent, _) in snapshot.items() if parent in roots}
            if children <= roots:
                break
            roots.update(children)
        for pid in roots:
            if pid in snapshot:
                tracked[pid] = snapshot[pid][1]
        return {pid: identity for pid, identity in tracked.items()
                if pid in snapshot and snapshot[pid][1] == identity}

    # SIGINT lets ROS and go2_base publish their final StopMove and clean up.
    for sig, timeout in ((signal.SIGINT, 12.), (signal.SIGTERM, 4.), (signal.SIGKILL, 2.)):
        deadline = time.monotonic() + timeout
        while True:
            alive = remaining()
            if not alive:
                print('导航已完全关闭，可以使用语音 / 手势控制', flush=True)
                return True
            for pid, identity in alive.items():
                if signalled.get((pid, identity)) == sig:
                    continue
                # Recheck identity immediately before signalling (PID reuse).
                try:
                    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
                except (OSError, IndexError):
                    continue
                if fields[0] == 'Z' or fields[19] != identity:
                    continue
                try:
                    os.kill(pid, sig)
                    signalled[(pid, identity)] = sig
                except ProcessLookupError:
                    pass
            if time.monotonic() >= deadline:
                break
            time.sleep(.1)
    alive = remaining()
    if not alive:
        print('导航已完全关闭，可以使用语音 / 手势控制', flush=True)
        return True
    print(f'导航关闭失败，仍有控制进程 {sorted(alive)}；请重试关闭', flush=True)
    return False


if __name__ == '__main__':
    try:
        sys.exit(0 if stop_navigation() else 1)
    except Exception as error:
        print(f'导航关闭失败：{error}', flush=True)
        sys.exit(1)
