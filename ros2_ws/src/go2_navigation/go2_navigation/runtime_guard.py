"""Keep one navigation server and one final velocity gate per ROS domain."""

import fcntl
import os
from pathlib import Path

_locks = {}


def acquire_process_lock(role):
    if role in _locks:
        return
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    path = Path(f'/tmp/go2_{role}_{os.getuid()}_{domain}.lock')
    handle = path.open('a+')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        handle.close()
        raise RuntimeError(f'Another Go2 {role} is running in ROS domain {domain}') from error
    handle.seek(0)
    handle.truncate()
    handle.write(str(os.getpid())+'\n')
    handle.flush()
    _locks[role] = handle


def conflicting_navigation_processes(include_gate=True, include_launch=False):
    """Also recognize pre-lock/legacy processes; never kill one implicitly."""
    found = []
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    for directory in Path('/proc').glob('[0-9]*'):
        try:
            if directory.stat().st_uid != os.getuid() or int(directory.name) == os.getpid():
                continue
            if (directory/'stat').read_text().rsplit(')', 1)[1].split()[0] == 'Z':
                continue
            args = [v.decode(errors='replace') for v in
                    (directory/'cmdline').read_bytes().split(b'\0') if v]
            names = [os.path.basename(v) for v in args]
            launching = include_launch and any(name in names for name in (
                'run_lio_navigation.sh', 'go2_lio_navigation.launch.py',
                'check_navigation_stance.py', 'stack_launcher.py'))
            # A paused/orphaned base or stand guard still owns sport commands.
            if not launching and not (include_gate and any(name in names for name in (
                    'scan_safety_gate.py', 'go2_base', 'idle_stand_guard.py'))) and (
                    '__node:=nav2_container' not in args):
                continue
            env = dict(v.split('=', 1) for v in
                       (directory/'environ').read_text().split('\0') if '=' in v)
            if env.get('ROS_DOMAIN_ID', '0') == domain:
                found.append(int(directory.name))
        except (OSError, ValueError):
            continue
    return found


def acquire_navigation_lock():
    """Allow an identified reload under the existing full launch's lock.

    A composed Nav2 included in the LIO launch acquires its lock in the main
    launch process, not in the component container. Keep that primary lock and
    require a separate exclusive reload lock for its supervised replacement.
    """
    conflicts = conflicting_navigation_processes(include_gate=False)
    if conflicts:
        raise RuntimeError(f'Existing navigation processes {conflicts}; refuse duplicate Nav2')
    try:
        acquire_process_lock('nav2')
        return
    except RuntimeError:
        owner = os.environ.get('GO2_NAV2_RELOAD_OWNER', '')
        try:
            pid, start = owner.split(':')
            proc = Path('/proc')/str(int(pid))
            fields = (proc/'stat').read_text().rsplit(')', 1)[1].split()
            args = (proc/'cmdline').read_bytes().split(b'\0')
            env = dict(v.split('=', 1) for v in
                       (proc/'environ').read_text().split('\0') if '=' in v)
            domain = os.environ.get('ROS_DOMAIN_ID', '0')
            primary = Path(f'/tmp/go2_nav2_{os.getuid()}_{int(domain)}.lock')
            valid = (proc.stat().st_uid == os.getuid() and fields[0] != 'Z' and
                     fields[19] == start and b'go2_lio_navigation.launch.py' in args and
                     env.get('ROS_DOMAIN_ID', '0') == domain and primary.read_text().strip() == pid)
        except (OSError, ValueError, IndexError):
            valid = False
        if not valid:
            raise RuntimeError('Nav2 lock is owned by another session; reload identity rejected')
    acquire_process_lock('nav2_reload')
