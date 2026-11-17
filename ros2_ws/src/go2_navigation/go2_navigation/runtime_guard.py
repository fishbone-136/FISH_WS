"""Keep one navigation server and one final velocity gate per ROS domain."""

import fcntl
import os
from pathlib import Path
import json
import subprocess

_locks = {}


def navigation_network_error(interface=None, target=None):
    """Read-only preflight before creating a DDS participant on the robot NIC."""
    interface = interface or os.environ.get('GO2_NETWORK_INTERFACE', 'eth0')
    target = target or os.environ.get('GO2_IP', '192.168.123.161')
    device = Path('/sys/class/net') / interface
    if not device.is_dir():
        return f'机器狗网卡 {interface} 不存在，请检查网卡连接和 GO2_NETWORK_INTERFACE 配置。'
    try:
        state = (device / 'operstate').read_text().strip()
        if state in ('down', 'lowerlayerdown', 'notpresent'):
            return (f'机器狗网口 {interface} 未连接，请检查机器狗供电和网线；'
                    '连接恢复后点击启动 / 重试，无需重启 UI。')
        addresses = json.loads(subprocess.check_output(
            ['ip', '-j', '-4', 'addr', 'show', 'dev', interface],
            text=True, stderr=subprocess.DEVNULL, timeout=1))
        if not any(a.get('family') == 'inet' for d in addresses for a in d.get('addr_info', [])):
            return f'机器狗网卡 {interface} 没有 IPv4 地址，请启用机器狗的有线网络配置后重试。'
        routes = json.loads(subprocess.check_output(
            ['ip', '-j', 'route', 'get', target], text=True,
            stderr=subprocess.DEVNULL, timeout=1))
        if not routes or routes[0].get('dev') != interface:
            return f'机器狗 {target} 未通过 {interface} 路由，请检查有线网络的静态地址和路由后重试。'
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        return f'无法检查机器狗网络 {interface} → {target}：{error}'
    return ''


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
