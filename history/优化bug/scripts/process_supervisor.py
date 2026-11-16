"""Own runtime-reload children and stop them when their navigation owner exits."""

import ctypes
import os
from pathlib import Path
import signal
import subprocess
import time


def process_identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] != 'Z' else None
    except (OSError, IndexError):
        return None


def _parent_death_signal(parent):
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGINT, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        os._exit(126)
    if os.getppid() != parent:
        os._exit(125)


class ManagedChildren:
    def __init__(self, owner_pid):
        self.owner_pid = owner_pid
        self.owner_identity = process_identity(owner_pid)
        if self.owner_identity is None:
            raise RuntimeError('Navigation owner is not alive')
        self.children = []

    def owner_alive(self):
        return process_identity(self.owner_pid) == self.owner_identity

    def start(self, args, **kwargs):
        parent = os.getpid()
        child = subprocess.Popen(
            args, start_new_session=True,
            preexec_fn=lambda: _parent_death_signal(parent), **kwargs)
        self.children.append(child)
        return child

    def close(self):
        def signal_groups(sig):
            for child in self.children:
                try:
                    os.killpg(child.pid, sig)
                except ProcessLookupError:
                    pass

        for child in reversed(self.children):
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic()+10
        for child in self.children:
            if child.poll() is None:
                try:
                    child.wait(timeout=max(.1, deadline-time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
        # A launcher may exit before a native component. Signal the owned
        # session group too, rather than leaving that grandchild orphaned.
        signal_groups(signal.SIGTERM)
        deadline = time.monotonic()+3
        while time.monotonic()<deadline:
            alive = False
            for child in self.children:
                try:
                    os.killpg(child.pid, 0)
                    alive = True
                except ProcessLookupError:
                    pass
            if not alive:
                break
            time.sleep(.05)
        signal_groups(signal.SIGKILL)
        for child in self.children:
            if child.poll() is None:
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    child.kill()
