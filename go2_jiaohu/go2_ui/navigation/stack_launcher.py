"""Own only the LIO stack explicitly started by the UI."""
import os
from pathlib import Path
import signal
import sys
import time


def main():
    root = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(root / 'scripts'))
    from process_supervisor import ManagedChildren, _parent_death_signal

    stopping = False
    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    parent = os.getppid()
    _parent_death_signal(parent)
    managed = ManagedChildren(parent)
    try:
        child = managed.start(['bash', str(root/'scripts/run_lio_navigation.sh'),
                               'lidar_source:=unitree', 'use_rviz:=false'], cwd=str(root))
        while not stopping and managed.owner_alive() and child.poll() is None:
            time.sleep(.1)
        return child.returncode if child.poll() is not None else 0
    finally:
        managed.close()


if __name__ == '__main__':
    sys.exit(main())
