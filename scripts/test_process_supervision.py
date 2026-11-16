"""Regression checks for the old reload-process orphan failure; no ROS or robot."""

import os
from pathlib import Path
import subprocess
import sys
import time
import unittest

from process_supervisor import ManagedChildren, process_identity


class ProcessSupervisionTests(unittest.TestCase):
    def test_launcher_exit_does_not_leave_a_native_grandchild(self):
        owner = subprocess.Popen(['/bin/sleep', '30'])
        managed = ManagedChildren(owner.pid)
        launcher = managed.start(
            [sys.executable, '-c',
             "import subprocess; p=subprocess.Popen(['/bin/sleep','30']);print(p.pid,flush=True)"],
            stdout=subprocess.PIPE, text=True)
        native_pid = int(launcher.stdout.readline())
        try:
            launcher.wait(timeout=3)
            managed.close()
            self.assertIsNone(process_identity(native_pid))
        finally:
            launcher.stdout.close()
            owner.terminate()
            owner.wait()
            if process_identity(native_pid) is not None:
                os.kill(native_pid, 9)

    def test_owner_exit_is_detected_and_children_are_stopped(self):
        owner = subprocess.Popen(['/bin/sleep', '.2'])
        managed = ManagedChildren(owner.pid)
        child = managed.start(['/bin/sleep', '30'])
        try:
            owner.wait(timeout=2)
            self.assertFalse(managed.owner_alive())
        finally:
            managed.close()
        self.assertIsNotNone(child.poll())

    def test_supervisor_crash_also_stops_its_child(self):
        code = ("from process_supervisor import ManagedChildren;import os,time;"
                "m=ManagedChildren(os.getpid());p=m.start(['/bin/sleep','30']);"
                "print(p.pid,flush=True);time.sleep(30)")
        supervisor = subprocess.Popen(
            [sys.executable, '-c', code], cwd=Path(__file__).parent,
            stdout=subprocess.PIPE, text=True)
        child_pid = int(supervisor.stdout.readline())
        try:
            supervisor.kill()
            supervisor.wait(timeout=3)
            deadline = time.monotonic()+3
            while process_identity(child_pid) is not None and time.monotonic()<deadline:
                time.sleep(.02)
            self.assertIsNone(process_identity(child_pid))
        finally:
            supervisor.stdout.close()
            if process_identity(child_pid) is not None:
                os.kill(child_pid, 9)


if __name__ == '__main__':
    unittest.main()
