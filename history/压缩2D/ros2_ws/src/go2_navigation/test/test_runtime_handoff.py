"""Keep duplicate locks intact while handing an idle full session to a reload."""
import os
from pathlib import Path
import subprocess
import sys
import unittest


class RuntimeHandoffTests(unittest.TestCase):
    def setUp(self):
        self.children = []
        self.env = os.environ.copy()
        self.env['ROS_DOMAIN_ID'] = '86'
        self.env['PYTHONPATH'] = str(Path(__file__).resolve().parents[1])
        self.env.pop('GO2_NAV2_RELOAD_OWNER', None)
        code = ('from go2_navigation.runtime_guard import acquire_process_lock; '
                'import os,sys; acquire_process_lock("nav2"); '
                'print(os.getpid(),flush=True); sys.stdin.read()')
        owner = self.spawn(code, 'go2_lio_navigation.launch.py')
        self.assertEqual(owner.stdout.readline().strip(), str(owner.pid))
        fields = Path(f'/proc/{owner.pid}/stat').read_text().rsplit(')', 1)[1].split()
        self.token = f'{owner.pid}:{fields[19]}'

    def spawn(self, code, *args, env=None):
        p = subprocess.Popen([sys.executable, '-c', code, *args], env=env or self.env,
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True)
        self.children.append(p)
        return p

    def probe(self, token=None):
        env = self.env.copy()
        if token:
            env['GO2_NAV2_RELOAD_OWNER'] = token
        p = self.spawn('from go2_navigation.runtime_guard import acquire_navigation_lock; '
                       'acquire_navigation_lock(); print("acquired",flush=True)', env=env)
        out, err = p.communicate(timeout=3)
        return p.returncode, out, err

    def tearDown(self):
        for p in reversed(self.children):
            if p.poll() is None:
                p.terminate()
                p.wait(timeout=3)
            for stream in (p.stdin, p.stdout, p.stderr):
                if stream and not stream.closed:
                    stream.close()

    def test_ordinary_duplicate_is_rejected(self):
        self.assertNotEqual(self.probe()[0], 0)

    def test_identified_same_session_reload_is_allowed(self):
        self.assertEqual(self.probe(self.token)[0], 0)

    def test_reused_pid_or_wrong_start_identity_is_rejected(self):
        self.assertNotEqual(self.probe(self.token.split(':')[0]+':0')[0], 0)

    def test_two_replacements_cannot_hold_the_reload_lock(self):
        env = dict(self.env, GO2_NAV2_RELOAD_OWNER=self.token)
        p = self.spawn('from go2_navigation.runtime_guard import acquire_navigation_lock; '
                       'import sys; acquire_navigation_lock(); '
                       'print("acquired",flush=True); sys.stdin.read()', env=env)
        self.assertEqual(p.stdout.readline().strip(), 'acquired')
        self.assertNotEqual(self.probe(self.token)[0], 0)

    def test_live_legacy_container_still_blocks_a_valid_reload_owner(self):
        p = self.spawn('import sys; print("ready",flush=True); sys.stdin.read()',
                       '__node:=nav2_container')
        self.assertEqual(p.stdout.readline().strip(), 'ready')
        self.assertNotEqual(self.probe(self.token)[0], 0)


if __name__ == '__main__':
    unittest.main()
