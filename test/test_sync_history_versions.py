"""Check snapshot fidelity, exclusions, repeatability and update safety."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest


spec = importlib.util.spec_from_file_location(
    'sync_history', Path(__file__).resolve().parents[1] / 'scripts/sync_history_versions.py')
sync_history = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sync_history)


class HistorySyncTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.sources = self.root / '历史版本'
        self.source = self.sources / '旧版本'
        self.destination = self.root / 'repo/history'
        self.write('go2_jiaohu/main.py', 'print("old version")\n')
        self.write('ros2_ws/src/pkg/config.yaml', 'value: 1\n')

    def write(self, name, data):
        path = self.source / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(data)
        return path

    def run_sync(self, dry_run=False):
        with contextlib.redirect_stdout(io.StringIO()):
            return sync_history.sync(self.sources, self.destination,
                                     b'models/\n.env\n', dry_run)

    def test_preserves_source_and_excludes_runtime_and_nested_git(self):
        for name in ['go2_jiaohu/.env', 'go2_jiaohu/voice/models/model.bin',
                     'go2_jiaohu/third_party/torch/lib/library.so',
                     'ros2_ws/src/pkg/yolo.pt', 'ros2_ws/src/fast_lio/.git/config',
                     'ros2_ws/src/pkg/.gitignore',
                     'ros2_ws/src/fast_lio/include/ikd-Tree/.git',
                     'ros2_ws/src/fast_lio/Log/mat_pre.txt',
                     'reports/old.json', '1.md']:
            self.write(name, 'excluded\n')
        self.write('ros2_ws/src/fast_lio/include/ikd-Tree/tree.h', 'int old_tree;\n')
        self.write('go2_jiaohu/.env.example', 'WHISPER_DEVICE=cpu\n')
        executable = self.write('scripts/run.sh', '#!/bin/sh\ntrue\n')
        executable.chmod(0o755)
        (self.source / 'go2_jiaohu/external.py').symlink_to(executable)
        self.run_sync()
        dest = self.destination / '旧版本'
        manifest = json.loads((dest / 'SOURCE_MANIFEST.json').read_text())
        names = set(manifest['files'])
        self.assertIn('go2_jiaohu/.env.example', names)
        self.assertIn('ros2_ws/src/fast_lio/include/ikd-Tree/tree.h', names)
        self.assertNotIn('go2_jiaohu/.env', names)
        self.assertNotIn('go2_jiaohu/external.py', names)
        self.assertEqual(len(names), 7)
        for name, metadata in manifest['files'].items():
            data = (dest / name).read_bytes()
            self.assertEqual(sync_history.digest(data), metadata['sha256'])
            if name not in {'.gitignore', 'SNAPSHOT.md'}:
                self.assertEqual(data, (self.source / name).read_bytes())
        self.assertTrue((dest / 'scripts/run.sh').stat().st_mode & 0o100)
        self.assertTrue((self.source / 'go2_jiaohu/.env').exists())
        before = (dest / 'SOURCE_MANIFEST.json').read_bytes()
        self.run_sync()
        self.assertEqual(before, (dest / 'SOURCE_MANIFEST.json').read_bytes())

    def test_updates_and_removes_only_previously_managed_files(self):
        old = self.write('scripts/old.py', 'old = True\n')
        self.run_sync()
        dest = self.destination / '旧版本'
        (dest / 'personal-note.txt').write_text('keep me')
        old.unlink()
        self.write('go2_jiaohu/main.py', 'print("updated")\n')
        self.run_sync()
        self.assertFalse((dest / 'scripts/old.py').exists())
        self.assertEqual((dest / 'personal-note.txt').read_text(), 'keep me')
        (dest / 'go2_jiaohu/main.py').write_text('local edit')
        with self.assertRaisesRegex(ValueError, '本地修改'):
            self.run_sync()
        self.assertEqual((dest / 'go2_jiaohu/main.py').read_text(), 'local edit')

    def test_dry_run_secrets_and_symlinks_do_not_write(self):
        self.run_sync(dry_run=True)
        self.assertFalse(self.destination.exists())
        self.write('go2_jiaohu/secret.py', 'ghp_' + 'x' * 36)
        with self.assertRaisesRegex(ValueError, '疑似凭据'):
            self.run_sync()
        self.assertFalse(self.destination.exists())
        (self.source / 'go2_jiaohu/secret.py').unlink()
        self.destination.mkdir(parents=True)
        (self.destination / '旧版本').symlink_to(self.root / 'elsewhere')
        with self.assertRaisesRegex(ValueError, '符号链接'):
            self.run_sync()


if __name__ == '__main__':
    unittest.main()
