from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from reload_lio_navigation import ROOT, retained_command_helpers

BASE = ROOT/'ros2_ws/install/go2_core/lib/go2_core/go2_base'
STAND = ROOT/'ros2_ws/install/go2_lio_navigation/lib/go2_lio_navigation/idle_stand_guard.py'


def process(root, pid, owner, command):
    p=root/str(pid);p.mkdir()
    (p/'stat').write_text(f'{pid} (test process) S {owner} 0 0')
    (p/'cmdline').write_bytes(b'\0'.join(str(v).encode() for v in command)+b'\0')


def test_retains_only_direct_same_launch_known_helpers(tmp_path):
    process(tmp_path,1,100,[BASE]);process(tmp_path,2,100,['python3',STAND])
    process(tmp_path,3,200,[BASE]);process(tmp_path,4,200,['python3',STAND])
    assert retained_command_helpers(100,{1,2,3,4},tmp_path)=={1,2}


def test_duplicate_own_base_is_still_a_conflict(tmp_path):
    process(tmp_path,1,100,[BASE]);process(tmp_path,2,100,[BASE])
    assert retained_command_helpers(100,{1,2},tmp_path)==set()


def test_same_names_from_other_workspace_are_not_whitelisted(tmp_path):
    process(tmp_path,1,100,['/other/go2_base'])
    process(tmp_path,2,100,['python3','/other/idle_stand_guard.py'])
    assert retained_command_helpers(100,{1,2},tmp_path)==set()


def test_unknown_or_exited_process_is_not_whitelisted(tmp_path):
    process(tmp_path,1,100,['python3','unknown.py'])
    assert retained_command_helpers(100,{1,2},tmp_path)==set()
