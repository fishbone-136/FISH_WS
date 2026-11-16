from types import SimpleNamespace
from unittest.mock import Mock
import time

import pytest
from PyQt5.QtCore import QCoreApplication

from go2_ui.navigation.relative_handoff import RelativeMotionHandoff
from voice.text_motion import parse_relative_actions


@pytest.fixture
def handoff():
    app=QCoreApplication.instance() or QCoreApplication([])
    page=SimpleNamespace(_control_pending='',_capabilities={'online':True,'goal_active':False},
        _goal_active=False,_goal_submitting=False,_navigation_closed=False,_close_failed=False,
        _close_navigation=Mock(),_launch_process=SimpleNamespace(state=Mock(return_value=0)),
        worker=SimpleNamespace(isRunning=Mock(return_value=False)))
    execute,update=Mock(return_value=True),Mock()
    mode, idle=[True],[True]
    task=RelativeMotionHandoff(lambda:page,execute,update,lambda:mode[0],idle=lambda:idle[0])
    result=SimpleNamespace(app=app,page=page,task=task,execute=execute,update=update,mode=mode,idle=idle,
        command=parse_relative_actions('转身，然后直行半米'))
    yield result
    task.cancel()


def test_sdk_sequence_waits_for_confirmed_full_navigation_exit(handoff):
    h=handoff
    assert h.task.request(*h.command)
    h.page._close_navigation.assert_called_once()
    h.task._poll()
    h.execute.assert_not_called()
    h.page._navigation_closed=True
    h.page.worker.isRunning.return_value=True
    h.task._poll()
    h.execute.assert_not_called()
    h.page.worker.isRunning.return_value=False
    h.idle[0]=False
    h.task._poll()
    h.execute.assert_not_called()
    h.idle[0]=True
    h.task._poll()
    h.execute.assert_called_once()
    assert not h.task.active
    h.task._poll()
    assert h.execute.call_count==1


@pytest.mark.parametrize('reason',['cancel','mode_change','timeout','close_failure'])
def test_stale_or_canceled_handoff_cannot_replay_a_motion(handoff,reason):
    h=handoff
    h.task.request(*h.command)
    if reason=='cancel': h.task.cancel()
    if reason=='mode_change': h.mode[0]=False
    if reason=='timeout': h.task._deadline=time.monotonic()-1
    if reason=='close_failure': h.page._close_failed=True
    h.task._poll()
    h.page._navigation_closed=True
    h.page._close_failed=False
    h.mode[0]=True
    h.task._poll()
    assert not h.task.active
    h.execute.assert_not_called()


@pytest.mark.parametrize('flag',['_goal_active','_goal_submitting'])
def test_active_navigation_goal_is_not_preempted_by_relative_text(handoff,flag):
    h=handoff
    setattr(h.page,flag,True)
    assert not h.task.request(*h.command)
    h.page._close_navigation.assert_not_called()
    h.execute.assert_not_called()


def test_external_goal_or_unknown_navigation_status_prevents_handoff(handoff):
    h=handoff
    h.page._capabilities['goal_active']=True
    assert not h.task.request(*h.command)
    h.page._capabilities={}
    assert not h.task.request(*h.command)
    h.page._close_navigation.assert_not_called()


def test_second_command_cannot_replace_a_pending_authorized_sequence(handoff):
    h=handoff
    assert h.task.request(*h.command)
    assert not h.task.request(*parse_relative_actions('后退半米'))
    h.page._navigation_closed=True
    h.task._poll()
    actions,_=h.execute.call_args.args
    assert actions[-1].action=='forward'
