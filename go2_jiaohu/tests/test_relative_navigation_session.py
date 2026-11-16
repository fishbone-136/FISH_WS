from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from PyQt5.QtCore import QObject, QCoreApplication, pyqtSignal

from go2_ui.navigation.relative_session import RelativeNavigationSession
from voice.text_motion import parse_relative_actions


class Worker(QObject):
    relative_action_updated=pyqtSignal(str,str,str)
    def __init__(self):
        super().__init__(); self.commands=[]
    def enqueue(self,*args): self.commands.append(args)


@pytest.fixture
def task():
    app=QCoreApplication.instance() or QCoreApplication([])
    worker=Worker()
    page=SimpleNamespace(worker=worker,_navigation_closed=False,_control_pending='',_close_failed=False,
        _capabilities={'goal_active':False},_goal_active=False,_goal_submitting=False,
        _navigation_state='ready',_last_navigation_message='',_start_stack=Mock())
    mode=[True]; update=Mock()
    session=RelativeNavigationSession(lambda:page,update,lambda:mode[0])
    result=SimpleNamespace(app=app,worker=worker,page=page,mode=mode,update=update,session=session)
    yield result
    session.cancel()


def test_ready_navigation_dispatches_without_startup_or_shutdown(task):
    n=task
    n.session.request(*parse_relative_actions('转身，然后直行半米'))
    assert n.worker.commands[-1][0]=='relative_sequence'
    n.page._start_stack.assert_not_called()
    ticket=n.worker.commands[-1][1]
    n.worker.relative_action_updated.emit(ticket,'succeeded','done')
    assert not n.session.active
    assert not n.page._navigation_closed


def test_cancel_waits_for_own_terminal_reply_and_ignores_late_acceptance(task):
    n=task
    n.session.request(*parse_relative_actions('转身'))
    ticket=n.worker.commands[-1][1]
    n.session.cancel()
    assert n.session.active
    assert n.worker.commands[-1]==('cancel_relative',ticket)
    n.worker.relative_action_updated.emit(ticket,'accepted','late')
    assert n.update.call_args.args[0]!='accepted'
    n.worker.relative_action_updated.emit(ticket,'finished','stopped')
    assert not n.session.active


def test_cancel_before_navigation_ready_never_dispatches(task):
    n=task
    n.page._navigation_state='waiting'
    n.session.request(*parse_relative_actions('转身'))
    n.mode[0]=False
    n.session._poll()
    n.page._navigation_state='ready'
    n.session._poll()
    assert not n.worker.commands


def test_existing_goal_is_never_preempted(task):
    n=task
    n.page._capabilities['goal_active']=True
    assert not n.session.request(*parse_relative_actions('转身'))
    assert not n.worker.commands
