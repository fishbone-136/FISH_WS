from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from PyQt5.QtCore import QCoreApplication

from go2_ui.navigation.near_session import NearApproachSession
from voice.visual_navigation import ApproachIntent


@pytest.fixture
def session():
    app = QCoreApplication.instance() or QCoreApplication([])
    update, fallback = Mock(), Mock()
    mode = [True]
    controller = SimpleNamespace(active=False, start=Mock(return_value=True), close=Mock(return_value=True))
    job = NearApproachSession(lambda:None, update, fallback, lambda:mode[0],
        controller_factory=lambda *args:controller)
    return SimpleNamespace(app=app,job=job,update=update,fallback=fallback,controller=controller,mode=mode)


def test_preflight_fallback_hands_off_after_controller_releases_sdk(session):
    s = session
    intent = ApproachIntent('人')
    assert s.job.request(intent)
    ticket = s.controller.start.call_args.args[1]
    s.job.updated.emit(ticket, 'fallback', '有障碍')
    s.fallback.assert_called_once_with(intent)
    assert not s.job.active


def test_stop_invalidates_an_already_queued_nav_fallback(session):
    s = session
    s.job.request(ApproachIntent('人'))
    ticket = s.controller.start.call_args.args[1]
    s.job.cancel()
    s.job.updated.emit(ticket, 'fallback', 'queued fallback')
    s.fallback.assert_not_called()
    assert not s.job.active
    assert s.update.call_args.args[0] == 'finished'


def test_old_status_cannot_terminate_or_redirect_new_request(session):
    s = session
    s.job.request(ApproachIntent('人'))
    old_ticket = s.controller.start.call_args.args[1]
    s.job.updated.emit(old_ticket,'finished','done')
    s.job.request(ApproachIntent('左侧的人'))
    s.job.updated.emit(old_ticket,'fallback','stale')
    assert s.job.active
    s.fallback.assert_not_called()


def test_mode_change_prevents_late_navigation_start(session):
    s = session
    s.job.request(ApproachIntent('人'))
    ticket = s.controller.start.call_args.args[1]
    s.mode[0] = False
    s.job.updated.emit(ticket,'fallback','delayed')
    s.fallback.assert_not_called()


def test_control_remains_owned_until_worker_finishes(session):
    s = session
    s.job.request(ApproachIntent('人'))
    ticket = s.controller.start.call_args.args[1]
    s.controller.active = True
    s.job.cancel()
    assert s.job.active
    assert not s.job.request(ApproachIntent('人'))
    s.controller.active = False
    s.job.updated.emit(ticket,'finished','stopped')
    assert not s.job.active
