from concurrent.futures import Future
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus

from voice.navigation_actions import navigation_steps
from voice.text_motion import parse_relative_actions
from go2_ui.navigation.relative_executor import RelativeNavigationExecutor


def done(value):
    f=Future(); f.set_result(value); return f


def test_metric_goals_remove_sdk_time_compensation_and_merge_split_turns():
    steps=navigation_steps(parse_relative_actions('转身，然后直行半米')[0])
    assert len(steps)==2
    assert steps[0].kind=='spin' and steps[0].quantity==pytest.approx(math.pi)
    assert steps[1].kind=='forward' and steps[1].quantity==pytest.approx(.5)
    assert navigation_steps(parse_relative_actions('后退半米')[0])[0].quantity==pytest.approx(-.5)


@pytest.fixture
def executor():
    events=[]
    worker=SimpleNamespace(_node=None,_relative_preflight=Mock(),_validate_relative_path=Mock(),
        _relative_stationary=Mock(return_value=True),
        relative_action_updated=SimpleNamespace(emit=lambda *args:events.append(args)))
    pending=[]
    def send(goal,**kwargs):
        response=Future()
        pending.append(SimpleNamespace(goal=goal,response=response,kwargs=kwargs))
        return response
    clients={name:SimpleNamespace(server_is_ready=lambda:True,send_goal_async=Mock(side_effect=send),destroy=Mock())
             for name in ('spin','forward','backup')}
    job=RelativeNavigationExecutor(worker,clients=clients,activity=SimpleNamespace(publish=Mock()))
    return SimpleNamespace(job=job,worker=worker,events=events,pending=pending,clients=clients)


def accept(request):
    result=Future()
    handle=SimpleNamespace(accepted=True,goal_id=request.kwargs['goal_uuid'],
        get_result_async=lambda:result,cancel_goal_async=Mock())
    request.response.set_result(handle)
    return handle,result


def test_sequence_uses_actual_action_result_and_fresh_stationary_feedback(executor):
    n=executor
    n.job.start('relative',parse_relative_actions('转身，然后直行半米')[0])
    assert len(n.pending)==1
    handle,result=accept(n.pending[0])
    n.job.tick()
    assert len(n.pending)==1
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    n.job.next_step_at=0
    n.worker._relative_stationary.return_value=False
    n.job.tick()
    assert len(n.pending)==1
    n.worker._relative_stationary.return_value=True
    n.job.tick()
    assert len(n.pending)==2
    assert n.pending[1].goal.target.x==pytest.approx(.5)
    handle,result=accept(n.pending[1])
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    n.job.next_step_at=0
    n.job.tick()
    assert not n.job.active and n.events[-1][1]=='succeeded'


def test_cancel_during_goal_acceptance_cancels_late_handle_and_never_sends_next(executor):
    n=executor
    n.job.start('relative',parse_relative_actions('转身，然后直行半米')[0])
    n.job.cancel('relative')
    handle,result=accept(n.pending[0])
    handle.cancel_goal_async.assert_called_once()
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    n.job.tick()
    assert len(n.pending)==1 and not n.job.active


def test_cancel_is_scoped_to_its_own_request(executor):
    n=executor
    n.job.start('relative',parse_relative_actions('左转90度')[0])
    handle,result=accept(n.pending[0])
    n.job.cancel('someone-else')
    handle.cancel_goal_async.assert_not_called()


def test_external_task_or_localization_failure_prevents_next_step(executor):
    n=executor
    n.job.start('relative',parse_relative_actions('左转90度，然后直行半米')[0])
    handle,result=accept(n.pending[0])
    n.worker._relative_preflight.side_effect=ValueError('定位失效')
    n.job.tick()
    handle.cancel_goal_async.assert_called_once()
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert len(n.pending)==1


def test_unready_behavior_service_does_not_fall_back_to_sdk(executor):
    n=executor
    n.clients['spin'].server_is_ready=lambda:False
    n.job.start('relative',parse_relative_actions('转身')[0])
    assert not n.pending and n.events[-1][1]=='rejected'


def test_navigation_rejection_and_unsafe_path_stop_the_sequence(executor):
    n=executor
    n.worker._validate_relative_path.side_effect=ValueError('障碍')
    n.job.start('relative',parse_relative_actions('直行半米')[0])
    assert not n.pending and not n.job.active


def test_idle_guard_heartbeat_spans_the_whole_sequence(executor):
    n=executor
    n.job.start('relative',parse_relative_actions('转身，然后直行半米')[0])
    assert n.job.activity.publish.call_args.args[0].data
    n.job._heartbeat_at=0
    n.job.tick()
    assert n.job.activity.publish.call_count==2


def test_stale_acceptance_error_does_not_cancel_a_new_sequence(executor):
    n=executor
    n.job.owner='new'
    f=Future(); f.set_exception(RuntimeError('old request'))
    n.job._accepted('old',0,f,10.)
    assert not n.job.cancel_requested
