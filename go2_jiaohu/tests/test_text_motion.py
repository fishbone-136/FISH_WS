"""Explicit visual text actions with fake robot RPCs only."""
import threading
from unittest.mock import Mock

import pytest

from deploy.text_motion_controller import TextMotionController
from voice import config
from voice.command_safety import SafeAction
from voice.text_motion import parse_relative_actions


@pytest.mark.parametrize('text', ['转身','掉头','调头','向后转','现在转身'])
def test_turnaround_preserves_180_degrees_without_raising_motor_limits(text):
    actions,warnings = parse_relative_actions(text)
    assert all(a.action=='turn_left' for a in actions)
    assert sum(a.angle for a in actions)==pytest.approx(180.)
    assert all(a.duration<=config.MAX_ACTION_DURATION for a in actions)
    assert sum(a.duration for a in actions)==pytest.approx(3.141592653589793/config.DEFAULT_TURN_SPEED*config.TURN_TIME_MULTIPLIER)
    assert not warnings


def test_observed_turnaround_then_straight_half_metre_is_a_complete_sequence():
    actions,_ = parse_relative_actions('转身，然后直行半米')
    assert sum(a.angle for a in actions if a.action=='turn_left')==pytest.approx(180.)
    assert actions[-1].action=='forward'
    assert actions[-1].duration==pytest.approx(.5/config.DEFAULT_LINEAR_SPEED*config.LINEAR_TIME_MULTIPLIER)


def test_unsupported_motion_is_explained_but_questions_stay_readonly():
    from voice.text_motion import is_motion_request
    text='现在转身，然后寻找黑色衣服的人，走到他前一米'
    assert parse_relative_actions(text) is None
    assert is_motion_request(text)
    for question in ['转身是什么意思','如果转身会怎样','不要转身','黑衣服的人在哪里']:
        assert parse_relative_actions(question) is None
        assert not is_motion_request(question)


@pytest.mark.parametrize('text',['转身零度','左转270度'])
def test_turn_angle_is_not_silently_changed_or_zero_promoted_to_motion(text):
    with pytest.raises(ValueError):
        parse_relative_actions(text)


@pytest.mark.parametrize('text', ['往前走半米','向前走0.5米','前进半米','请帮我往前走半米','执行指令，前进半米'])
def test_observed_relative_command_is_an_action_not_a_scene_question(text):
    actions,warnings=parse_relative_actions(text)
    assert not warnings
    assert actions[0].action=='forward'
    assert actions[0].duration==pytest.approx(.5/config.DEFAULT_LINEAR_SPEED*config.LINEAR_TIME_MULTIPLIER)


@pytest.mark.parametrize('text',['移动到大型黑色设备前一米处','前进到黑色椅子前一米',
    '往前走半米是什么意思','不要前进半米','如果前进半米会怎样','左边有什么','前进到障碍物旁边'])
def test_object_requests_and_questions_never_fall_back_to_blind_forward_motion(text):
    assert parse_relative_actions(text) is None


def test_sequence_parsing_and_invalid_parameters():
    actions,_=parse_relative_actions('前进半米，然后左转九十度，再后退半米')
    assert [a.action for a in actions]==['forward','turn_left','backward']
    with pytest.raises(ValueError):
        parse_relative_actions('前进零米')
    with pytest.raises(ValueError):
        parse_relative_actions('前进，然后左转，再后退，再右转，再前进')


class Robot:
    def __init__(self):
        self.client=None
        self.last_error='test failure'
        self.calls=[]
        self.reconnect=Mock(side_effect=self.connect)
        self.shutdown=Mock(side_effect=self.disconnect)
    def connect(self):
        self.client=self
        return True
    def disconnect(self):
        self.client=None
    def execute_action(self,action,**kwargs):
        self.calls.append(action)
        return True


def test_typed_motion_waits_for_ack_then_stops_and_releases_bridge():
    robot=Robot()
    updates=[]
    controller=TextMotionController(lambda *args:updates.append(args),robot=robot,guard=lambda:True)
    assert controller.start([SafeAction('forward',.3,.01)],'r1')
    controller._thread.join(2.)
    assert not controller.is_busy
    assert robot.calls==['forward','stop']
    assert robot.client is None
    assert [update[1] for update in updates]==['connecting','accepted','finished']


def test_multiple_actions_execute_in_order_and_release_after_the_last_step():
    robot=Robot()
    updates=[]
    controller=TextMotionController(lambda *args:updates.append(args),robot=robot,guard=lambda:True)
    sequence=[SafeAction('turn_left',.5,.01,90.),SafeAction('turn_left',.5,.01,90.),SafeAction('forward',.3,.01)]
    assert controller.start(sequence,'sequence')
    controller._thread.join(2.)
    assert robot.calls==['turn_left','turn_left','forward','stop']
    assert len([update for update in updates if update[1]=='accepted'])==3
    assert robot.client is None


def test_cancel_after_first_step_prevents_the_remaining_sequence():
    robot=Robot()
    controller=TextMotionController(lambda *args:None,robot=robot,guard=lambda:True)
    def execute(action,**kwargs):
        robot.calls.append(action)
        if action=='turn_left':
            controller.cancel()
        return True
    robot.execute_action=execute
    controller.start([SafeAction('turn_left',.5,.01,90.),SafeAction('forward',.3,.01)],'sequence')
    controller._thread.join(2.)
    assert robot.calls==['turn_left','stop']
    assert robot.client is None


def test_navigation_conflict_does_not_connect_or_move():
    robot=Robot()
    updates=[]
    controller=TextMotionController(lambda *args:updates.append(args),robot=robot,guard=lambda:False)
    controller.start([SafeAction('forward',.3,.01)],'r1')
    controller._thread.join(2.)
    robot.reconnect.assert_not_called()
    assert not robot.calls
    assert updates[-1][1]=='error'


def test_cancel_while_connecting_prevents_a_late_move():
    robot=Robot()
    entered,release=threading.Event(),threading.Event()
    def reconnect():
        entered.set()
        assert release.wait(2.)
        return robot.connect()
    robot.reconnect=Mock(side_effect=reconnect)
    controller=TextMotionController(lambda *args:None,robot=robot,guard=lambda:True)
    controller.start([SafeAction('forward',.3,.01)],'r1')
    assert entered.wait(1.)
    controller.cancel()
    release.set()
    controller.close()
    assert 'forward' not in robot.calls
    assert robot.calls==['stop']


def test_navigation_appearing_mid_motion_triggers_stop():
    robot=Robot()
    updates=[]
    guard=Mock(side_effect=[True,True,False])
    controller=TextMotionController(lambda *args:updates.append(args),robot=robot,guard=guard)
    controller.start([SafeAction('forward',.3,1.)],'r1')
    controller._thread.join(2.)
    assert robot.calls==['forward','stop']
    assert updates[-1][1]=='error'
    assert '导航控制源' in updates[-1][2]


def test_sdk_rejection_is_visible_and_not_reported_as_accepted():
    robot=Robot()
    robot.execute_action=Mock(side_effect=lambda action,**kw:action=='stop')
    updates=[]
    controller=TextMotionController(lambda *args:updates.append(args),robot=robot,guard=lambda:True)
    controller.start([SafeAction('forward',.3,.01)],'r1')
    controller._thread.join(2.)
    assert not any(update[1]=='accepted' for update in updates)
    assert updates[-1][1]=='error'
    assert '未被接受' in updates[-1][2]
