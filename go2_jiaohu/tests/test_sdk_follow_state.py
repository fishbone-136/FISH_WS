"""SDK feedback protocol with fake DDS state and pipe input, no robot connection."""
import io
import json
import os
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from voice import sdk_worker
from voice.robot_interface import _SdkProcessBridge


def test_sport_state_reader_preserves_receipt_time_and_rejects_stale_data():
    reader = sdk_worker._SportStateReader()
    subscriber = Mock()
    reader._subscribers = [subscriber]
    with pytest.raises(ValueError, match='未就绪'):
        reader.get()
    message = SimpleNamespace(body_height=.25, imu_state=SimpleNamespace(rpy=[0.,0.,0.]),
                              position=[1.,2.,0.], velocity=[.1,0.,0.], yaw_speed=.1)
    reader._receive(message)
    state = reader.get()
    assert state['position'] == [1.,2.,0.]
    assert 0 <= time.monotonic()-state['observed_at'] < .3
    reader._state['observed_at'] -= 1.
    with pytest.raises(ValueError, match='过期'):
        reader.get()
    reader.close()
    subscriber.Close.assert_called_once()


def test_sdk_feedback_accepts_high_or_low_frequency_stream(monkeypatch):
    import sys
    subscribers = []
    def subscribe(topic, message_type):
        subscriber = SimpleNamespace(topic=topic, Init=Mock(), Close=Mock())
        subscribers.append(subscriber)
        return subscriber
    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.core.channel', SimpleNamespace(ChannelSubscriber=subscribe))
    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.idl.unitree_go.msg.dds_', SimpleNamespace(SportModeState_=object))
    reader = sdk_worker._SportStateReader()
    with pytest.raises(ValueError, match='未就绪'):
        reader.get()
    assert [s.topic for s in subscribers] == ['rt/sportmodestate', 'rt/lf/sportmodestate']
    # The low-rate stream alone can satisfy the feedback requirement.
    callback = subscribers[1].Init.call_args.args[0]
    callback(SimpleNamespace(body_height=.25, imu_state=SimpleNamespace(rpy=[0.,0.,0.]),
                             position=[0.,0.,0.], velocity=[0.,0.,0.], yaw_speed=0.))
    assert reader.get()['height'] == .25
    reader.close()
    for subscriber in subscribers:
        subscriber.Close.assert_called_once()


def test_state_request_returns_feedback_without_any_motor_call(monkeypatch):
    client = SimpleNamespace(Move=Mock(return_value=0), StopMove=Mock(return_value=0))
    reader = SimpleNamespace(get=Mock(return_value={'height':.25, 'observed_at':123.}))
    messages = []
    monkeypatch.setattr(sdk_worker, '_send_status', lambda *a, **k:messages.append((a,k)))
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd) as stream:
        worker = threading.Thread(target=sdk_worker._run_command_loop, args=(client,stream,reader))
        worker.start()
        try:
            os.write(write_fd, (json.dumps(dict(command='read_sport_state',request_id='state'))+'\n').encode())
            deadline = time.monotonic()+1.
            while not messages and time.monotonic() < deadline:
                time.sleep(.005)
        finally:
            os.close(write_fd)
            worker.join(timeout=2.)
    assert not worker.is_alive()
    assert messages == [(('accepted',), dict(request_id='state', state=reader.get.return_value))]
    client.Move.assert_not_called()
    client.StopMove.assert_not_called()


def test_bridge_returns_state_payload_and_keeps_boolean_action_ack():
    bridge = _SdkProcessBridge.__new__(_SdkProcessBridge)
    bridge._process = SimpleNamespace(poll=lambda:None, stdin=io.StringIO())
    bridge._lock = threading.Lock()
    bridge._request_counter = 0
    bridge.error = ''
    def response(timeout):
        last = json.loads(bridge._process.stdin.getvalue().splitlines()[-1])
        return dict(request_id=last['request_id'], status='accepted', state={'height':.25})
    bridge._read_response = response
    assert bridge.read_sport_state() == {'height':.25}
    assert bridge.send_visual_velocity(.1, .1) is True
    assert bridge.send_follow_velocity(0., .5) is True
    assert json.loads(bridge._process.stdin.getvalue().splitlines()[-1])['command'] == 'follow_velocity'
    assert bridge.send_action('stop', 0., 0., 0.) is True


def test_bridge_reports_feedback_errors_instead_of_returning_a_fake_state():
    bridge = _SdkProcessBridge.__new__(_SdkProcessBridge)
    bridge._process = None
    bridge.error = 'feedback unavailable'
    with pytest.raises(ValueError, match='unavailable'):
        bridge.read_sport_state()
