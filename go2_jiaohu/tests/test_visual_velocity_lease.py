import json
import os
import threading
import time

import pytest

from voice import config, sdk_worker


@pytest.mark.parametrize('changes', [dict(vx=-.1),dict(vx=.3),dict(wz=1.),dict(lease=1.),
    dict(issued_at=90.),dict(issued_at=101.),dict(vx=float('nan'))])
def test_invalid_or_stale_visual_velocity_is_rejected(changes):
    payload = dict(vx=.1,wz=.1,issued_at=100.,lease=.25)
    with pytest.raises(ValueError):
        sdk_worker._visual_velocity_spec({**payload,**changes},100.)


def test_input_timestamp_sets_deadline_instead_of_extending_a_delayed_command():
    motion, deadline = sdk_worker._visual_velocity_spec(dict(vx=.1,wz=.1,issued_at=100.,lease=.25),100.1)
    assert motion == (.1,0.,.1)
    assert deadline == 100.25


@pytest.mark.parametrize('command',['visual_velocity','follow_velocity'])
def test_worker_expires_velocity_lease_when_perception_stops(monkeypatch, command):
    calls, statuses = [], []
    client = type('Client',(),{'Move':lambda self,*v:calls.append(v) or 0,
        'StopMove':lambda self:calls.append('stop') or 0})()
    monkeypatch.setattr(config,'ROBOT_SDK_MOVE_INTERVAL',.005)
    monkeypatch.setattr(sdk_worker,'_send_status',lambda *a,**k:statuses.append((a,k)))
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd) as stream:
        thread = threading.Thread(target=sdk_worker._run_command_loop,args=(client,stream))
        thread.start()
        try:
            os.write(write_fd,(json.dumps(dict(command=command,vx=.1,wz=.1,
                issued_at=time.monotonic(),lease=.15,request_id='lease'))+'\n').encode())
            deadline=time.monotonic()+1.
            while 'stop' not in calls and time.monotonic()<deadline:
                time.sleep(.01)
            assert calls[0] == (.1,0.,.1)
            assert 'stop' in calls
            assert any(args[0]=='accepted' for args,kwargs in statuses)
        finally:
            os.close(write_fd)
            thread.join(timeout=2.)
        assert not thread.is_alive()


def test_follow_lease_accepts_rotation_at_zero_forward_speed(monkeypatch):
    monkeypatch.setattr(config,'MAX_TURN_SPEED',.65)
    motion,deadline = sdk_worker._follow_velocity_spec(
        dict(vx=0.,wz=.5,issued_at=100.,lease=.25),100.1)
    assert motion == (0.,0.,.5)
    assert deadline == 100.25


@pytest.mark.parametrize('changes',[dict(vx=-.1),dict(vx=.41),dict(wz=.51),dict(lease=.5),
                                   dict(issued_at=99.),dict(wz=float('nan'))])
def test_follow_speed_and_timestamp_limits(changes):
    with pytest.raises(ValueError):
        sdk_worker._follow_velocity_spec({**dict(vx=.2,wz=.3,issued_at=100.,lease=.25),**changes},100.)
