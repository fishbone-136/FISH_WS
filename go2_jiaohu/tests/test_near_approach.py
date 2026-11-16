import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from voice import config
from voice.near_approach import (NearUnavailable, body_projection, valid_state,
    near_limits, approach_velocity, corridor_clear, target_in_body, track_person, appearance)
from voice.person_detector import person_side, select_person
from voice.visual_navigation import ApproachIntent, RgbdSnapshot
from deploy.near_approach_controller import NearApproachController


BOX = (.44, .10, .56, .70)


def project(points):
    points = np.asarray(points)
    return np.stack((points[..., 2], -points[..., 0], .18-points[..., 1]), axis=-1)


def state():
    return dict(observed_at=time.monotonic(), height=.25, rpy=(0., 0., 0.),
        position=(0., 0., 0.), velocity=(0., 0., 0.), yaw_speed=0.)


def snapshot(distance=2., obstacle=False):
    depth = np.full((480, 640), 8000, np.uint16)
    for v in range(241, 480):
        depth[v, :] = min(8000, int(.43*400/(v-240)*1000))
    depth[48:336, 281:359] = round(distance*1000)
    if obstacle:
        depth[320:380, 280:360] = 500
    color = np.full((480, 640, 3), 100, np.uint8)
    return RgbdSnapshot(color, depth, time.time(), time.monotonic(),
        lambda u, v, z: ((u-320)*z/400, (v-240)*z/400, z))


def test_sdk_geometry_uses_recorded_body_mount_without_navigation_tf(monkeypatch):
    monkeypatch.setattr(config, 'VISUAL_CAMERA_MOUNT', '')
    transform = body_projection()
    assert transform((0., 0., 3.)) == pytest.approx((3.1565155736, .0261072594, .2231170938))


@pytest.mark.parametrize('target, expected', [('人','any'), ('前方的人','any'),
    ('左边的人','left'), ('右侧的人','right'), ('黑衣服的人',None), ('黑色椅子',None)])
def test_fast_person_vocabulary_never_discards_attributes(target, expected):
    assert person_side(target) == expected


def test_multiple_people_never_automatically_choose_nearest():
    boxes = [(.1,.1,.3,.9),(.6,.1,.8,.9)]
    with pytest.raises(ValueError, match='不唯一'):
        select_person(boxes, 'any')
    assert select_person(boxes, 'left') == tuple(boxes[0])


def test_plain_floor_is_clear_but_obstacle_and_invalid_depth_are_rejected():
    assert corridor_clear(snapshot(), state(), project, (2., 0., .2), 1.)
    with pytest.raises(NearUnavailable, match='障碍'):
        corridor_clear(snapshot(obstacle=True), state(), project, (2.,0.,.2),1.)
    sample = snapshot()
    sample.depth_mm[:] = 0
    with pytest.raises(NearUnavailable, match='深度空洞'):
        corridor_clear(sample, state(), project, (2.,0.,.2),1.)


def test_tilted_stale_and_nonstationary_robot_cannot_start():
    for changes in [dict(rpy=(.3,0.,0.)), dict(observed_at=time.monotonic()-1), dict(velocity=(.2,0.,0.))]:
        with pytest.raises(ValueError):
            valid_state({**state(), **changes}, stationary=True)


def test_range_bearing_and_standoff_limits():
    for point, stop in [((5.,0.,0.),1.), ((2.,1.,0.),1.), ((-2.,0.,0.),1.), ((2.,0.,0.),.6)]:
        with pytest.raises(NearUnavailable):
            near_limits(point, stop)
    assert approach_velocity((1.05,0.,0.), 1.) == (0.,0.)
    assert 0 < approach_velocity((1.2,0.,0.), 1.)[0] < approach_velocity((2.,0.,0.), 1.)[0] <= .2


def test_tracking_loss_or_crossing_never_switches_person():
    frame = snapshot().color
    original = appearance(frame, BOX)
    with pytest.raises(ValueError, match='丢失'):
        track_person([], BOX, frame, original)
    with pytest.raises(ValueError):
        track_person([BOX,(.45,.12,.58,.69)], BOX, frame, original)


def fixture_controller(monkeypatch, *, distance=2., obstacle=False):
    monkeypatch.setattr(config, 'ROBOT_MOTION_BACKEND', 'sdk')
    import deploy.near_approach_controller as module
    monkeypatch.setattr(module, 'body_projection', lambda: project)
    robot = SimpleNamespace(distance=distance, last_error='', calls=[])
    robot.reconnect = Mock(return_value=True)
    def velocity(vx, wz):
        robot.calls.append((vx,wz))
        if vx:
            robot.distance = max(1.04, robot.distance-.06)
        return True
    robot.send_visual_velocity = velocity
    robot.execute_action = Mock(return_value=True)
    robot.shutdown = Mock()
    monitor = SimpleNamespace(get=state, close=Mock())
    events = []
    controller = NearApproachController(lambda:snapshot(robot.distance,obstacle),
        lambda *args:events.append(args), robot=robot,
        people=SimpleNamespace(detect=lambda image:[BOX]), state_factory=lambda:monitor, guard=lambda:True)
    return controller, robot, monitor, events


def test_closed_loop_slows_stops_and_releases_sdk_before_success(monkeypatch):
    controller, robot, monitor, events = fixture_controller(monkeypatch)
    controller._run(ApproachIntent('人'), 'test', True)
    assert events[-1][1] == 'succeeded', events
    assert len(robot.calls) > 5
    moving = [v[0] for v in robot.calls if v[0] > 0]
    assert moving[-1] < moving[0]
    assert robot.calls[-1] == (0.,0.)
    robot.execute_action.assert_called_once_with('stop',speed=0.,duration=0.)
    robot.shutdown.assert_called_once()
    monitor.close.assert_called_once()


@pytest.mark.parametrize('distance, obstacle', [(5.,False),(2.,True)])
def test_fallback_to_nav_is_only_possible_before_any_sdk_motion(monkeypatch, distance, obstacle):
    controller, robot, monitor, events = fixture_controller(monkeypatch,distance=distance,obstacle=obstacle)
    controller._run(ApproachIntent('人'), 'test', True)
    assert events[-1][1] == 'fallback', events
    robot.reconnect.assert_not_called()
    assert not robot.calls


def test_cancel_during_connection_cannot_replay_a_late_move(monkeypatch):
    controller, robot, monitor, events = fixture_controller(monkeypatch)
    def connect():
        controller.cancel()
        return True
    robot.reconnect.side_effect = connect
    controller._run(ApproachIntent('人'), 'test', True)
    assert not robot.calls
    assert events[-1][1] == 'finished'
    robot.shutdown.assert_called_once()


def test_target_loss_during_motion_stops_without_nav_fallback(monkeypatch):
    controller, robot, monitor, events = fixture_controller(monkeypatch)
    controller._detector.detect = lambda image: [] if robot.calls else [BOX]
    controller._run(ApproachIntent('人'), 'test', True)
    assert robot.calls
    assert events[-1][1] == 'rejected'
    assert all(event[1] != 'fallback' for event in events)
    robot.execute_action.assert_called_once()


def test_navigation_conflict_prevents_sdk_connection(monkeypatch):
    controller, robot, monitor, events = fixture_controller(monkeypatch)
    controller._guard = lambda: False
    controller._run(ApproachIntent('人'), 'test', True)
    assert events[-1][1] == 'fallback'
    robot.reconnect.assert_not_called()
