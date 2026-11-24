"""Exercise leg gaps/background, continuous following and motor stop boundaries."""
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from deploy.person_follow_controller import PersonFollowController
from voice import config
from voice.person_follow import (FollowFilter, PersonMeasurement, depth_cloud,
                                 measure_person, check_path)
from voice.visual_navigation import RgbdSnapshot


BOX = (.35, .1, .65, .9)


def project(points):
    points = np.asarray(points)
    return np.stack((points[..., 2], -points[..., 0], .18-points[..., 1]), axis=-1)


def state():
    return dict(observed_at=time.monotonic(), height=.25, rpy=(0., 0., 0.),
                position=(0., 0., 0.), velocity=(0., 0., 0.), yaw_speed=0.)


def scene(distance=1.5, legs=True, obstacle=False):
    depth = np.full((240, 320), 5000, np.uint16)
    for v in range(121, 240):
        depth[v] = min(5000, round(.43*250/(v-120)*1000))
    if legs:
        bottom = min(180, int(120+.32*250/distance))
        depth[30:bottom, 125:145] = round(distance*1000)
        depth[30:bottom, 175:195] = round(distance*1000)
    if obstacle:
        depth[40:110, 145:175] = 500
    return RgbdSnapshot(np.full((240, 320, 3), 100, np.uint8), depth,
        time.time(), time.monotonic(), lambda u,v,z: ((u-160)*z/250, (v-120)*z/250, z))


def measurement(distance, bearing=0.):
    return PersonMeasurement((distance*math.cos(bearing), distance*math.sin(bearing), 0.),
                             np.zeros((0, 2)), distance)


def test_leg_gap_floor_and_isolated_noise_do_not_become_target():
    sample = scene()
    sample.depth_mm[60:64, 160:164] = 300
    assert sample.depth_mm[60, 160] == 300  # Single-pixel sampling would fail here.
    result = measure_person(sample, BOX, depth_cloud(sample, state(), project))
    assert result.distance == pytest.approx(1.5, abs=.02)
    assert result.pixels.shape[0] >= 16
    # All selected pixels lie on one of the two legs, never the gap/background.
    assert np.all(((result.pixels[:, 0] >= 125) & (result.pixels[:, 0] < 145)) |
                  ((result.pixels[:, 0] >= 175) & (result.pixels[:, 0] < 195)))


def test_background_only_cannot_authorize_a_target():
    sample = scene(legs=False)
    with pytest.raises(ValueError, match='无法区分'):
        measure_person(sample, BOX, depth_cloud(sample, state(), project))


def test_invalid_depth_and_range_jump_are_rejected():
    sample = scene()
    sample.depth_mm[:] = 0
    with pytest.raises(ValueError, match='有效深度不足'):
        measure_person(sample, BOX, depth_cloud(sample, state(), project))
    sample = scene(2.5)
    with pytest.raises(ValueError, match='跳变'):
        measure_person(sample, BOX, depth_cloud(sample, state(), project), previous=1.5)


def test_floor_removal_uses_robot_pitch():
    sample = scene(legs=False)
    cloud = depth_cloud(sample, state(), project)
    floor = cloud.valid & (cloud.body[..., 0] < 2.)
    assert np.median(np.abs(cloud.height[floor])) < .01


def test_follow_stops_on_raw_close_range_and_resumes_without_finishing():
    feedback = FollowFilter()
    assert feedback.velocity(measurement(2.), 1.)[0] > 0
    assert feedback.velocity(measurement(1.5), 1.1) == (0., 0.)
    for index in range(1, 12):
        vx, _ = feedback.velocity(measurement(2.), 1.1+index*.1)
    assert 0 < vx <= .35
    assert feedback.velocity(measurement(.8), 3.) == (0., 0.)


def test_steering_sign_acceleration_limit_and_bearing_boundary():
    left = FollowFilter()
    vx, wz = left.velocity(measurement(2., .2), 0.)
    assert 0 < vx <= .0601
    assert wz > 0
    assert FollowFilter().velocity(measurement(2., -.2), 0.)[1] < 0
    with pytest.raises(ValueError, match='45°'):
        left.velocity(measurement(2., math.radians(50)), .1)


@pytest.mark.parametrize('distance,speed', [(float('nan'), .2), (1.5, float('nan')), (.5,.2),(1.5,.5)])
def test_invalid_follow_settings(distance, speed):
    with pytest.raises(ValueError):
        FollowFilter(distance, speed)


def test_front_obstacle_and_missing_floor_block_the_motion_corridor():
    assert check_path(depth_cloud(scene(2.), state(), project), .2, .15) is None
    with pytest.raises(ValueError, match='障碍'):
        check_path(depth_cloud(scene(2., obstacle=True), state(), project), .2, .15)
    cloud = depth_cloud(scene(), state(), project)
    cloud.valid[:] = False
    with pytest.raises(ValueError, match='空洞'):
        check_path(cloud, .2, 0.)


def fixture_controller(monkeypatch):
    monkeypatch.setattr(config, 'ROBOT_MOTION_BACKEND', 'sdk')
    robot = SimpleNamespace(last_error='', calls=[], distance=2.)
    robot.reconnect = Mock(return_value=True)
    robot.read_sport_state = Mock(side_effect=state)
    robot.execute_action = Mock(return_value=True)
    robot.shutdown = Mock()
    events = []
    controller = PersonFollowController(lambda:scene(robot.distance),
        lambda message, **details: events.append((message, details)), robot=robot,
        people=SimpleNamespace(detect=Mock(return_value=[BOX])), guard=lambda:True, project=project)
    def send(vx, wz):
        robot.calls.append((vx, wz))
        if len(robot.calls) >= 6:
            controller.cancel()
        return True
    robot.send_follow_velocity = Mock(side_effect=send)
    return controller, robot, events


def test_controller_keeps_tracking_when_stopped_and_releases_sdk(monkeypatch):
    controller, robot, events = fixture_controller(monkeypatch)
    robot.distance = 1.5
    def send(vx, wz):
        robot.calls.append((vx, wz))
        robot.distance += .15  # Target walks away after the initial stop.
        if len(robot.calls) >= 6:
            controller.cancel()
        return True
    robot.send_follow_velocity.side_effect = send
    controller._run(FollowFilter(), 10.)
    assert robot.calls[0] == (0., 0.)
    assert any(vx > 0 for vx, wz in robot.calls[1:])
    robot.execute_action.assert_called_once_with('stop', speed=0., duration=0.)
    robot.shutdown.assert_called_once()
    assert events[-1][1]['stopped']


def test_person_loss_latches_stop_instead_of_reacquiring(monkeypatch):
    controller, robot, events = fixture_controller(monkeypatch)
    controller._people.detect.side_effect = lambda _: [] if robot.calls else [BOX]
    robot.send_follow_velocity.side_effect = lambda vx,wz:robot.calls.append((vx,wz)) or True
    controller._run(FollowFilter(), 10.)
    assert robot.calls[0][0] > 0
    assert all(call == (0.,0.) for call in robot.calls[1:])
    assert '丢失' in events[-1][0]
    robot.execute_action.assert_called_once()
    robot.shutdown.assert_called_once()


@pytest.mark.parametrize('failure', ['conflict', 'cancel', 'stale', 'obstacle', 'people', 'tilt'])
def test_preflight_or_cancellation_never_sends_motion(monkeypatch, failure):
    controller, robot, events = fixture_controller(monkeypatch)
    if failure == 'conflict':
        controller._guard = lambda:False
    elif failure == 'cancel':
        robot.reconnect.side_effect = lambda:controller.cancel() or True
    elif failure == 'stale':
        sample = scene()
        sample = RgbdSnapshot(sample.color, sample.depth_mm, time.time(), time.monotonic()-1., sample.deproject)
        controller._snapshot = lambda:sample
    elif failure == 'obstacle':
        controller._snapshot = lambda:scene(obstacle=True)
    elif failure == 'people':
        controller._people.detect.return_value = [BOX, (.1,.1,.3,.9)]
    else:
        robot.read_sport_state.side_effect = [state(), {**state(), 'rpy': (.3, 0., 0.)}]
    controller._run(FollowFilter(), 10.)
    assert all(call == (0.,0.) for call in robot.calls)


def test_failed_velocity_and_failed_stop_are_reported(monkeypatch):
    controller, robot, events = fixture_controller(monkeypatch)
    robot.send_follow_velocity.side_effect = None
    robot.send_follow_velocity.return_value = False
    robot.execute_action.return_value = False
    controller._run(FollowFilter(), 10.)
    assert '停止回执异常' in events[-1][0]
    robot.shutdown.assert_called_once()


def test_late_inference_cannot_send_a_motor_lease(monkeypatch):
    controller, robot, events = fixture_controller(monkeypatch)
    def detect(_):
        if controller._people.detect.call_count > 1:
            time.sleep(.22)
        return [BOX]
    controller._people.detect.side_effect = detect
    controller._run(FollowFilter(), 10.)
    assert not robot.calls
    assert '200ms' in events[-1][0]


def test_preview_does_not_connect_to_sdk_or_send_motion(monkeypatch):
    from deploy import person_follow as preview
    camera = SimpleNamespace(start=Mock(), snapshot=lambda:scene(), close=Mock())
    robot = SimpleNamespace(reconnect=Mock(), send_visual_velocity=Mock(),
                            execute_action=Mock(), shutdown=Mock())
    controller = PersonFollowController(camera.snapshot, lambda *a, **k:None,
                                        robot=robot, project=project)
    people = SimpleNamespace(detect=Mock(return_value=[BOX]), close=Mock())
    monkeypatch.setattr(preview, 'FollowCamera', lambda **kwargs:camera)
    monkeypatch.setattr(preview, 'PersonFollowController', lambda *a, **k:controller)
    monkeypatch.setattr(preview, 'body_projection', lambda:project)
    monkeypatch.setattr(preview, 'detector', people)
    monkeypatch.setattr(preview.cv2, 'namedWindow', Mock())
    monkeypatch.setattr(preview.cv2, 'imshow', Mock())
    monkeypatch.setattr(preview.cv2, 'waitKey', lambda _:ord('q'))
    monkeypatch.setattr(preview.cv2, 'destroyAllWindows', Mock())
    assert preview.run_preview(SimpleNamespace(preview_height=.25)) == 0
    robot.reconnect.assert_not_called()
    robot.send_visual_velocity.assert_not_called()
    robot.execute_action.assert_not_called()
    camera.close.assert_called_once()
    people.close.assert_called_once()


def preview_fixture(monkeypatch):
    from deploy import person_follow as preview
    camera = SimpleNamespace(start=Mock(), snapshot=lambda:scene(), close=Mock())
    controller = SimpleNamespace(active=False, start=Mock(), close=Mock(return_value=True), cancel=Mock())
    people = SimpleNamespace(detect=Mock(return_value=[BOX]), close=Mock())
    monkeypatch.setattr(preview, 'FollowCamera', lambda **kwargs:camera)
    monkeypatch.setattr(preview, 'PersonFollowController', lambda *a, **k:controller)
    monkeypatch.setattr(preview, 'body_projection', lambda:project)
    monkeypatch.setattr(preview, 'detector', people)
    for name in ('namedWindow', 'imshow', 'destroyAllWindows'):
        monkeypatch.setattr(preview.cv2, name, Mock())
    monkeypatch.setattr(preview.cv2, 'getWindowProperty', lambda *a:1.)
    return preview, camera, controller, people


def test_model_loading_keeps_gui_responsive_and_blocks_start(monkeypatch):
    import threading
    preview, camera, controller, people = preview_fixture(monkeypatch)
    entered, release = threading.Event(), threading.Event()
    def detect(_):
        entered.set()
        assert release.wait(2.)
        return [BOX]
    people.detect.side_effect = detect
    keys = []
    def wait_key(_):
        keys.append(1)
        if len(keys) == 1:  # Paint the startup window before camera/model startup.
            camera.start.assert_not_called()
            return -1
        assert entered.wait(.5)
        if len(keys) == 2:
            assert not release.is_set()
            return ord(' ')
        controller.start.assert_not_called()
        release.set()
        return ord('q')
    monkeypatch.setattr(preview.cv2, 'waitKey', wait_key)
    assert preview.run_preview(SimpleNamespace(preview_height=.25)) == 0
    assert len(keys) == 3
    assert preview.cv2.imshow.call_count >= 3
    controller.start.assert_not_called()
    camera.close.assert_called_once()


def test_model_load_error_is_reported_and_all_resources_are_closed(monkeypatch, caplog):
    preview, camera, controller, people = preview_fixture(monkeypatch)
    people.detect.side_effect = RuntimeError('CUDA model startup failed')
    def wait_key(_):
        time.sleep(.005)
        return -1
    monkeypatch.setattr(preview.cv2, 'waitKey', wait_key)
    assert preview.run_preview(SimpleNamespace(preview_height=.25)) == 1
    assert 'CUDA model startup failed' in caplog.text
    controller.start.assert_not_called()
    camera.close.assert_called_once()
    people.close.assert_called_once()


def test_preview_only_disables_space_even_after_detection_ready(monkeypatch):
    preview, camera, controller, people = preview_fixture(monkeypatch)
    keys = iter([-1, -1, ord(' '), ord('q')])
    def wait_key(_):
        time.sleep(.02)
        return next(keys)
    monkeypatch.setattr(preview.cv2, 'waitKey', wait_key)
    assert preview.run_preview(SimpleNamespace(preview_height=.25, preview_only=True)) == 0
    assert people.detect.call_count > 0
    controller.start.assert_not_called()


@pytest.mark.parametrize('visible,rect,closed', [
    (-1., (0,0,640,480), False),  # GTK does not implement WND_PROP_VISIBLE.
    (-1., (-1,-1,-1,-1), True),
    (0., (0,0,640,480), True),
    (1., (0,0,640,480), False),
])
def test_window_visibility_unsupported_is_not_treated_as_closed(monkeypatch, visible, rect, closed):
    from deploy import person_follow as preview
    monkeypatch.setattr(preview.cv2, 'getWindowProperty', lambda *a:visible)
    monkeypatch.setattr(preview.cv2, 'getWindowImageRect', lambda *a:rect)
    assert preview.preview_window_closed('test') is closed


def test_gtk_preview_stays_open_until_quit(monkeypatch):
    preview, camera, controller, people = preview_fixture(monkeypatch)
    monkeypatch.setattr(preview.cv2, 'getWindowProperty', lambda *a:-1.)
    monkeypatch.setattr(preview.cv2, 'getWindowImageRect', lambda *a:(0,0,640,480))
    wait_key = Mock(side_effect=[-1, -1, ord('q')])
    monkeypatch.setattr(preview.cv2, 'waitKey', wait_key)
    assert preview.run_preview(SimpleNamespace(preview_height=.25)) == 0
    assert wait_key.call_count == 3
    controller.start.assert_not_called()
