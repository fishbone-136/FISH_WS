"""Check real bridge goal construction using fake TF/Nav2; never initialize ROS."""
from types import SimpleNamespace
from unittest.mock import Mock
import time

import numpy as np
import pytest
from rclpy.time import Time

from go2_ui.navigation.ros_bridge import RosNavigationWorker
from voice import config
from voice.visual_navigation import ApproachIntent, GroundedTarget, RgbdSnapshot


@pytest.fixture
def bridge(monkeypatch):
    worker = RosNavigationWorker()
    monkeypatch.setattr(config, 'VISUAL_CAMERA_MOUNT', '.2,0,.3,0,0,0')
    results = []
    worker.visual_goal_updated.connect(lambda *args: results.append(args))
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=10., y=20., z=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    def lookup(target, source, when):
        if source == 'base_link':
            return transform
        raise LookupError('No camera TF')
    worker._tf_buffer = SimpleNamespace(lookup_transform=lookup)
    worker._node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=time.time_ns())))
    worker._map_payload = dict(resolution=1., width=100,height=100,origin_x=0.,origin_y=0.,origin_yaw=0.,data=bytes(10000))
    worker._map_pose_seen_at = time.monotonic()
    worker._localization_valid = True
    worker._lifecycle_state = 3
    future = SimpleNamespace(add_done_callback=Mock())
    worker._navigator = SimpleNamespace(server_is_ready=lambda:True, send_goal_async=Mock(return_value=future))
    snapshot = RgbdSnapshot(np.zeros((100,100,3),np.uint8),np.full((100,100),3000,np.uint16),
        time.time(),time.monotonic(),lambda *args:(0,0,3))
    target = GroundedTarget('visual-test',ApproachIntent('黑色椅子',1.),snapshot,(.1,.1,.9,.9),(0.,0.,3.))
    return SimpleNamespace(worker=worker,target=target,results=results,transform=transform)


def test_depth_target_produces_nav2_goal_in_map_not_an_sdk_motion(bridge):
    b = bridge
    b.worker._send_visual_goal(b.target)
    goal = b.worker._navigator.send_goal_async.call_args.args[0]
    assert goal.pose.header.frame_id == 'odom_lio'
    assert goal.pose.pose.position.x == pytest.approx(12.2)
    assert goal.pose.pose.position.y == pytest.approx(20.)
    assert b.worker._goal_owner == 'visual-test'


def test_no_calibration_no_goal(bridge, monkeypatch):
    monkeypatch.setattr(config,'VISUAL_CAMERA_MOUNT','')
    monkeypatch.setattr(config,'VISUAL_CAMERA_CALIBRATION','')
    bridge.worker._visual_preflight('visual-test')
    assert bridge.results[-1][1] == 'rejected'
    assert '标定' in bridge.results[-1][2]
    bridge.worker._send_visual_goal(bridge.target)
    bridge.worker._navigator.send_goal_async.assert_not_called()


def test_fresh_capture_waits_for_tf_and_cancel_never_replays(bridge):
    b=bridge
    b.worker._tf_buffer.can_transform=Mock(return_value=False)
    b.worker._queue_visual_capture(b.target)
    b.worker._flush_visual_capture()
    b.worker._navigator.send_goal_async.assert_not_called()
    b.worker._cancel_visual_goal(b.target.request_id)
    b.worker._tf_buffer.can_transform.return_value=True
    b.worker._flush_visual_capture()
    b.worker._navigator.send_goal_async.assert_not_called()


def test_capture_tf_timeout_does_not_dispatch_after_late_tf_arrives(bridge):
    b=bridge
    b.worker._tf_buffer.can_transform=Mock(return_value=False)
    b.worker._queue_visual_capture(b.target)
    b.worker._capture_pending=(b.target,time.monotonic()-1.)
    b.worker._flush_visual_capture()
    assert b.results[-1][1]=='rejected'
    b.worker._tf_buffer.can_transform.return_value=True
    b.worker._flush_visual_capture()
    b.worker._navigator.send_goal_async.assert_not_called()


def test_tf_arriving_after_deadline_is_not_sent_even_if_executor_was_delayed(bridge):
    b=bridge
    b.worker._tf_buffer.can_transform=Mock(return_value=True)
    b.worker._capture_pending=(b.target,time.monotonic()-1.)
    b.worker._flush_visual_capture()
    b.worker._navigator.send_goal_async.assert_not_called()
    assert b.results[-1][1]=='rejected'


def test_file_can_prepare_navigation_but_missing_body_tf_cannot_send_goal(bridge, monkeypatch):
    monkeypatch.setattr(config, 'VISUAL_CAMERA_MOUNT', '')
    bridge.worker._camera_calibration_ready()  # Nav2 has not yet published body TF.
    bridge.worker._visual_preflight('visual-test')
    assert '变换未就绪' in bridge.results[-1][2]
    bridge.worker._send_visual_goal(bridge.target)
    bridge.worker._navigator.send_goal_async.assert_not_called()


def test_measured_calibration_uses_capture_time_and_live_lidar_body_tf(bridge, monkeypatch):
    monkeypatch.setattr(config, 'VISUAL_CAMERA_MOUNT', '')
    when = Time(seconds=123.)
    lookup = Mock(side_effect=[LookupError('No direct camera TF'), SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=.14, y=0., z=.13),
        # Body tilted relative to lidar: must use the live full rotation.
        rotation=SimpleNamespace(x=0., y=0., z=2**-.5, w=2**-.5)))])
    bridge.worker._tf_buffer.lookup_transform = lookup
    from voice.camera_calibration import load_camera_calibration
    calibration = load_camera_calibration(config.VISUAL_CAMERA_CALIBRATION, config.VISUAL_CAMERA_FRAME)
    x, y, z = calibration.point_in_parent((0., 0., 3.))
    assert bridge.worker._camera_point_in_base((0., 0., 3.), when) == pytest.approx((.14-y, x, .13+z))
    assert lookup.call_args_list[1].args == ('base_link', 'livox_frame', when)


def test_existing_direct_tf_takes_precedence_over_file_and_mount(bridge, monkeypatch):
    monkeypatch.setattr(config, 'VISUAL_CAMERA_CALIBRATION', '/nonexistent/file.json')
    bridge.worker._tf_buffer.lookup_transform = Mock(return_value=bridge.transform)
    assert bridge.worker._camera_point_in_base((0., 0., 3.), Time()) == pytest.approx((10., 20., 3.))
    bridge.worker._camera_calibration_ready()


def test_invalid_explicit_mount_cannot_silently_fall_back_to_bundled_calibration(bridge, monkeypatch):
    monkeypatch.setattr(config, 'VISUAL_CAMERA_MOUNT', 'invalid')
    with pytest.raises(ValueError, match='实测数值'):
        bridge.worker._camera_calibration_ready()


@pytest.mark.parametrize('occupancy', [100,255,80])
def test_unknown_or_occupied_approach_point_never_moves(bridge, occupancy):
    data = bytearray(10000)
    data[20*100+12] = occupancy
    bridge.worker._map_payload['data'] = bytes(data)
    bridge.worker._send_visual_goal(bridge.target)
    bridge.worker._navigator.send_goal_async.assert_not_called()
    assert bridge.results[-1][1] == 'rejected'


def test_robot_moving_during_model_inference_rejects_old_target(bridge):
    original = bridge.worker._tf_buffer.lookup_transform
    def lookup(frame, source, when):
        if source == 'base_link' and when.nanoseconds == 0:
            return SimpleNamespace(transform=SimpleNamespace(
                translation=SimpleNamespace(x=10.5,y=20.,z=0.), rotation=bridge.transform.transform.rotation))
        return original(frame,source,when)
    bridge.worker._tf_buffer.lookup_transform = lookup
    bridge.worker._send_visual_goal(bridge.target)
    bridge.worker._navigator.send_goal_async.assert_not_called()
    assert '已移动' in bridge.results[-1][2]


def test_external_goal_and_invalid_localization_are_not_preempted(bridge):
    bridge.worker._external_goal_active = True
    bridge.worker._send_visual_goal(bridge.target)
    assert '已有' in bridge.results[-1][2]
    bridge.worker._navigator.send_goal_async.assert_not_called()
    bridge.worker._external_goal_active = False
    bridge.worker._localization_valid = False
    bridge.worker._send_visual_goal(bridge.target)
    assert '定位' in bridge.results[-1][2]
    bridge.worker._navigator.send_goal_async.assert_not_called()


def test_cancel_is_scoped_to_visual_goal_and_handles_late_acceptance(bridge):
    worker = bridge.worker
    cancel = Mock()
    worker._goal_handle = SimpleNamespace(cancel_goal_async=cancel)
    worker._goal_owner = 'other'
    worker._cancel_visual_goal('visual-test')
    cancel.assert_not_called()
    worker._goal_owner = 'visual-test'
    worker._cancel_visual_goal('visual-test')
    cancel.assert_called_once()
    worker._goal_handle = None
    worker._goal_request_pending = True
    worker._cancel_visual_goal('visual-test')
    handle = SimpleNamespace(accepted=True,cancel_goal_async=Mock(),
        get_result_async=lambda:SimpleNamespace(add_done_callback=Mock()))
    worker._on_goal_response(SimpleNamespace(result=lambda:handle),'visual-test')
    handle.cancel_goal_async.assert_called_once()
    assert all(result[1] != 'accepted' for result in bridge.results)


def test_accepted_goal_is_canceled_if_result_subscription_fails(bridge):
    handle = SimpleNamespace(accepted=True,cancel_goal_async=Mock(),
        get_result_async=Mock(side_effect=RuntimeError('result client failed')))
    bridge.worker._goal_owner = 'visual-test'
    bridge.worker._on_goal_response(SimpleNamespace(result=lambda:handle),'visual-test')
    handle.cancel_goal_async.assert_called_once()
    assert bridge.results[-1][1] == 'rejected'
    assert bridge.worker._goal_owner == 'visual-test'  # Still owned for shutdown cleanup.
