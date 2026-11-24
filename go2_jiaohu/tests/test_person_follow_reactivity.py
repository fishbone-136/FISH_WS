"""Wide-view calibration, fast feedback, and orientation independent of range."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from deploy.person_follow_controller import ControlGuard
from voice.person_detector import PersonDetector
from voice.person_detector_worker import decode_people
from voice.person_follow import (FollowFilter, camera_projection, depth_cloud, measure_person,
                                 person_appearance, match_person, check_path, PersonMeasurement)
from tests.test_person_follow import scene, state, project, measurement, fixture_controller, BOX


def test_standoff_keeps_turning_to_keep_person_centred():
    feedback = FollowFilter()
    vx,wz = feedback.velocity(measurement(1.5,math.radians(15)), 1.)
    assert vx == 0.
    assert .3 < wz <= .5
    assert FollowFilter().velocity(measurement(1.5,math.radians(-15)),1.)[1] < -.3
    assert FollowFilter().velocity(measurement(.5,math.radians(15)),1.) == (0.,0.)


def test_turn_first_then_resume_forward_without_leaving_follow_mode():
    feedback = FollowFilter(max_speed=.15)
    vx,wz = feedback.velocity(measurement(2.,math.radians(30)),1.)
    assert vx == 0. and wz == .5
    vx,wz = feedback.velocity(measurement(2.,math.radians(10)),1.1)
    assert vx > 0. and wz > 0.
    for i in range(2,6):
        vx,wz = feedback.velocity(measurement(2.),1.+i*.1)
    assert vx == pytest.approx(.15)
    assert wz == 0.


def test_heading_error_converges_while_holding_range():
    feedback = FollowFilter()
    bearing = math.radians(30)
    for i in range(35):
        vx,wz = feedback.velocity(measurement(1.5,bearing),i*.1)
        assert vx == 0.
        bearing -= wz*.1
    assert abs(bearing) <= math.radians(3.1)


def test_ir_to_colour_extrinsics_are_applied_after_scaling_and_rays_are_cached():
    calls = []
    def deproject(intrinsics, uv, z):
        calls.append(uv)
        return [(uv[0]-1.)*z/2., (uv[1]-1.)*z/2., z]
    rs = SimpleNamespace(rs2_deproject_pixel_to_point=deproject)
    intrinsics = SimpleNamespace(width=4,height=3)
    rotation = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    extrinsics = SimpleNamespace(rotation=rotation.flatten(order='F').tolist(),translation=[.2,-.1,.05])
    one,many = camera_projection(rs,intrinsics,extrinsics)
    u,v,z = np.array([0,2,3]),np.array([0,1,2]),np.array([1.,2.,3.])
    expected = np.array([one(float(x),float(y),float(d)) for x,y,d in zip(u,v,z)])
    assert np.allclose(many(u,v,z),expected)
    before = len(calls)
    assert np.allclose(many(u,v,z),expected)
    assert len(calls) == before
    expected2 = np.array([one(float(x),float(y),float(d*2)) for x,y,d in zip(u,v,z)])
    assert np.allclose(many(u,v,z*2),expected2)


def test_batch_projection_is_used_instead_of_per_pixel_sdk_calls():
    from dataclasses import replace
    sample = scene()
    many = Mock(side_effect=lambda u,v,z:np.column_stack(((u-160)*z/250,(v-120)*z/250,z)))
    single = Mock(side_effect=AssertionError('per-pixel projection must not run'))
    cloud = depth_cloud(replace(sample,deproject=single,deproject_many=many),state(),project)
    assert cloud.valid.any()
    many.assert_called_once()
    single.assert_not_called()


def test_bbox_heading_is_not_biased_toward_one_swinging_leg():
    sample = scene()
    # Only one leg has usable samples, but the detection still covers the person.
    sample.depth_mm[30:180,175:195] = 0
    result = measure_person(sample,BOX,depth_cloud(sample,state(),project),project=project)
    assert abs(result.point[1]) < 1e-6
    assert result.distance == pytest.approx(1.5)


def test_control_guard_avoids_repeated_full_proc_scans_but_refreshes():
    now = [0.]
    scan = Mock(side_effect=[True,False])
    guard = ControlGuard(scan=scan,clock=lambda:now[0])
    assert guard()
    for value in (.05,.1,.2):
        now[0] = value
        assert guard()
    assert scan.call_count == 1
    now[0] = .26
    assert not guard()
    now[0] = .3
    assert not guard()
    assert scan.call_count == 2


def test_partial_body_confidence_is_local_to_follow_detector():
    output = np.zeros((84,3),np.float32)
    output[:4,0] = (320,240,80,240)
    output[4,0] = .4
    assert decode_people(output,640,480,1.,(0,0)) == []
    assert len(decode_people(output,640,480,1.,(0,0),confidence=.35)) == 1
    assert PersonDetector().confidence == .55
    assert PersonDetector(confidence=.35).confidence == .35


def test_monochrome_appearance_still_distinguishes_different_intensity():
    frame = np.full((240,320,3),40,np.uint8)
    original = person_appearance(frame,BOX)
    assert match_person([BOX],BOX,frame,original) == BOX
    bright = np.full_like(frame,210)
    with pytest.raises(ValueError,match='匹配'):
        match_person([BOX],BOX,bright,original)


def test_rotation_checks_its_swept_corridor():
    cloud = depth_cloud(scene(),state(),project)
    locations = np.argwhere(cloud.valid)[:3]
    for r,c in locations:
        cloud.body[r,c] = (.55,.45,.25)
        cloud.height[r,c] = .5
    with pytest.raises(ValueError,match='障碍'):
        check_path(cloud,0.,.5)


def test_short_dropout_stops_before_recovering_same_target(monkeypatch):
    controller, robot, events = fixture_controller(monkeypatch)
    detections = [[BOX],[BOX],[],[],[BOX],[BOX],[BOX],[BOX]]
    controller._people.detect.side_effect = lambda _:detections.pop(0) if detections else [BOX]
    controller._run(FollowFilter(),10.)
    assert robot.calls[0][0] > 0.
    assert robot.calls[1:3] == [(0.,0.),(0.,0.)]
    assert any(vx > 0 for vx,wz in robot.calls[3:])
    assert any('短暂漏检' in text for text,details in events)


def test_default_speed_reaches_035_without_exceeding_the_cap():
    feedback = FollowFilter()
    velocities = [feedback.velocity(measurement(2.),i*.1)[0] for i in range(12)]
    assert max(velocities) == pytest.approx(.35)
    assert all(0. <= vx <= .35 for vx in velocities)
    assert all(b-a <= .0601 for a,b in zip(velocities,velocities[1:]))


def test_image_centre_overrides_mounting_axis_offset():
    sample = scene()
    angle = math.radians(12)
    rotation = np.array([[math.cos(angle),-math.sin(angle),0.],
                         [math.sin(angle),math.cos(angle),0.],[0.,0.,1.]])
    def mount(points):
        return project(points) @ rotation.T
    target = measure_person(sample,BOX,depth_cloud(sample,state(),mount),project=mount)
    assert math.atan2(target.point[1],target.point[0]) == pytest.approx(angle)
    assert target.image_bearing == pytest.approx(0.,abs=1e-6)
    assert FollowFilter().velocity(target,1.) == (0.,0.)


def test_small_screen_offset_turns_before_person_gets_near_edge():
    target = PersonMeasurement((1.5,0.,0.),np.zeros((0,2)),1.5,math.radians(2))
    vx,wz = FollowFilter().velocity(target,1.)
    assert vx == 0.
    assert wz > .09


def test_lateral_motion_and_direction_reversal_remain_near_image_centre():
    feedback = FollowFilter()
    bearing,actual_yaw = .15,0.
    for phase,target_rate in enumerate((.25,-.25,0.)):
        errors = []
        for step in range(60):
            target = PersonMeasurement((1.5*math.cos(bearing),1.5*math.sin(bearing),0.),
                                       np.zeros((0,2)),1.5,bearing)
            vx,wz = feedback.velocity(target,(phase*60+step)*.1,yaw_speed=actual_yaw)
            assert vx == 0.
            assert abs(wz) <= .5
            actual_yaw += .35*(wz-actual_yaw)  # Include actuator lag.
            bearing += (target_rate-actual_yaw)*.1
            errors.append(abs(bearing))
        assert max(errors) < math.radians(12)
        assert errors[-1] <= math.radians(1.2)
    assert abs(actual_yaw) < .01


def test_rate_feedback_bias_cannot_keep_a_centred_person_turning():
    feedback = FollowFilter()
    feedback.velocity(measurement(1.5,.2),1.,yaw_speed=.5)
    assert feedback.velocity(measurement(1.5,0.),1.1,yaw_speed=.5) == (0.,0.)
    feedback.reset()
    assert feedback.last_bearing is None
    assert feedback.turn_integral == 0.
    assert not feedback.turning


def test_small_detection_jitter_does_not_trigger_continuous_rotation():
    feedback = FollowFilter()
    for i in range(30):
        target = measurement(1.5,math.radians(.25 if i % 2 else -.25))
        assert feedback.velocity(target,i*.1,yaw_speed=0.) == (0.,0.)
