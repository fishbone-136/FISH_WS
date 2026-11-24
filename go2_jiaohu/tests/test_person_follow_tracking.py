"""Temporal target association and centring with actuator deadband and lag."""
from dataclasses import replace
import math

import numpy as np
import pytest

from voice.person_follow import PersonTrack, TargetUncertain, FollowFilter, depth_cloud, measure_person
from tests.test_person_follow import scene, state, project, measurement, BOX, fixture_controller


def track_frame(track, now, *, distance=2., boxes=None, shift=0, intensity=100):
    sample = scene(distance)
    sample.color[:] = intensity
    if shift:
        sample.depth_mm[:] = np.roll(sample.depth_mm,shift,axis=1)
    sample = replace(sample,captured_monotonic=now)
    cloud = depth_cloud(sample,state(),project)
    return track.update([BOX] if boxes is None else boxes,sample,cloud,project)


def test_full_body_to_partial_body_and_back_keeps_the_target():
    track = PersonTrack()
    track_frame(track,1.)
    partial = (.385,.12,.46,.75)
    box,target = track_frame(track,1.1,boxes=[partial])
    assert box == partial
    assert abs(target.distance-2.) < .05
    box,target = track_frame(track,1.2)
    assert box == BOX
    assert np.linalg.norm(track.velocity) < 1e-6


def test_illumination_and_turning_appearance_adapt_with_strong_depth_geometry():
    track = PersonTrack()
    track_frame(track,1.,intensity=40)
    for i in range(1,8):
        box,target = track_frame(track,1.+i*.1,intensity=100+i*8)
        assert box == BOX
    assert track.last_seen == pytest.approx(1.7)


def test_long_gap_does_not_accept_a_different_appearance_at_the_same_position():
    track = PersonTrack()
    track_frame(track,1.,intensity=40)
    with pytest.raises(TargetUncertain):
        track_frame(track,1.5,intensity=220)
    assert track.last_seen == 1.


def test_ambiguous_candidates_do_not_update_the_confirmed_target():
    track = PersonTrack()
    track_frame(track,1.)
    other = (.36,.1,.66,.9)
    with pytest.raises(TargetUncertain,match='多人'):
        track_frame(track,1.1,boxes=[BOX,other])
    assert track.box == BOX
    assert track.last_seen == 1.


def test_bad_range_frame_can_recover_without_changing_identity():
    track = PersonTrack()
    track_frame(track,1.)
    with pytest.raises(TargetUncertain):
        track_frame(track,1.1,distance=3.5)
    assert track.distance == pytest.approx(2.)
    box,target = track_frame(track,1.2,distance=2.1)
    assert box == BOX
    assert target.distance == pytest.approx(2.1,abs=.01)


def test_reacquisition_after_gap_requires_two_consecutive_confirmations():
    track = PersonTrack()
    track_frame(track,1.)
    with pytest.raises(TargetUncertain,match='两帧'):
        track_frame(track,1.4)
    assert track.last_seen == 1.
    box,target = track_frame(track,1.5)
    assert box == BOX and track.last_seen == 1.5


def test_missing_frame_breaks_reacquisition_confirmation():
    track = PersonTrack()
    track_frame(track,1.)
    with pytest.raises(TargetUncertain):
        track_frame(track,1.4)
    with pytest.raises(TargetUncertain):
        track_frame(track,1.5,boxes=[])
    with pytest.raises(TargetUncertain,match='两帧'):
        track_frame(track,1.6)
    assert track.last_seen == 1.


def test_velocity_prediction_handles_lateral_motion_and_brief_occlusion():
    track = PersonTrack()
    track_frame(track,1.)
    for i,shift in enumerate((20,40),1):
        box = tuple(x+shift/320 if j in (0,2) else x for j,x in enumerate(BOX))
        track_frame(track,1.+i*.1,boxes=[box],shift=shift)
    shift = 80
    box = tuple(x+shift/320 if j in (0,2) else x for j,x in enumerate(BOX))
    with pytest.raises(TargetUncertain,match='两帧'):
        track_frame(track,1.5,boxes=[box],shift=shift)
    result,_ = track_frame(track,1.6,boxes=[box],shift=shift)
    assert result == box


def test_depth_prior_ignores_a_new_closer_patch_in_the_leg_gap():
    sample = scene(2.)
    sample.depth_mm[30:150,150:174] = 1000
    target = measure_person(sample,BOX,depth_cloud(sample,state(),project),previous=2.,project=project)
    assert target.distance == pytest.approx(2.)


@pytest.mark.parametrize('direction',[-1,1])
def test_small_offset_produces_breakaway_command_before_reaching_edge(direction):
    feedback = FollowFilter()
    _,wz = feedback.velocity(measurement(1.5,math.radians(1.6)*direction),0.,yaw_speed=0.)
    assert .22 <= abs(wz) <= .5
    assert wz*direction > 0.
    assert feedback.velocity(measurement(1.5,math.radians(.4)),.1,yaw_speed=0.) == (0.,0.)


def test_turn_hysteresis_avoids_chatter_in_the_centre_band():
    feedback = FollowFilter()
    for i,angle in enumerate((.9,-1.,.8,-.7)):
        assert feedback.velocity(measurement(1.5,math.radians(angle)),i*.1) == (0.,0.)
    assert feedback.velocity(measurement(1.5,math.radians(1.4)),.5)[1] >= .22
    assert feedback.velocity(measurement(1.5,math.radians(.9)),.6)[1] >= .22
    assert feedback.velocity(measurement(1.5,math.radians(.5)),.7)[1] == 0.


def test_deadband_and_actuator_lag_converge_to_centre():
    feedback = FollowFilter()
    bearing,actual = math.radians(3.),0.
    commands = []
    for i in range(80):
        _,cmd = feedback.velocity(measurement(1.5,bearing),i*.1,yaw_speed=actual)
        commands.append(cmd)
        drive = cmd if abs(cmd) >= .18 else 0.
        actual += .35*(drive-actual)
        bearing -= actual*.1
    assert commands[0] >= .22
    assert abs(bearing) < math.radians(1.2)
    assert abs(actual) < .01


def test_matching_failure_pauses_and_recovers_instead_of_ending_session(monkeypatch):
    controller,robot,events = fixture_controller(monkeypatch)
    # The full/partial detection alternation used to violate the area ratio gate.
    partial = (.385,.12,.46,.75)
    boxes = [[BOX],[BOX],[partial],[BOX],[BOX],[BOX],[BOX]]
    controller._people.detect.side_effect = lambda _:boxes.pop(0) if boxes else [BOX]
    controller._run(FollowFilter(),10.)
    assert len(robot.calls) == 6
    assert all(vx > 0 for vx,wz in robot.calls)
    assert events[-1][1]['stopped']
