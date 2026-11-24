from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from unitree_near_guard import UnitreeNearGuard, ROTATION, TRANSLATION, decode_cloud


def cloud(near=False):
    rng = np.random.default_rng(10)
    body = rng.uniform([1., -.5, -.3], [3., .5, .8], (100, 3))
    if near:
        body[0] = [.55, .02, -.1]
    raw = (body-TRANSLATION) @ np.linalg.inv(ROTATION.T)
    return raw, body, np.arange(len(raw), dtype=np.uint64)


def feed(g, i, near=False, shift=0.):
    raw, body, keys = cloud(near)
    stamp, received = 1793000000.+i*.07, 20.+i*.07
    g.observe_raw(raw, keys, stamp, received, (0.,0.,0.))
    g.observe_body(body+shift, keys, stamp, received)
    return received


def ready():
    g = UnitreeNearGuard()
    for i in range(6):
        now = feed(g, i)
    assert g.reason(now) == 'none'
    return g


def test_shifted_source_clock_still_requires_progress_and_transform():
    g = ready()
    assert g.calibration_error < 1.e-6
    assert g.reason(20.56) == 'unitree_unavailable'


def test_raw_near_presence_does_not_restart_health_recovery():
    g = ready()
    raw, _, keys = cloud(True)
    g.observe_raw(raw, keys, 1793000000.+.42, 20.42, (0.,0.,0.))
    assert g.reason(20.42) == 'none'
    assert len(g.near_points) == 1
    assert g.motion_scale(0.,0.,.7,20.42,(0.,0.,0.)) == (1., 'none')
    for i in range(7, 12):
        now = feed(g, i)
    assert g.reason(now) == 'none'


def observe_body_points(g, body, stamp=1793000000.+.42, received=20.42, pose=(0.,0.,0.)):
    raw, base, keys = cloud()
    base[:len(body)] = body
    raw = (base-TRANSLATION) @ np.linalg.inv(ROTATION.T)
    g.observe_raw(raw, keys, stamp, received, pose)


def test_one_thin_return_in_braking_path_still_stops_immediately():
    g = ready(); observe_body_points(g, [[.43,0.,-.1]])
    assert g.motion_scale(.6,0.,0.,20.42,(0.,0.,0.)) == (0., 'unitree_braking_arc_blocked')


def test_escape_from_front_obstacle_does_not_wait_for_empty_box():
    g = ready(); observe_body_points(g, [[.43,0.,-.1]])
    assert g.motion_scale(-.25,0.,0.,20.42,(0.,0.,0.)) == (1., 'none')
    assert g.reason(20.42) == 'none'


def test_point_outside_turn_sweep_allows_both_turn_directions():
    for angular in (-.70,.70):
        g=ready(); observe_body_points(g, [[.52,.08,-.15]])
        assert g.motion_scale(0.,0.,angular,20.42,(0.,0.,0.)) == (1., 'none')


def test_corner_contact_is_not_excused_as_a_pure_turn():
    g=ready(); observe_body_points(g, [[.401,-.05,-.1]])
    assert g.motion_scale(0.,0.,.7,20.42,(0.,0.,0.))[0] == 0.


def test_35cm_body_in_50cm_corridor_can_translate_without_hard_inflation():
    g=UnitreeNearGuard(half_width=.175)
    for i in range(6):feed(g,i)
    walls=[[x,y,-.1] for x in np.linspace(.41,.84,20) for y in (-.25,.25)]
    observe_body_points(g,walls)
    assert len(g.near_points) == len(walls)
    assert g.motion_scale(.3,0.,0.,20.42,(0.,0.,0.)) == (1., 'none')


def test_intermittent_points_still_block_collision_but_not_safe_turns():
    g=ready(); observe_body_points(g, [[.43,0.,-.1]])
    feed(g,7)
    assert g.motion_scale(.6,0.,0.,20.49,(0.,0.,0.))[0] == 0.
    assert g.motion_scale(0.,0.,.7,20.49,(0.,0.,0.))[0] == 1.
    for i in (8,9,10):feed(g,i)
    assert g.motion_scale(.6,0.,0.,20.70,(0.,0.,0.))[0] == 1.


def test_retained_geometry_follows_accepted_robot_pose():
    g=ready(); observe_body_points(g, [[.55,.1,-.1]])
    assert np.allclose(g.geometry(20.49,(.1,0.,0.)), [[.45,.1]])
    assert np.allclose(g.geometry(20.49,(0.,0.,np.pi/2)), [[.1,-.55]])


def test_missing_pose_cannot_treat_retained_obstacle_as_free():
    g=ready(); observe_body_points(g, [[.43,0.,-.1]],pose=None)
    assert g.motion_scale(.3,0.,0.,20.42,(0.,0.,0.)) == (0., 'unitree_pose_unavailable')


def test_health_failure_blocks_an_otherwise_safe_escape_command():
    g=ready(); observe_body_points(g, [[.55,0.,-.1]])
    assert g.motion_scale(-.3,0.,0.,20.63,(0.,0.,0.))[1] == 'unitree_unavailable'


def test_no_factory_pair_never_permits_motion():
    g = UnitreeNearGuard()
    raw, _, keys = cloud()
    for i in range(6):
        g.observe_raw(raw, keys, 100.+i*.07, 20.+i*.07)
    assert g.reason(20.35) == 'unitree_transform_unverified'


def test_wrong_factory_transform_closes_gate():
    g = ready()
    now = feed(g, 8, shift=.02)
    assert g.reason(now) == 'unitree_invalid'


def test_freeze_and_backward_source_clock_rejected():
    for stamp in (1793000000.+.35, 1793000000.):
        g = ready()
        raw, _, keys = cloud()
        g.observe_raw(raw, keys, stamp, 20.42)
        assert g.reason(20.42) == 'unitree_clock_invalid'


def test_queue_growth_rejected_and_burst_cannot_recover():
    g = ready()
    raw, _, keys = cloud()
    # Gradual delivery lag, without a single large inter-message gap.
    for i in range(6, 10):
        g.observe_raw(raw, keys, 1793000000.+i*.07, 20.+i*.07+(i-5)*.045)
    assert g.reason(20.81) == 'unitree_clock_invalid'
    for i in range(10, 13):
        g.observe_raw(raw, keys, 1793000000.+i*.07, 20.82+(i-10)*.001)
    assert g.reason(20.822) != 'none'


def test_empty_zero_and_invalid_returns_do_not_prove_clear():
    for raw in (np.empty((0, 3)), np.zeros((100, 3)), np.full((100, 3), np.nan)):
        g = ready()
        g.observe_raw(raw, np.arange(len(raw)), 1793000000.+.42, 20.42)
        assert g.reason(20.42) == 'unitree_blind'


def test_floor_and_narrow_corridor_walls_outside_stop_band():
    g = ready()
    raw, body, keys = cloud()
    body[:3] = [[.55, 0., -.5], [.55, .25, 0.], [.55, -.25, 0.]]
    raw = (body-TRANSLATION) @ np.linalg.inv(ROTATION.T)
    g.observe_raw(raw, keys, 1793000000.+.42, 20.42)
    assert g.reason(20.42) == 'none'


def test_stale_transform_and_timeout_require_new_recovery():
    g = ready()
    raw, _, keys = cloud()
    for i in range(6, 21):
        g.observe_raw(raw, keys, 1793000000.+i*.07, 20.+i*.07)
    assert g.reason(21.4) == 'unitree_transform_unverified'
    now = feed(g, 21)
    assert g.reason(now) == 'unitree_recovery'


@pytest.mark.parametrize('endian,big', [('<', False), ('>', True)])
def test_factory_decoder_checks_layout_and_byteorder(endian, big):
    dtype = np.dtype({'names': ['x','y','z','ring','time'],
        'formats': [endian+'f4']*3+[endian+'u2', endian+'f4'],
        'offsets': [0,4,8,20,24], 'itemsize': 32})
    a = np.zeros(2, dtype=dtype)
    a['x'] = [1., 2.]; a['ring'] = [3, 4]; a['time'] = [.01, .02]
    fields = [SimpleNamespace(name=k, datatype=4 if k=='ring' else 7, offset=o, count=1)
              for k,o in [('x',0),('y',4),('z',8),('ring',20),('time',24)]]
    m = SimpleNamespace(height=1, width=2, point_step=32, row_step=64,
                        is_bigendian=big, fields=fields, data=a.tobytes())
    points, keys = decode_cloud(m)
    assert list(points[:,0]) == [1.,2.]
    assert list(keys >> np.uint64(32)) == [3,4]
    m.data = b''
    with pytest.raises(ValueError): decode_cloud(m)
