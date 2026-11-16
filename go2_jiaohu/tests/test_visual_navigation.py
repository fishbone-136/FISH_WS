"""Metric visual approach tests; no camera, ROS context, model or robot."""
from dataclasses import replace
import json
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from voice.visual_navigation import (ApproachIntent, GroundedTarget, RgbdSnapshot,
    approach_goal, calibrated_mount_point, is_visual_stop, locate_target,
    measure_target, parse_approach_intent, revalidate_target, transform_point)


@pytest.fixture
def snapshot():
    return RgbdSnapshot(np.full((100, 160, 3), 80, np.uint8),
        np.full((100, 160), 3000, np.uint16), time.time(), time.monotonic(),
        lambda u,v,z: ((u-80)/100*z, (v-50)/100*z, z))


@pytest.mark.parametrize('text', ['走到黑色椅子一米前', '走到黑色椅子前一米',
    '前进到黑色椅子前1m处', '请走到黑色椅子前一米', '邦邦，走到黑色椅子前1米'])
def test_explicit_target_request_extracts_object_and_metric_standoff(text):
    assert parse_approach_intent(text) == ApproachIntent('黑色椅子', 1.)


@pytest.mark.parametrize('text', ['黑色椅子在哪里？', '能不能走到椅子前一米',
    '如果我让你走到椅子前一米', '不要走到椅子前一米', '画面里有一把椅子', '走到椅子前一米吗'])
def test_conversation_never_authorizes_motion(text):
    assert parse_approach_intent(text) is None


@pytest.mark.parametrize('text', ['走到椅子前零点一米', '走到椅子前五米',
    '走到椅子前20厘米', '走到椅子前一米，然后去桌子旁', '走到椅子-1米前'])
def test_invalid_distance_or_multiple_targets_are_rejected(text):
    with pytest.raises(ValueError):
        parse_approach_intent(text)


def test_stopping_is_explicit_and_not_a_question():
    assert is_visual_stop('停止') and is_visual_stop('取消导航')
    assert not is_visual_stop('如何停止？')


def test_model_only_selects_bbox_metric_position_comes_from_depth(snapshot):
    client = SimpleNamespace(chat=Mock(return_value=json.dumps(
        {'found': True, 'ambiguous': False, 'bbox': [250,200,750,800], 'distance': 999})))
    target = locate_target(client, snapshot, ApproachIntent('黑色椅子'), 'req')
    assert target.optical_point[2] == 3.
    assert target.request_id == 'req'
    assert client.chat.call_args.kwargs['json_output'] is True
    assert client.chat.call_args.args[0][-1]['images']


@pytest.mark.parametrize('payload', [
    {'found': False}, {'found': True, 'ambiguous': True},
    {'found': True, 'ambiguous': False, 'bbox': [-1,0,500,500]},
    {'found': True, 'ambiguous': False, 'bbox': [1,2,3,4]},
    {'found': True, 'ambiguous': False, 'bbox': [True,100,900,900]},
    {'found': 'true', 'ambiguous': False, 'bbox': [100,100,900,900]},
])
def test_unreliable_grounding_cannot_produce_navigation_target(snapshot, payload):
    with pytest.raises(ValueError):
        locate_target(SimpleNamespace(chat=lambda *a,**k: json.dumps(payload)), snapshot, ApproachIntent('椅子'), 'r')


def test_invalid_sparse_or_mixed_depth_is_rejected(snapshot):
    with pytest.raises(ValueError, match='深度不足'):
        measure_target(replace(snapshot, depth_mm=np.zeros((100,160), np.uint16)), (.1,.1,.9,.9))
    mixed = snapshot.depth_mm.copy()
    mixed[:,80:] = 6000
    with pytest.raises(ValueError, match='深度混杂'):
        measure_target(replace(snapshot, depth_mm=mixed), (.1,.1,.9,.9))


def test_late_image_or_changed_object_does_not_dispatch(snapshot):
    point = measure_target(snapshot, (.1,.1,.9,.9))
    target = GroundedTarget('r', ApproachIntent('椅子'), snapshot, (.1,.1,.9,.9), point)
    revalidate_target(target, snapshot)
    with pytest.raises(ValueError, match='画面发生变化'):
        revalidate_target(target, replace(snapshot, color=np.full_like(snapshot.color, 200)))
    with pytest.raises(ValueError, match='距离发生变化'):
        revalidate_target(target, replace(snapshot, depth_mm=np.full_like(snapshot.depth_mm, 3500)))
    with pytest.raises(ValueError, match='中断'):
        revalidate_target(target, replace(snapshot, captured_monotonic=time.monotonic()-3.))
    with pytest.raises(ValueError, match='过期'):
        revalidate_target(replace(target, snapshot=replace(snapshot, captured_monotonic=time.monotonic()-31.)), snapshot)


def test_approach_goal_keeps_one_metre_on_robot_side_and_uses_world_coordinates():
    x,y,yaw = approach_goal((10.,20.), (13.,24.,.5), 1.)
    assert (x,y) == pytest.approx((12.4,23.2))
    assert math.hypot(x-13., y-24.) == pytest.approx(1.)
    assert yaw == pytest.approx(math.atan2(4.,3.))
    with pytest.raises(ValueError, match='距离以内'):
        approach_goal((0,0), (.8,0,0), 1.)


def test_real_mount_converts_optical_axes_and_accounts_for_offset():
    assert calibrated_mount_point((.4,.2,3.), '.2,0,.4,0,0,0') == pytest.approx((3.2,-.4,.2))
    assert calibrated_mount_point((0,0,3.), '0,0,0,0,0,90') == pytest.approx((0,3,0), abs=1e-8)
    with pytest.raises(ValueError):
        calibrated_mount_point((0,0,3), '')
    transform = SimpleNamespace(translation=SimpleNamespace(x=10,y=20,z=0),
        rotation=SimpleNamespace(x=0,y=0,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))
    assert transform_point((3,0,1), transform) == pytest.approx((10,23,1))
