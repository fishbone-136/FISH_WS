import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from go2_perception.unitree_navigation_source import (
    interpolate_poses, near_clearance_rays, rotate, scan_geometry,
)
from go2_perception.unitree_cloud import ROTATION, TRANSLATION


def test_missing_near_return_is_added_once_and_keeps_its_measurement_time():
    base = np.column_stack((np.linspace(1., 2., 32), np.zeros(32), np.zeros(32)))
    near = np.array([[.6, .1, -.1]])
    body_raw = np.vstack((base, near))
    raw = (body_raw - TRANSLATION) @ ROTATION
    seconds = (np.arange(33) * .001).astype(np.float32)
    keys = seconds.view(np.uint32).astype(np.uint64)
    points, offsets = scan_geometry((raw, keys), (base, keys[:32]))
    np.testing.assert_allclose(points, body_raw, atol=2e-6)
    assert offsets[-1] == pytest.approx(32_000_000, abs=2)
    points, _ = scan_geometry((raw, keys), (body_raw, keys))
    np.testing.assert_array_equal(points, body_raw)
    with pytest.raises(ValueError, match='transform'):
        scan_geometry((raw, keys), (base + .04, keys[:32]))


def test_deskew_uses_per_point_pose_and_rejects_pose_gaps():
    stamps = np.array([0, 25_000_000, 50_000_000])
    positions = np.array([[0., 0., 0.], [.02, 0., 0.], [.04, 0., 0.]])
    orientations = np.tile([0., 0., 0., 1.], (3, 1))
    orientations[1] *= -1
    query = np.array([10_000_000, 40_000_000])
    offset, rotation = interpolate_poses(stamps, positions, orientations, query)
    points = np.array([[1. - .008, 0., 0.], [1. - .032, 0., 0.]])
    np.testing.assert_allclose(rotate(rotation, points) + offset, [[1., 0., 0.]] * 2)
    with pytest.raises(ValueError, match='gap'):
        interpolate_poses(stamps * 3, positions, orientations, query)


def test_near_clearance_cloud_keeps_native_rays_only_with_covered_times_and_stable_origin():
    aligned = np.array([[-.8, .12, -.3], [.09, .01, -.35], [2.1, 0., -.3], [.4, 0., -.3]])
    seconds = np.array([.010, .040, .020, .070], dtype=np.float32)
    raw = (aligned @ ROTATION, seconds.view(np.uint32).astype(np.uint64))
    stamps = np.array([0, 25_000_000, 50_000_000])
    positions = np.zeros((3, 3))
    rotations = np.tile([0., 0., 0., 1.], (3, 1))
    args = (raw, 0, 0, stamps[-1], (stamps, positions, rotations))
    points = near_clearance_rays(*args, TRANSLATION, rotations[-1])
    np.testing.assert_allclose(points, aligned[:2], atol=1.e-8)

    positions[:, 0] = [0., .006, .014]
    assert near_clearance_rays(*args, TRANSLATION + positions[-1], rotations[-1]) is None
