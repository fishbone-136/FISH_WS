"""Regression checks for persistent metric environment-map construction."""

import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

source = Path(__file__).resolve().parents[1] / 'scripts' / 'environment_grid.py'
spec = importlib.util.spec_from_file_location('environment_grid', source)
grid = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grid)


class EnvironmentGridTests(unittest.TestCase):
    def test_noisy_floor_stays_traversable_while_true_steep_slope_is_blocked(self):
        def points(degrees=0.,noise=False):
            values=[]
            for y in range(11):
                for x in range(11):
                    z=.08*x*np.tan(np.radians(degrees))
                    if noise and x==5 and y==5:z+=.05
                    values.extend([[x*.08+.01,y*.08+.01,z],[x*.08+.02,y*.08+.02,z]])
            return np.array(values)
        flat=grid.build_layers(points(noise=True),resolution=.08)
        self.assertLess(flat['surface_costs'].max(),90)
        steep=grid.build_layers(points(35.),resolution=.08)
        self.assertGreater(np.count_nonzero(steep['surface_costs']==100),60)

    def test_confirmed_clear_volume_preserves_floor_neighbour_and_unobserved_height(self):
        points=np.array([[1.,0.,-.37],[1.,0.,-.20],[1.,0.,.30],
                         [1.,.10,-.20],[3.,0.,-.20]])
        removed=grid.cleared_point_mask(points,[[1.,0.,-.25,.10,.04]])
        np.testing.assert_array_equal(removed,[False,True,False,False,False])
        self.assertFalse(grid.cleared_point_mask(points,[]).any())

    def test_stable_bounds_do_not_keep_old_obstacle_cells(self):
        points = np.array([[0.02, 0.02, 0], [0.04, 0.04, 0],
                           [2.02, 0.02, 0], [2.04, 0.04, 0]])
        before = grid.build_layers(points)
        bounds = np.r_[before['origin'], before['origin'] +
                       before['resolution'] * np.array([before['width'], before['height']])]
        after = grid.build_layers(points[:2], previous_bounds=bounds)
        self.assertEqual((before['width'], before['height']),
                         (after['width'], after['height']))
        np.testing.assert_allclose(before['origin'], after['origin'])
        self.assertEqual(np.count_nonzero(after['costs'] >= 0), 1)

    def test_flat_floor_has_metric_height_and_zero_cost(self):
        floor = np.array([[x + dx, y + dy, -0.78]
                          for x in [0, 0.2, 0.4] for y in [0, 0.2, 0.4]
                          for dx, dy in [(0.02, 0.02), (0.04, 0.04)]])
        layers = grid.build_layers(floor, resolution=0.2)
        self.assertEqual(len(layers['surface']), 9)
        np.testing.assert_allclose(layers['surface'][:, 2], -0.78)
        np.testing.assert_array_equal(layers['surface_costs'], 0)
        self.assertTrue(np.isnan(layers['elevation']).any())
        self.assertTrue((layers['costs'] == -1).any())

    def test_wall_vertical_extent_becomes_high_cost(self):
        wall = np.array([[0.02, 0.02, -0.78], [0.04, 0.04, -0.1]])
        layers = grid.build_layers(wall)
        self.assertEqual(layers['surface_costs'][0], 100)

    def test_step_marks_both_sides(self):
        points = np.array([[0.02, 0.02, 0], [0.04, 0.04, 0],
                           [0.22, 0.02, 0.3], [0.24, 0.04, 0.3]])
        layers = grid.build_layers(points, resolution=0.2)
        np.testing.assert_array_equal(layers['surface_costs'], [100, 100])

    def test_history_stays_when_live_scan_moves_away(self):
        old = np.array([[0.02, 0.02, 0], [0.04, 0.04, 0]])
        new = old + np.array([2, 0, 0])
        layers = grid.build_layers(np.concatenate([old, new]))
        self.assertEqual(len(layers['surface']), 2)
        self.assertGreater(layers['surface'][:, 0].ptp(), 1.5)

    def test_pose_correction_rebuild_does_not_leave_old_ghost(self):
        scan = np.array([[0.02, 0.02, 0], [0.04, 0.04, 0]])
        before = grid.pose_matrix([2, 0, 0, 0, 0, 0, 1])
        after = grid.pose_matrix([1, 0, 0, 0, 0, 0, 1])
        old = grid.build_layers(grid.transform_points(scan, before))
        corrected = grid.build_layers(grid.transform_points(scan, after))
        self.assertLess(corrected['surface'][:, 0].max(), old['surface'][:, 0].min())
        self.assertEqual(len(corrected['surface']), 1)

    def test_quaternion_is_normalized(self):
        transform = grid.pose_matrix([0, 0, 0, 0, 0, 2, 2])
        result = grid.transform_points(np.array([[1, 0, 0]]), transform)
        np.testing.assert_allclose(result, [[0, 1, 0]], atol=1.e-6)
        with self.assertRaises(ValueError):
            grid.pose_matrix([0, 0, 0, 0, 0, 0, 0])

    def test_voxel_filter_and_invalid_points(self):
        points = np.array([[0.01, 0.02, 0.03], [0.02, 0.03, 0.04], [np.nan, 0, 0]])
        self.assertEqual(len(grid.voxel_filter(points, 0.1)), 1)
        self.assertIsNone(grid.build_layers(np.array([[np.inf, 0, 0]])))

    def test_packed_voxels_preserve_first_points_and_negative_coordinate_order(self):
        random = np.random.default_rng(42)
        for points, leaf in [(random.uniform(-8, 8, (10000, 3)), 0.15),
                             (np.array([[-1.e6, 0, 0], [1.e6, 1.e6, 1.e6],
                                        [-1.e6, 0, 0]]), 1.e-4)]:
            points = np.asarray(points, dtype=np.float32)
            keys = np.floor(points / leaf).astype(np.int64)
            _, indices = np.unique(keys, axis=0, return_index=True)
            expected = points[np.sort(indices)]
            np.testing.assert_array_equal(grid.voxel_filter(points, leaf), expected)

    def test_reference_transform_precision_at_map_extent(self):
        points = np.random.default_rng(5).uniform(-60, 60, (1000, 3)).astype(np.float32)
        matrix = grid.pose_matrix([30, -30, 1, 0.02, 0.1, 0.3, 0.94])
        expected = points @ matrix[:3, :3].T + matrix[:3, 3]
        result = grid.transform_points(points, matrix)
        self.assertEqual(result.dtype, np.float32)
        np.testing.assert_allclose(result, expected, atol=2.e-5)

    def test_extent_bounds_memory_and_npz_preserves_unknown(self):
        points = np.array([[-100, 0, 0], [-99.99, 0.01, 0],
                           [100, 0, 0], [100.01, 0.01, 0]])
        layers = grid.build_layers(points, max_extent=6.0)
        self.assertTrue(layers['clipped'])
        self.assertLessEqual(layers['width'], 41)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'elevation.npz'
            np.savez_compressed(path, elevation_m=layers['elevation'])
            with np.load(path) as saved:
                np.testing.assert_array_equal(saved['elevation_m'], layers['elevation'])


if __name__ == '__main__':
    unittest.main()
