"""Protection geometry can be reused; age and source changes must not be hidden."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from reactive_safety import ObstacleGeometryCache, motion_stop_reason


class GeometryCacheTests(unittest.TestCase):
    def test_reuses_only_identical_sources_and_preserves_a_new_thin_obstacle(self):
        cache = ObstacleGeometryCache()
        scan, memory, pose = [(1., .4)], np.array([[.5, .3]]), (0., 0., 0.)
        first = cache.update(scan, memory, pose, True)
        second = cache.update(scan, memory, pose, True)
        self.assertIs(first[0], second[0])
        self.assertIs(first[1], second[1])
        changed = cache.update([(.44, .1)], memory, pose, True)
        self.assertIn((.44, .1), changed[1])
        changed = cache.update(scan, np.array([[.4, .2]]), pose, True)
        self.assertEqual(changed[0], [[.4, .2]])

    def test_pose_change_reprojects_memory_and_frame_loss_does_not_reuse_it(self):
        cache = ObstacleGeometryCache()
        scan, memory = [], np.array([[1., 0.], [3., 0.]])
        self.assertEqual(cache.update(scan, memory, (0., 0., 0.), True)[0], [[1., 0.]])
        points, _ = cache.update(scan, memory, (1., 0., np.pi / 2), True)
        self.assertAlmostEqual(points[0][0], 0.)
        self.assertAlmostEqual(points[1][1], -2.)
        self.assertEqual(cache.update(scan, memory, (1., 0., 0.), False), ([], []))

    def test_freshness_blockers_are_evaluated_even_with_unchanged_geometry(self):
        good = [True] * 7
        self.assertEqual(motion_stop_reason(*good), 'none')
        for i, expected in enumerate(('scan_unavailable', 'footprint_unavailable',
                                      'memory_unavailable', 'command_timeout', 'invalid_command',
                                      'occupied_footprint', 'braking_arc_blocked')):
            conditions = good.copy()
            conditions[i] = False
            self.assertEqual(motion_stop_reason(*conditions), expected)

    def test_numeric_geometry_is_reused_and_a_new_obstacle_invalidates_it(self):
        cache=ObstacleGeometryCache()
        scan=[(1.,.4)];memory=np.array([[.5,.3]])
        pose=(0.,0.,0.)
        cache.update(scan,memory,pose,True)
        previous=cache.combined_array
        np.testing.assert_array_equal(previous,cache.combined)
        cache.update(scan,memory,pose,True)
        self.assertIs(cache.combined_array,previous)
        cache.update([(.2,.1)],memory,pose,True)
        self.assertIsNot(cache.combined_array,previous)
        np.testing.assert_array_equal(cache.combined_array,[[.2,.1],[.5,.3]])
        cache.update([],memory,pose,False)
        self.assertEqual(cache.combined_array.shape,(0,2))


if __name__ == '__main__':
    unittest.main()
