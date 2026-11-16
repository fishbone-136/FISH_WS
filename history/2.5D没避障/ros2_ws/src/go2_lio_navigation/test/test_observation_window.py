"""Ensure sparse scans do not create stop pulses or keep stale coverage alive."""

import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'observation_window', Path(__file__).resolve().parents[1] / 'scripts/observation_window.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ForwardObservationTests(unittest.TestCase):
    def test_startup_has_no_fabricated_forward_coverage(self):
        window = module.ForwardObservationWindow()
        window.observe(0, 10., 20.)
        self.assertFalse(window.fresh(10., 20., .35))

    def test_one_sparse_frame_preserves_recent_real_coverage(self):
        window = module.ForwardObservationWindow()
        window.observe(2, 10., 20.)
        window.observe(0, 10.1, 20.1)
        self.assertTrue(window.fresh(10.1, 20.1, .35))
        window.observe(1, 10.2, 20.2)
        self.assertTrue(window.fresh(10.2, 20.2, .35))

    def test_sparse_frames_never_refresh_forward_observation_deadline(self):
        window = module.ForwardObservationWindow()
        window.observe(2, 10., 20.)
        for step in range(1, 8):
            window.observe(0, 10.+step*.1, 20.+step*.1)
        self.assertFalse(window.fresh(10.351, 20.351, .35))

    def test_stale_sensor_time_and_stale_receipt_each_block(self):
        window = module.ForwardObservationWindow()
        window.observe(2, 10., 20.)
        self.assertFalse(window.fresh(10.36, 20.1, .35))
        self.assertFalse(window.fresh(10.1, 20.36, .35))

    def test_wrong_frame_revokes_coverage_and_new_good_observation_recovers(self):
        window = module.ForwardObservationWindow()
        window.observe(2, 10., 20.)
        window.observe(2, 10.1, 20.1, supported_frame=False)
        self.assertFalse(window.fresh(10.1, 20.1, .35))
        window.observe(2, 10.2, 20.2)
        self.assertTrue(window.fresh(10.2, 20.2, .35))

    def test_sparse_scan_cannot_release_previous_stop_or_start_a_new_turn(self):
        self.assertEqual(module.bound_sparse_command((.45, 0., 0.), (0., 0., 0.)), (0., 0., 0.))
        self.assertEqual(module.bound_sparse_command((.3, 0., .5), (.3, 0., 0.)), (.3, 0., 0.))
        self.assertEqual(module.bound_sparse_command((.3, 0., -.5), (.3, 0., .5)), (.3, 0., 0.))

    def test_sparse_scan_retains_motion_without_acceleration_or_curvature_change(self):
        self.assertEqual(module.bound_sparse_command((.3, 0., .2), (.3, 0., .2)), (.3, 0., .2))
        limited = module.bound_sparse_command((.45, 0., .4), (.3, 0., .2))
        self.assertAlmostEqual(limited[0], .225)
        self.assertAlmostEqual(limited[2], .2)
        self.assertAlmostEqual(limited[0]/limited[2], .45/.4)
        self.assertEqual(module.bound_sparse_command((0., 0., 0.), (.3, 0., .2)), (0., 0., 0.))


if __name__ == '__main__':
    unittest.main()
