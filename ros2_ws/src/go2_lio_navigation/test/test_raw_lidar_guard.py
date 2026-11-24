"""Regression: a fresh empty/occluded scan must never authorize walking."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from raw_lidar_guard import RawLidarGuard
from background_budget import BackgroundBudget

CLEAR = [[2., 0., 0.], [2., .05, .1], [2., -.05, -.1]]


class RawLidarGuardTests(unittest.TestCase):
    def ready(self):
        guard = RawLidarGuard()
        for i in range(3):
            guard.observe(CLEAR, 10.+i*.05, 20.+i*.05)
        self.assertEqual(guard.reason(10.1, 20.1), 'none')
        return guard

    def test_startup_requires_real_observations_and_recovery(self):
        guard = RawLidarGuard()
        self.assertEqual(guard.reason(10., 20.), 'raw_lidar_unavailable')
        guard.observe(CLEAR, 10., 20.)
        self.assertEqual(guard.reason(10., 20.), 'raw_lidar_recovery')
        self.ready()

    def test_full_cover_zero_placeholders_stops_on_first_packet(self):
        guard = self.ready()
        guard.observe(np.zeros((20000, 3)), 10.15, 20.15)
        self.assertEqual(guard.reason(10.15, 20.15), 'raw_lidar_blind')

    def test_empty_or_nonfinite_packet_cannot_be_free_space(self):
        for points in ([], [[np.nan, 0., 0.]], [[np.inf, 0., 0.]]):
            guard = self.ready()
            guard.observe(points, 10.15, 20.15)
            self.assertEqual(guard.reason(10.15, 20.15), 'raw_lidar_blind')

    def test_occluder_inside_projection_crop_stops_before_filtering(self):
        for x in (.01, .03, .05, .15, .28):
            guard = self.ready()
            guard.observe(CLEAR+[[x, 0., 0.]], 10.15, 20.15)
            self.assertEqual(guard.reason(10.15, 20.15), 'raw_lidar_near_contact')

    def test_single_near_return_stops_even_with_many_clear_returns(self):
        guard = self.ready()
        guard.observe(CLEAR*100+[[.2, .02, 0.]], 10.15, 20.15)
        self.assertEqual(guard.near_returns, 1)
        self.assertEqual(guard.reason(10.15, 20.15), 'raw_lidar_near_contact')

    def test_floor_and_side_returns_do_not_become_near_contact(self):
        guard = self.ready()
        guard.observe(CLEAR+[[.2, 0., -.5], [.2, .3, 0.]], 10.15, 20.15)
        self.assertEqual(guard.reason(10.15, 20.15), 'none')

    def test_stale_receipt_and_sensor_time_independently_stop(self):
        guard = self.ready()
        self.assertEqual(guard.reason(10.36, 20.12), 'raw_lidar_unavailable')
        guard = self.ready()
        self.assertEqual(guard.reason(10.12, 20.26), 'raw_lidar_unavailable')

    def test_wrong_frame_and_duplicate_packet_revoke_permission(self):
        for supported, stamp in ((False, 10.15), (True, 10.1)):
            guard = self.ready()
            guard.observe(CLEAR, stamp, 20.15, supported)
            self.assertEqual(guard.reason(stamp, 20.15), 'raw_lidar_invalid')

    def test_recovery_cannot_release_on_one_good_frame_or_callback_burst(self):
        guard = self.ready()
        guard.observe([], 10.15, 20.15)
        for i in range(3):
            guard.observe(CLEAR, 10.2+i*.05, 20.2+i*.001)
        self.assertEqual(guard.reason(10.3, 20.202), 'raw_lidar_recovery')
        guard.observe(CLEAR, 10.35, 20.31)
        self.assertEqual(guard.reason(10.35, 20.31), 'none')

    def test_side_and_behind_sensor_returns_do_not_certify_forward(self):
        guard = self.ready()
        guard.observe([[0., 1., 0.], [-1., 0., 0.], [1., 1., 0.]], 10.15, 20.15)
        self.assertEqual(guard.reason(10.15, 20.15), 'raw_lidar_blind')


class BackgroundBudgetTests(unittest.TestCase):
    def test_load_hysteresis_preserves_spare_cpu(self):
        budget = BackgroundBudget()
        budget.update(100, 50)
        budget.update(200, 60)
        self.assertTrue(budget.overloaded)
        budget.update(300, 80)
        self.assertTrue(budget.overloaded)
        budget.update(400, 110)
        self.assertFalse(budget.overloaded)

    def test_background_work_backs_off_without_queueing_jobs(self):
        budget = BackgroundBudget()
        budget.last_sample = 10.
        self.assertTrue(budget.due('maps', 12., 6., 10.))
        self.assertFalse(budget.due('maps', 12., 6., 10.04))
        self.assertTrue(budget.due('maps', 12., 6., 10.09))
        budget.overloaded = True
        self.assertFalse(budget.due('maps', 12., 6., 10.18))
        self.assertTrue(budget.due('maps', 12., 6., 10.27))

    def test_map_polling_reaches_the_budget_without_catchup_bursts(self):
        for loaded, rate in ((False, 15.), (True, 8.)):
            budget = BackgroundBudget()
            budget.last_sample = 100.
            budget.overloaded = loaded
            interval = .025
            ticks = [i * interval for i in range(200)]
            updates = [now for now in ticks
                       if budget.due('maps', 15., 8., now, poll_interval=interval)]
            self.assertLessEqual(abs(len(updates) - 5 * rate), 1)
            self.assertTrue(budget.due('maps', 15., 8., 10., poll_interval=interval))
            self.assertFalse(budget.due('maps', 15., 8., 10.025, poll_interval=interval))
