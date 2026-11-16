"""Check terminal-action settling, acknowledgement and motion arbitration."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from idle_stand import IdleStand


class IdleStandTests(unittest.TestCase):
    def completed(self):
        guard = IdleStand()
        guard.navigation(True, 0.)
        guard.velocity(True, 1.)
        guard.navigation(False, 1.)
        return guard

    def test_idle_startup_and_historical_terminal_status_do_not_change_pose(self):
        g = IdleStand()
        g.velocity(True, 0.)
        g.navigation(False, 1.)
        self.assertIsNone(g.tick(10., True, 10))

    def test_an_active_goal_is_not_settled_during_a_safety_pause(self):
        g = IdleStand()
        g.navigation(True, 0.)
        g.velocity(True, 1.)
        self.assertIsNone(g.tick(10., True, 10))

    def test_stop_ack_then_stand_once_after_continuous_idle(self):
        g = self.completed()
        self.assertIsNone(g.tick(1.79, True, 1))
        self.assertEqual(g.tick(1.81, True, 1), 1003)
        self.assertIsNone(g.tick(1.9, True, 2))
        self.assertFalse(g.response(999, 1003, 0, 1.9))
        self.assertFalse(g.response(1, 1002, 0, 1.9))
        self.assertTrue(g.response(1, 1003, 0, 1.9))
        self.assertIsNone(g.tick(2.1, True, 2))
        self.assertEqual(g.tick(2.21, True, 2), 1002)
        self.assertTrue(g.response(2, 1002, 0, 2.3))
        for t in (3., 10., 100.):
            g.navigation(False, t)
            g.velocity(True, t)
            self.assertIsNone(g.tick(t, True, 3))

    def test_smoother_deceleration_after_terminal_does_not_lose_settle(self):
        g = self.completed()
        g.velocity(False, 1.1)
        self.assertIsNone(g.tick(2., True, 1))
        g.velocity(True, 2.)
        self.assertIsNone(g.tick(2.7, True, 1))
        self.assertEqual(g.tick(2.81, True, 1), 1003)

    def test_fresh_zero_and_stationary_feedback_are_required(self):
        g = self.completed()
        self.assertIsNone(g.tick(5., False, 1))
        self.assertEqual(g.tick(5.1, True, 1), 1003)

    def test_failure_or_missing_stop_ack_never_falls_through_to_stand(self):
        for code in (-1, 3101, None):
            with self.subTest(code=code):
                g = self.completed()
                self.assertEqual(g.tick(2., True, 1), 1003)
                if code is not None:
                    g.response(1, 1003, code, 2.1)
                for t in (4.1, 5., 10.):
                    self.assertIsNone(g.tick(t, True, 2))
                self.assertIsNotNone(g.error)

    def test_missing_stand_ack_is_not_retried(self):
        g = self.completed()
        g.tick(2., True, 1)
        g.response(1, 1003, 0, 2.1)
        self.assertEqual(g.tick(2.5, True, 2), 1002)
        self.assertIsNone(g.tick(5., True, 3))
        self.assertFalse(g.armed)

    def test_late_ack_cannot_bypass_the_timeout_between_timer_callbacks(self):
        g = self.completed()
        g.tick(2., True, 1)
        self.assertFalse(g.response(1, 1003, 0, 4.01))
        self.assertIsNone(g.tick(5., True, 2))

    def test_new_goal_invalidates_pending_and_late_ack(self):
        g = self.completed()
        g.tick(2., True, 1)
        g.navigation(True, 2.1)
        self.assertFalse(g.response(1, 1003, 0, 2.2))
        self.assertIsNone(g.tick(5., True, 2))
        g.navigation(False, 5.)
        self.assertEqual(g.tick(5.81, True, 3), 1003)

    def test_resumed_motion_invalidates_pending_and_requires_new_idle_interval(self):
        g = self.completed()
        g.tick(2., True, 1)
        g.velocity(False, 2.1)
        self.assertFalse(g.response(1, 1003, 0, 2.2))
        g.velocity(True, 3.)
        self.assertIsNone(g.tick(3.7, True, 2))
        self.assertEqual(g.tick(3.81, True, 2), 1003)

    def test_interactive_override_disarms_until_a_new_navigation_episode(self):
        g = self.completed()
        g.tick(2., True, 1)
        g.suspend()
        self.assertFalse(g.response(1, 1003, 0, 2.1))
        self.assertIsNone(g.tick(10., True, 2))
        g.navigation(True, 11.)
        g.navigation(False, 12.)
        self.assertEqual(g.tick(12.81, True, 3), 1003)


if __name__ == '__main__':
    unittest.main()
