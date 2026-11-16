import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'stance', Path(__file__).resolve().parents[1] / 'scripts/check_navigation_stance.py')
stance = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stance)


class StanceTests(unittest.TestCase):
    def test_measured_lying_height_and_invalid_feedback_are_rejected(self):
        self.assertIsNotNone(stance.stance_error(.072, 0., 0., 0.))
        for x in (float('nan'), float('inf')):
            self.assertIsNotNone(stance.stance_error(x, 0., 0., 0.))
            self.assertIsNotNone(stance.stance_error(.27, x, 0., 0.))

    def test_motion_and_turning_prevent_stationary_initialization(self):
        self.assertIsNotNone(stance.stance_error(.27, .05, .05, 0.))
        self.assertIsNotNone(stance.stance_error(.27, 0., 0., .11))
        self.assertIsNone(stance.stance_error(.27, .01, -.01, .01))

    def test_one_packet_cannot_satisfy_stable_duration(self):
        s = stance.StableStance()
        s.observe(10., .27, 0., 0., 0.)
        self.assertFalse(s.ready(11.6))
        s.observe(11.6, .27, 0., 0., 0.)
        self.assertFalse(s.ready(11.6))

    def test_stable_standing_succeeds_but_stale_feedback_does_not(self):
        s = stance.StableStance()
        for i in range(17):
            s.observe(10.+i*.1, .27, 0., 0., 0.)
        self.assertTrue(s.ready(11.6))
        self.assertFalse(s.ready(12.))

    def test_lying_or_motion_resets_the_stable_window(self):
        for values in ((.072, 0., 0., 0.), (.27, .2, 0., 0.)):
            s = stance.StableStance()
            for i in range(17):
                s.observe(10.+i*.1, .27, 0., 0., 0.)
            s.observe(11.7, *values)
            self.assertFalse(s.ready(11.7))
            s.observe(11.8, .27, 0., 0., 0.)
            self.assertFalse(s.ready(11.8))


if __name__ == '__main__':
    unittest.main()
