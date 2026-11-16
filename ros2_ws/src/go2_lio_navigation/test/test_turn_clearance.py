"""Regression cases for a single thin obstacle in a Go2 turning sweep."""

import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'turn_clearance', Path(__file__).resolve().parents[1] / 'scripts/turn_clearance.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TurnClearanceTests(unittest.TestCase):
    def test_explicit_zero_dimensions_do_not_restore_a_body_sweep(self):
        self.assertTrue(module.turn_is_clear([(.4,.1)], .3, 0., .4, 0., 0., 1.5))
        self.assertTrue(module.turn_is_clear([(.1,0.)], 0., 0., .65, 0., 0., 1.5))
        self.assertFalse(module.turn_is_clear([(0.,0.)], 0., 0., .65, 0., 0., 1.5))
        self.assertFalse(module.turn_is_clear([], .3, 0., 0., 0., -.1, 1.5))

    def clear(self, points, angular=.65, linear=0., lateral=0.):
        return module.turn_is_clear(points, linear, lateral, angular, .51, .33, 1.5)

    def test_one_point_outside_current_body_blocks_corner_sweep(self):
        # Outside even the new static stop rectangle: x<.58, y>.38.
        # A left turn brings this one point inside the front corner.
        self.assertFalse(self.clear([(.4, .43)]))
        # Turning away remains possible; a full-circle veto would block it.
        self.assertTrue(self.clear([(.4, .43)], angular=-.65))

    def test_sweep_is_symmetric_and_includes_rear_legs(self):
        self.assertFalse(self.clear([(.4, -.43)], angular=-.65))
        self.assertTrue(self.clear([(.4, -.43)], angular=.65))
        self.assertFalse(self.clear([(-.4, -.43)]))

    def test_arc_translation_is_checked_together_with_rotation(self):
        self.assertTrue(self.clear([(.7, .2)], linear=0., angular=.4))
        self.assertFalse(self.clear([(.7, .2)], linear=.3, angular=.4))
        self.assertFalse(self.clear([(.1, .6)], lateral=.3, angular=.1))

    def test_clear_scene_and_points_outside_swept_radius(self):
        self.assertTrue(self.clear([]))
        self.assertTrue(self.clear([(2., 0.), (0., 2.), (-2., 0.)]))

    def test_obstacle_already_inside_and_invalid_command_are_rejected(self):
        self.assertFalse(self.clear([(.49, .1)]))
        self.assertFalse(self.clear([], angular=float('nan')))


if __name__ == '__main__':
    unittest.main()
