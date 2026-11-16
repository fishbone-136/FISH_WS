"""Check shared physical-envelope geometry and matching emergency protection."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_navigation.safety_geometry import configured_protection, protected_rectangle, rectangle_points


class SafetyGeometryTests(unittest.TestCase):
    def test_compact_configuration_preserves_nominal_body_and_matches_stop_polygon(self):
        root = Path(__file__).resolve().parents[2]
        config = root/'go2_navigation/config/nav2_params.yaml'
        local = yaml.safe_load(config.read_text())['local_costmap']['local_costmap']['ros__parameters']
        self.assertEqual(yaml.safe_load(local['footprint']),
                         [[.38,.20],[.38,-.20],[-.38,-.20],[-.38,.20]])
        front, rear, half_width = configured_protection(config)
        self.assertAlmostEqual(front+rear, .90)
        self.assertAlmostEqual(2*half_width, .54)
        self.assertGreaterEqual(local['inflation_layer']['inflation_radius'], half_width)
        stop = yaml.safe_load((root/'go2_lio_navigation/config/collision_monitor.yaml').read_text())
        self.assertEqual(stop['collision_monitor']['ros__parameters']['EmergencyStop']['points'],
                         rectangle_points(front, rear, half_width))

    def test_asymmetric_body_is_bounded_without_shortening_any_side(self):
        p={'footprint':[[.4,.2],[.4,-.3],[-.3,-.3],[-.3,.2]],'footprint_padding':.07}
        before=copy.deepcopy(p)
        self.assertEqual(protected_rectangle(p),(.47,.37,.37))
        self.assertEqual(p,before)

    def test_invalid_or_negative_margin_cannot_silently_shrink_the_body(self):
        for pad in (-.01, float('nan'), float('inf')):
            with self.subTest(padding=pad), self.assertRaises(ValueError):
                protected_rectangle({'footprint':[[.38,.2],[.38,-.2],[-.38,-.2],[-.38,.2]],
                                     'footprint_padding':pad})

    def test_mismatched_local_and_global_envelopes_fail_closed(self):
        p={'footprint':[[.38,.2],[.38,-.2],[-.38,-.2],[-.38,.2]],'footprint_padding':.07}
        other=copy.deepcopy(p);other['footprint_padding']=.13
        d={'local_costmap':{'local_costmap':{'ros__parameters':p}},
           'global_costmap':{'global_costmap':{'ros__parameters':other}}}
        with tempfile.TemporaryDirectory() as folder:
            f=Path(folder)/'params.yaml';f.write_text(yaml.safe_dump(d))
            with self.assertRaises(ValueError):configured_protection(f)


if __name__ == '__main__':unittest.main()
