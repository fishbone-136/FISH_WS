"""Point planning must not collapse the independent physical stop envelope."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from go2_navigation.safety_geometry import (configured_protection, protected_rectangle,
                                           rectangle_points, collision_monitor_parameters)


class SafetyGeometryTests(unittest.TestCase):
    def test_explicit_point_experiment_keeps_soft_inflation_and_disables_polygon_actions(self):
        root = Path(__file__).resolve().parents[2]
        config = root/'go2_navigation/config/nav2_params.yaml'
        document = yaml.safe_load(config.read_text())
        for name in ('local_costmap', 'global_costmap'):
            parameters = document[name][name]['ros__parameters']
            self.assertEqual(protected_rectangle(parameters, allow_point=True), (0., 0., 0.))
            self.assertEqual(parameters['inflation_layer']['plugin'], 'nav2_costmap_2d::InflationLayer')
            self.assertEqual(parameters['inflation_layer']['inflation_radius'], .30)
        self.assertNotIn('ObstacleFootprint',
                         document['controller_server']['ros__parameters']['FollowPath']['critics'])
        front, rear, half_width = configured_protection(config)
        self.assertEqual((front, rear, half_width), (0., 0., 0.))
        self.assertEqual(collision_monitor_parameters(config),
                         {'EmergencyStop.enabled': False, 'FootprintApproach.enabled': False})
        stop = yaml.safe_load((root/'go2_lio_navigation/config/collision_monitor.yaml').read_text())
        self.assertFalse(stop['collision_monitor']['ros__parameters']['EmergencyStop']['enabled'])
        self.assertFalse(stop['collision_monitor']['ros__parameters']['FootprintApproach']['enabled'])
        self.assertEqual(stop['collision_monitor']['ros__parameters']['FootprintApproach']['footprint_topic'],
                         '/safety/body_footprint')

    def test_switching_back_to_rectangle_restores_both_polygon_actions(self):
        root = Path(__file__).resolve().parents[2]
        document = yaml.safe_load((root/'go2_navigation/config/nav2_params.yaml').read_text())
        document['physical_safety']['ros__parameters']['collision_model'] = 'rectangle'
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder)/'params.yaml';config.write_text(yaml.safe_dump(document))
            self.assertEqual(configured_protection(config), (.38, .38, .15))
            self.assertEqual(collision_monitor_parameters(config), {
                'EmergencyStop.enabled': True, 'FootprintApproach.enabled': True,
                'EmergencyStop.points': rectangle_points(.38, .38, .15)})

    def test_point_planning_without_physical_safety_cannot_disable_stopping(self):
        p={'footprint':[[0.,0.]]*4, 'footprint_padding':0.}
        d={'local_costmap':{'local_costmap':{'ros__parameters':p}},
           'global_costmap':{'global_costmap':{'ros__parameters':p}}}
        with tempfile.TemporaryDirectory() as folder:
            f=Path(folder)/'params.yaml';f.write_text(yaml.safe_dump(d))
            with self.assertRaisesRegex(ValueError, 'physical_safety'):configured_protection(f)
            d['physical_safety']={'ros__parameters':p}
            f.write_text(yaml.safe_dump(d))
            with self.assertRaises(ValueError):configured_protection(f)

    def test_legacy_rectangle_profiles_keep_their_protection(self):
        p={'footprint':[[.38,.2],[.38,-.2],[-.38,-.2],[-.38,.2]],'footprint_padding':.05}
        d={'local_costmap':{'local_costmap':{'ros__parameters':p}},
           'global_costmap':{'global_costmap':{'ros__parameters':p}}}
        with tempfile.TemporaryDirectory() as folder:
            f=Path(folder)/'params.yaml';f.write_text(yaml.safe_dump(d))
            self.assertEqual(configured_protection(f), (.43,.43,.25))

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
