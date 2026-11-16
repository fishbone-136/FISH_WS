"""Exercise moving hazards and smooth recovery without any robot endpoint."""

import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from turn_clearance import braking_horizon, braking_speed_scale, turn_is_clear

spec = importlib.util.spec_from_file_location(
    'reactive_safety', Path(__file__).resolve().parents[1] / 'scripts/reactive_safety.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReactiveSafetyTests(unittest.TestCase):
    def test_resampling_a_static_slanted_surface_does_not_invent_approach(self):
        guard = module.ClosingApproach()
        for i in range(6):
            # Slide sampling locations along a stationary diagonal wall while
            # the robot moves forward. Longitudinal nearest-point differences
            # were consistently nonzero in this case despite ego compensation.
            shift = -.03*i
            points = [(2.+1.2*y-.03*i, y) for y in
                      (-.4+j*.08+shift for j in range(13))]
            guard.observe(points, 10.+i/15, (.45, 0., 0.))
            self.assertAlmostEqual(guard.closing, 0.)
            self.assertEqual(guard.scale(.18, 10.+i/15), 1.)

    def test_a_supported_moving_face_brakes_early_despite_ego_motion(self):
        guard = module.ClosingApproach()
        for i in range(3):
            points = [(1.80-.13*i, y) for y in (-.08, -.04, 0., .04, .08)]
            guard.observe(points, 10.+i/15, (.45, 0., 0.))
        self.assertGreater(guard.closing, 1.)
        self.assertLess(guard.scale(.45, 10.15), .35)

    def test_a_corner_or_surface_extrapolation_is_not_a_supported_normal_track(self):
        f = module.ClosingApproach.surface_rate
        self.assertIsNone(f([(1.,0.,0.), (1.,.1,0.), (1.1,0.,0.)], .9, 0., .1))
        self.assertIsNone(f([(1.,-.1,0.), (1.,0.,0.), (1.,.1,0.)], .9, .3, .1))

    def test_one_return_stops_at_any_edge_of_the_existing_emergency_rectangle(self):
        for point in ((.51, .33), (-.51, -.33), (0., -.33)):
            self.assertFalse(module.stop_zone_clear([point]))
        self.assertTrue(module.stop_zone_clear([(.52, 0.), (0., .34)]))

    def test_new_close_obstacle_brakes_on_its_first_frame(self):
        guard = module.ClosingApproach()
        guard.observe([(.90, 0.)], 10.)
        self.assertLess(guard.scale(.45, 10.), .8)
        guard.observe([(.66, 0.)], 10.1)
        self.assertEqual(guard.scale(.45, 10.1), 0.)

    def test_oncoming_obstacle_limits_speed_before_ego_only_approach(self):
        static, moving = module.ClosingApproach(), module.ClosingApproach()
        ego = (.45,0.,0.)
        for guard in (static, moving): guard.observe([(1.80, .3)], 10., ego)
        static.observe([(1.77, .3)], 10.+1/15, ego)
        moving.observe([(1.67, .3)], 10.+1/15, ego)
        static.observe([(1.74, .3)], 10.+2/15, ego)
        moving.observe([(1.54, .3)], 10.+2/15, ego)
        self.assertEqual(static.scale(.45, 10.15), 1.)
        self.assertLess(moving.scale(.45, 10.15), .35)

    def test_sample_age_is_deducted_from_contact_time(self):
        guard = module.ClosingApproach()
        guard.observe([(1., 0.)], 10.)
        self.assertLess(guard.scale(.45, 10.2), guard.scale(.45, 10.))

    def test_clear_corridor_resets_closing_and_does_not_slow_an_away_turn(self):
        guard = module.ClosingApproach()
        guard.observe([(1.8, 0.)], 10.)
        guard.observe([(1.6, 0.)], 10.1)
        guard.observe([(1.4, 0.)], 10.2)
        self.assertLess(guard.scale(.45, 10.2), 1.)
        self.assertEqual(guard.scale(0., 10.2), 1.)
        guard.observe([(1.6, .60)], 10.3)
        self.assertEqual(guard.scale(.45, 10.3), 1.)

    def test_side_return_switch_cannot_project_motion_onto_a_static_frontal_wall(self):
        guard = module.ClosingApproach()
        guard.observe([(1.8,0.), (1.,.48)], 10.)
        guard.observe([(1.8,0.), (.7,.48)], 10.+1/15)
        self.assertEqual(guard.scale(.45,10.+1/15),1.)

    def test_static_ego_motion_is_not_reclassified_when_recovery_command_is_small(self):
        guard = module.ClosingApproach()
        for i in range(4):
            guard.observe([(1.8-.03*i,0.)],10.+i/15, (.45,0.,0.))
        self.assertEqual(guard.closing,0.)
        self.assertEqual(guard.scale(.02,10.2),1.)

    def test_recovery_does_not_repeatedly_multiply_a_soft_approach_limit(self):
        guard = module.ClosingApproach()
        for i in range(3):
            guard.observe([(1.94-i/15,0.)],10.+i/15)
        now = 10.+2/15+.10
        target = .45*guard.scale(.45,now)
        self.assertLess(target,.45)
        ramp = module.SafetyRecoveryRamp()
        ramp.apply((.10,0.,0.),now,False)
        previous = .10
        for i in range(1,4):
            time = now+i*.02
            recovered = ramp.apply((target,0.,0.),time,True)
            ratio = guard.braking_scale(*recovered,time,.51,.33)
            checked = tuple(ratio*v for v in recovered)
            ramp.commit_checked(checked)
            self.assertGreaterEqual(checked[0],previous)
            previous = checked[0]
        self.assertGreater(previous,.14)

    def test_recovery_hard_check_still_stops_imminent_moving_contact(self):
        guard = module.ClosingApproach()
        guard.observe([(.9,0.)],10.)
        guard.observe([(.66,0.)],10.1)
        self.assertEqual(guard.braking_scale(.18,0.,0.,10.12,.51,.33),0.)

    def test_safety_stop_and_reduction_are_never_delayed_by_the_recovery_ramp(self):
        ramp = module.SafetyRecoveryRamp()
        ramp.apply((.45, 0., .9), 10., False)
        self.assertEqual(ramp.apply((.1, 0., .2), 10.02, True), (.1, 0., .2))
        self.assertEqual(ramp.apply((0., 0., 0.), 10.04, True), (0., 0., 0.))

    def test_recovery_is_acceleration_bounded_without_amplifying_any_axis(self):
        ramp = module.SafetyRecoveryRamp()
        ramp.apply((0., 0., 0.), 10., True)
        old = (0., 0., 0.)
        for i in range(1, 50):
            new = ramp.apply((.45, 0., .30), 10.+i*.02, False)
            self.assertLessEqual(new[0]-old[0], .7*.02+1.e-9)
            self.assertLessEqual(new[2]-old[2], 1.2*.02+1.e-9)
            self.assertLessEqual(new[0], .45)
            self.assertLessEqual(new[2], .30)
            old = new
        self.assertEqual(old, (.45, 0., .30))

    def test_turn_caution_uses_whole_protected_sweep_and_keeps_near_obstacles_slow(self):
        far=module.close_clearance_scale([(.90,0.)],0.,0.,1.2,.45,.27)
        close=module.close_clearance_scale([(.60,0.)],0.,0.,1.2,.45,.27)
        inside=module.close_clearance_scale([(.45,.28)],0.,0.,1.2,.45,.27)
        self.assertEqual(far,1.)
        self.assertAlmostEqual(close,.65/1.2)
        self.assertAlmostEqual(inside,.65/1.2)

    def test_yaw_reversal_during_caution_does_not_restart_the_forward_gait(self):
        ramp = module.SafetyRecoveryRamp()
        ramp.apply((.18, 0., .1), 10., False)
        result = ramp.apply((.18, 0., -.1), 10.02, True)
        self.assertAlmostEqual(result[0], .18)
        self.assertAlmostEqual(result[2], -.024)
        ramp.commit_checked((0., 0., 0.))
        result = ramp.apply((.18, 0., -.1), 10.04, True)
        self.assertLessEqual(result[0], .014000001)

    def test_clearance_speed_changes_continuously_at_the_old_binary_boundary(self):
        a = module.close_clearance_scale([(.6, .49999)], .45, 0., 0.)
        b = module.close_clearance_scale([(.6, .50001)], .45, 0., 0.)
        self.assertLess(abs(a-b), .0001)
        speeds = [module.close_clearance_scale([(.48, y)], .45, 0., 0.)
                  for y in (.40, .45, .50, .55, .60)]
        self.assertEqual(speeds, sorted(speeds))

    def test_relaxing_a_partial_dynamic_limit_is_ramped_too(self):
        ramp = module.SafetyRecoveryRamp()
        ramp.apply((.1, 0., 0.), 10., False)
        ramp.apply((.1, 0., 0.), 10.02, True)
        self.assertLessEqual(ramp.apply((.3, 0., 0.), 10.04, True)[0], .114000001)

    def test_idle_zero_does_not_forget_a_previous_safety_stop(self):
        ramp = module.SafetyRecoveryRamp()
        ramp.apply((0., 0., 0.), 10., True)
        ramp.apply((0., 0., 0.), 10.02, False)
        self.assertLessEqual(ramp.apply((.45, 0., 0.), 10.04, False)[0], .014000001)

    def test_long_callback_gap_cannot_bypass_recovery_acceleration(self):
        ramp = module.SafetyRecoveryRamp()
        ramp.apply((0., 0., 0.), 10., True)
        self.assertLessEqual(ramp.apply((.45, 0., .9), 15., False)[0], .035000001)

    def test_new_direction_is_bounded_and_invalid_commands_stop(self):
        ramp = module.SafetyRecoveryRamp()
        ramp.apply((.3, 0., .3), 10., False)
        result = ramp.apply((.3, 0., -.3), 10.02, True)
        self.assertLessEqual(abs(result[2]), .024000001)
        self.assertEqual(ramp.apply((float('nan'), 0., 0.), 10.04, False), (0., 0., 0.))

    def test_side_obstacle_does_not_fake_a_frontal_contact_on_an_away_arc(self):
        guard = module.ClosingApproach()
        guard.observe([(.515, .335)], 10.)
        self.assertEqual(guard.scale(.25, 10.05, wz=-.65), 1.)
        self.assertEqual(guard.scale(.25, 10.05, wz=.65), 0.)

    def test_narrow_corridor_is_slow_but_not_a_static_emergency(self):
        points = [(x, side*.36) for x in (.60, .80, 1.0) for side in (-1, 1)]
        self.assertTrue(module.stop_zone_clear(points))
        guard = module.ClosingApproach();guard.observe(points, 10.)
        self.assertEqual(guard.scale(.3, 10.05), 1.)
        safe_speed=.3*module.close_clearance_scale(points,.3,0.,0.)
        self.assertGreaterEqual(safe_speed,.22,'Soft caution must not prevent a stable gait')
        self.assertLess(safe_speed,.3)
        self.assertEqual(braking_speed_scale(points,safe_speed,0.,0.,.51,.33,.05), 1.)

    def test_braking_extent_grows_with_speed_and_retains_the_scan_age_margin(self):
        low = .51+.18*braking_horizon(.18,0.,0.,.08)
        high = .51+.55*braking_horizon(.55,0.,0.,.08)
        self.assertGreater(high, .66)
        self.assertGreater(high, low)

    def test_close_obstacle_in_the_protected_footprint_stops_every_arc(self):
        for vx,wz in ((.3,0.),(0.,.65),(0.,-.65),(.18,-.4)):
            self.assertEqual(braking_speed_scale([(.46,.1)],vx,0.,wz,.51,.33,.08),0.)

    def test_no_full_speed_turn_beyond_braking_distance_is_required(self):
        points=[(.4,.42)]
        self.assertFalse(turn_is_clear(points,0.,0.,.3,.51,.33,1.5))
        self.assertEqual(braking_speed_scale(points,0.,0.,.3,.51,.33,.03),1.)


if __name__ == '__main__': unittest.main()
