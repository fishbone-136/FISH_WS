"""Regression for seen box -> blind rear -> turned front with missing returns."""
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from obstacle_memory_model import (ObstacleMemory,RayEvidence,navigation_grid,clearing_delta,
                                   reference_clear_volumes)
import numpy as np


class ObstacleMemoryTests(unittest.TestCase):
    def test_near_sensor_returns_can_support_obstacle_history(self):
        # A real return 6 cm ahead of HAP must not disappear behind the old
        # 25 cm ray-evidence cutoff; the sensor origin itself is not a return.
        rays = RayEvidence([[.14, 0., .13], [.20, 0., .20]], [.14, 0., .13])
        self.assertEqual(len(rays.points), 1)
        np.testing.assert_allclose(rays.points[0], [.20, 0., .20])

    def test_near_history_uses_real_crossing_rays_without_return_range_cutoff(self):
        origin=[.14,0.,.13]
        far=RayEvidence([[3.,0.,-.37]],origin)
        self.assertTrue(far.clears(.38,0.,-.25,.30))
        self.assertTrue(far.clears_many([[.38,0.]],[-.25],[.30])[0])
        self.assertTrue(far.sees_beyond([[.38,0.]])[0])
        # No return, the wrong height, and an actual closer occluder cannot
        # clear the same near history merely because it has become old.
        self.assertFalse(RayEvidence([],origin).clears(.38,0.,-.25,.30))
        self.assertFalse(far.clears(.38,0.,-.28,-.23))
        blocked=RayEvidence([[.40,0.,.10],[3.,0.,-.37]],origin)
        self.assertFalse(blocked.clears(.38,0.,-.25,.30))

    def test_last_foot_return_keeps_continuously_measured_body_height(self):
        m=ObstacleMemory(require_3d_clearing=True,max_observation_gap=.60)
        m.mark_xyz([[.39,0.,-.25],[.39,0.,.30]],(0.,0.,0.),10.)
        m.mark_xyz([[.39,0.,-.26]],(0.,0.,0.),10.2)
        cell=next(iter(m.cells.values()))
        self.assertLessEqual(cell[5],-.26)
        self.assertGreaterEqual(cell[6],.30)
        # A farther ground return crosses the formerly observed body volume
        # but cannot cross a standalone low foot slice near the sensor.
        rays=RayEvidence([[3.,0.,-.37]],[.14,0.,.13])
        for i in range(8):
            snapshot=m.cells.copy();keys=list(snapshot)
            geometry=np.array([[np.nan if v is None else v for v in row]
                               for row in snapshot.values()]).reshape(-1,7)
            stamp=11.+i*.2
            updates=clearing_delta(geometry,rays,(0.,0.,0.),stamp,stamp-.2)
            m.merge_delta(snapshot,keys,*updates,stamp)
        self.assertEqual(m.points(),[])

    def test_height_history_does_not_join_objects_after_an_observation_gap(self):
        m=ObstacleMemory(max_observation_gap=.60)
        m.mark_xyz([[1.,0.,-.25],[1.,0.,.8]],(0.,0.,0.),10.)
        m.mark_xyz([[1.,0.,.1]],(0.,0.,0.),11.)
        np.testing.assert_allclose(next(iter(m.cells.values()))[5:],[.075,.125])

    def test_clearance_keeps_thin_leg_and_shadow_protection(self):
        rays=RayEvidence([[3.,.045,.25]],[0.,0.,.25])
        self.assertTrue(rays.clears(1.,0.,.20,.30))
        self.assertFalse(rays.clears(1.,.20,.20,.30))
        np.testing.assert_array_equal(rays.clears_many([[1.,0.],[1.,.20]], [.2,.2],[.3,.3]),
                                      [True,False])
        beside_leg=RayEvidence([[3.,.09,.25]],[0.,0.,.25])
        self.assertFalse(beside_leg.clears(1.,0.,.20,.30))
        occluded=RayEvidence([[.5,0.,.25],[3.,.045,.25]],[0.,0.,.25])
        self.assertFalse(occluded.clears(1.,0.,.20,.30))
        self.assertFalse(occluded.clears_many([[1.,0.]],[.2],[.3])[0])

    def test_angular_prefilter_keeps_support_beside_ray_and_across_angle_wrap(self):
        rays = RayEvidence([[1., .023, .25], [-1., .012, .25], [3., 0., .25]],
                           [0., 0., .25])
        free, support = rays.classify_many([[1., 0.], [-1., -.01], [0., 1.]],
                                           [.20] * 3, [.30] * 3)
        np.testing.assert_array_equal(support, [True, True, False])
        np.testing.assert_array_equal(free, [False, False, False])

    def test_shared_reference_clearance_never_includes_the_floor(self):
        rows=[[1.,0.,10.,None,0,-.395,-.345],[1.,0.,10.,None,0,-.26,.30]]
        volumes=reference_clear_volumes(rows,[0.,0.,1.,.37])
        self.assertEqual(volumes.shape,(1,5))
        self.assertGreater(volumes[0,2],-.29)
        self.assertEqual(len(reference_clear_volumes(rows,None)),0)

    def test_only_successfully_merged_removal_is_shared_with_reference(self):
        m=ObstacleMemory();m.mark_xyz([[1.,0.,.2],[2.,0.,.2]],(0.,0.,0.),10.)
        snapshot=m.cells.copy();keys=list(snapshot)
        m.mark_xyz([[2.,0.,.3]],(0.,0.,0.),11.)
        removed,removed_keys=[],[]
        count=m.merge_delta(snapshot,keys,[0,1],[[np.nan,8],[np.nan,8]],
                            [True,True],10.5,cleared_cells=removed,cleared_keys=removed_keys)
        self.assertEqual(count,1);self.assertEqual(len(removed),1)
        self.assertEqual(removed[0][:2],(1.,0.))
        self.assertEqual(removed_keys,[keys[0]])
        self.assertIn((2.,0.),m.points())

    def test_genuine_geometry_never_records_artificial_scan_spread_inside_the_footprint(self):
        m=ObstacleMemory();m.mark_xyz([[.40,.29,-.20],[.40,.30,.30]],(0.,0.,0.),10.)
        self.assertTrue(all(y>=.29 for x,y in m.points()))
        self.assertTrue(all(not(abs(x)<=.45 and abs(y)<=.27) for x,y in m.points()))
        self.assertTrue(all(c[5] is not None and c[6] is not None for c in m.cells.values()))

    def test_genuine_points_acquire_height_and_choose_an_actual_near_endpoint(self):
        m=ObstacleMemory();m.mark_xyz([[1.01,0.,-.2],[1.04,.01,.3]],(0.,0.,0.),10.)
        self.assertEqual(m.points(),[(1.01,0.)]);self.assertEqual(next(iter(m.cells.values()))[5:],(-.2,.3))

    def test_refining_grid_preserves_overlapping_source_obstacles_and_unknown(self):
        grid,res=navigation_grid([100,0,-1,50],2,2,.10,.04)
        self.assertEqual(grid.shape,(5,5));self.assertEqual(res,.04)
        # The output cell [0.08,0.12] overlaps the old obstacle: centre-only
        # sampling would have incorrectly erased that occupied edge.
        self.assertEqual(grid[0,2],100)
        self.assertEqual(grid[4,0],-1)
        self.assertEqual(grid[4,4],50)

    def test_memory_point_uses_the_navigation_resolution_without_coarse_reinflation(self):
        grid,res=navigation_grid([0]*10000,100,100,.10,.04)
        x,y=.40,.29;ix=int((x+5.)/res);iy=int((y+5.)/res)
        grid[iy,ix]=100
        self.assertEqual(grid[int((.26+5.)/res),ix],0)
        self.assertEqual(grid[iy,ix],100)

    def test_delayed_clear_cannot_erase_a_newer_seen_obstacle(self):
        m=ObstacleMemory();self.observe(m,[1.],10.)
        snapshot=m.cells.copy()
        m.mark([1.],0.,.01,.25,5.5,(0.,0.,0.),10.1)
        m.merge_clearing(snapshot,{},10.)
        self.assertIn((1.,0.),m.points())

    def test_delayed_clear_never_rewinds_an_occupied_cell_position(self):
        m=ObstacleMemory();self.observe(m,[1.01],10.)
        snapshot=m.cells.copy()
        m.mark([1.04],0.,.01,.25,5.5,(0.,0.,0.),10.1)
        result={k:(*v[:5],-.2,.4) for k,v in snapshot.items()}
        m.merge_clearing(snapshot,result,10.)
        self.assertEqual(m.points(),[(1.04,0.)])
        self.assertEqual(next(iter(m.cells.values()))[5:],(-.2,.4))

    def test_confirmed_clear_applies_only_to_unchanged_old_cells(self):
        m=ObstacleMemory();self.observe(m,[1.],10.)
        snapshot=m.cells.copy();m.merge_clearing(snapshot,{},10.1)
        self.assertEqual(m.points(),[]);self.assertEqual(m.cleared,1)

    def test_fast_observations_do_not_erase_history_when_raw_is_missing(self):
        m=ObstacleMemory();m.mark([1.],0.,.01,.25,5.5,(0.,0.,0.),10.)
        for i in range(50):m.mark([math.inf],0.,.01,.25,5.5,(0.,0.,0.),11.+i/15)
        self.assertIn((1.,0.),m.points())

    def test_batch_ray_evidence_matches_scalar_height_and_occlusion_rules(self):
        rng=np.random.default_rng(741)
        points=rng.uniform([-4.,-4.,-.5],[4.,4.,.8],size=(5000,3))
        rays=RayEvidence(points,[0.,0.,.13])
        xy=rng.uniform(-4.,4.,size=(250,2))
        low=rng.uniform(-.4,.1,250);high=low+rng.uniform(.05,.6,250)
        scalar=[rays.clears(x,y,a,b) for (x,y),a,b in zip(xy,low,high)]
        np.testing.assert_array_equal(rays.clears_many(xy,low,high),scalar)
        self.assertEqual(rays.bounds_many(xy),[rays.bounds(x,y) for x,y in xy])

    def observe(self,m,ranges,stamp,pose=(0.,0.,0.)):
        m.observe(ranges,0.,.01,.25,5.5,pose,stamp)

    def test_nan_ground_and_time_passage_never_erase_a_seen_box(self):
        m=ObstacleMemory();self.observe(m,[1.],10.)
        for i in range(100):self.observe(m,[math.nan if i%2 else math.inf],11.+i*.1)
        self.assertIn((1.,0.),m.points())

    def test_passing_box_and_turning_back_cannot_clear_it_from_front_only_scans(self):
        m=ObstacleMemory();self.observe(m,[1.],10.)
        for i in range(20):self.observe(m,[3.],11.+i*.1,(2.,0.,0.))
        self.assertIn((1.,0.),m.points())
        for i in range(20):self.observe(m,[math.inf],14.+i*.1,(2.,0.,math.pi))
        self.assertIn((1.,0.),m.points())

    def test_a_closer_occluder_cannot_erase_the_box_behind_it(self):
        m=ObstacleMemory();self.observe(m,[1.],10.)
        for i in range(20):self.observe(m,[.5],11.+i*.1)
        self.assertIn((1.,0.),m.points())

    def test_only_repeated_finite_farther_rays_clear_an_obstacle(self):
        m=ObstacleMemory();self.observe(m,[1.],10.)
        for i in range(7):self.observe(m,[3.],11.+i/15)
        self.assertIn((1.,0.),m.points())
        for i in range(7,11):self.observe(m,[3.],11.+i/15)
        self.assertNotIn((1.,0.),m.points())
        self.assertIn((3.,0.),m.points())

    def test_unobserved_frame_resets_the_clear_confirmation(self):
        m=ObstacleMemory();self.observe(m,[1.],10.)
        for i in range(7):self.observe(m,[3.],11.+i/15)
        self.observe(m,[math.nan],11.5)
        self.observe(m,[3.],11.6)
        self.assertIn((1.,0.),m.points())

    def test_capacity_exhaustion_is_an_error_not_silent_near_obstacle_eviction(self):
        m=ObstacleMemory(max_cells=1);self.observe(m,[1.],10.)
        with self.assertRaises(RuntimeError):self.observe(m,[2.],11.)

    def test_old_or_duplicate_scans_do_not_supply_clear_evidence(self):
        m=ObstacleMemory();self.observe(m,[1.],10.)
        for _ in range(50):self.observe(m,[3.],10.)
        self.assertEqual(m.points(),[(1.,0.)])

    def test_long_sensor_gap_restarts_clear_confirmation_without_losing_the_obstacle(self):
        m=ObstacleMemory();self.observe(m,[1.],10.)
        for i in range(5):self.observe(m,[3.],11.+i/15)
        for i in range(4):self.observe(m,[3.],13.+i/15)
        self.assertIn((1.,0.),m.points())

    def test_raw_box_support_vetoes_clear_even_if_scan_filter_misses_it(self):
        m=ObstacleMemory(require_3d_clearing=True)
        box=RayEvidence([[1.,0.,-.25],[1.,0.,0.],[1.,0.,.30]],[0.,0.,.13])
        m.observe([1.],0.,.01,.25,5.5,(0.,0.,0.),10.,rays=box)
        for i in range(20):m.observe([3.],0.,.01,.25,5.5,(0.,0.,0.),11.+i/15,rays=box)
        self.assertIn((1.,0.),m.points())

    def test_ground_ray_over_a_low_object_does_not_clear_its_height_volume(self):
        rays=RayEvidence([[3.,0.,-.37]],[0.,0.,.13])
        self.assertFalse(rays.clears(1.,0.,-.37,-.25))
        self.assertTrue(rays.clears(1.,0.,-.25,.30))

    def test_actual_ground_rays_through_removed_box_volume_can_clear_it(self):
        m=ObstacleMemory(require_3d_clearing=True)
        box=RayEvidence([[1.,0.,-.25],[1.,0.,0.],[1.,0.,.30]],[0.,0.,.13])
        floor=RayEvidence([[3.,0.,-.37]],[0.,0.,.13])
        m.observe([1.],0.,.01,.25,5.5,(0.,0.,0.),10.,rays=box)
        for i in range(12):m.observe([math.inf],0.,.01,.25,5.5,(0.,0.,0.),11.+i/15,rays=floor)
        self.assertEqual(m.points(),[])

    def memory_with_box(self):
        m=ObstacleMemory(require_3d_clearing=True,max_observation_gap=.60)
        m.mark_xyz([[1.,0.,-.25],[1.,0.,.30]],(0.,0.,0.),10.)
        return m

    def clear_frame(self,m,stamp,points):
        rays=RayEvidence(points,[0.,0.,.13])
        m.observe([],0.,1.,0.,0.,(0.,0.,0.),stamp,rays=rays)

    def test_sparse_3d_rays_clear_removed_box_without_consecutive_frames(self):
        m=self.memory_with_box()
        for i in range(15):
            self.clear_frame(m,11.+i*.2,[[3.,0.,-.37]] if i%2==0 else [])
            if i<14:self.assertIn((1.,0.),m.points())
        self.assertEqual(m.points(),[])

    def test_seven_rays_then_unobserved_space_never_clears(self):
        m=self.memory_with_box()
        for i in range(7):self.clear_frame(m,11.+i*.2,[[3.,0.,-.37]])
        for i in range(100):self.clear_frame(m,12.4+i*.2,[])
        self.assertIn((1.,0.),m.points())
        self.assertEqual(next(iter(m.cells.values()))[3:5],(None,0))

    def test_occupied_3d_support_resets_accumulated_clear_evidence(self):
        m=self.memory_with_box()
        for i in range(7):self.clear_frame(m,11.+i*.2,[[3.,0.,-.37]])
        self.clear_frame(m,12.4,[[1.,0.,0.],[3.,0.,-.37]])
        self.assertEqual(next(iter(m.cells.values()))[3:5],(None,0))
        self.clear_frame(m,12.6,[[3.,0.,-.37]])
        self.assertIn((1.,0.),m.points())
        self.assertEqual(next(iter(m.cells.values()))[4],1)

    def test_3d_evidence_outside_window_or_after_sensor_gap_does_not_accumulate(self):
        for gap in (False,True):
            m=self.memory_with_box()
            for i in range(7):self.clear_frame(m,11.+i*.2,[[3.,0.,-.37]])
            if gap:
                self.clear_frame(m,13.4,[[3.,0.,-.37]])
            else:
                for i in range(15):self.clear_frame(m,12.4+i*.2,[])
                self.clear_frame(m,15.4,[[3.,0.,-.37]])
            self.assertIn((1.,0.),m.points())
            self.assertEqual(next(iter(m.cells.values()))[4],1)

    def test_new_occupied_hit_cancels_sparse_sequence_and_delayed_clear(self):
        m=self.memory_with_box()
        for i in range(7):self.clear_frame(m,11.+i*.2,[[3.,0.,-.37]])
        snapshot=m.cells.copy()
        self.clear_frame(m,12.4,[[3.,0.,-.37]])
        cleared=m.cells.copy()
        m.cells=snapshot.copy()
        m.mark_xyz([[1.,0.,0.]],(0.,0.,0.),12.5)
        m.merge_clearing(snapshot,cleared,12.4)
        self.assertIn((1.,0.),m.points())
        self.assertEqual(next(iter(m.cells.values()))[3:5],(None,0))

    def test_closer_3d_occluder_cannot_supply_clear_evidence(self):
        m=self.memory_with_box()
        for i in range(30):self.clear_frame(m,11.+i*.2,[[.5,0.,0.]])
        self.assertIn((1.,0.),m.points())

    def test_batch_rays_match_scalar_with_empty_unknown_and_out_of_range_cells(self):
        for points in ([],[[3.,0.,-.37],[1.,0.,0.]]):
            rays=RayEvidence(points,[0.,0.,.13])
            xy=np.array([[1.,0.],[.1,0.],[9.,0.],[1.,1.]])
            low,high=[-.25,-.25,-.25,None],[.3,.3,.3,None]
            scalar=[rays.clears(x,y,a,b) for (x,y),a,b in zip(xy,low,high)]
            np.testing.assert_array_equal(rays.clears_many(xy,low,high),scalar)

    def test_clearing_snapshot_keeps_far_fresh_and_unknown_cells_in_main_memory(self):
        m=self.memory_with_box()
        m.mark_xyz([[9.,0.,.1]],(0.,0.,0.),10.1)
        m.mark_xyz([[2.,0.,.1]],(0.,0.,0.),11.9)
        m.cells[(60,0)]=(3.,0.,10.,None,0,None,None)
        before=m.cells.copy()
        snapshot=m.clearing_snapshot((0.,0.),12.)
        self.assertEqual(set(snapshot),{(20,0)})
        self.assertEqual(m.cells,before)
        m.merge_clearing(snapshot,{},12.)
        self.assertEqual(set(m.cells),set(before)-{(20,0)})

    def test_aligned_grid_refinement_replicates_unknown_and_single_cell_leg_exactly(self):
        source=np.array([[-1,0,100],[30,0,-1]],dtype=np.int8)
        result,res=navigation_grid(source.ravel(),3,2,float(np.float32(.08)),.04)
        np.testing.assert_array_equal(result,np.repeat(np.repeat(source,2,axis=0),2,axis=1))
        self.assertEqual(res,.04)
        self.assertEqual(np.count_nonzero(result==100),4)

    def test_sparse_numeric_delta_matches_existing_3d_model(self):
        from itertools import chain
        rng=np.random.default_rng(73)
        for gap in (.2,.7):
            m=ObstacleMemory(require_3d_clearing=True,max_observation_gap=.60)
            for i in range(500):
                x,y=rng.uniform(-6.,6.,2);low=rng.uniform(-.4,.1)
                m.cells[(i,0)]=(x,y,8.,None if i%3 else 9.6,0 if i%3 else 7,low,low+.3)
            snapshot=m.cells.copy();keys=list(snapshot);m.last_stamp=10.-gap
            geometry=np.fromiter(chain.from_iterable(snapshot.values()),float).reshape(-1,7)
            before=geometry.copy()
            rays=RayEvidence(rng.uniform([-5.,-5.,-.5],[5.,5.,.8],(3000,3)),[0.,0.,.13])
            updates=clearing_delta(geometry,rays,(0.,0.,0.),10.,m.last_stamp)
            merged=ObstacleMemory();merged.cells=snapshot.copy()
            merged.merge_delta(snapshot,keys,*updates,10.)
            m.observe([],0.,1.,0.,0.,(0.,0.,0.),10.,rays=rays)
            self.assertEqual(merged.cells,m.cells)
            np.testing.assert_array_equal(geometry,before)

    def test_sparse_numeric_merge_never_erases_newer_occupied_write(self):
        m=self.memory_with_box();snapshot=m.cells.copy();keys=list(snapshot)
        m.mark_xyz([[1.,0.,.1]],(0.,0.,0.),11.)
        m.merge_delta(snapshot,keys,np.array([0]),np.array([[10.,8.]]),np.array([True]),10.9)
        self.assertIn((1.,0.),m.points())
        self.assertEqual(next(iter(m.cells.values()))[3:5],(None,0))

    def floor_correction(self,point,rays_points,steps=10):
        m=ObstacleMemory(require_3d_clearing=True,max_observation_gap=.60)
        m.mark_xyz([point],(0.,0.,0.),10.)
        for i in range(steps):
            snapshot=m.cells.copy();keys=list(snapshot)
            geometry=np.array([[np.nan if v is None else v for v in c] for c in snapshot.values()]).reshape(-1,7)
            stamp=11.+i*.2
            updates=clearing_delta(geometry,RayEvidence(rays_points,[0.,0.,.5]),
                (0.,0.,0.),stamp,stamp-.2,ground_plane=[0.,0.,1.,0.])
            m.merge_delta(snapshot,keys,*updates,stamp)
        return m

    def test_repeated_stable_ground_evidence_reclassifies_old_floor_memory(self):
        self.assertEqual(self.floor_correction([1.,0.,0.],[[3.,0.,0.]]).points(),[])
        self.assertTrue(self.floor_correction([1.,0.,0.],[[3.,0.,0.]],steps=7).points())

    def test_ground_correction_keeps_ten_cm_leg_even_in_visible_column(self):
        m=self.floor_correction([1.,0.,.10],[[1.,0.,.10],[3.,0.,0.]])
        self.assertIn((1.,0.),m.points())

    def test_ground_correction_cannot_expire_blind_or_occluded_floor_cells(self):
        for rays in ([],[[3.,1.,0.]],[[.5,0.,.1],[3.,0.,0.]]):
            self.assertIn((1.,0.),self.floor_correction([1.,0.,0.],rays).points())

    def test_measured_ground_support_can_correct_distant_floor_without_a_farther_ray(self):
        self.assertEqual(self.floor_correction([3.,0.,0.],[[3.,0.,0.]]).points(),[])
        self.assertIn((3.,0.),self.floor_correction([3.,0.,0.],[[5.,0.,0.]]).points())


if __name__ == '__main__':unittest.main()
