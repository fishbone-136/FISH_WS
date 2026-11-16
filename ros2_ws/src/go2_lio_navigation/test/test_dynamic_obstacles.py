from pathlib import Path
import sys
import unittest
import math
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from dynamic_obstacles import DynamicObstacles,cells_in_bounds
from obstacle_memory_model import (ObstacleMemory,RayEvidence,clearing_delta,memory_checkpoint,
                                  restore_checkpoint,clear_confirmed_region)


def person(y,z=.30):
    return np.array([[1.,y+d,h] for d in np.linspace(-.18,.18,9) for h in (z,z+.2,z+.4)])


class DynamicEvidenceTests(unittest.TestCase):
    def test_compact_grid_query_matches_full_map_at_negative_and_exact_boundaries(self):
        memory=ObstacleMemory()
        xy=np.array([[x,y,.3] for x in np.arange(-2.,2.,.073)
                     for y in np.arange(-2.,2.,.081)])
        memory.mark_xyz(xy,(0.,0.,0.),10.)
        for lo,hi in (([-.30,-.20],[.20,.30]),([-5.,-5.],[5.,5.]),
                      ([-.05,-.05],[0.,0.]),([10.,10.],[11.,11.])):
            expected={k:v for k,v in memory.cells.items()
                      if lo[0]<=v[0]<=hi[0] and lo[1]<=v[1]<=hi[1]}
            self.assertEqual(dict(cells_in_bounds(memory.cells,lo,hi,memory.resolution)),expected)

    def test_confirmed_right_clear_rotates_with_robot_and_preserves_fresh_hits(self):
        memory=ObstacleMemory();motion=DynamicObstacles()
        memory.mark_xyz([[1.30,2.,.4],[.7,2.,.4],[1.30,3.,.4]],(1.,2.,math.pi/2),10.)
        memory.mark_xyz([[1.35,2.05,.4]],(1.,2.,math.pi/2),19.)
        memory.mark_xyz([],(1.,2.,math.pi/2),20.)
        before=memory_checkpoint(memory,'odom_lio',{'pid':123},motion)
        after,removed=clear_confirmed_region(before,(1.,2.,math.pi/2))
        self.assertEqual(len(removed),1)
        self.assertAlmostEqual(removed[0][2],1.30)
        self.assertAlmostEqual(removed[0][3],2.)
        self.assertEqual(len(before['cells']),4)
        self.assertEqual(len(after['cells']),3)
        self.assertEqual(after['session'],before['session'])
        allowed={tuple(row[:2]):row[4] for row in removed}
        memory.mark_xyz([[1.30,2.,.4]],(1.,2.,math.pi/2),21.)
        memory.mark_xyz([],(1.,2.,math.pi/2),30.)
        updated=memory_checkpoint(memory,'odom_lio',{'pid':123},motion)
        _,removed_again=clear_confirmed_region(updated,(1.,2.,math.pi/2),allowed_hits=allowed)
        self.assertEqual(removed_again,[],'An observation after the confirmation must survive')

    def observe(self,tracker,memory,points,stamp,pose=(0.,0.,0.)):
        tracker.observe(points,pose,stamp,memory.cells)
        memory.mark_xyz(points,pose,stamp);tracker.prune(memory.cells)

    def test_lateral_pass_is_confirmed_but_stationary_body_is_not(self):
        for moving in (False,True):
            tracker=DynamicObstacles();memory=ObstacleMemory()
            for i in range(12):
                self.observe(tracker,memory,person(-.9+i*.12 if moving else 0.),10.+i*.2)
            self.assertEqual(bool(tracker.dynamic_cells),moving)
            self.assertEqual(tracker.confirmed_tracks,int(moving))

    def test_noise_and_robot_rotation_cannot_supply_motion_confirmation(self):
        for turning in (False,True):
            tracker=DynamicObstacles();memory=ObstacleMemory()
            for i in range(12):
                y=-.9+i*.12 if turning else (.08 if i%2 else -.08)
                self.observe(tracker,memory,person(y),10.+i*.2,pose=(0.,0.,i*.15 if turning else 0.))
            self.assertFalse(tracker.dynamic_cells)

    def test_a_thin_leg_never_gets_a_dynamic_lifetime(self):
        tracker=DynamicObstacles();memory=ObstacleMemory()
        for i in range(12):
            points=np.array([[1.,i*.12,h] for h in (.1,.3,.5)])
            self.observe(tracker,memory,points,10.+i*.2)
        self.assertFalse(tracker.dynamic_cells)

    def test_paused_then_moving_person_can_be_confirmed(self):
        tracker=DynamicObstacles();memory=ObstacleMemory()
        for i in range(30):self.observe(tracker,memory,person(-.8),10.+i*.2)
        self.assertFalse(tracker.dynamic_cells)
        for i in range(12):self.observe(tracker,memory,person(-.8+i*.12),16.+i*.2)
        self.assertTrue(tracker.dynamic_cells)

    def test_fast_crossing_is_confirmed_at_the_actual_scan_rate(self):
        tracker=DynamicObstacles();memory=ObstacleMemory()
        for i in range(8):
            self.observe(tracker,memory,person(-.5+i*.08),10.+i/15.)
        self.assertTrue(tracker.dynamic_cells)

    def test_confirmed_person_keeps_ownership_when_last_returns_are_fragments(self):
        tracker=DynamicObstacles();memory=ObstacleMemory(max_observation_gap=.60)
        for i in range(8):
            self.observe(tracker,memory,person(-.5+i*.08),10.+i/15.)
        last=person(.06)
        fragments=last[:2]
        key=tuple(np.floor(fragments[0,:2]/memory.resolution).astype(int))
        self.assertIn(key,tracker.dynamic_cells)
        owner=tracker.dynamic_cells[key]
        self.observe(tracker,memory,fragments,10.+8/15.)
        self.assertEqual(tracker.dynamic_cells.get(key),owner)
        self.assertNotIn(key,tracker.untracked_landmarks)
        # Fresh independent support after the owner is no longer recent
        # must not inherit the person's relaxed column-clearing permission.
        self.observe(tracker,memory,fragments,12.)
        self.assertNotIn(key,tracker.dynamic_cells)

    def test_independent_sparse_landmark_is_not_reclassified_by_a_nearby_crossing(self):
        tracker=DynamicObstacles();memory=ObstacleMemory()
        arm=np.array([[.78,-.5,h] for h in (.3,.5,.7)])
        self.observe(tracker,memory,arm,10.)
        for i in range(4):self.observe(tracker,memory,person(-.5+i*.08),10.07+i/15.)
        for i in range(5):self.observe(tracker,memory,person(.25+i*.08),10.6+i/15.)
        self.assertTrue(tracker.dynamic_cells)
        self.assertNotIn((15,-10),tracker.dynamic_cells)

    def test_established_leg_is_not_reassigned_to_a_passing_body(self):
        tracker=DynamicObstacles();memory=ObstacleMemory()
        leg=np.array([[1.,0.,h] for h in (.1,.3,.5)])
        for i in range(10):self.observe(tracker,memory,leg,10.+i*.2)
        for i in range(12):self.observe(tracker,memory,np.r_[leg,person(-.9+i*.12)],12.+i*.2)
        self.assertNotIn((20,0),tracker.dynamic_cells)

    def test_person_passing_against_static_box_is_separated_before_tracking(self):
        tracker=DynamicObstacles();memory=ObstacleMemory()
        box=np.array([[x,y,z] for x in np.linspace(.7,1.3,7)
            for y in np.linspace(.45,1.05,7) for z in (.12,.35,.6)])
        for i in range(30):self.observe(tracker,memory,box,10.+i*.2)
        established=set(memory.cells)
        tagged=set()
        for i in range(16):
            self.observe(tracker,memory,np.r_[box,person(-.9+i*.08)],16.+i*.2)
            tagged.update(tracker.dynamic_cells)
        self.assertTrue(tagged,'Merging with a static box lost the moving person')
        self.assertFalse(tagged&established,'The static box was assigned dynamic clearing')

    def test_checkpoint_roundtrip_preserves_geometry_and_rejects_other_session(self):
        tracker=DynamicObstacles();memory=ObstacleMemory()
        self.observe(tracker,memory,person(0.),10.)
        session={'pid':10,'start':'123','domain':'0'}
        state=memory_checkpoint(memory,'odom_lio',session,tracker)
        restored=ObstacleMemory();motion=DynamicObstacles()
        restore_checkpoint(state,restored,'odom_lio',session,motion)
        self.assertEqual(memory.cells,restored.cells)
        self.assertEqual(tracker.first_seen,motion.first_seen)
        self.assertEqual(tracker.static_confirmed,motion.static_confirmed)
        self.assertEqual(tracker.support_history,motion.support_history)
        with self.assertRaises(ValueError):
            restore_checkpoint(state,restored,'odom_lio',{'pid':11},motion)

    def test_background_floor_clears_only_motion_confirmed_upper_body(self):
        for dynamic,occluded in ((False,False),(True,False),(True,True)):
            memory=ObstacleMemory(require_3d_clearing=True)
            memory.mark_xyz([[1.,0.,.7]],(0.,0.,0.),10.)
            for i in range(12):
                snapshot=memory.cells.copy();keys=list(snapshot)
                geometry=np.array([[np.nan if x is None else x for x in c] for c in snapshot.values()]).reshape(-1,7)
                points=[[3.,0.,0.]]+([[.5,0.,.3]] if occluded else [])
                stamp=11.+i*.2
                changes=clearing_delta(geometry,RayEvidence(points,[0.,0.,.4]),(0.,0.,0.),stamp,
                    stamp-.2,ground_plane=[0.,0.,1.,0.],dynamic_flags=[dynamic]*len(keys))
                memory.merge_delta(snapshot,keys,*changes,stamp)
            self.assertEqual(bool(memory.cells),not(dynamic and not occluded))

    def test_strong_current_floor_can_reclassify_old_ground_but_not_a_leg(self):
        points=[[x,y,0.] for x in np.linspace(2.,3.,10) for y in np.linspace(-1.,1.,10)]
        geometry=np.array([[1.,0.,10.,np.nan,0,-.025,.025],[1.,.4,10.,np.nan,0,.075,.125]])
        for i in range(8):
            stamp=11.+i*.2
            ids,states,deleted=clearing_delta(geometry,RayEvidence(points,[0.,0.,.4]),
                (0.,0.,0.),stamp,stamp-.2,ground_plane=[0.,0.,1.,0.])
            geometry[ids,3:5]=states
        np.testing.assert_array_equal(ids[deleted],[0])


if __name__=='__main__':unittest.main()
