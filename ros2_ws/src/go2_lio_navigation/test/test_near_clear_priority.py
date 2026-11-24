"""Near confirmed empty space must clear promptly, regardless of dwell time."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from obstacle_memory_model import ObstacleMemory,RayEvidence,clearing_delta
from dynamic_obstacles import DynamicObstacles


class NearClearPriorityTests(unittest.TestCase):
    def step(self,memory,stamp,points,ground=None,transient=None,dynamic=None,
             near_points=None,origin=(0.,0.,.4)):
        snapshot=memory.cells.copy();keys=list(snapshot)
        geometry=np.asarray(list(snapshot.values()),dtype=float).reshape(-1,7)
        near_rays=(None if near_points is None else
                   RayEvidence(near_points,origin,minimum_horizontal_range=.01))
        updates=clearing_delta(geometry,RayEvidence(points,origin),
            (0.,0.,0.),stamp,stamp-.1,ground_plane=ground,
            transient_flags=transient,dynamic_flags=dynamic,near_fast=True,near_rays=near_rays)
        memory.merge_delta(snapshot,keys,*updates,stamp)

    def test_unitree_raw_crossings_clear_two_near_body_cells(self):
        origin=(.28,0.,0.)
        ground=[0.,0.,1.,.35]
        floor=[[x,y,-.35] for x in np.linspace(.5,2.,21) for y in np.linspace(-.6,.6,21)]
        front=(.33,.001,-.195)
        rear=(-.30,.001,-.22)
        # The front beam runs 3.9 cm past the cell in XY, but 15.5 cm
        # in height. The rear beam is absent in the factory-filtered scan.
        rays=[(.369,.002,-.35),(-.64,.002,-.35)]
        for mode,keep in (('filtered',2),('no_ground',1),('measured',0),
                          ('occupied',1),('shadow',1)):
            with self.subTest(mode=mode):
                memory=ObstacleMemory(require_3d_clearing=True)
                memory.mark_xyz([front,rear],(0.,0.,0.),10.)
                extra=[] if mode=='filtered' else rays.copy()
                if mode=='occupied':extra.append(front)
                if mode=='shadow':extra.append((.30,.001,-.078))
                for i in range(3):
                    self.step(memory,11.+i*.125,floor,None if mode=='no_ground' else ground,
                              near_points=extra,origin=origin)
                    if i<2:self.assertEqual(len(memory.cells),2)
                self.assertEqual(len(memory.cells),keep)
                if keep==1:self.assertAlmostEqual(next(iter(memory.cells.values()))[0],front[0])

    def test_supplemental_rays_do_not_clear_blind_or_distant_history(self):
        memory=ObstacleMemory(require_3d_clearing=True)
        memory.mark_xyz([[.65,0.,.4],[.33,.16,.4]],(0.,0.,0.),10.)
        for i in range(10):
            self.step(memory,11.+i*.125,[],near_points=[[1.,0.,.4]])
        self.assertEqual(len(memory.cells),2)

    def test_unitree_corridor_exit_accepts_short_floor_beam_with_occupied_veto(self):
        origin=(.28,0.,0.)
        ground=[0.,0.,1.,.315]
        floor=[[x,y,-.315] for x in np.linspace(.5,2.,21) for y in np.linspace(-.6,.6,21)]
        cell=(.15,-.15,-.21)
        # The measured horizontal overrun is only 9.9 cm at this oblique
        # angle, but the beam travels another 14.4 cm to the floor.
        ray=(.085,-.225,-.315)
        for extra,keep in (([ray],False),([ray,cell],True),
                           ([ray,(.215,-.075,-.105)],True),
                           ([(.1227,-.1815,-.265)],True),
                           ([(.111,-.195,-.273)],True)):
            with self.subTest(extra=extra):
                memory=ObstacleMemory(require_3d_clearing=True)
                memory.mark_xyz([cell],(0.,0.,0.),10.)
                for i in range(4):
                    self.step(memory,11.+i*.125,floor,ground,near_points=extra,origin=origin)
                    if i<2:self.assertTrue(memory.cells)
                self.assertEqual(bool(memory.cells),keep)

    def test_below_floor_return_beyond_volume_does_not_shadow_a_unitree_crossing(self):
        origin=(.28,0.,0.)
        ground=[0.,0.,1.,.335]
        floor=[[x,y,-.335] for x in np.linspace(.5,2.,21) for y in np.linspace(-.6,.6,21)]
        cell=(.36,-.01,-.221)
        crossing=(.419,-.017375,-.38225)
        below_floor=(.407,-.015875,-.4)
        occluder=(.32,-.005,-.111)
        for rays,keep in (([crossing,below_floor],False),([below_floor],True),
                          ([crossing,below_floor,occluder],True)):
            with self.subTest(rays=rays):
                memory=ObstacleMemory(require_3d_clearing=True)
                memory.mark_xyz([cell],(0.,0.,0.),10.)
                for i in range(3):
                    self.step(memory,11.+i*.125,floor,ground,near_points=rays,origin=origin)
                self.assertEqual(bool(memory.cells),keep)

    def test_actual_3d_clearance_time_does_not_grow_with_parked_duration(self):
        for dwell in (.1,2.,20.):
            memory=ObstacleMemory(require_3d_clearing=True,max_observation_gap=.6)
            for stamp in np.arange(10.,10.+dwell+.001,.1):
                memory.mark_xyz([[.9,0.,.2],[.9,0.,.5]],(0.,0.,0.),float(stamp))
            departed=10.+dwell
            for i in range(2):
                self.step(memory,departed+.11+i*.125,[[3.,0.,.4]])
                self.assertTrue(memory.cells)
            self.step(memory,departed+.36,[[3.,0.,.4]])
            self.assertFalse(memory.cells,dwell)

    def test_fresh_near_hits_are_excluded_but_newly_vacated_near_hits_are_prioritized(self):
        memory=ObstacleMemory()
        body=(0.,0.)
        sensor=(.14,0.)
        memory.mark_xyz([[.9,0.,.3],[-1.75,0.,.3],[1.85,0.,.3]],(*body,0.),10.)
        self.assertFalse(memory.clearing_snapshot(sensor,10.05,near_position=body))
        selected=memory.clearing_snapshot(sensor,10.15,near_position=body)
        self.assertCountEqual([v[:2] for v in selected.values()],[(.9,0.),(-1.75,0.)])
        self.assertEqual(len(memory.clearing_snapshot(sensor,10.6,near_position=body)),3)
        self.assertEqual(len(memory.clearing_snapshot(
            sensor,10.6,near_position=body,near_only=True)),2)

    def test_extended_priority_requires_observed_empty_volume(self):
        for points,expected in (([[3.,0.,.4]],False),
                                ([[1.,0.,.4],[3.,0.,.4]],True),([],True)):
            with self.subTest(points=points):
                memory=ObstacleMemory(require_3d_clearing=True)
                memory.mark_xyz([[1.7,0.,.2],[1.7,0.,.5]],(0.,0.,0.),10.)
                for i in range(3):
                    self.step(memory,10.125+i*.125,points)
                self.assertEqual(bool(memory.cells),expected)

    def test_close_support_shadow_and_no_returns_never_clear_a_parked_obstacle(self):
        for points in ([[.9,0.,.3],[3.,0.,.4]],[[.5,0.,.4],[3.,0.,.4]],[]):
            memory=ObstacleMemory(require_3d_clearing=True)
            memory.mark_xyz([[.9,0.,.2],[.9,0.,.5]],(0.,0.,0.),10.)
            for i in range(10):self.step(memory,11.+i*.125,points)
            self.assertTrue(memory.cells,points)

    def test_sparse_near_ground_inference_still_requires_four_observations(self):
        memory=ObstacleMemory(require_3d_clearing=True)
        memory.mark_xyz([[.9,0.,.65]],(0.,0.,0.),10.)
        floor=[[x,y,0.] for x in np.linspace(2.,4.,21) for y in np.linspace(-1.,1.,41)]
        for i in range(3):
            self.step(memory,11.+i*.125,floor,[0.,0.,1.,0.],[True])
            self.assertTrue(memory.cells)
        self.step(memory,11.375,floor,[0.,0.,1.,0.],[True])
        self.assertFalse(memory.cells)

    def test_far_ordinary_obstacles_keep_the_original_eight_observation_rule(self):
        memory=ObstacleMemory(require_3d_clearing=True)
        memory.mark_xyz([[2.,0.,.2],[2.,0.,.5]],(0.,0.,0.),10.)
        for i in range(7):
            self.step(memory,11.+i*.125,[[4.,0.,.4]])
            self.assertTrue(memory.cells)
        self.step(memory,11.875,[[4.,0.,.4]])
        self.assertFalse(memory.cells)

    def test_tracked_motion_gets_priority_beyond_the_near_radius(self):
        memory=ObstacleMemory(require_3d_clearing=True)
        memory.mark_xyz([[2.3,0.,.2],[2.3,0.,.5]],(0.,0.,0.),10.)
        moving=set(memory.cells)
        self.assertFalse(memory.clearing_snapshot((0.,0.),10.15,near_position=(0.,0.),near_only=True))
        self.assertEqual(set(memory.clearing_snapshot(
            (0.,0.),10.15,near_position=(0.,0.),near_only=True,moving_cells=moving)),moving)
        for i in range(3):
            self.step(memory,11.+i*.125,[[1.,0.,.4],[4.,0.,.4]],dynamic=[True])
        self.assertTrue(memory.cells)
        for i in range(3):
            self.step(memory,11.375+i*.125,[[4.,0.,.4]],dynamic=[True])
        self.assertFalse(memory.cells)

    def test_near_leg_tracks_keep_their_identity_after_a_long_pause(self):
        for pause in (.2,2.,10.):
            memory=ObstacleMemory();tracker=DynamicObstacles()
            for i in range(round(pause/.1)+1):
                points=np.array([[.9,-.4+d,z] for d in (-.06,0.,.06) for z in (.1,.3)])
                stamp=10.+i*.1
                tracker.observe(points,(0.,0.,0.),stamp,memory.cells)
                memory.mark_xyz(points,(0.,0.,0.),stamp)
            for i in range(1,10):
                points=np.array([[.9,-.4+i*.08+d,z] for d in (-.06,0.,.06) for z in (.1,.3)])
                stamp=10.+pause+i*.1
                tracker.observe(points,(0.,0.,0.),stamp,memory.cells)
                memory.mark_xyz(points,(0.,0.,0.),stamp)
            self.assertTrue(tracker.dynamic_cells,pause)
