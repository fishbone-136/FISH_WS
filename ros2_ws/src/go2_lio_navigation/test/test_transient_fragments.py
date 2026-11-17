"""Sparse crossing fragments need visible ground proof, never a blind TTL."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from dynamic_obstacles import DynamicObstacles
from obstacle_memory_model import ObstacleMemory,RayEvidence,clearing_delta


class TransientFragmentsTests(unittest.TestCase):
    def fixture(self):
        memory=ObstacleMemory(require_3d_clearing=True,max_observation_gap=.6)
        tracker=DynamicObstacles()
        leg=np.array([[1.5,.7,z] for z in (.12,.3,.5)])
        for i in range(8):
            stamp=10.+i*.2
            tracker.observe(leg,(0.,0.,0.),stamp,memory.cells)
            memory.mark_xyz(leg,(0.,0.,0.),stamp)
        fragments=np.array([[1.,y,.65] for y in (-.2,-.1,0.,.1,.2)],dtype=np.float32)
        tracker.observe(fragments,(0.,0.,0.),12.,memory.cells)
        memory.mark_xyz(fragments,(0.,0.,0.),12.)
        self.assertFalse(tracker.dynamic_cells,'Sparse initial fragments must not invent motion')
        return memory,tracker,fragments

    def clear(self,memory,tracker,points,steps):
        rays=RayEvidence(points,[0.,0.,.4])
        for i in range(steps):
            snapshot=memory.cells.copy();keys=list(snapshot)
            geometry=np.asarray(list(snapshot.values()),dtype=float).reshape(-1,7)
            stamp=13.+i*.2
            updates=clearing_delta(geometry,rays,(0.,0.,0.),stamp,stamp-.2,
                ground_plane=[0.,0.,1.,0.],dynamic_flags=tracker.flags(keys),
                transient_flags=tracker.transient_flags(keys,snapshot))
            memory.merge_delta(snapshot,keys,*updates,stamp)

    def floor(self):
        return [[x,y,0.] for x in np.linspace(2.,4.,21) for y in np.linspace(-1.,1.,41)]

    def test_sparse_crossing_clears_after_eight_ground_observations_and_retains_leg(self):
        memory,tracker,fragments=self.fixture()
        self.clear(memory,tracker,self.floor(),7)
        self.assertEqual(len(memory.cells),6)
        # A separate confirmation pass, preserving the original since/count.
        snapshot=memory.cells.copy();keys=list(snapshot)
        changes=clearing_delta(np.asarray(list(snapshot.values()),dtype=float),
            RayEvidence(self.floor(),[0.,0.,.4]),(0.,0.,0.),14.4,14.2,
            ground_plane=[0.,0.,1.,0.],transient_flags=tracker.transient_flags(keys,snapshot))
        memory.merge_delta(snapshot,keys,*changes,14.4)
        self.assertEqual(memory.points(),[(1.5,.7)])

    def test_occluders_and_missing_floor_cannot_erase_fragments(self):
        for points in ([],[[3.,0.,.1]],self.floor()+[[.5,y/2.,.3] for y in (-.2,-.1,0.,.1,.2)]):
            with self.subTest(points=len(points)):
                memory,tracker,_=self.fixture();before=set(memory.cells)
                self.clear(memory,tracker,points,12)
                self.assertEqual(set(memory.cells),before)

    def test_single_new_landmark_never_acquires_fragment_clearing(self):
        memory=ObstacleMemory();tracker=DynamicObstacles()
        points=np.array([[1.,0.,z] for z in (.1,.3,.5)])
        tracker.observe(points,(0.,0.,0.),10.,memory.cells)
        memory.mark_xyz(points,(0.,0.,0.),10.)
        self.assertFalse(tracker.transient_flags(list(memory.cells),memory.cells).any())

    def test_ground_visibility_requires_actual_floor_in_the_same_column(self):
        rays=RayEvidence([[3.,0.,.15]],[0.,0.,.4])
        self.assertTrue(rays.sees_beyond([[1.,0.]],[0.,0.,1.,0.])[0])
        self.assertFalse(rays.sees_beyond([[1.,0.]],[0.,0.,1.,0.],ground_only=True)[0])


if __name__=='__main__':unittest.main()
