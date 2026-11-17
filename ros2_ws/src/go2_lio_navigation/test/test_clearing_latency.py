"""Delayed geometry evidence must progress without weakening occupied writes."""
from collections import deque
from concurrent.futures import Future
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from sensor_msgs.msg import PointCloud2,PointField

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from obstacle_memory import RememberedObstacles
from obstacle_memory_model import ObstacleMemory


class ClearingLatencyTests(unittest.TestCase):
    def fixture(self):
        memory=ObstacleMemory(require_3d_clearing=True,max_observation_gap=.60)
        memory.mark_xyz([[1.,0.,.2]],(0.,0.,0.),8.)
        node=SimpleNamespace(
            memory=memory,localized=True,fault=None,clearing=None,
            clearing_result_max_age=1.5,clearing_reset_stamp=9.,
            last_observation_stamp=9.,clearing_observation_resets=0,
            last_clearing_stamp=9.,last_clearing_result_age=None,
            next_clearing=math.inf,clearing_rate=6.,clearing_jobs=0,
            clearing_removed=0,stale_clearing_results=0,
            max_processing_ms=0.,ray_failures=0,occupancy_revision=0,
            motion=Mock(),publish_points=Mock(),publish_maps=Mock(),
            publish_clear_volumes=Mock(),clearing_ground=None,
            processed=deque(),worker=Mock(),clouds={},ground_planes={},
            observation_transform=Mock())
        node.now=10.
        node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(
            nanoseconds=int(node.now*1.e9)))
        return node

    def finish(self,node,stamp,age,updates,snapshot):
        node.now=stamp+age
        node.clearing=Future();node.clearing.set_result((updates,350.))
        node.clearing_context=(snapshot,stamp);node.clearing_keys=list(snapshot)
        RememberedObstacles.update_clearing(node)

    def test_delayed_clear_refreshes_maps_but_cannot_delete_newer_hit(self):
        node=self.fixture()
        node.memory.mark_xyz([[2.,0.,.2]],(0.,0.,0.),8.1)
        snapshot=node.memory.cells.copy()
        node.memory.mark_xyz([[2.,0.,.2]],(0.,0.,0.),10.2)
        updates=(np.array([0,1]),np.array([[9.,8.],[9.,8.]]),np.array([True,True]))
        self.finish(node,10.,.9,updates,snapshot)
        self.assertEqual(node.memory.points(),[(2.,0.)])
        self.assertEqual(node.clearing_jobs,1)
        node.publish_points.assert_called_once()
        node.publish_maps.assert_called_once()
        self.assertEqual(len(node.publish_clear_volumes.call_args.args[0]),1)

    def test_expired_results_and_results_before_sensor_gap_do_not_clear(self):
        for age,gap in ((1.6,False),(.9,True)):
            with self.subTest(age=age,gap=gap):
                node=self.fixture();snapshot=node.memory.cells.copy()
                if gap:RememberedObstacles.record_observation_continuity(node,10.5)
                self.finish(node,10.,age,(np.array([0]),np.array([[9.,8.]]),
                            np.array([True])),snapshot)
                self.assertEqual(node.memory.cells,snapshot)
                self.assertEqual(node.stale_clearing_results,1)
                node.publish_maps.assert_not_called()

    def test_dispatch_gap_with_continuous_scans_keeps_confirmation_progress(self):
        for sensor_gap in (False,True):
            with self.subTest(sensor_gap=sensor_gap):
                node=self.fixture();node.last_clearing_stamp=10.
                key=next(iter(node.memory.cells));cell=node.memory.cells[key]
                node.memory.cells[key]=(*cell[:3],9.5,7,*cell[5:])
                node.last_observation_stamp=10.
                if not sensor_gap:
                    for stamp in (10.1,10.2,10.3,10.4,10.5,10.6):
                        RememberedObstacles.record_observation_continuity(node,stamp)
                RememberedObstacles.record_observation_continuity(node,10.7)
                scan=PointCloud2();scan.header.stamp.sec=10
                scan.header.stamp.nanosec=700000000
                scan.height=scan.width=1;scan.point_step=scan.row_step=12
                scan.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
                             for i,k in enumerate(('x','y','z'))]
                scan.data=np.array([[3.,0.,0.]],dtype='<f4').tobytes()
                node.processed.append((scan,(0.,0.,0.)))
                node.clouds[(10,700000000)]=scan
                node.observation_transform.return_value=SimpleNamespace(transform=
                    SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.,z=.2),
                                    rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
                node.now=10.8;node.next_clearing=0.
                RememberedObstacles.update_clearing(node)
                args=node.worker.submit.call_args.args
                changes,_=args[0](*args[1:])
                self.assertEqual(bool(changes[2][0]),not sensor_gap)
                self.assertEqual(int(changes[1][0,1]),1 if sensor_gap else 8)

    def test_validated_geometry_survives_dispatch_latency_but_expires_at_bound(self):
        for age,dispatched in ((.65,True),(.80,False)):
            with self.subTest(age=age):
                node=self.fixture();node.next_clearing=0.
                scan=PointCloud2();scan.header.stamp.sec=10
                node.processed.append((scan,(0.,0.,0.)))
                node.clouds[(10,0)]=scan
                node.observation_transform.return_value=SimpleNamespace(transform=
                    SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.,z=.2)))
                node.now=10.+age
                RememberedObstacles.update_clearing(node)
                self.assertEqual(node.worker.submit.called,dispatched)

    def test_unmeasured_ground_still_allows_paired_real_3d_rays(self):
        node=self.fixture();node.next_clearing=0.
        key=next(iter(node.memory.cells));cell=node.memory.cells[key]
        node.memory.cells[key]=(*cell[:3],9.,7,*cell[5:])
        scan=PointCloud2();scan.header.stamp.sec=10
        scan.fields=[PointField(name='positive_only',offset=12,
                                datatype=PointField.UINT8,count=1)]
        cloud=PointCloud2();cloud.header.stamp.sec=10
        cloud.height=cloud.width=1;cloud.point_step=cloud.row_step=12
        cloud.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
                      for i,k in enumerate(('x','y','z'))]
        cloud.data=np.array([[3.,0.,0.]],dtype='<f4').tobytes()
        node.processed.append((scan,(0.,0.,0.)));node.clouds[(10,0)]=cloud
        node.observation_transform.return_value=SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.,y=0.,z=.2),
            rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        node.now=10.1
        RememberedObstacles.update_clearing(node)
        self.assertTrue(node.worker.submit.called)
        args=node.worker.submit.call_args.args
        self.assertIsNone(args[8],'No floor/column inference without measured ground')
        changes,_=args[0](*args[1:])
        self.assertTrue(changes[2][0],'Eight real rays through the volume should clear it')

    def test_unmeasured_ground_without_paired_raw_cloud_cannot_clear(self):
        node=self.fixture();node.next_clearing=0.
        scan=PointCloud2();scan.header.stamp.sec=10
        scan.fields=[PointField(name='positive_only',datatype=PointField.UINT8,count=1)]
        node.processed.append((scan,(0.,0.,0.)));node.now=10.1
        RememberedObstacles.update_clearing(node)
        node.worker.submit.assert_not_called()


if __name__=='__main__':unittest.main()
