"""Persistent geometry uses a complete physical pose, including mounting."""
from pathlib import Path
import math
import sys
import unittest

import numpy as np
from geometry_msgs.msg import TransformStamped,PoseStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from obstacle_memory import full_pose_transform,cloud_points,compute_clearing,world_ground_plane
from obstacle_memory_model import ObstacleMemory
from environment_mapper import cloud_message


class ObservationGeometryTests(unittest.TestCase):
    def test_sensor_pitch_and_height_are_not_flattened_into_a_false_obstacle_position(self):
        odom=Odometry();odom.header.frame_id='odom_lio';odom.child_frame_id='livox_frame'
        odom.pose.pose.orientation.y=math.sin(math.radians(20)/2)
        odom.pose.pose.orientation.w=math.cos(math.radians(20)/2)
        odom.pose.pose.position.z=.35
        relative=TransformStamped();relative.transform.rotation.w=1.
        relative.transform.translation.x=-.14;relative.transform.translation.z=-.13
        actual=full_pose_transform(odom,relative)
        cloud=cloud_message(np.array([[1.,.3,.2]]),odom.header.stamp,'base_link')
        points,_=cloud_points(cloud,actual)
        angle=math.radians(20)
        expected=[math.cos(angle)*.86+math.sin(angle)*.07,.3,
                  .35-math.sin(angle)*.86+math.cos(angle)*.07]
        np.testing.assert_allclose(points[0],expected,atol=1.e-7)
        self.assertGreater(abs(points[0,0]-.86),.02)

    def test_rejects_nonfinite_or_nonunit_pose_before_geometry_is_remembered(self):
        odom=Odometry();odom.pose.pose.orientation.w=1.
        relative=TransformStamped();relative.transform.rotation.w=1.
        odom.pose.pose.position.x=math.nan
        with self.assertRaises(ValueError):full_pose_transform(odom,relative)
        odom.pose.pose.position.x=0.;odom.pose.pose.orientation.w=0.
        with self.assertRaises(ValueError):full_pose_transform(odom,relative)

    def test_clearing_worker_never_adds_a_point_from_an_angularly_expanded_scan(self):
        memory=ObstacleMemory(require_3d_clearing=True)
        memory.mark_xyz([[1.,0.,.1]],(0.,0.,0.),1.)
        memory.last_stamp=0.
        header=cloud_message(np.empty((0,3)),Time(sec=2),'base_link')
        rays=cloud_message(np.array([[3.,0.,-.37]]),header.header.stamp,'base_link')
        transform=TransformStamped();transform.transform.rotation.w=1.
        cells,_=compute_clearing(memory,header,(0.,0.,0.),rays,transform)
        self.assertTrue(set(cells).issubset({(20,0)}))

    def test_ground_plane_uses_the_same_full_pose_as_obstacle_heights(self):
        plane=PoseStamped();plane.pose.orientation.w=1.;plane.pose.position.z=-.37
        transform=TransformStamped();angle=math.radians(15.)
        transform.transform.rotation.y=math.sin(angle/2.);transform.transform.rotation.w=math.cos(angle/2.)
        transform.transform.translation.x=.2;transform.transform.translation.z=.3
        world=world_ground_plane(plane,transform)
        cloud=cloud_message(np.array([[1.,.1,-.37],[1.,.1,-.27]]),Time(sec=1),'base_link')
        points,_=cloud_points(cloud,transform)
        heights=points@world[:3]+world[3]
        np.testing.assert_allclose(heights,[0.,.10],atol=1.e-7)


if __name__=='__main__':unittest.main()
