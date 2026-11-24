#!/usr/bin/python3
"""Isolated full-pose/planar scan alignment regression; sends no motion goals."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
if os.environ.get('ROS_DOMAIN_ID')!='89' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=89 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI']=('<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast>'
    '</General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
    '</Peers></Discovery></Domain></CycloneDDS>')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,LaserScan,PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster
from ament_index_python.packages import get_package_prefix


def main():
    rclpy.init();node=rclpy.create_node('full_pose_scan_alignment_test')
    children=[]
    def start(package,executable,args):
        p=subprocess.Popen([str(Path(get_package_prefix(package))/'lib'/package/executable),
                            '--ros-args',*args]);children.append(p)
    try:
        start('go2_lio_navigation','lio_base_bridge',['-p','startup_samples:=1',
            '-p','recovery_samples:=1','-p','planar_output:=true',
            '-p','full_body_frame:=base_link_3d','-p','cancel_navigation_on_invalid:=false',
            '-p','clear_costmaps_on_invalid:=false','--log-level','warn'])
        start('go2_perception','pointcloud_to_laserscan_node',['-r','cloud_in:=/test/tilted_cloud',
            '-r','scan:=/test/aligned_scan','-r','obstacle_points:=/test/body_obstacles',
            '-r','ground_plane:=/test/aligned_ground',
            '-p','target_frame:=base_link_3d','-p','scan_frame:=base_link','-p','level_scan:=true',
            '-p','ground_filter_enabled:=true','-p','observed_free_space_only:=true',
            '-p','require_ground_for_scan:=true',
            '-p','ground_pose_topic:=/Odometry_validated','-p','range_min:=0.25',
            '-p','range_max:=5.5','-p','point_spread_radius:=0.0',
            '-p','max_fill_gap_bins:=0','-p','angle_increment:=0.00698','--log-level','warn'])
        start('go2_lio_navigation','obstacle_memory.py',['-p','body_frame:=base_link_3d',
            '-p','observation_topic:=/test/body_obstacles','-p','raw_cloud_topic:=/test/tilted_cloud',
            '-p','ground_plane_topic:=/test/aligned_ground','--log-level','warn'])
        buffer=Buffer();listener=TransformListener(buffer,node)
        static=StaticTransformBroadcaster(node);mount=TransformStamped()
        mount.header.frame_id='base_link_3d';mount.child_frame_id='livox_frame'
        mount.transform.translation.x=.14;mount.transform.translation.z=.13
        mount.transform.rotation.w=1.;static.sendTransform(mount)
        odom=node.create_publisher(Odometry,'/Odometry',10)
        cloud=node.create_publisher(PointCloud2,'/test/tilted_cloud',qos_profile_sensor_data)
        latest={}
        node.create_subscription(LaserScan,'/test/aligned_scan',lambda m:latest.update(scan=m),qos_profile_sensor_data)
        node.create_subscription(PointCloud2,'/test/body_obstacles',lambda m:latest.update(points=m),qos_profile_sensor_data)
        node.create_subscription(PointCloud2,'/safety/remembered_obstacles',lambda m:latest.update(memory=m),qos_profile_sensor_data)
        yaw=.65;pitch=.22;roll=-.15
        cy,sy,cp,sp,cr,sr=math.cos(yaw),math.sin(yaw),math.cos(pitch),math.sin(pitch),math.cos(roll),math.sin(roll)
        rz=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        rotation=rz@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
        base=np.array([1.,2.,.10]);offset=np.array([.14,0.,.13])
        # Build a flat world floor and a genuine small box in the yaw frame.
        ground=np.array([[x,y,-.40] for x in np.arange(.6,3.5,.10) for y in np.arange(-1.,1.,.10)])
        box=np.array([[1.5+d,.25+e,z] for d in (0.,.01) for e in (-.01,0.,.01) for z in (-.15,.05,.25)])
        points=(np.r_[ground,box]@rz.T)@rotation-offset
        fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(('x','y','z'))]
        age=[0.]
        include_ground=[False]
        def publish():
            stamp=node.get_clock().now().to_msg();m=Odometry();m.header.stamp=stamp
            m.header.frame_id='odom_lio';m.child_frame_id='livox_frame'
            position=base+rotation@offset
            m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,position)
            q=m.pose.pose.orientation
            q.w=math.cos(yaw/2)*math.cos(pitch/2)*math.cos(roll/2)+math.sin(yaw/2)*math.sin(pitch/2)*math.sin(roll/2)
            q.x=math.cos(yaw/2)*math.cos(pitch/2)*math.sin(roll/2)-math.sin(yaw/2)*math.sin(pitch/2)*math.cos(roll/2)
            q.y=math.cos(yaw/2)*math.sin(pitch/2)*math.cos(roll/2)+math.sin(yaw/2)*math.cos(pitch/2)*math.sin(roll/2)
            q.z=math.sin(yaw/2)*math.cos(pitch/2)*math.cos(roll/2)-math.cos(yaw/2)*math.sin(pitch/2)*math.sin(roll/2)
            if age[0]:m.header.stamp=Time(nanoseconds=node.get_clock().now().nanoseconds-int(age[0]*1e9)).to_msg()
            odom.publish(m)
            h=TransformStamped().header;h.frame_id='livox_frame';h.stamp=stamp
            frame = points if include_ground[0] else points[-len(box):]
            cloud.publish(point_cloud2.create_cloud(h,fields,frame.astype(np.float32)))
        node.create_timer(1./15.,publish)
        def until(check,timeout=12.):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.02)
                if check():return
            raise AssertionError('Timed out waiting for '+str(latest.keys()))
        until(lambda:'points' in latest and 'memory' in latest
              and any(field.name=='unclassified' for field in latest['points'].fields))
        end=time.monotonic()+.3;until(lambda:time.monotonic()>=end,timeout=1.)
        assert 'scan' not in latest
        assert latest['points'].width==0
        assert latest['memory'].header.frame_id=='unavailable'
        print('PASS no ground: unclassified input cannot paint a navigation obstacle',flush=True)
        include_ground[0]=True
        until(lambda:'scan' in latest and 'points' in latest and 'memory' in latest
              and latest['memory'].header.frame_id=='odom_lio' and latest['memory'].width>0)
        scan=latest['scan'];assert scan.header.frame_id=='base_link'
        assert latest['points'].header.frame_id=='base_link_3d'
        measured=np.array([[r*math.cos(scan.angle_min+i*scan.angle_increment),r*math.sin(scan.angle_min+i*scan.angle_increment)]
                          for i,r in enumerate(scan.ranges) if math.isfinite(r)])
        assert len(measured)>0
        error=np.linalg.norm(measured-[1.5,.25],axis=1)
        assert error.min()<.025,(error.min(),measured)
        assert error.max()<.06,('Floor/self returns appeared as scan obstacles',measured)
        memory=np.array([list(v) for v in point_cloud2.read_points(latest['memory'],field_names=('x','y','z'))])
        expected=base+rz@np.array([1.5,.25,0.])
        memory_error=np.linalg.norm(memory[:,:2]-expected[:2],axis=1)
        assert memory_error.min()<.025,('Persistent XYZ was tilted twice',memory_error.min())
        physical=buffer.lookup_transform('odom_lio','base_link_3d',Time())
        planar=buffer.lookup_transform('odom_lio','base_link',Time())
        assert abs(physical.transform.translation.z-.10)<1e-5
        assert abs(planar.transform.translation.z)<1e-6
        assert abs(planar.transform.rotation.x)+abs(planar.transform.rotation.y)<1e-6
        assert abs(physical.transform.rotation.y)>.05
        print('PASS tilted physical body and planar navigation have distinct, coherent TF frames',flush=True)
        print('PASS tilted box scan aligns to full-pose world geometry within %.1f mm'%(1000*error.min()),flush=True)
        print('PASS measured XYZ remains physical; ground/self returns do not become obstacles',flush=True)
        print('PASS the real memory node and the levelled scan occupy the same world XY',flush=True)
        age[0]=.8
        end=time.monotonic()+.6;until(lambda:time.monotonic()>=end,timeout=1.)
        stamp=latest['scan'].header.stamp
        end=time.monotonic()+.4;until(lambda:time.monotonic()>=end,timeout=1.)
        assert latest['scan'].header.stamp==stamp,'A stale attitude manufactured fresh navigation scans'
        print('PASS stale full-pose attitude cannot refresh the navigation scan',flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()
        for p in children:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in children:p.wait(timeout=6)


if __name__=='__main__':main()
