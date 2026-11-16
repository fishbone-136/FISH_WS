#!/usr/bin/env python3
"""Verify stable world cells, real observation timestamps and 8 Hz terrain."""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

if os.environ.get('ROS_DOMAIN_ID')!='91' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=91 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI']=(
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
    '</Peers></Discovery></Domain></CycloneDDS>')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from environment_mapper import cloud_message

rclpy.init();node=rclpy.create_node('terrain_grid_regression')
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
health=node.create_publisher(Bool,'/lio/localization_valid',latched)
cloud=node.create_publisher(PointCloud2,'/test/terrain_cloud',1)
tf=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom_lio';t.transform.rotation.w=1.
static.sendTransform(t)
received=[];stamps=set();state={'enabled':True,'ticks':0,'valid':True}
global_grids={};surfaces=[]
points=np.array([[1.025,.025,.10],[1.025,.025,.50],[3.10,.01,-.50],[3.11,.02,-.50]])
def tick():
    state['ticks']+=1
    stamp=(node.get_clock().now()-rclpy.duration.Duration(seconds=.03)).to_msg()
    t=TransformStamped();t.header.frame_id='odom_lio';t.child_frame_id='base_link';t.header.stamp=stamp
    t.transform.rotation.w=1.;t.transform.translation.x=.009*math.sin(state['ticks'])
    t.transform.translation.y=.009*math.cos(state['ticks']);t.transform.translation.z=.5
    tf.sendTransform(t);health.publish(Bool(data=state['valid']))
    if state['enabled']:
        stamps.add((stamp.sec,stamp.nanosec));cloud.publish(cloud_message(points,stamp,'odom_lio'))
node.create_timer(1./15.,tick)
node.create_subscription(OccupancyGrid,'/traversability/local',
    lambda m:received.append((time.monotonic(),m)),latched)
node.create_subscription(OccupancyGrid,'/traversability/global',
    lambda m:global_grids.update({(m.header.stamp.sec,m.header.stamp.nanosec):m}),latched)
node.create_subscription(PointCloud2,'/traversability/terrain_surface',
    lambda m:surfaces.append(m),latched)
prefix=Path(get_package_prefix('go2_lio_navigation'))
config=Path(get_package_share_directory('go2_lio_navigation'))/'config/traversability.yaml'
child=subprocess.Popen([str(prefix/'lib/go2_lio_navigation/traversability_mapper'),
    '--ros-args','--params-file',str(config),'-p','local_cloud_topic:=/test/terrain_cloud','--log-level','warn'])
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if child.poll() is not None:raise AssertionError('Terrain child exited')
        rclpy.spin_once(node,timeout_sec=.01)
try:
    spin(3.);began=time.monotonic();received.clear();spin(3.)
    samples=list(received);assert len(samples)>=18,len(samples)
    for _,m in samples:
        res=m.info.resolution;o=m.info.origin.position
        assert abs(res-.08)<1.e-6
        assert abs(o.x/.08-round(o.x/.08))<1.e-6
        assert abs(o.y/.08-round(o.y/.08))<1.e-6
        ix=math.floor((1.025-o.x)/res);iy=math.floor((.025-o.y)/res)
        assert m.data[iy*m.info.width+ix]==100
        assert abs(o.x+(ix+.5)*res-1.00)<1.e-5
        assert (m.header.stamp.sec,m.header.stamp.nanosec) in stamps
        g=global_grids[(m.header.stamp.sec,m.header.stamp.nanosec)]
        assert g.header.frame_id==m.header.frame_id=='odom_lio'
        gx=round((m.info.origin.position.x-g.info.origin.position.x)/res)
        gy=round((m.info.origin.position.y-g.info.origin.position.y)/res)
        source=np.asarray(g.data).reshape(g.info.height,g.info.width)
        np.testing.assert_array_equal(np.asarray(m.data).reshape(m.info.height,m.info.width),
                                      source[gy:gy+m.info.height,gx:gx+m.info.width])
    hz=(len(samples)-1)/(samples[-1][0]-samples[0][0])
    print('PASS stable 8 cm occupied world cell under +/-9 mm robot jitter; %.2f Hz'%hz,flush=True)
    print('PASS terrain map timestamps match actual cloud acquisitions',flush=True)
    print('PASS local/global terrain share exactly the same measured cells and acquisition stamp',flush=True)
    assert surfaces
    from environment_mapper import xyz_from_cloud
    z=xyz_from_cloud(surfaces[-1])[:,2]
    assert np.any(np.isclose(z,-.5)) and np.any(np.isclose(z,.1))
    print('PASS live 2.5D terrain surface retains measured metric heights',flush=True)
    points=points[2:];spin(.7)
    m=received[-1][1];res=m.info.resolution;o=m.info.origin.position
    ix=math.floor((1.025-o.x)/res);iy=math.floor((.025-o.y)/res)
    assert m.data[iy*m.info.width+ix]!=100
    print('PASS a removed dynamic obstacle leaves live terrain within 0.7 seconds',flush=True)
    state['enabled']=False;spin(.6);count=len(received);spin(.5)
    assert len(received)==count,'Stale cloud kept producing newly stamped terrain'
    print('PASS stale input stops terrain refresh rather than manufacturing fresh evidence',flush=True)
    state['valid']=False;spin(.3)
    assert surfaces[-1].width==0
    assert all(v==-1 for v in received[-1][1].data)
    print('PASS invalid localization clears latched live 2.5D geometry and terrain costs',flush=True)
finally:
    child.send_signal(signal.SIGINT)
    try:child.wait(timeout=5)
    except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)
    node.destroy_node();rclpy.shutdown()
