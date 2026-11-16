#!/usr/bin/python3
"""Check projection options at the actual ROS node boundary, off hardware."""
import math
import os
import resource
import signal
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=87 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI']=('<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast>'
    '</General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
resource.setrlimit(resource.RLIMIT_CORE,(0,0))
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointField, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def main():
    exe=get_package_prefix('go2_perception')+'/lib/go2_perception/pointcloud_to_laserscan_node'
    base=[exe,'--ros-args']
    for args, expected in [
        (['-p','target_frame:=body','-p','scan_frame:=planar'], 'relabelling is not a transform'),
        (['-p','target_frame:=body','-p','scan_frame:=planar','-p','level_scan:=true'],
         'requires target_frame and ground_pose_topic'),
        (['-p','angle_increment:=0.0'], 'Invalid scan angular/range limits'),
        (['-p','ground_pose_topic:=/pose'], 'nonempty target_frame')]:
        result=subprocess.run(base+args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=5.)
        assert result.returncode!=0 and expected in result.stdout.decode(),result.stdout.decode()
    print('PASS invalid frame and angular configurations fail explicitly before processing clouds',flush=True)
    rclpy.init();node=rclpy.create_node('scan_configuration_regression')
    process=subprocess.Popen(base+['-p','target_frame:=body','-p','scan_frame:=planar',
        '-p','level_scan:=true','-p','ground_pose_topic:=/test/pose',
        '-p','ground_filter_enabled:=false','-p','range_max:=5.5',
        '-r','cloud_in:=/test/cloud','-r','scan:=/test/scan','--log-level','warn'])
    scans=[]
    node.create_subscription(LaserScan,'/test/scan',scans.append,qos_profile_sensor_data)
    poses=node.create_publisher(Odometry,'/test/pose',10)
    clouds=node.create_publisher(PointCloud2,'/test/cloud',qos_profile_sensor_data)
    fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
    def publish():
        stamp=node.get_clock().now().to_msg()
        pose=Odometry();pose.header=Header(stamp=stamp,frame_id='odom_lio');pose.child_frame_id='body'
        pose.pose.pose.orientation.y=math.sin(.2/2);pose.pose.pose.orientation.w=math.cos(.2/2)
        poses.publish(pose)
        clouds.publish(point_cloud2.create_cloud(Header(stamp=stamp,frame_id='body'),fields,
                                                [[1.5,.2,.3]]))
    timer=node.create_timer(1./15.,publish)
    try:
        deadline=time.monotonic()+6.
        while not scans and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        assert scans,'level_scan silently stalls when ground filtering is disabled'
        scan=scans[-1];assert scan.header.frame_id=='planar'
        measured=[(r*math.cos(scan.angle_min+i*scan.angle_increment),
                   r*math.sin(scan.angle_min+i*scan.angle_increment))
                  for i,r in enumerate(scan.ranges) if math.isfinite(r)]
        expected=(math.cos(.2)*1.5+math.sin(.2)*.3,.2)
        assert measured and min(math.dist(v,expected) for v in measured)<.03,(measured,expected)
        print('PASS attitude levelling works with ground filtering disabled and preserves actual XY',flush=True)
    finally:
        node.destroy_timer(timer)
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5.)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
