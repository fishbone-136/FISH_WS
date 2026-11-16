#!/usr/bin/python3
"""Healthy-to-sparse FAST-LIO regression, localhost only, no robot commands."""
import os
if os.environ.get('ROS_DOMAIN_ID') != '90' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=90 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
import json
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry

root=Path(os.environ['GO2_HUMBLE_PROJECT_ROOT'])


def main():
    rclpy.init();n=rclpy.create_node('fastlio_sparse_fixture')
    imu_pub=n.create_publisher(Imu,'/livox/imu',2000)
    cloud_pub=n.create_publisher(PointCloud2,'/livox/lidar',qos_profile_sensor_data)
    odom=[]
    n.create_subscription(Odometry,'/Odometry',lambda m:odom.append(m),qos_profile_sensor_data)
    # Three differently oriented planes give the estimator ordinary room geometry.
    a=np.arange(.3,4.01,.12);b=np.arange(-2.,2.01,.12);z=np.arange(-.37,1.51,.12)
    points=np.array([(x,y,-.37) for x in a for y in b]+
                    [(4.,y,h) for y in b for h in z]+
                    [(x,2.,h) for x in a for h in z],dtype='<f4')
    sparse=points[np.linspace(0,len(points)-1,33,dtype=int)]
    state={'sparse':False,'last_imu':None}
    dtype=np.dtype({'names':['x','y','z','intensity','tag','line','timestamp'],
        'formats':['<f4','<f4','<f4','<f4','u1','u1','<f8'],
        'offsets':[0,4,8,12,16,17,18],'itemsize':26})
    def imu():
        m=Imu();m.header.frame_id='livox_frame'
        stamp=n.get_clock().now()-rclpy.duration.Duration(seconds=.005)
        m.header.stamp=stamp.to_msg();m.linear_acceleration.z=1.
        imu_pub.publish(m)
    def cloud():
        xyz=sparse if state['sparse'] else points
        p=np.zeros(len(xyz),dtype=dtype)
        for i,k in enumerate(('x','y','z')):p[k]=xyz[:,i]
        p['intensity']=100.;p['line']=np.arange(len(p))%6
        stamp=n.get_clock().now()-rclpy.duration.Duration(seconds=.08)
        p['timestamp']=stamp.nanoseconds+np.linspace(0,45e6,len(p))
        m=PointCloud2();m.header.frame_id='livox_frame';m.header.stamp=stamp.to_msg()
        m.height=1;m.width=len(p);m.point_step=26;m.row_step=26*len(p);m.is_dense=True
        m.fields=[PointField(name=k,offset=dtype.fields[k][1],
            datatype=PointField.UINT8 if k in ('tag','line') else
                     PointField.FLOAT64 if k=='timestamp' else PointField.FLOAT32,count=1)
            for k in dtype.names]
        m.data=p.tobytes();cloud_pub.publish(m)
    n.create_timer(.005,imu);n.create_timer(1./15.,cloud)
    log_path=root/'reports/navigation_lyingdown_fastlio_sparse_native_20261102.log'
    with log_path.open('w') as log:
        p=subprocess.Popen([str(root/'ros2_ws/install/fast_lio/lib/fast_lio/fastlio_mapping'),
            '--ros-args','--params-file',str(root/'ros2_ws/src/go2_lio_navigation/config/fast_lio_hap.yaml'),
            '-p','preprocess.scan_rate:=15','-p','point_filter_num:=1',
            '-p','mapping.imu_init_duration:=0.6','-p','mapping.initial_map_duration:=0.5',
            '-p','mapping.initial_map_max_duration:=3.0','-p','mapping.input_warmup_duration:=0.1'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            end=time.monotonic()+12.
            while len(odom)<8 and time.monotonic()<end and p.poll() is None:
                rclpy.spin_once(n,timeout_sec=.005)
            assert len(odom)>=8, ('healthy geometry did not track',log_path.read_text()[-3000:])
            before=len(odom);state['sparse']=True
            end=time.monotonic()+4.
            while p.poll() is None and time.monotonic()<end:
                rclpy.spin_once(n,timeout_sec=.005)
            assert p.poll() is not None, 'Sparse geometry failed to stop estimator'
            text=log_path.read_text()
            assert 'persistent weak scan matching' in text, text[-3000:]
            assert 'minimum_matches=50' in text, text[-3000:]
            assert 'IMU initialization in progress' in text, text[:3000]
            report={'healthy_odometry_samples':before,'odom_after_sparse_switch':len(odom)-before,
                    'quality_min_matches_preserved':50,'persistent_sparse_geometry_stopped':True,
                    'process_exit_code':p.returncode,'isolated_ros_domain':90}
            (root/'reports/navigation_lyingdown_fastlio_sparse_20261102.json').write_text(json.dumps(report,indent=2))
            print(json.dumps(report),flush=True)
        finally:
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3.)
            n.destroy_node()
            if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
