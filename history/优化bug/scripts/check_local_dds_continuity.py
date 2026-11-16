#!/usr/bin/python3
"""Check inter-process local DDS continuity while observing robot NIC carrier."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--publisher',action='store_true');p.add_argument('--seconds',type=float,default=30.)
    p.add_argument('--output',type=Path);args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='86':raise SystemExit('Requires test ROS_DOMAIN_ID=86')
    rclpy.init();node=rclpy.create_node('local_dds_probe_publisher' if args.publisher else 'local_dds_probe_reader')
    begin=time.monotonic()
    if args.publisher:
        pub=node.create_publisher(Imu,'/test/local_dds_continuity',qos_profile_sensor_data)
        def send():
            m=Imu();m.header.stamp=node.get_clock().now().to_msg();pub.publish(m)
        node.create_timer(.005,send)
        while time.monotonic()-begin<args.seconds:rclpy.spin_once(node,timeout_sec=.01)
    else:
        stamps=[];ages=[];carrier=[]
        def receive(m):
            stamps.append(time.monotonic());ages.append(node.get_clock().now().nanoseconds/1e9-m.header.stamp.sec-m.header.stamp.nanosec/1e9)
        node.create_subscription(Imu,'/test/local_dds_continuity',receive,qos_profile_sensor_data)
        child=subprocess.Popen(['/usr/bin/python3',str(Path(__file__).resolve()),'--publisher','--seconds',str(args.seconds+2.)])
        next_carrier=0.
        try:
            while time.monotonic()-begin<args.seconds:
                rclpy.spin_once(node,timeout_sec=.01)
                if time.monotonic()>=next_carrier:
                    next_carrier=time.monotonic()+.05
                    try:value=Path('/sys/class/net/eno1/carrier').read_text().strip()
                    except OSError:value='unavailable'
                    if not carrier or value!=carrier[-1][1]:carrier.append([round(time.monotonic()-begin,3),value])
            gaps=[b-a for a,b in zip(stamps,stamps[1:])]
            rate=(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 else 0.
            result={'duration':args.seconds,'messages':len(stamps),'received_hz':round(rate,2),
                    'maximum_gap_ms':round(1000*max(gaps,default=0.),2),
                    'maximum_age_ms':round(1000*max(ages,default=0.),2),'robot_nic_carrier_events':carrier,
                    'pass':len(stamps)>100 and rate>150. and max(gaps,default=1.)<.15}
            if args.output:args.output.write_text(json.dumps(result,indent=2))
            print(json.dumps(result,indent=2),flush=True)
        finally:
            if child.poll() is None:child.send_signal(signal.SIGINT)
            child.wait(timeout=5.)
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
