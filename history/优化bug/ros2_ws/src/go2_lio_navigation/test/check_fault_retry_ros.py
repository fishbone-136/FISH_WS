#!/usr/bin/python3
"""Run the real fault guard against delayed, initially rejecting services."""
import json
import os
import signal
import subprocess
import time

if os.environ.get('ROS_DOMAIN_ID')!='91' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=91 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI']=('<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast>'
    '</General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_prefix
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String


def main():
    rclpy.init();node=rclpy.create_node('fault_request_regression')
    process=subprocess.Popen([get_package_prefix('go2_lio_navigation')+
                              '/lib/go2_lio_navigation/navigation_fault_guard.py'])
    health=[];statuses=[];counts={'cancel':0,'pause':0}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool,'/lio/localization_valid',lambda m:health.append(m.data),qos)
    node.create_subscription(String,'/safety/navigation_fault',lambda m:statuses.append(json.loads(m.data)),qos)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def cancel(_,response):
        counts['cancel']+=1
        response.return_code=(CancelGoal.Response.ERROR_REJECTED if counts['cancel']==1 else
                              CancelGoal.Response.ERROR_NONE)
        return response
    def pause(_,response):
        counts['pause']+=1;response.success=counts['pause']>1;return response
    try:
        spin(1.5)
        assert health and health[-1] is False,'Unavailable services prevented immediate motion stop'
        node.create_service(CancelGoal,'/navigate_to_pose/_action/cancel_goal',cancel)
        node.create_service(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes',pause)
        end=time.monotonic()+7.
        while time.monotonic()<end:
            spin(.1)
            if statuses and statuses[-1].get('navigation_paused') and statuses[-1].get('navigation_cancelled'):
                break
        else:raise AssertionError(('Failed RPCs never retried',counts,statuses[-1:]))
        assert counts=={'cancel':2,'pause':2},counts
        spin(1.5)
        assert counts=={'cancel':2,'pause':2},'Acknowledged RPCs were flooded'
        assert all(v is False for v in health)
        print('PASS delayed services and rejected cancel/pause requests recover without opening motion',flush=True)
        print('PASS successful acknowledgements stop RPC retries',flush=True)
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5.)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
