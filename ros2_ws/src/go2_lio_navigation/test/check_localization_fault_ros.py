#!/usr/bin/python3
"""Unexpected LIO exit parks navigation while the owning launch stays alive."""
import os
from pathlib import Path
import signal
import subprocess
import time
if os.environ.get('ROS_DOMAIN_ID')!='90' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=90 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI']=('<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast>'
    '</General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
    '</Peers></Discovery></Domain></CycloneDDS>')
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ManageLifecycleNodes
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


def main():
    rclpy.init();node=rclpy.create_node('unexpected_lio_exit_test')
    received={'cancel':0,'pause':0,'valid':None,'marker':None}
    def cancel(_,response):received['cancel']+=1;return response
    def pause(request,response):
        assert request.command==ManageLifecycleNodes.Request.PAUSE
        received['pause']+=1;response.success=True;return response
    node.create_service(CancelGoal,'/navigate_to_pose/_action/cancel_goal',cancel)
    node.create_service(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes',pause)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool,'/lio/localization_valid',lambda m:received.update(valid=m.data),qos)
    node.create_subscription(Marker,'/safety/navigation_fault_marker',lambda m:received.update(marker=m),qos)
    process=subprocess.Popen(['ros2','launch','go2_lio_navigation','go2_lio_navigation.launch.py',
        'start_hap_driver:=false','enable_go2_base:=false','use_rviz:=false','enable_nav2:=false',
        'enable_terrain:=false','enable_perception:=false','enable_loop_closure:=false',
        'enable_environment_mapping:=false'],start_new_session=True)
    def until(check,timeout=15.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if check():return
        raise AssertionError(str(received))
    def lio_child():
        for p in Path('/proc').glob('[0-9]*'):
            try:
                fields=(p/'stat').read_text().rsplit(')',1)[1].split()
                argv=(p/'cmdline').read_bytes().split(b'\0')
                if int(fields[1])==process.pid and argv and Path(argv[0].decode()).name=='fastlio_mapping':
                    return int(p.name)
            except (OSError,ValueError):pass
        return None
    try:
        until(lambda:lio_child() is not None)
        os.kill(lio_child(),signal.SIGTERM)
        until(lambda:received['cancel']>0 and received['pause']>0 and received['marker'] is not None)
        deadline=time.monotonic()+1.;until(lambda:time.monotonic()>deadline,timeout=2.)
        assert process.poll() is None,'Unexpected LIO exit still closes the owning launch / RViz'
        assert received['valid'] is False
        assert received['marker'].type==Marker.TEXT_VIEW_FACING
        assert received['marker'].color.r>.9 and 'STOPPED' in received['marker'].text
        print('PASS unexpected FAST-LIO exit retains the owning launch and visualization process tree',flush=True)
        print('PASS the fault latches motion off, cancels the goal and pauses the Nav2 lifecycle',flush=True)
        print('PASS a persistent visible fault replaces the disappearing navigation window',flush=True)
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=8.)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
