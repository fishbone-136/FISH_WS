#!/usr/bin/python3
"""Closed-loop rear goal with a nearby corner obstacle, using no robot hardware."""
import math
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

if os.environ.get('ROS_DOMAIN_ID')!='88' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=88 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI']=(
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
    '</Peers></Discovery></Domain></CycloneDDS>')
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'go2_navigation/test'))
from check_obstacle_pipeline import Fixture
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Bool,String
from ament_index_python.packages import get_package_prefix,get_package_share_directory


def main():
    rclpy.init();fixture=Fixture();children=[]
    def start(args):
        p=subprocess.Popen(args,start_new_session=True);children.append(p);return p
    try:
        diagnostic={}
        fixture.create_subscription(String,'/safety/motion_diagnostics',
            lambda m:diagnostic.update(json.loads(m.data)),qos_profile_sensor_data)
        health=fixture.create_publisher(Bool,'/lio/localization_valid',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        fixture.create_timer(.05,lambda:health.publish(Bool(data=True)))
        fixture.floor_z=-.37
        fixture.floor_points=np.array([[x,y] for x in np.arange(-5.,5.1,.12)
                                            for y in np.arange(-5.,5.1,.12)])
        # Keep the initial pose outside the projector's existing 4 cm point
        # spread as well as the protected footprint. The old (0.40,0.30)
        # setup already put expanded returns inside the emergency rectangle.
        fixture.box_x=.42;fixture.box_y=.34;fixture.box_width=.03
        fixture.box_heights=[.11,.15,.20,.30,.45,.60,.70]
        fixture.simulate_motion=True
        projection=Path(get_package_prefix('go2_perception'))/'lib/go2_perception/pointcloud_to_laserscan_node'
        for name,topic,scan in [('return_safety','/test/cloud_fast','/scan_safety'),
                                ('return_deskew','/test/cloud','/scan')]:
            start([str(projection),'--ros-args','-r','__node:='+name,'-r','cloud_in:='+topic,
                '-r','obstacle_points:='+('/obstacles/deskewed' if scan=='/scan' else '/obstacles/raw'),
                '-r','scan:='+scan,'-p','target_frame:=base_link','-p','ground_filter_enabled:=true',
                '-p','observed_free_space_only:=true','-p','range_min:=0.25','-p','range_max:=5.5',
                '--log-level','warn'])
        executable=Path(get_package_prefix('go2_lio_navigation'))/'lib/go2_lio_navigation'
        start(['/usr/bin/python3',str(executable/'obstacle_memory.py'),'--ros-args',
               '-p','raw_cloud_topic:=/test/cloud_fast','-p','full_pose_topic:=/odom'])
        start(['/usr/bin/python3',str(executable/'scan_safety_gate.py')])
        config=Path(get_package_share_directory('go2_lio_navigation'))/'config/collision_monitor.yaml'
        start(['/opt/ros/humble/lib/nav2_collision_monitor/collision_monitor',
               '--ros-args','--params-file',str(config),'--log-level','warn'])
        fixture.lifecycle(Transition.TRANSITION_CONFIGURE)
        fixture.lifecycle(Transition.TRANSITION_ACTIVATE)
        start(['ros2','launch','go2_navigation','go2_nav2.launch.py',
               'mapping_mode:=true','lio_mode:=true','log_level:=info'])
        fixture.until(fixture.navigator.server_is_ready,timeout=30.)
        client=fixture.create_client(GetState,'/bt_navigator/get_state')
        fixture.until(client.service_is_ready)
        deadline=time.monotonic()+15.
        while time.monotonic()<deadline:
            state=client.call_async(GetState.Request());fixture.until(state.done)
            if state.result().current_state.label=='active':break
            until=time.monotonic()+.1;fixture.until(lambda:time.monotonic()>=until)
        else:raise AssertionError('Navigator did not activate')
        fixture.until(lambda:fixture.costmap is not None and fixture.safe is not None)
        fixture.command_enabled=False
        target=fixture.goal_pose();target.header.frame_id='odom_lio'
        target.header.stamp=rclpy.time.Time().to_msg()
        target.pose.position.x=-1.5;target.pose.position.y=0.
        target.pose.orientation.z=0.;target.pose.orientation.w=1.
        goal=NavigateToPose.Goal();goal.pose=target
        started=time.monotonic();sent=fixture.navigator.send_goal_async(goal)
        fixture.until(sent.done);assert sent.result().accepted
        result=sent.result().get_result_async()
        fixture.until(lambda:any(abs(v[2])>.01 for v in fixture.raw_history),timeout=12.)
        first=next(v for v in fixture.raw_history if abs(v[2])>.01)
        assert first[2]<0.,('Rear turn started toward the obstructed side',first)
        print('PASS rear goal immediately selects the clear initial turn side',flush=True)
        fixture.until(result.done,timeout=45.)
        if result.result().status!=4:
            points=[(v*math.cos(fixture.scan.angle_min+i*fixture.scan.angle_increment),
                     v*math.sin(fixture.scan.angle_min+i*fixture.scan.angle_increment))
                    for i,v in enumerate(fixture.scan.ranges) if math.isfinite(v)]
            print('RETURN DEBUG',diagnostic,'monitor',fixture.monitor_command,'scan_points',points,flush=True)
        assert result.result().status==4,(result.result(),fixture.x,fixture.y,fixture.yaw)
        error=math.hypot(fixture.x+1.5,fixture.y)
        assert error<=fixture.goal_tolerance+1.e-6,error
        assert fixture.body_clearances and min(fixture.body_clearances)>=fixture.protection_margin-.001
        print('PASS closed-loop rear goal reaches %.3f m error in %.2f s without entering the padded obstacle' %
              (error,time.monotonic()-started),flush=True)
        first_safe=next((v[0]-started for v in fixture.safe_history
                        if v[0]>=started and (abs(v[1])>.001 or abs(v[2])>.001)),None)
        print('TIMING first rear-goal guarded motion after %.3f s' % first_safe,flush=True)
        # A second goal reverses the travel direction again, as the user's
        # outbound/return sequence does. Keep the same memory and costmaps.
        target.pose.position.x=0.
        goal=NavigateToPose.Goal();goal.pose=target
        started=time.monotonic();sent=fixture.navigator.send_goal_async(goal)
        fixture.until(sent.done);assert sent.result().accepted
        result=sent.result().get_result_async();fixture.until(result.done,timeout=45.)
        assert result.result().status==4,(result.result(),fixture.x,fixture.y,fixture.yaw)
        error=math.hypot(fixture.x,fixture.y)
        assert error<=fixture.goal_tolerance+1.e-6,error
        assert min(fixture.body_clearances)>=fixture.protection_margin-.001
        first_safe=next((v[0]-started for v in fixture.safe_history
                        if v[0]>=started and (abs(v[1])>.001 or abs(v[2])>.001)),None)
        print('PASS second reverse-direction goal reaches %.3f m error in %.2f s; first guarded motion %.3f s' %
              (error,time.monotonic()-started,first_safe),flush=True)
    finally:
        fixture.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for p in reversed(children):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in children:
            try:p.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
            try:os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError:pass


if __name__=='__main__':main()
