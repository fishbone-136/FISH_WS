"""Real Nav2 behaviors on a simulated base; localhost domain 98, no SDK/base driver."""
import os
if os.environ.get('ROS_DOMAIN_ID') != '98' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires ROS_DOMAIN_ID=98 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
os.environ['QT_QPA_PLATFORM']='offscreen'

import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import numpy as np

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from PyQt5.QtWidgets import QApplication
from geometry_msgs.msg import TransformStamped, Twist, PolygonStamped, Point32
from lifecycle_msgs.srv import GetState, ChangeState
from nav2_msgs.msg import Costmap
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from unitree_go.msg import SportModeState
from unitree_api.msg import Request, Response

from go2_ui.navigation.ros_bridge import RosNavigationWorker
from voice.text_motion import parse_relative_actions
from voice.visual_navigation import ApproachIntent, GroundedTarget, RgbdSnapshot


def main():
    root=Path(os.environ['GO2_HUMBLE_PROJECT_ROOT'])
    app=QApplication([])
    rclpy.init()
    node=rclpy.create_node('relative_navigation_simulated_base')
    executor=MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    sys.path.insert(0,str(root/'ros2_ws/src/go2_lio_navigation/scripts'))
    from idle_stand_guard import IdleStandGuard
    guard=IdleStandGuard()
    executor.add_node(guard)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps=node.create_publisher(OccupancyGrid,'/safety/navigation_global',qos)
    costs=node.create_publisher(Costmap,'/local_costmap/costmap_raw',qos)
    footprints=node.create_publisher(PolygonStamped,'/local_costmap/published_footprint',10)
    health=node.create_publisher(Bool,'/lio/localization_valid',qos)
    odoms=node.create_publisher(Odometry,'/odom',10)
    sport=node.create_publisher(SportModeState,'/lf/sportmodestate',10)
    guarded=node.create_publisher(Twist,'/cmd_vel_guarded',10)
    responses=node.create_publisher(Response,'/api/sport/response',10)
    dynamic,static=TransformBroadcaster(node),StaticTransformBroadcaster(node)
    state=dict(x=0.,y=0.,yaw=0.,vx=0.,wz=0.,last=time.monotonic(),relative=False)
    api_requests=[]
    def api(message):
        api_requests.append((state['relative'],message.header.identity.api_id))
        answer=Response(); answer.header.identity=message.header.identity; answer.header.status.code=0
        responses.publish(answer)
    node.create_subscription(Request,'/api/sport/request',api,10)
    node.create_subscription(Bool,'/go2/relative_navigation_active',lambda m:state.update(relative=m.data),10)
    node.create_subscription(Twist,'/cmd_vel_nav',lambda m:state.update(vx=m.linear.x,wz=m.angular.z),10)
    grid=OccupancyGrid(); grid.header.frame_id='odom_lio'; grid.info.resolution=.1
    grid.info.width=grid.info.height=200
    grid.info.origin.position.x=grid.info.origin.position.y=-10.; grid.info.origin.orientation.w=1.
    grid.data=[0]*40000
    cost=Costmap(); cost.header.frame_id='odom_lio'; cost.metadata.resolution=.1
    cost.metadata.size_x=cost.metadata.size_y=200
    cost.metadata.origin=grid.info.origin; cost.data=[0]*40000
    def pose_tick():
        now=time.monotonic(); dt=min(.05,now-state['last']); state['last']=now
        state['x']+=math.cos(state['yaw'])*state['vx']*dt
        state['y']+=math.sin(state['yaw'])*state['vx']*dt
        state['yaw']+=state['wz']*dt
        pose=TransformStamped(); pose.header.frame_id='odom_lio'; pose.child_frame_id='base_link'
        pose.header.stamp=node.get_clock().now().to_msg()
        pose.transform.translation.x=state['x']; pose.transform.translation.y=state['y']
        pose.transform.rotation.z=math.sin(state['yaw']/2); pose.transform.rotation.w=math.cos(state['yaw']/2)
        dynamic.sendTransform(pose)
        odom=Odometry(); odom.header=pose.header; odom.child_frame_id='base_link'
        odom.pose.pose.position.x=state['x']; odom.pose.pose.position.y=state['y']
        odom.pose.pose.orientation=pose.transform.rotation
        odom.twist.twist.linear.x=state['vx']; odom.twist.twist.angular.z=state['wz']
        odoms.publish(odom)
        footprints_msg=PolygonStamped(); footprints_msg.header.frame_id='base_link'
        footprints_msg.header.stamp=pose.header.stamp
        footprints_msg.polygon.points=[Point32(x=x,y=y,z=0.) for x,y in [(.3,.2),(.3,-.2),(-.3,-.2),(-.3,.2)]]
        footprints.publish(footprints_msg)
        feedback=SportModeState(); feedback.body_height=.25
        feedback.velocity=[state['vx'],0.,0.]; feedback.yaw_speed=state['wz']
        sport.publish(feedback)
        cmd=Twist(); cmd.linear.x=state['vx']; cmd.angular.z=state['wz']; guarded.publish(cmd)
    def map_tick():
        stamp=node.get_clock().now().to_msg()
        grid.header.stamp=stamp; cost.header.stamp=stamp
        maps.publish(grid); costs.publish(cost); health.publish(Bool(data=True))
    node.create_timer(.02,pose_tick); node.create_timer(.1,map_tick)
    def lifecycle(request,response): response.current_state.id=3; return response
    node.create_service(GetState,'/bt_navigator/get_state',lifecycle)
    navigated=[]
    def navigate(handle):
        navigated.append(True); handle.succeed(); return NavigateToPose.Result()
    nav_server=ActionServer(node,NavigateToPose,'navigate_to_pose',navigate)
    camera=TransformStamped(); camera.header.frame_id='base_link'; camera.child_frame_id='livox_frame'
    camera.transform.translation.x=.14; camera.transform.translation.z=.13; camera.transform.rotation.w=1.
    static.sendTransform(camera)
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    logfile=root/'reports/relative_navigation_native_behavior_20261111.log'
    output=logfile.open('w')
    args=['/opt/ros/humble/lib/nav2_behaviors/behavior_server','--ros-args','-r','__node:=behavior_server',
        '-r','behavior_server:cmd_vel:=cmd_vel_nav','-p','global_frame:=odom_lio','-p','robot_base_frame:=base_link',
        '-p','costmap_topic:=/local_costmap/costmap_raw','-p','footprint_topic:=/local_costmap/published_footprint',
        '-p','cycle_frequency:=20.0','-p','max_rotational_vel:=0.65','-p','min_rotational_vel:=0.4',
        '-p','rotational_acc_lim:=0.9','-p',"behavior_plugins:=['spin','drive_on_heading','backup']",
        '-p','spin.plugin:=nav2_behaviors/Spin','-p','drive_on_heading.plugin:=nav2_behaviors/DriveOnHeading',
        '-p','backup.plugin:=nav2_behaviors/BackUp']
    behavior=subprocess.Popen(args,stdout=output,stderr=subprocess.STDOUT)
    worker=RosNavigationWorker()
    statuses,updates=[],[]
    worker.status_updated.connect(lambda *args:statuses.append(args))
    worker.relative_action_updated.connect(lambda *args:updates.append(args))
    def until(predicate,timeout=10.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            app.processEvents()
            if predicate(): return
            if behavior.poll() is not None: raise AssertionError(logfile.read_text())
            time.sleep(.01)
        raise AssertionError(dict(statuses=statuses[-3:],updates=updates[-5:],state=state,log=logfile.read_text()[-2500:]))
    try:
        client=node.create_client(ChangeState,'/behavior_server/change_state')
        until(lambda:client.service_is_ready())
        for transition in (1,3):
            request=ChangeState.Request(); request.transition.id=transition
            future=client.call_async(request); until(future.done)
            assert future.result().success,logfile.read_text()
        worker.start(); until(lambda:statuses and statuses[-1][0]=='ready')
        started=time.monotonic()
        worker.enqueue('relative_sequence','native-sequence',parse_relative_actions('转身，然后直行半米')[0])
        until(lambda:any(item[:2]==('native-sequence','accepted') for item in updates))
        first_accept=time.monotonic()-started
        until(lambda:any(item[:2]==('native-sequence','succeeded') for item in updates),25.)
        assert abs(abs(state['yaw'])-math.pi)<.18,state
        assert abs(state['x']+.5)<.1 and abs(state['y'])<.15,state
        assert not any(active for active,api_id in api_requests),api_requests
        assert worker.isRunning() and behavior.poll() is None
        worker.enqueue('visual_preflight','warm-next-target')
        prepared=[]
        worker.visual_goal_updated.connect(lambda *args:prepared.append(args))
        until(lambda:any(item[:2]==('warm-next-target','prepared') for item in prepared))
        sample=RgbdSnapshot(np.full((60,80,3),80,np.uint8),np.full((60,80),3000,np.uint16),
            time.time()-.1,time.monotonic()-.1,lambda *args:(0.,0.,3.))
        worker.enqueue('visual_goal',GroundedTarget('warm-next-target',ApproachIntent('柜子',1.),sample,(.1,.1,.9,.9),(0.,0.,3.)))
        until(lambda:any(item[:2]==('warm-next-target','succeeded') for item in prepared))
        assert navigated
        until(lambda:not worker._external_goal_active and worker._goal_handle is None)
        worker.enqueue('relative_sequence','native-reverse',parse_relative_actions('右转九十度，然后后退半米')[0])
        until(lambda:any(item[:2]==('native-reverse','succeeded') for item in updates),20.)
        worker.enqueue('relative_sequence','native-cancel',parse_relative_actions('左转90度，然后直行半米')[0])
        until(lambda:any(item[:2]==('native-cancel','accepted') for item in updates))
        worker.enqueue('cancel_relative','native-cancel')
        until(lambda:any(item[:2]==('native-cancel','finished') for item in updates))
        report={'isolated_domain':98,'localhost_only':True,'real_nav2_behavior_server':True,
            'simulated_base':True,'first_action_acceptance_seconds':first_accept,
            'behavior_pid_kept_alive':behavior.pid,'warm_navigation_reused':True,
            'spin_180_then_drive_05_completed':True,'owned_sequence_cancel_verified':True,
            'right_turn_and_backup_completed':True,'visual_goal_after_relative_sequence':bool(navigated),
            'no_idle_stand_rpc_during_sequence':not any(active for active,api_id in api_requests),
            'physical_sdk_or_base_started':False,'updates':updates}
        (root/'reports/relative_navigation_native_ros_20261111.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k!='updates'},ensure_ascii=False),flush=True)
    finally:
        worker.stop(); worker.wait(5000)
        behavior.send_signal(signal.SIGINT)
        try: behavior.wait(timeout=4.)
        except subprocess.TimeoutExpired: behavior.kill(); behavior.wait()
        output.close()
        executor.shutdown(timeout_sec=2.); thread.join(timeout=2.)
        nav_server.destroy(); guard.destroy_node(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
