#!/usr/bin/env python3
"""Reload an idle session's obstacle memory while retaining live LIO mapping.

--check-only is read-only. New nodes support --preserve-session-memory using
a same-LIO-session checkpoint. Legacy nodes require --reset-session-memory.
Run with scripts/env.sh sourced. The replacement follows the owning launch.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from process_supervisor import ManagedChildren, process_identity

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--main-pid',type=int,required=True)
    parser.add_argument('--memory-pid',type=int,required=True)
    parser.add_argument('--report-prefix',default='memory_reload_'+time.strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--check-only',action='store_true')
    parser.add_argument('--reset-session-memory',action='store_true')
    parser.add_argument('--preserve-session-memory',action='store_true')
    parser.add_argument('--confirmed-right-clear',action='store_true',
                        help='Only after operator confirms the right-foot region is empty: '
                             'remove stale records there from a preserved checkpoint')
    parser.add_argument('--confirmed-clear-bounds',type=float,nargs=4,
                        metavar=('X_MIN','X_MAX','Y_MIN','Y_MAX'),
                        help='Operator-confirmed empty body-frame rectangle; preserve fresh hits')
    parser.add_argument('--confirmed-region-reference',type=Path,
                        help='Checkpoint-derived records and pose for the operator-confirmed region')
    parser.add_argument('--detach',action='store_true')
    args=parser.parse_args()
    regional_clear=args.confirmed_right_clear or args.confirmed_clear_bounds is not None
    bounds=args.confirmed_clear_bounds or [-.5,.5,-.4,-.2]
    if not all(math.isfinite(v) for v in bounds) or not (bounds[0]<bounds[1] and bounds[2]<bounds[3]):
        parser.error('Invalid confirmed rectangle')
    if Path(args.report_prefix).name!=args.report_prefix:parser.error('Invalid report prefix')
    if args.reset_session_memory and args.preserve_session_memory:
        parser.error('Choose either a reset or a preserved checkpoint')
    if regional_clear and not args.preserve_session_memory:
        parser.error('Confirmed regional clearing requires --preserve-session-memory')
    if regional_clear and args.confirmed_region_reference is None:
        parser.error('Confirmed clearing requires the captured region reference')
    if not args.check_only and not (args.reset_session_memory or args.preserve_session_memory):
        parser.error('Applying requires --preserve-session-memory or --reset-session-memory')
    if args.detach:
        with (ROOT/'reports'/f'{args.report_prefix}_supervisor.log').open('x') as stream:
            child=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),
                *[a for a in sys.argv[1:] if a!='--detach']],cwd=ROOT,stdin=subprocess.DEVNULL,
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        print(json.dumps({'supervisor_pid':child.pid,'report_prefix':args.report_prefix}))
        return

    def identity(pid,expected):
        proc=Path('/proc')/str(pid)
        argv=[v.decode() for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
        if proc.stat().st_uid!=os.getuid() or expected not in [Path(v).name for v in argv]:
            raise RuntimeError('Process identity mismatch: '+str(pid))
        return argv

    identity(args.main_pid,'go2_lio_navigation.launch.py')
    argv=identity(args.memory_pid,'obstacle_memory.py')
    old_token=process_identity(args.memory_pid)
    if old_token is None:raise RuntimeError('Original memory process already exited')
    managed=ManagedChildren(args.main_pid)
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from nav2_msgs.srv import ClearEntireCostmap
    from unitree_go.msg import SportModeState
    rclpy.init()
    node=rclpy.create_node('idle_obstacle_memory_reload')
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state={'statuses':[]}
    velocities=deque(maxlen=100)
    node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',
        lambda m:state.update(statuses=[v.status for v in m.status_list]),latched)
    node.create_subscription(Twist,'/cmd_vel_guarded',
        lambda m:velocities.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),
        qos_profile_sensor_data)
    def sport(m):
        values=[float(m.velocity[0]),float(m.velocity[1]),float(m.yaw_speed)]
        state.update(sport_time=time.monotonic(),stationary=all(math.isfinite(v) for v in values)
            and math.hypot(*values[:2])<.04 and abs(values[2])<.10)
    node.create_subscription(SportModeState,'/lf/sportmodestate',sport,qos_profile_sensor_data)
    node.create_subscription(String,'/safety/obstacle_memory_status',
        lambda m:state.update(memory=json.loads(m.data)),latched)
    def pose(m):
        p,q=m.pose.pose.position,m.pose.pose.orientation
        state['pose']={'frame':m.header.frame_id,'x':p.x,'y':p.y,
                       'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                       'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1.e9}
    node.create_subscription(Odometry,'/odom',pose,qos_profile_sensor_data)
    report={'owner_pid':args.main_pid,'original_memory_pid':args.memory_pid,'applied':False,
            'reset_session_memory':args.reset_session_memory,'supervisor_pid':os.getpid()}
    path=ROOT/'reports'/f'{args.report_prefix}.json'
    log=None
    def write():path.write_text(json.dumps(report,indent=2))
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if not managed.owner_alive():raise RuntimeError('Owning LIO launch exited')
            rclpy.spin_once(node,timeout_sec=.02)
    def idle():
        recent=[v for v in velocities if time.monotonic()-v[0]<.75]
        if (any(v in (1,2,3) for v in state['statuses']) or len(recent)<10 or
                any(abs(v)>.001 for row in recent for v in row[1:]) or
                time.monotonic()-state.get('sport_time',-math.inf)>.5 or not state.get('stationary')):
            raise RuntimeError('Active goal, motion or stale feedback; reload refused')
    def call(kind,topic,request):
        client=node.create_client(kind,topic)
        try:
            if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('Unavailable: '+topic)
            future=client.call_async(request)
            end=time.monotonic()+30.
            while not future.done() and time.monotonic()<end:spin(.02)
            if not future.done():raise RuntimeError('Service timeout: '+topic)
            return future.result()
        finally:node.destroy_client(client)
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupted)
    signal.signal(signal.SIGTERM,interrupted)
    try:
        spin(2.)
        idle()
        report['before']=dict(state)
        report['original_memory_identity']=old_token
        report['ready']=True
        write()
        if args.check_only:
            print(json.dumps(report,indent=2))
            return
        saved=call(Trigger,'/mapping/save',Trigger.Request())
        if not saved.success:raise RuntimeError('Reference map save failed: '+saved.message)
        report['saved_map']=saved.message
        if '--ros-args' not in argv:argv.append('--ros-args')
        if args.preserve_session_memory:
            checkpoint=call(Trigger,'/safety/save_obstacle_memory',Trigger.Request())
            if not checkpoint.success:raise RuntimeError('Memory checkpoint failed: '+checkpoint.message)
            report['checkpoint']=checkpoint.message
            restore_file=checkpoint.message
            if regional_clear:
                sys.path.insert(0,str(ROOT/'ros2_ws/src/go2_lio_navigation/scripts'))
                from obstacle_memory_model import clear_confirmed_region
                source=json.loads(Path(checkpoint.message).read_text())
                reference=json.loads(args.confirmed_region_reference.read_text())
                if reference['session']!=source['session'] or reference['frame']!=source['frame']:
                    raise RuntimeError('Confirmed region belongs to a different localization session')
                # Exporting a large checkpoint can briefly queue pose callbacks.
                # Drain them before checking freshness; never relax the limit.
                spin(.2)
                p=state.get('pose',{})
                if (p.get('frame')!=source['frame'] or
                    not -.05<=node.get_clock().now().nanoseconds/1e9-p.get('stamp',0)<=.35):
                    raise RuntimeError('No matching fresh pose for confirmed clearing')
                if (math.hypot(p['x']-reference['pose'][0],p['y']-reference['pose'][1])>.05 or
                        abs(math.remainder(p['yaw']-reference['pose'][2],2*math.pi))>.05):
                    raise RuntimeError('Robot moved since the confirmed-region reference')
                hits={tuple(row[:2]):row[4] for row in reference['records']}
                filtered,removed=clear_confirmed_region(source,reference['pose'],bounds=bounds,allowed_hits=hits)
                target=ROOT/'reports'/f'{args.report_prefix}_confirmed_region.json'
                with target.open('x') as stream:json.dump(filtered,stream,allow_nan=False)
                report['confirmed_region_clear']={'removed_cells':len(removed),
                    'body_bounds':bounds,'minimum_age':5.,'pose':reference['pose'],
                    'current_pose':p,'reference':str(args.confirmed_region_reference),
                    'removed_records':removed,'original_checkpoint':checkpoint.message}
                if not removed:
                    report['no_op']='Confirmed records are already absent or have newer observations'
                    write()
                    print(json.dumps(report,indent=2),flush=True)
                    return
                restore_file=str(target)
            argv+=['-p','restore_state_file:='+restore_file]
        else:
            # A previous preserved reload may have a restore override in argv.
            # An explicit reset must not silently restore that old snapshot.
            argv+=['-p',"restore_state_file:=''"]
        # Large checkpoint serialization can take longer than the feedback
        # freshness bound. Drain queued feedback before the final idle check.
        spin(.3)
        idle()
        identity(args.memory_pid,'obstacle_memory.py')
        if process_identity(args.memory_pid)!=old_token:raise RuntimeError('Original memory PID changed')
        os.kill(args.memory_pid,signal.SIGINT)
        end=time.monotonic()+8.
        while process_identity(args.memory_pid)==old_token and time.monotonic()<end:spin(.05)
        if process_identity(args.memory_pid)==old_token:
            raise RuntimeError('Original node did not stop; refuse duplicate replacement')
        state.pop('memory',None)
        argv[0]='/usr/bin/python3'
        log=(ROOT/'reports'/f'{args.report_prefix}_memory.log').open('x')
        child=managed.start(argv,cwd=ROOT,env=os.environ.copy(),stdout=log,stderr=subprocess.STDOUT)
        report['memory_pid']=child.pid
        report['history_reset']=args.reset_session_memory
        write()
        deadline=time.monotonic()+45.
        healthy_since=None
        first_accepted=None
        while time.monotonic()<deadline:
            spin(.05)
            if child.poll() is not None:raise RuntimeError('Replacement memory node exited')
            m=state.get('memory',{})
            report['replacement_last_status']=m
            healthy=(m.get('clearing_policy')=='motion_evidence_v7_near_volume' and
                m.get('clearing_transport')=='numeric_delta_v1' and
                (not args.preserve_session_memory or m.get('checkpoint_restored')) and
                (not args.reset_session_memory or not m.get('checkpoint_restored')) and
                m.get('accepted_scans',0)>=15 and m.get('localized') and
                m.get('scan_age',math.inf)<.60 and m.get('fault') is None)
            # Diagnostics are sampled at 5 Hz, between 15 Hz observations.
            # Requiring *every diagnostic* below the final freshness bound
            # rejects a healthy updating node just for its sample phase.
            # Demand continued scan progress and a fresh final observation;
            # the robot's motion gate retains its stricter independent check.
            if healthy:
                if healthy_since is None:
                    healthy_since=time.monotonic()
                    first_accepted=m['accepted_scans']
                if (time.monotonic()-healthy_since>=2. and
                        m['accepted_scans']-first_accepted>=15 and m['scan_age']<.35):break
            else:
                healthy_since=None
                first_accepted=None
        else:raise RuntimeError('Updated memory did not become continuously healthy')
        idle()
        for topic in ('/local_costmap/clear_entirely_local_costmap',
                      '/global_costmap/clear_entirely_global_costmap'):
            call(ClearEntireCostmap,topic,ClearEntireCostmap.Request())
        report['applied']=True
        report['after']=dict(state)
        write()
        print(json.dumps(report,indent=2),flush=True)
        while managed.owner_alive():
            spin(.2)
            if child.poll() is not None:raise RuntimeError('Replacement memory node exited')
    except KeyboardInterrupt:
        report['shutdown']='Supervisor interrupted'
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        managed.close()
        report['children_stopped']=True
        write()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if log:log.close()


if __name__=='__main__':main()
