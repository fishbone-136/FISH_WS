#!/usr/bin/python3
"""Reproduce passing a box then returning while fresh forward scans miss it."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=87 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI']=(
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
    '</Peers></Discovery></Domain></CycloneDDS>')
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'go2_navigation/test'))
from check_obstacle_pipeline import Fixture,cloud_message
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2,LaserScan
from std_msgs.msg import Bool,String
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_prefix


def cpu_heavy_clearing(*args):
    """Importable spawn worker that reproduces Python CPU contention."""
    from obstacle_memory import compute_clearing_delta
    until=time.monotonic()+.25
    value=0
    while time.monotonic()<until:
        value=(value+1)%65536
    return compute_clearing_delta(*args)


class MemoryFixture(Fixture):
    def __init__(self):
        super().__init__()
        self.destroy_publisher(self.command)
        self.command=self.create_publisher(Twist,'/cmd_vel_safe',1)
        self.health=self.create_publisher(Bool,'/lio/localization_valid',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05,lambda:self.health.publish(Bool(data=True)))
        self.memory_map=None
        self.memory_status={}
        self.remembered_cloud=None
        self.clear_volumes=[]
        self.create_subscription(PointCloud2,'/safety/cleared_obstacle_volumes',
                                 lambda m:self.clear_volumes.append(m),10)
        self.empty_observations=self.create_publisher(PointCloud2,'/obstacles/deskewed',1)
        self.create_timer(self.frame_dt,self.publish_empty_observations)
        self.create_subscription(PointCloud2,'/safety/remembered_obstacles',
            lambda m:setattr(self,'remembered_cloud',m),qos_profile_sensor_data)
        self.motion_diagnostic={}
        self.fast_scan=None
        self.create_subscription(LaserScan,'/scan_safety',lambda m:setattr(self,'fast_scan',m),
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/safety/navigation_global',
            lambda m:setattr(self,'memory_map',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/safety/obstacle_memory_status',
            lambda m:setattr(self,'memory_status',json.loads(m.data)),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/safety/motion_diagnostics',
            lambda m:setattr(self,'motion_diagnostic',json.loads(m.data)),qos_profile_sensor_data)

    def publish_empty_observations(self):
        if self.scan_override is not None:
            stamp=self.frame_stamp
            if stamp is None:return
            self.empty_observations.publish(cloud_message(np.empty((0,3)),stamp,'base_link'))

    def wait(self,seconds):
        end=time.monotonic()+seconds
        self.until(lambda:time.monotonic()>=end,timeout=seconds+2.)

    def box_cost(self,grid=None):
        m=grid or self.memory_map
        if m is None:return None
        x=int((1.-m.info.origin.position.x)/m.info.resolution)
        y=int((0.-m.info.origin.position.y)/m.info.resolution)
        return m.data[y*m.info.width+x] if 0<=x<m.info.width and 0<=y<m.info.height else None

    def old_box_cells(self):
        m=self.memory_map
        if m is None:return None
        count=0
        for y in range(m.info.height):
            wy=m.info.origin.position.y+(y+.5)*m.info.resolution
            if abs(wy)>.45:continue
            for x in range(m.info.width):
                wx=m.info.origin.position.x+(x+.5)*m.info.resolution
                if .85<=wx<=1.15 and m.data[y*m.info.width+x]>=90:count+=1
        return count


def obstacle_refresh(f,start):
    """A removed box's complete inflation halo must return to baseline."""
    no_ground='--no-ground-rays' in sys.argv
    f.floor_z=.30 if no_ground else -.37
    if no_ground:
        # A real farther return can prove a 3-D segment free even when no
        # ground plane is visible. No infinity/floor inference is available.
        f.floor_points=np.array([[x,y] for x in np.linspace(3.,4.,6)
                                for y in np.linspace(-1.,1.,21)])
    f.extra_points=[[1.6,1.6,f.floor_z+h] for h in (.11,.30,.50)]
    start(['ros2','launch','go2_navigation','go2_nav2.launch.py',
           'mapping_mode:=true','lio_mode:=true','log_level:=error'])
    f.until(lambda:f.memory_map is not None and f.costmap is not None,timeout=25.)
    def region_count():
        m=f.costmap
        yy,xx=np.indices((m.info.height,m.info.width));o=m.info.origin.position
        wx=o.x+(xx+.5)*m.info.resolution;wy=o.y+(yy+.5)*m.info.resolution
        region=(wx>=.5)&(wx<=1.5)&(np.abs(wy)<=.65)
        return int(np.count_nonzero(np.asarray(m.data).reshape(region.shape)[region]>0))
    def leg_cost():
        m=f.memory_map
        ix=int((1.6-m.info.origin.position.x)/m.info.resolution)
        iy=int((1.6-m.info.origin.position.y)/m.info.resolution)
        return m.data[iy*m.info.width+ix]
    f.until(lambda:leg_cost()==100)
    f.wait(1.)
    baseline=region_count()
    sparse='--sparse-fragments' in sys.argv
    if sparse:
        # Five upper-body returns never form compact(): the original ghost
        # was retained forever despite repeatedly seeing the floor behind it.
        f.extra_points += [[1.,y,f.floor_z+.65] for y in (-.2,-.1,0.,.1,.2)]
        f.until(lambda:f.box_cost()==100 and region_count()>baseline,timeout=10.)
    else:
        f.box_x=1.;f.box_width=.40
        f.box_heights=[-.20] if no_ground else [.11+i*.025 for i in range(24)]+[.70]
        f.until(lambda:f.box_cost()==100 and region_count()>baseline,timeout=10.)
        f.wait(1.)
    occupied=region_count()
    f.box_x=None
    if sparse:f.extra_points=[[1.6,1.6,f.floor_z+h] for h in (.11,.30,.50)]
    removed=time.monotonic()
    try:
        f.until(lambda:f.old_box_cells()==0 and region_count()==baseline,timeout=12.)
    except AssertionError:
        print('REFRESH DEBUG',json.dumps({'memory':f.memory_status,
            'remaining_box_cells':f.old_box_cells(),'cost_cells':region_count(),
            'baseline':baseline}),flush=True)
        raise
    elapsed=time.monotonic()-removed
    assert leg_cost()==100,'Stationary leg must survive removal of the box'
    print('PASS removed box and entire inflation halo return to baseline in %.3f s; static leg retained'%elapsed,flush=True)
    root=Path(os.environ.get('GO2_HUMBLE_PROJECT_ROOT',Path(__file__).resolve().parents[4]))
    name='pedestrian_fragment_refresh_measurement.json' if sparse else 'obstacle_refresh_measurement.json'
    if no_ground:name='no_ground_volume_refresh_measurement.json'
    (root/'reports'/name).write_text(json.dumps({
        'baseline_cost_cells':baseline,'occupied_cost_cells':occupied,
        'final_cost_cells':region_count(),'inflation_clear_seconds':elapsed,
        'static_leg_retained':True},indent=2))


def dynamic_crossing(f,start,executable,memory):
    """Upper-body returns + floor-only background reproduce the real ghost."""
    f.floor_z=-.37;f.box_x=None;f.box_width=.36
    f.box_heights=[.70,.80,.95,1.10]
    f.floor_points=np.array([[x,y] for x in np.arange(2.,4.6,.12)
                            for y in np.arange(-2.5,2.6,.12)])
    f.extra_points=[[1.6,1.6,f.floor_z+h] for h in (.11,.30,.50)]
    mixed='--mixed-crossing' in sys.argv
    if mixed:
        f.extra_points += [[x,y,f.floor_z+h] for x in np.arange(.7,1.31,.05)
                          for y in np.arange(.45,1.06,.05) for h in (.12,.35,.6)]
    start(['ros2','launch','go2_navigation','go2_nav2.launch.py',
           'mapping_mode:=true','lio_mode:=true','log_level:=error'])
    f.plan()
    def region_count(message,threshold=1):
        if message is None:return 0
        yy,xx=np.indices((message.info.height,message.info.width));o=message.info.origin.position;r=message.info.resolution
        mask=(xx*r+o.x>=.55)&(xx*r+o.x<=1.4)&(np.abs(yy*r+o.y)<=1.3)
        return int(np.count_nonzero(np.asarray(message.data).reshape(mask.shape)[mask]>=threshold))
    def leg_cost():
        m=f.memory_map
        if m is None:return None
        ix=int((1.6-m.info.origin.position.x)/m.info.resolution)
        iy=int((1.6-m.info.origin.position.y)/m.info.resolution)
        return m.data[iy*m.info.width+ix]
    f.until(lambda:leg_cost()==100)
    f.wait(4.)
    baseline_memory=region_count(f.memory_map,90)
    baseline_cost=region_count(f.costmap)
    peak=0
    for center in np.arange(-.80,.81,.10):
        f.box_x=1.;f.box_y=float(center);f.wait(.07 if '--fast-crossing' in sys.argv else .22)
        peak=max(peak,region_count(f.memory_map,90))
    assert peak>5
    f.until(lambda:f.memory_status.get('dynamic_tracks_confirmed',0)>=1)
    f.box_x=None;removed=time.monotonic()
    f.until(lambda:region_count(f.memory_map,90)<=baseline_memory,timeout=5.)
    memory_seconds=time.monotonic()-removed
    f.until(lambda:f.costmap is not None and region_count(f.costmap)<=baseline_cost,timeout=3.)
    cost_seconds=time.monotonic()-removed
    assert leg_cost()==100
    print('PASS lateral crossing with only upper-body returns is tracked as motion',flush=True)
    print('PASS floor-only background clears the entire moving trail in %.3f s'%memory_seconds,flush=True)
    print('PASS complete former inflation region returns to its static baseline in %.3f s'%cost_seconds,flush=True)
    if mixed:
        msg=f.memory_map
        ix=int((1.-msg.info.origin.position.x)/msg.info.resolution)
        iy=int((.7-msg.info.origin.position.y)/msg.info.resolution)
        assert msg.data[iy*msg.info.width+ix]==100,'The stationary box was erased'
        print('PASS a person merging with a stationary box clears without deleting the box',flush=True)
    print('PASS the stationary thin leg stays occupied while the moving trail clears',flush=True)
    client=f.create_client(Trigger,'/safety/save_obstacle_memory')
    f.until(client.service_is_ready);future=client.call_async(Trigger.Request());f.until(future.done)
    saved=future.result();assert saved.success,saved.message
    f.scan_override=[]
    memory.send_signal(signal.SIGINT);f.until(lambda:memory.poll() is not None,timeout=6.)
    start(['/usr/bin/python3',str(executable/'obstacle_memory.py'),'--ros-args',
           '-p','raw_cloud_topic:=/test/cloud_fast','-p','full_pose_topic:=/odom',
           '-p','restore_state_file:='+saved.message])
    f.until(lambda:f.memory_status.get('checkpoint_restored') and leg_cost()==100,timeout=8.)
    f.wait(1.);assert leg_cost()==100
    print('PASS checkpoint reload retains an unobserved leg in the same session',flush=True)
    from pathlib import Path
    report={'peak_trail_occupied_cells':peak,'memory_clear_seconds':memory_seconds,
            'inflation_clear_seconds':cost_seconds,'static_leg_retained':True,
            'checkpoint_restored':True,'checkpoint':saved.message}
    report['static_baseline_memory_cells']=baseline_memory
    report['static_baseline_cost_cells']=baseline_cost
    name='mixed_crossing_measurement.json' if mixed else 'dynamic_crossing_measurement.json'
    Path('/home/fishbone/HAP_ws/reports',name).write_text(
        json.dumps(report,indent=2))


def main():
    rclpy.init()
    f=MemoryFixture();children=[]
    executable=Path(get_package_prefix('go2_lio_navigation'))/'lib/go2_lio_navigation'
    def start(args):
        p=subprocess.Popen(args,start_new_session=True);children.append(p);return p
    try:
        projection=Path(get_package_prefix('go2_perception'))/'lib/go2_perception/pointcloud_to_laserscan_node'
        start([str(projection),'--ros-args','-r','__node:=memory_test_projector',
               '-r','cloud_in:=/test/cloud_fast','-r','scan:=/scan_safety',
               '-r','obstacle_points:=/obstacles/deskewed',
               '-r','ground_plane:=/perception/deskewed_ground_plane',
               '-p','target_frame:=base_link','-p','ground_filter_enabled:=true',
               '-p','observed_free_space_only:=true','-p','range_min:=0.25',
               '-p','range_max:=5.5','--log-level','warn'])
        if '--slow-clearing' in sys.argv:
            code=('import sys;sys.path.insert(0,'+repr(str(executable))+');'
                  'sys.path.insert(0,'+repr(str(Path(__file__).resolve().parent))+');'
                  'import obstacle_memory as m;from check_obstacle_memory_ros import cpu_heavy_clearing;'
                  'm.compute_clearing_delta=cpu_heavy_clearing;'
                  "sys.argv=['obstacle_memory','--ros-args','-p','raw_cloud_topic:=/test/cloud_fast','-p','full_pose_topic:=/odom'];m.main()")
            memory=start(['/usr/bin/python3','-c',code])
        else:
            memory=start(['/usr/bin/python3',str(executable/'obstacle_memory.py'),
                          '--ros-args','-p','raw_cloud_topic:=/test/cloud_fast','-p','full_pose_topic:=/odom'])
        if '--refresh-only' in sys.argv:
            obstacle_refresh(f,start)
            return
        start(['/usr/bin/python3',str(executable/'scan_safety_gate.py')])
        if '--dynamic-crossing' in sys.argv or '--mixed-crossing' in sys.argv:
            dynamic_crossing(f,start,executable,memory)
            return
        f.floor_z=-.37;f.box_x=1.;f.box_width=.60
        f.box_heights=[.11+i*.025 for i in range(24)]+[.70]
        f.scan_override=None
        f.tf_lag=.09
        # Map, scan, safety diagnostics and memory diagnostics are independent
        # topics. Their relative delivery order is not guaranteed by DDS.
        # Waiting for the map alone intermittently indexed an empty status.
        def box_ready():
            return (f.box_cost()==100 and f.motion_diagnostic.get('memory_valid') is True and
                    f.memory_status.get('cells',0)>0 and f.fast_scan is not None and
                    sum(math.isfinite(v) and v<1.3 for v in f.fast_scan.ranges)>20)
        f.until(box_ready,timeout=12.)
        f.until(lambda:f.remembered_cloud is not None and f.remembered_cloud.width>0)
        xyz=np.frombuffer(f.remembered_cloud.data,dtype='<f4').reshape(-1,3)
        assert np.all(np.abs(xyz[:,0]-1.)<1.e-5),xyz.tolist()
        print('PASS memory contains genuine box endpoints, never artificial angular spread points',flush=True)
        print('PASS a 70 cm box at 1 m survives the real 3-D ground/self filter and enters obstacle memory',flush=True)
        for _ in range(10):
            f.wait(.2)
            assert f.memory_status.get('scan_age',1.)<.65,f.memory_status
        print('PASS continuously delayed TF uses an older ready scan without starving obstacle memory',flush=True)
        if '--slow-clearing' in sys.argv:
            assert f.memory_status.get('max_fast_processing_ms',1000.)<50.,f.memory_status
            print('PASS slow 3-D clearing cannot block fresh occupied writes or memory health',flush=True)
        f.tf_lag=0.
        # This fixture teleports rather than walking through intermediate
        # poses. Drain old box frames before the jump so interpolated TF does
        # not assign them an artificial in-between robot position.
        f.enabled=False
        f.wait(.5)
        # Move past the box, observing only the new forward sector. Terrain
        # source grids keep publishing FREE everywhere, as a short history does.
        f.x,f.y,f.yaw=2.,0.,0.
        f.scan_override=[]
        f.enabled=True
        f.wait(2.)
        assert f.box_cost()==100
        print('PASS a seen box behind the robot survives fresh front-only scans and overwritten free terrain',flush=True)
        start(['ros2','launch','go2_navigation','go2_nav2.launch.py',
               'mapping_mode:=true','lio_mode:=true','log_level:=error'])
        target=f.goal_pose();target.pose.position.x=target.pose.position.y=0.
        target.header.frame_id='odom_lio';target.header.stamp=rclpy.time.Time().to_msg()
        route=f.plan(target)
        assert max(abs(p.pose.position.y) for p in route.poses)>.25
        print('PASS real Smac planning detours around the remembered rear box',flush=True)
        clear=f.create_client(ClearEntireCostmap,'/global_costmap/clear_entirely_global_costmap')
        f.until(clear.service_is_ready)
        done=clear.call_async(ClearEntireCostmap.Request());f.until(done.done)
        f.wait(.8)
        route=f.plan(target)
        assert max(abs(p.pose.position.y) for p in route.poses)>.25
        print('PASS a Nav2 clear-costmap recovery does not erase independent obstacle memory',flush=True)
        def current_scan_cost():
            m=f.costmap
            if m is None:return None
            values=[]
            for dx in (-.06,0.,.06):
                for dy in (-.06,0.,.06):
                    ix=int((3.+dx-m.info.origin.position.x)/m.info.resolution)
                    iy=int((.8+dy-m.info.origin.position.y)/m.info.resolution)
                    if 0<=ix<m.info.width and 0<=iy<m.info.height:
                        values.append(m.data[iy*m.info.width+ix])
            return max(values) if values else None
        f.forward_coverage=False;f.scan_override=[(1.,.8)]
        f.until(lambda:current_scan_cost()==100)
        f.scan_override=[]
        f.until(lambda:current_scan_cost()==0,timeout=5.)
        assert f.box_cost()==100
        f.forward_coverage=True
        print('PASS current scan obstacle and inflation disappear without clearing rays; blind memory stays',flush=True)
        f.yaw=math.pi
        f.wait(1.)
        assert f.box_cost()==100
        print('PASS after turning back, ground infinity/unknown rays cannot erase the box',flush=True)
        f.x=1.65;f.command_linear=.55;f.command_angular=0.;f.command_enabled=True
        f.until(lambda:f.safe is not None and 0.<f.safe.linear.x<.4,timeout=8.)
        assert f.motion_diagnostic.get('remembered_points',0)>0
        print('PASS retained front obstacle slows motion even when the current scan misses it',flush=True)
        f.wait(1.)
        recent=[v for v in f.safe_history if v[0]>time.monotonic()-.75]
        assert recent and all(v[1]>.03 for v in recent),recent
        print('PASS missing matching 3-D frames cannot starve memory updates or pulse an otherwise safe approach',flush=True)
        f.x=1.43
        f.until(lambda:f.safe.linear.x==0.)
        print('PASS the final gate stops before the remembered box despite missing current returns',flush=True)
        f.command_linear=0.;f.command_angular=.65;f.x=1.46;f.y=-.50;f.yaw=0.
        f.wait(.6)
        assert f.safe.angular.z==0.
        f.command_angular=-.65
        f.until(lambda:f.safe.angular.z<-.2,timeout=8.)
        print('PASS remembered rear corner blocks a turn into it while allowing the away turn',flush=True)
        f.command_linear=f.command_angular=0.;f.x=1.43;f.y=0.;f.yaw=math.pi
        f.wait(.4)
        # Remove the physical box and resume real 3-D floor observations.
        # Ground rays only clear if they cross the box's old height volume.
        f.box_x=None;f.scan_override=None
        try:
            f.until(lambda:f.old_box_cells()==0 and f.memory_status.get('cleared_cells',0)>0,
                    timeout=12.)
        except AssertionError:
            print('CLEAR DEBUG',f.memory_status,'old_box_cells',f.old_box_cells(),flush=True)
            raise
        print('PASS actual 3-D rays through the old box volume clear it after removal; no time-only expiry',flush=True)
        f.until(lambda:f.memory_status.get('ground_frame_paired') is True,timeout=5.)
        print('PASS stable ground plane pairs with the same validated observation frame',flush=True)
        f.until(lambda:bool(f.clear_volumes),timeout=5.)
        assert any(m.width>0 for m in f.clear_volumes)
        print('PASS confirmed 3-D removal is published for the environment reference to consume',flush=True)
        f.command_linear=.3
        f.until(lambda:f.safe.linear.x>.1,timeout=8.)
        began=time.monotonic()
        memory.send_signal(signal.SIGINT);f.until(lambda:memory.poll() is not None,timeout=5.)
        f.until(lambda:f.safe.linear.x==0.,timeout=2.)
        assert time.monotonic()-began<1.2
        f.until(lambda:f.motion_diagnostic.get('scan_valid') is True and
                        f.motion_diagnostic.get('memory_valid') is False)
        print('PASS loss of the memory process closes the final motion gate with raw scans still fresh',flush=True)
    finally:
        f.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for p in reversed(children):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in children:
            try:p.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
            try:os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError:pass


if __name__ == '__main__':main()
