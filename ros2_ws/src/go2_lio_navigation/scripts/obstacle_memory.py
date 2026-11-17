#!/usr/bin/python3
"""Remember genuine deskewed obstacle endpoints independently of terrain grids."""
from array import array
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from copy import deepcopy
from itertools import chain
import json
import math
import multiprocessing
import time
import os
import signal
from pathlib import Path
# Small 3-D transforms lose time and CPU to threaded BLAS on the AGX.
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from go2_navigation.runtime_guard import acquire_process_lock
from obstacle_memory_model import (ObstacleMemory,RayEvidence,navigation_grid,clearing_delta,
                                   reference_clear_volumes,memory_checkpoint,restore_checkpoint)
from dynamic_obstacles import DynamicObstacles


def initialize_clearing_worker():
    # The ROS parent handles Ctrl-C and drains this pool. Interrupting both
    # processes at once used to break the pool during ordinary launch exit.
    signal.signal(signal.SIGINT,signal.SIG_IGN)


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def lio_session_identity():
    domain=os.environ.get('ROS_DOMAIN_ID','0')
    found=[]
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            argv=[v for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
            if not argv or Path(argv[0].decode()).name!='fastlio_mapping':continue
            env=dict(v.split('=',1) for v in (proc/'environ').read_text().split('\0') if '=' in v)
            if env.get('ROS_DOMAIN_ID','0')!=domain:continue
            fields=(proc/'stat').read_text().rsplit(')',1)[1].split()
            found.append({'pid':int(proc.name),'start':fields[19],'domain':domain})
        except (OSError,ValueError):continue
    if len(found)==1:return found[0]
    if not found and os.environ.get('ROS_LOCALHOST_ONLY')=='1' and domain!='0':
        return {'isolated_test_domain':domain}
    raise ValueError('Cannot identify one owning FAST-LIO process')


def full_pose_transform(odometry,relative):
    """Compose the actual 6-DoF sensor pose before projecting to memory XY."""
    result=deepcopy(relative);a=odometry.pose.pose.orientation;b=relative.transform.rotation
    p=odometry.pose.pose.position;t=relative.transform.translation
    values=[p.x,p.y,p.z,t.x,t.y,t.z,a.x,a.y,a.z,a.w,b.x,b.y,b.z,b.w]
    if not all(math.isfinite(v) for v in values):raise ValueError('Non-finite observation transform')
    if abs(a.x*a.x+a.y*a.y+a.z*a.z+a.w*a.w-1.)>.001 or abs(b.x*b.x+b.y*b.y+b.z*b.z+b.w*b.w-1.)>.001:
        raise ValueError('Non-unit observation transform quaternion')
    x,y,z,w=a.x,a.y,a.z,a.w
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    xyz=r@np.array([t.x,t.y,t.z])+[p.x,p.y,p.z]
    result.transform.translation.x,result.transform.translation.y,result.transform.translation.z=map(float,xyz)
    q=result.transform.rotation
    q.x=w*b.x+x*b.w+y*b.z-z*b.y;q.y=w*b.y-x*b.z+y*b.w+z*b.x
    q.z=w*b.z+x*b.y-y*b.x+z*b.w;q.w=w*b.w-x*b.x-y*b.y-z*b.z
    result.header.frame_id=odometry.header.frame_id;result.header.stamp=odometry.header.stamp
    return result


def cloud_points(cloud,transform):
    fields={f.name:f for f in cloud.fields}
    if cloud.height!=1 or not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):
        raise ValueError('Unsupported raw XYZ cloud layout')
    q=transform.transform.rotation;x,y,z,w=q.x,q.y,q.z,q.w
    rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    p=transform.transform.translation;origin=np.array([p.x,p.y,p.z])
    dtype=np.dtype({'names':['x','y','z'],'formats':['>f4' if cloud.is_bigendian else '<f4']*3,
        'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':cloud.point_step})
    data=np.frombuffer(cloud.data,dtype=dtype,count=cloud.width)
    return np.einsum('ij,kj->ki',rotation,np.column_stack([data[k] for k in ('x','y','z')]))+origin,origin


def compute_clearing(memory,scan,pose,cloud,transform):
    """No ROS or shared-state mutation in the background clearing worker."""
    started=time.perf_counter()
    points,origin=cloud_points(cloud,transform)
    rays=RayEvidence(points,origin)
    stamp=scan.header.stamp.sec+scan.header.stamp.nanosec/1.e9
    # Clearing workers never manufacture/mark angularly expanded scan points.
    memory.observe([],0.,1.,0.,0.,pose,stamp,rays=rays)
    return memory.cells,1000*(time.perf_counter()-started)


def world_ground_plane(message,transform):
    q=message.pose.orientation
    if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.001:
        raise ValueError('Non-unit ground orientation')
    normal=np.array([2*(q.x*q.z+q.y*q.w),2*(q.y*q.z-q.x*q.w),1-2*(q.x*q.x+q.y*q.y)])
    q=transform.transform.rotation;x,y,z,w=q.x,q.y,q.z,q.w
    rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    p=message.pose.position;t=transform.transform.translation
    normal=rotation@normal;point=rotation@np.array([p.x,p.y,p.z])+[t.x,t.y,t.z]
    return np.r_[normal,-normal@point]


def compute_clearing_delta(geometry,stamp,previous_stamp,pose,cloud,transform,clear_window,
                           ground_plane=None,dynamic_flags=None,transient_flags=None):
    """Transfer compact numeric geometry in and sparse confirmation changes out."""
    started=time.perf_counter()
    points,origin=cloud_points(cloud,transform)
    # Apply the same own-body exclusion as the positive-obstacle projector.
    # Robot returns must not veto clearing of external obstacle history.
    c,s=math.cos(pose[2]),math.sin(pose[2]);delta=points[:,:2]-pose[:2]
    bx=c*delta[:,0]+s*delta[:,1];by=-s*delta[:,0]+c*delta[:,1]
    points=points[(np.abs(bx)>=.40)|(np.abs(by)>=.24)]
    updates=clearing_delta(geometry,RayEvidence(points,origin),pose,stamp,previous_stamp,
                           # The parent checks sensor continuity; this gap is
                           # between background jobs, bounded by the evidence window.
                           clear_window=clear_window,max_gap=clear_window,ground_plane=ground_plane,
                           dynamic_flags=dynamic_flags,transient_flags=transient_flags)
    return updates,1000*(time.perf_counter()-started)


class RememberedObstacles(Node):
    def __init__(self):
        super().__init__('obstacle_memory')
        acquire_process_lock('obstacle_memory')
        self.frame = self.declare_parameter('memory_frame','odom_lio').value
        self.body_frame = self.declare_parameter('body_frame','base_link').value
        clear_window=self.declare_parameter('clearing_confirmation_window',4.).value
        self.memory = ObstacleMemory(require_3d_clearing=True,max_observation_gap=.60,
                                     clear_window=clear_window)
        self.motion=DynamicObstacles(self.memory.resolution)
        restore_file=self.declare_parameter('restore_state_file','').value
        self.checkpoint_restored=False
        if restore_file:
            restore_checkpoint(json.loads(Path(restore_file).read_text()),self.memory,
                               self.frame,lio_session_identity(),self.motion)
            self.checkpoint_restored=True
        self.memory_publish_rate=float(self.declare_parameter('memory_publish_rate',15.).value)
        self.navigation_publish_rate=float(self.declare_parameter('navigation_publish_rate',10.).value)
        self.clearing_rate=float(self.declare_parameter('clearing_rate',6.).value)
        # A worker result is a timestamped geometric observation, not current
        # motion permission. Allow bounded computation/dispatch latency; the
        # merge still rejects every cell hit again after its snapshot.
        self.clearing_result_max_age=float(self.declare_parameter(
            'clearing_result_max_age',1.5).value)
        if not .60<=self.clearing_result_max_age<clear_window:
            raise ValueError('Clearing result age must be below the confirmation window')
        if any(not math.isfinite(v) or v<=0. for v in (
                self.memory_publish_rate,self.navigation_publish_rate,self.clearing_rate)):
            raise ValueError('Memory publication and clearing rates must be finite and positive')
        self.geometry = None
        self.occupancy_revision=0
        self.points_cache=None
        self.points_cache_key=None
        self.published_maps={}
        self.clearing_ground=None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.pending = deque(maxlen=8)
        self.localized = False
        self.pose = None
        self.stamp = None
        self.tf_failures = 0
        self.received_scans = self.received_poses = self.accepted_scans = 0
        self.skipped_pose = self.skipped_stale = 0
        self.fault = None
        self.clouds={}
        self.ground_planes={}
        self.ground_frame_paired=False
        self.cloud_order=deque()
        self.full_poses={}
        self.pose_order=deque()
        self.raw_paired=False
        self.last_processing_ms=self.max_processing_ms=0.
        self.ray_failures=0
        self.last_ray_error=None
        self.last_observation_error=None
        self.base_world_z=0.
        self.observation_mode='unavailable'
        # CPU-heavy clearing must not share Python's GIL with ROS callbacks.
        # Spawn a clean interpreter: never fork an initialized DDS participant.
        self.worker=ProcessPoolExecutor(max_workers=1,
            mp_context=multiprocessing.get_context('spawn'),initializer=initialize_clearing_worker)
        self.clearing=None
        self.clearing_context=None
        self.clearing_keys=None
        self.last_clearing_stamp=-math.inf
        self.last_observation_stamp=-math.inf
        self.clearing_reset_stamp=-math.inf
        self.clearing_observation_resets=0
        self.last_clearing_result_age=None
        self.next_clearing=0.
        self.last_prune=0.
        self.processed=deque(maxlen=8)
        self.fast_processing_ms=self.max_fast_processing_ms=0.
        self.tracking_processing_ms=0.
        self.stale_clearing_results=0
        self.clearing_jobs=0
        self.clearing_candidates=0
        self.transient_clearing_candidates=0
        self.clearing_removed=0
        latched = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.points_pub = self.create_publisher(PointCloud2,'/safety/remembered_obstacles',1)
        self.clear_pub = self.create_publisher(PointCloud2,'/safety/cleared_obstacle_volumes',10)
        self.status_pub = self.create_publisher(String,'/safety/obstacle_memory_status',latched)
        self.maps = {}
        self.prepared_maps={}
        self.map_resolutions={
            'local':self.declare_parameter('local_navigation_resolution',.04).value,
            'global':self.declare_parameter('global_navigation_resolution',.08).value}
        if any(not math.isfinite(v) or v<=0. for v in self.map_resolutions.values()):
            raise ValueError('Navigation grid resolutions must be finite and positive')
        self.map_publishers = {}
        for label in ('local','global'):
            self.map_publishers[label] = self.create_publisher(
                OccupancyGrid,'/safety/navigation_'+label,latched)
            self.create_subscription(OccupancyGrid,'/traversability/'+label,
                lambda m,key=label:self.maps.update({key:m}),latched)
        observations=self.declare_parameter('observation_topic','/obstacles/deskewed').value
        latest_cloud=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2,observations,self.on_scan,latest_cloud)
        cloud_topic=self.declare_parameter('raw_cloud_topic','/cloud_registered_body').value
        # This callback only caches messages. Retain a short DDS queue so
        # occupied processing cannot drop the exact raw frame needed to pair
        # an independently projected observation for geometric clearing.
        raw_cloud_qos=QoSProfile(depth=8,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2,cloud_topic,self.on_cloud,raw_cloud_qos)
        ground_topic=self.declare_parameter('ground_plane_topic','/perception/deskewed_ground_plane').value
        self.create_subscription(PoseStamped,ground_topic,self.on_ground_plane,latest_cloud)
        # The bridge publishes this full 6-DoF pose only after the same health,
        # jump and timestamp checks that admit /odom and its navigation TF.
        # Waiting for that TF again can starve this Python callback queue even
        # while fresh accepted poses and obstacle endpoints are available.
        full_pose_topic=self.declare_parameter('full_pose_topic','/Odometry_validated').value
        self.validated_pose_source=full_pose_topic == '/Odometry_validated'
        # The internal timestamp cache retains received poses for pairing.
        # A separate DDS backlog of 16 poses can keep callbacks processing old
        # frames until the newest matching obstacle scan has already expired.
        self.create_subscription(Odometry,full_pose_topic,self.on_full_pose,1)
        self.create_subscription(Bool,'/lio/localization_valid',
            lambda m:setattr(self,'localized',m.data),latched)
        # Occupied writes track the 15 Hz scan; slow 3-D clearing cannot block
        # TF reception, fresh obstacle writes or the 10 Hz protection snapshot.
        self.create_timer(.02,self.process)
        # Snapshot/pickle/merge work must not sit ahead of occupied writes in
        # every 20 ms processing tick. It has its own lower-rate callback.
        # Poll completion without paying another entire 200 ms dispatch period.
        # Dispatch itself stays limited to 5 Hz and never queues a second job.
        self.create_timer(.04,self.update_clearing)
        self.memory_timer=self.create_timer(1./self.memory_publish_rate,self.publish_points)
        # Diagnostics need not reserialize the protective cloud or scan every
        # stored cell at the same rate as fresh obstacle evidence.
        self.create_timer(.20,self.publish)
        self.maps_timer=self.create_timer(1./self.navigation_publish_rate,self.publish_maps)
        self.add_on_set_parameters_callback(self.update_rates)
        self.create_service(Trigger,'/safety/save_obstacle_memory',self.save_state)
        self.get_logger().info('Obstacle memory: bounded 3-D clear evidence; blind sectors retained')

    def save_state(self,request,response):
        try:
            if not self.localized or self.pose is None or self.fault or self.stamp is None:
                raise ValueError('Memory is not healthy for a checkpoint')
            state=memory_checkpoint(self.memory,self.frame,lio_session_identity(),self.motion)
            root=Path(os.environ.get('GO2_HUMBLE_PROJECT_ROOT',Path(__file__).resolve().parents[4]))/'reports'
            path=root/('obstacle_memory_state_'+str(time.time_ns())+'.json')
            temporary=path.with_suffix('.tmp')
            temporary.write_text(json.dumps(state,allow_nan=False));temporary.replace(path)
            response.success,response.message=True,str(path)
        except Exception as exc:response.success,response.message=False,str(exc)
        return response

    def update_rates(self,parameters):
        limits={'memory_publish_rate':30.,'navigation_publish_rate':15.,'clearing_rate':10.}
        updates={p.name:p.value for p in parameters if p.name in limits}
        for name,value in updates.items():
            if (not isinstance(value,(float,int)) or isinstance(value,bool) or
                    not math.isfinite(value) or not 1.<=value<=limits[name]):
                return SetParametersResult(successful=False,reason='Invalid memory rate: '+name)
        for name,value in updates.items():
            setattr(self,name,float(value))
            timer={'memory_publish_rate':self.memory_timer,
                   'navigation_publish_rate':self.maps_timer}.get(name)
            if timer is not None:timer.timer_period_ns=int(1.e9/value)
        return SetParametersResult(successful=True)

    def on_cloud(self,msg):
        key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if key not in self.clouds:self.cloud_order.append(key)
        self.clouds[key]=msg
        # Keep paired rays through the bounded clearing dispatch latency.
        # Eight raw frames could expire while the occupied/tracking callbacks
        # were still processing the matching observation.
        while len(self.cloud_order)>16:
            self.clouds.pop(self.cloud_order.popleft(),None)

    def on_ground_plane(self,msg):
        if msg.header.frame_id!=self.body_frame:return
        key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        self.ground_planes[key]=msg
        while len(self.ground_planes)>16:self.ground_planes.pop(next(iter(self.ground_planes)))

    def on_full_pose(self,msg):
        if msg.header.frame_id!=self.frame or not msg.child_frame_id:return
        self.received_poses += 1
        key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if key not in self.full_poses:self.pose_order.append(key)
        self.full_poses[key]=msg
        while len(self.pose_order)>16:self.full_poses.pop(self.pose_order.popleft(),None)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1.e9
        age=self.get_clock().now().nanoseconds/1.e9-stamp
        if -.05<=age<=.35:
            # Sensor continuity belongs to the validated pose stream, not the
            # smaller subset of scans selected by the occupied-write worker.
            self.record_observation_continuity(stamp)
        self.process()

    def observation_transform(self,header):
        odom=self.full_poses[(header.stamp.sec,header.stamp.nanosec)]
        relative=self.tf.lookup_transform(odom.child_frame_id,header.frame_id,Time())
        return full_pose_transform(odom,relative)

    def update_clearing(self):
        if not self.localized or self.fault:return
        if self.clearing is not None and self.clearing.done():
            snapshot,stamp=self.clearing_context
            try:
                updates,elapsed=self.clearing.result()
                self.last_processing_ms=elapsed
                self.max_processing_ms=max(self.max_processing_ms,elapsed)
                self.last_clearing_result_age=self.get_clock().now().nanoseconds/1.e9-stamp
                if (-.05<=self.last_clearing_result_age<=self.clearing_result_max_age and
                        stamp>=self.clearing_reset_stamp):
                    cleared=[]
                    removed=self.memory.merge_delta(snapshot,self.clearing_keys,
                        *updates,stamp,cleared_cells=cleared)
                    self.clearing_removed+=removed
                    if removed:
                        self.occupancy_revision+=1
                        self.geometry=None
                        self.motion.prune(self.memory.cells)
                        self.publish_clear_volumes(cleared,self.clearing_ground,stamp)
                        self.publish_points()
                        self.publish_maps()
                    self.clearing_jobs+=1
                    self.last_clearing_stamp=stamp
                else:self.stale_clearing_results+=1
            except BrokenProcessPool as exc:
                self.fail_clearing_worker(exc)
            except Exception as exc:
                self.ray_failures+=1;self.last_ray_error=str(exc)
            self.clearing=None;self.clearing_context=None;self.clearing_keys=None
        if self.fault:return
        if self.clearing is not None or time.monotonic()<self.next_clearing:return
        now=self.get_clock().now().nanoseconds/1.e9
        for scan,pose in reversed(self.processed):
            stamp=scan.header.stamp.sec+scan.header.stamp.nanosec/1.e9
            # Geometric clearing is not motion permission. Its result still
            # expires at clearing_result_max_age and cannot erase a newer hit.
            # The old 350 ms dispatch cutoff starved every job after normal
            # projection, pairing and tracking latency under navigation load.
            if stamp<=self.last_clearing_stamp or not -.05<=now-stamp<=min(.75,self.clearing_result_max_age):continue
            # Missing measured ground forbids floor/column inference, not
            # genuine free 3-D beam segments in the paired raw cloud. Infinity
            # and projected/expanded endpoints are never used as ray proof.
            positive_only=any(f.name=='positive_only' for f in scan.fields)
            cloud=self.clouds.get((scan.header.stamp.sec,scan.header.stamp.nanosec))
            if cloud is None:continue
            try:
                transform=self.observation_transform(cloud.header)
            except Exception:continue
            # Fresh occupied writes already reset their confirmation counters.
            # Only old, in-range cells need costly 3-D ray tests and IPC. Keep
            # the rest in the main memory, including every blind rear obstacle.
            # Use the actual 6-DoF sensor translation for the range envelope.
            origin=transform.transform.translation
            snapshot=self.memory.clearing_snapshot((origin.x,origin.y),stamp)
            self.clearing_candidates=len(snapshot)
            if not snapshot:
                self.last_clearing_stamp=stamp
                self.next_clearing=time.monotonic()+1./self.clearing_rate
                return
            self.clearing_context=(snapshot,stamp)
            self.clearing_keys=list(snapshot)
            ground=None
            plane=(None if positive_only else
                   self.ground_planes.get((scan.header.stamp.sec,scan.header.stamp.nanosec)))
            if plane is not None:
                try:ground=world_ground_plane(plane,self.observation_transform(plane.header))
                except Exception:ground=None
            self.ground_frame_paired=ground is not None
            self.clearing_ground=ground
            geometry=np.fromiter(chain.from_iterable(snapshot.values()),dtype=float,
                                 count=7*len(snapshot)).reshape(-1,7)
            try:
                # Worker scheduling gaps do not imply missing sensor data.
                # Reset only after an actual accepted-observation gap. The
                # per-cell confirmation window still bounds sparse evidence.
                previous_stamp=(self.last_clearing_stamp
                                if self.last_clearing_stamp>=self.clearing_reset_stamp
                                else -math.inf)
                transient=(self.motion.transient_flags(self.clearing_keys,snapshot)
                           if ground is not None else np.zeros(len(snapshot),dtype=bool))
                self.transient_clearing_candidates=int(np.count_nonzero(transient))
                self.clearing=self.worker.submit(compute_clearing_delta,geometry,stamp,
                    previous_stamp,pose,cloud,transform,self.memory.clear_window,ground,
                    self.motion.flags(self.clearing_keys),transient)
            except BrokenProcessPool as exc:
                self.fail_clearing_worker(exc)
            self.next_clearing=time.monotonic()+1./self.clearing_rate
            return

    def fail_clearing_worker(self,error):
        self.fault='Obstacle clearing worker stopped: '+str(error)
        self.get_logger().error(self.fault)
        # Retain geometry for diagnosis/restart, but immediately withdraw
        # usable safety evidence. Never silently run with clearing disabled.
        self.publish_points()

    def on_scan(self,msg):
        if msg.header.frame_id == self.body_frame:
            self.received_scans += 1
            self.pending.append(msg)
            self.process()

    def process(self):
        if not self.localized or self.fault:
            return
        now=self.get_clock().now().nanoseconds/1.e9
        selected=None
        for i in range(len(self.pending)-1,-1,-1):
            m=self.pending[i]
            age=now-m.header.stamp.sec-m.header.stamp.nanosec/1.e9
            if not -.05<=age<=.35:
                self.skipped_stale += 1
                continue
            if (m.header.stamp.sec,m.header.stamp.nanosec) not in self.full_poses:
                self.skipped_pose += 1
                continue
            # LIO's transform may arrive just after the raw scan. Select the
            # newest transform-ready frame, rather than repeatedly waiting
            # for a different unavailable newest frame on every timer tick.
            if self.validated_pose_source or self.tf.can_transform(
                    self.frame,m.header.frame_id,Time.from_msg(m.header.stamp)):
                selected=i
                break
            self.tf_failures+=1
        if selected is None:
            while self.pending and now-self.pending[0].header.stamp.sec-self.pending[0].header.stamp.nanosec/1.e9>.35:
                self.pending.popleft()
            return
        for _ in range(selected):self.pending.popleft()
        while self.pending:
            scan = self.pending[0]
            age = (self.get_clock().now()-Time.from_msg(scan.header.stamp)).nanoseconds/1.e9
            if age > .35 or age < -.05:
                self.pending.popleft()
                continue
            try:
                tf = self.observation_transform(scan.header)
                if not self.validated_pose_source:
                    # Custom/raw pose inputs retain the independent TF check.
                    accepted_tf=self.tf.lookup_transform(self.frame,self.body_frame,Time.from_msg(scan.header.stamp))
                    if (math.hypot(tf.transform.translation.x-accepted_tf.transform.translation.x,
                                   tf.transform.translation.y-accepted_tf.transform.translation.y)>.05 or
                        abs(math.remainder(yaw_of(tf.transform.rotation)-yaw_of(accepted_tf.transform.rotation),2*math.pi))>.05):
                        raise ValueError('Full observation pose disagrees with accepted navigation pose')
            except Exception as exc:
                self.tf_failures += 1
                self.last_observation_error=str(exc)
                return
            pose = (tf.transform.translation.x,tf.transform.translation.y,yaw_of(tf.transform.rotation))
            stamp = scan.header.stamp.sec+scan.header.stamp.nanosec/1.e9
            try:
                started=time.perf_counter()
                points,_=cloud_points(scan,tf)
                # Preserve the pre-hit view for tracking, but publish occupied
                # protection before clustering/motion classification. Tracking
                # only relaxes subsequent clearing; it must not delay new hits.
                tracking_memory=self.memory.cells.copy()
                accepted=self.memory.mark_xyz(points,pose,stamp)
                self.geometry = None
                self.raw_paired=(scan.header.stamp.sec,scan.header.stamp.nanosec) in self.clouds
                self.fast_processing_ms=1000*(time.perf_counter()-started)
                self.max_fast_processing_ms=max(self.max_fast_processing_ms,self.fast_processing_ms)
            except Exception as exc:
                self.fault = str(exc)
                self.get_logger().error(self.fault)
                return
            if accepted:
                self.occupancy_revision+=1
                self.accepted_scans += 1
                self.pose,self.stamp=pose,scan.header.stamp
                self.base_world_z=tf.transform.translation.z
                self.last_observation_error=None
                self.observation_mode=('positive_only' if any(
                    f.name=='positive_only' for f in scan.fields) else 'measured_ground')
                self.processed.append((scan,pose))
                # Send new protection evidence immediately rather than wait
                # for another timer turn behind clouds, maps and TF callbacks.
                self.publish_points()
                try:
                    tracking_started=time.perf_counter()
                    self.motion.observe(points,pose,stamp,tracking_memory)
                    if time.monotonic()-self.last_prune>=1.:
                        self.memory.cells={k:p for k,p in self.memory.cells.items()
                            if (p[0]-pose[0])**2+(p[1]-pose[1])**2<=self.memory.radius**2}
                        self.geometry=None
                        self.motion.prune(self.memory.cells)
                        self.last_prune=time.monotonic()
                    self.tracking_processing_ms=1000*(time.perf_counter()-tracking_started)
                except Exception as exc:
                    self.fault=str(exc)
                    self.get_logger().error(self.fault)
                    self.publish_points()
                    return
            self.pending.popleft()
            return

    def record_observation_continuity(self,stamp):
        if stamp-self.last_observation_stamp>self.memory.max_observation_gap:
            self.clearing_reset_stamp=stamp
            self.clearing_observation_resets+=1
        self.last_observation_stamp=stamp

    def geometry_snapshot(self):
        if self.geometry is None:
            # Reading tuples through NumPy's nested-sequence discovery consumed
            # the scan callback budget. Iterate their unchanged scalar values
            # directly, including the same None-to-NaN conversion.
            self.geometry = np.fromiter(
                chain.from_iterable(self.memory.cells.values()), dtype=float,
                count=7*len(self.memory.cells)).reshape(-1, 7)
        return self.geometry

    def publish_points(self):
        valid = self.localized and self.pose is not None and self.fault is None
        key=(self.occupancy_revision,valid,self.pose,
             None if self.stamp is None else (self.stamp.sec,self.stamp.nanosec))
        if key==self.points_cache_key:
            self.points_pub.publish(self.points_cache)
            return
        msg = PointCloud2()
        if self.stamp is not None:
            msg.header.stamp = self.stamp  # Actual last processed scan time, never refreshed by this timer.
        msg.header.frame_id = self.frame if valid else 'unavailable'
        points = self.geometry_snapshot()[:, :2] if valid else np.empty((0, 2))
        if valid:
            delta = points - self.pose[:2]
            points = points[np.sum(delta*delta, axis=1) <= 4.]
        msg.height, msg.width, msg.point_step = 1,len(points),12
        msg.row_step = 12*len(points)
        msg.is_dense = True
        msg.fields = [PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
                      for i,k in enumerate(('x','y','z'))]
        xyz = np.zeros((len(points), 3), dtype='<f4')
        xyz[:, :2] = points
        # ROS's bytes setter validates every byte twice in Python. A typed
        # array uses the generated fast path and keeps the same wire layout.
        msg.data = array('B', xyz.tobytes())
        self.points_cache_key,self.points_cache=key,msg
        self.points_pub.publish(msg)

    def publish_clear_volumes(self,cells,ground,stamp):
        volumes=reference_clear_volumes(cells,ground)
        if not len(volumes):return
        msg=PointCloud2();msg.header.frame_id=self.frame
        msg.header.stamp=Time(seconds=stamp).to_msg()
        msg.height,msg.width,msg.point_step=1,len(volumes),20
        msg.row_step=msg.width*20;msg.is_dense=True
        msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1)
                    for i,k in enumerate(('x','y','z_min','z_max','radius'))]
        msg.data=array('B',np.asarray(volumes,dtype='<f4').tobytes())
        self.clear_pub.publish(msg)

    def publish(self):
        age = ((self.get_clock().now()-Time.from_msg(self.stamp)).nanoseconds/1.e9
               if self.stamp is not None else None)
        near_body=[]
        geometry = self.geometry_snapshot()
        if self.pose is not None:
            px,py,yaw=self.pose;c,s=math.cos(yaw),math.sin(yaw)
            now=self.get_clock().now().nanoseconds/1.e9
            delta = geometry[:, :2] - (px, py)
            x,y=c*delta[:,0]+s*delta[:,1],-s*delta[:,0]+c*delta[:,1]
            indices = np.flatnonzero((np.sum(delta*delta,axis=1)<=.6**2) &
                                     (np.abs(x)<=.50) & (np.abs(y)<=.35))[:20]
            for i in indices:
                cell = geometry[i]
                key=(math.floor(cell[0]/self.memory.resolution),
                     math.floor(cell[1]/self.memory.resolution))
                near_body.append({'body_xy':[round(float(x[i]),3),round(float(y[i]),3)],
                    'height_world':[float(v) if math.isfinite(v) else None for v in cell[5:7]],
                    'last_seen_age':round(float(now-cell[2]),3),
                    'motion_confirmed':key in self.motion.dynamic_cells,
                    'clear_confirmations':int(cell[4]),
                    'clear_evidence_age':(round(float(now-cell[3]),3)
                                          if math.isfinite(cell[3]) else None)})
        self.status_pub.publish(String(data=json.dumps({
            'localized':self.localized,'cells':len(self.memory.cells),'scan_age':age,
            'observation_mode':self.observation_mode,
            'validated_pose_source':self.validated_pose_source,
            'received_scans':self.received_scans,'received_poses':self.received_poses,
            'accepted_scans':self.accepted_scans,'skipped_pose':self.skipped_pose,
            'skipped_stale':self.skipped_stale,
            'cleared_cells':self.memory.cleared,'raw_frame_paired':self.raw_paired,
            'clearing_policy':'motion_evidence_v7_near_volume',
            'dynamic_tracks_confirmed':self.motion.confirmed_tracks,
            'dynamic_history_cells':len(self.motion.dynamic_cells),
            'checkpoint_restored':self.checkpoint_restored,
            'ground_frame_paired':self.ground_frame_paired,
            'clearing_transport':'numeric_delta_v1',
            'memory_publish_rate':self.memory_publish_rate,
            'navigation_publish_rate':self.navigation_publish_rate,
            'clearing_rate':self.clearing_rate,
            'clearing_confirmation_window':self.memory.clear_window,
            'clearing_result_max_age':self.clearing_result_max_age,
            'last_clearing_result_age':self.last_clearing_result_age,
            'clearing_observation_resets':self.clearing_observation_resets,
            'clearing_jobs':self.clearing_jobs,'clearing_candidates':self.clearing_candidates,
            'transient_clearing_policy':'grouped_ground_evidence_v1',
            'transient_clearing_candidates':self.transient_clearing_candidates,
            'clearing_removed_cells':self.clearing_removed,
            'processing_ms':round(self.last_processing_ms,3),
            'max_processing_ms':round(self.max_processing_ms,3),
            'fast_processing_ms':round(self.fast_processing_ms,3),
            'max_fast_processing_ms':round(self.max_fast_processing_ms,3),
            'tracking_processing_ms':round(self.tracking_processing_ms,3),
            'clearing_busy':self.clearing is not None,
            'clearing_source_age':(self.get_clock().now().nanoseconds/1.e9-self.clearing_context[1]
                                   if self.clearing_context is not None else None),
            'stale_clearing_results':self.stale_clearing_results,
            'ray_failures':self.ray_failures,'last_ray_error':self.last_ray_error,
            'last_observation_error':self.last_observation_error,
            'unconfirmed_height_cells':int(np.isnan(geometry[:,5]).sum()),
            'base_height_world':self.base_world_z,'near_body_observations':near_body[:20],
            'tf_waits':self.tf_failures,'fault':self.fault})))
    def publish_maps(self):
        if not self.localized or self.pose is None or self.fault is not None:
            return
        world=self.geometry_snapshot()[:, :2]
        for key,source in list(self.maps.items()):
            try:
                if source.header.frame_id == self.frame:
                    tx,ty,yaw = 0.,0.,0.
                else:
                    t = self.tf.lookup_transform(source.header.frame_id,self.frame,Time())
                    tx,ty,yaw = t.transform.translation.x,t.transform.translation.y,yaw_of(t.transform.rotation)
                revision=(id(source),self.occupancy_revision,tx,ty,yaw)
                if self.published_maps.get(key)==revision:continue
                c,s = math.cos(yaw),math.sin(yaw)
                origin = source.info.origin
                oc,os = math.cos(yaw_of(origin.orientation)),math.sin(yaw_of(origin.orientation))
                out = OccupancyGrid()
                out.header,out.info = source.header,deepcopy(source.info)
                cached=self.prepared_maps.get(key)
                if cached is None or cached[0] is not source:
                    base,resolution=navigation_grid(source.data,source.info.width,source.info.height,
                                                    source.info.resolution,self.map_resolutions[key])
                    cached=(source,base,resolution)
                    self.prepared_maps[key]=cached
                grid,resolution=cached[1].copy(),cached[2]
                out.info.resolution=resolution
                out.info.height,out.info.width=grid.shape
                dx=tx+c*world[:,0]-s*world[:,1]-origin.position.x
                dy=ty+s*world[:,0]+c*world[:,1]-origin.position.y
                ix=np.floor((oc*dx+os*dy)/resolution).astype(np.int64)
                iy=np.floor((-os*dx+oc*dy)/resolution).astype(np.int64)
                inside=(ix>=0)&(ix<out.info.width)&(iy>=0)&(iy<out.info.height)
                grid[iy[inside],ix[inside]]=100
                out.data=array('b');out.data.frombytes(grid.tobytes())
                self.map_publishers[key].publish(out)
                self.published_maps[key]=revision
            except Exception as exc:
                self.get_logger().warning('Memory map transform unavailable: '+str(exc),throttle_duration_sec=3.)


def main():
    # Python owns SIGINT; stop the executor/worker before closing ROS context.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RememberedObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.worker.shutdown(wait=True,cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':main()
