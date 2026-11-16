#!/usr/bin/env python3
"""Read-only static comparison of nearby 3-D returns, scans, memory and maps."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy,QoSProfile,qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2,LaserScan
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener


def xyz(message):
    fields={f.name:f for f in message.fields}
    if message.height!=1 or not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):
        raise ValueError('Unsupported XYZ cloud layout')
    dtype=np.dtype({'names':['x','y','z'],'formats':['>f4' if message.is_bigendian else '<f4']*3,
        'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':message.point_step})
    data=np.frombuffer(message.data,dtype=dtype,count=message.width)
    p=np.column_stack([data[k] for k in ('x','y','z')])
    return p[np.isfinite(p).all(axis=1)]


def transform(points,tf):
    q=tf.transform.rotation;x,y,z,w=q.x,q.y,q.z,q.w
    rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    p=tf.transform.translation
    return np.einsum('ij,kj->ki',rotation,points)+[p.x,p.y,p.z]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=10.)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--x',type=float,default=1.)
    parser.add_argument('--y',type=float,default=0.)
    parser.add_argument('--half-depth',type=float,default=.45)
    parser.add_argument('--half-width',type=float,default=.45)
    parser.add_argument('--min-z',type=float,default=-.20)
    parser.add_argument('--max-z',type=float,default=.80)
    args=parser.parse_args()
    if args.output.exists():parser.error('Refusing to overwrite a report')
    if not 1<=args.seconds<=60 or min(args.half_depth,args.half_width)<=0:
        parser.error('Invalid duration or inspection region')
    rclpy.init();node=rclpy.create_node('front_obstacle_static_readonly')
    buffer=Buffer();listener=TransformListener(buffer,node)
    recent={};rows=defaultdict(list);faults=defaultdict(int);map_data={}
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    clouds=('/livox/lidar','/cloud_registered_body','/safety/remembered_obstacles')
    scans=('/scan_safety','/scan')
    maps=('/traversability/local','/safety/navigation_local','/safety/navigation_global',
          '/local_costmap/costmap','/global_costmap/costmap')
    for topic in clouds:
        node.create_subscription(PointCloud2,topic,lambda m,t=topic:recent.update({t:m}),qos_profile_sensor_data)
    for topic in scans:
        node.create_subscription(LaserScan,topic,lambda m,t=topic:recent.update({t:m}),qos_profile_sensor_data)
    def full(m,t):
        recent[t]=m;map_data[t]=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width).copy()
    def update(m,t):
        if t in map_data:map_data[t][m.y:m.y+m.height,m.x:m.x+m.width]=np.asarray(m.data).reshape(m.height,m.width)
    for topic in maps:
        node.create_subscription(OccupancyGrid,topic,lambda m,t=topic:full(m,t),latched)
        if 'costmap' in topic:
            node.create_subscription(OccupancyGridUpdate,topic+'_updates',lambda m,t=topic:update(m,t),qos_profile_sensor_data)
    for topic in ('/safety/obstacle_memory_status','/safety/motion_diagnostics'):
        node.create_subscription(String,topic,lambda m,t=topic:recent.update({t:json.loads(m.data)}),
            latched if topic.endswith('status') else qos_profile_sensor_data)
    def region(points):
        return (np.abs(points[:,0]-args.x)<=args.half_depth)&(np.abs(points[:,1]-args.y)<=args.half_width)
    began=time.monotonic();next_sample=began+2.;seen={}
    while time.monotonic()-began<args.seconds+2.:
        rclpy.spin_once(node,timeout_sec=.02)
        if time.monotonic()<next_sample:continue
        next_sample=time.monotonic()+.25
        for topic in clouds+scans:
            m=recent.get(topic)
            if m is None:continue
            stamp=m.header.stamp.sec+m.header.stamp.nanosec/1.e9
            if seen.get(topic)==stamp:continue
            try:
                if topic in clouds:
                    p=xyz(m)
                    if topic=='/livox/lidar':
                        p=p[np.linalg.norm(p,axis=1)>1.e-4]  # Invalid zero returns are not objects at the sensor origin.
                    if m.header.frame_id!='base_link':
                        p=transform(p,buffer.lookup_transform('base_link',m.header.frame_id,Time()))
                    p=p[region(p)]
                    elevated=p[(p[:,2]>=args.min_z)&(p[:,2]<=args.max_z)]
                    rows[topic].append({'stamp':stamp,'roi_points':len(p),'elevated_points':len(elevated),
                        'xyz_min':p.min(axis=0).tolist() if len(p) else None,
                        'xyz_max':p.max(axis=0).tolist() if len(p) else None})
                else:
                    d=np.asarray(m.ranges);a=m.angle_min+np.arange(len(d))*m.angle_increment
                    finite=np.isfinite(d)&(d>=m.range_min)&(d<=m.range_max)
                    d,a=d[finite],a[finite]
                    p=np.column_stack((d*np.cos(a),d*np.sin(a)))
                    p=p[region(p)]
                    rows[topic].append({'stamp':stamp,'roi_obstacle_bins':len(p),
                        'xy_min':p.min(axis=0).tolist() if len(p) else None,
                        'xy_max':p.max(axis=0).tolist() if len(p) else None})
                seen[topic]=stamp
            except Exception as exc:faults[topic+': '+str(exc)]+=1
    summary={}
    for topic,v in rows.items():
        keys=('roi_points','elevated_points') if topic in clouds else ('roi_obstacle_bins',)
        summary[topic]={'samples':len(v),'last':v[-1]}
        for key in keys:summary[topic][key+'_min_median_max']=[int(min(r[key] for r in v)),float(np.median([r[key] for r in v])),int(max(r[key] for r in v))]
    probe=np.array([[x,y,0.] for x in np.arange(args.x-args.half_depth,args.x+args.half_depth,.04)
                               for y in np.arange(args.y-args.half_width,args.y+args.half_width,.04)])
    for topic,grid in map_data.items():
        m=recent[topic]
        try:
            p=transform(probe,buffer.lookup_transform(m.header.frame_id,'base_link',Time()))
            o=m.info.origin;q=o.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            d=p[:,:2]-[o.position.x,o.position.y];c,s=math.cos(yaw),math.sin(yaw)
            ix=np.floor((c*d[:,0]+s*d[:,1])/m.info.resolution).astype(int)
            iy=np.floor((-s*d[:,0]+c*d[:,1])/m.info.resolution).astype(int)
            valid=(ix>=0)&(iy>=0)&(ix<m.info.width)&(iy<m.info.height)
            indices=np.unique(iy[valid]*m.info.width+ix[valid]);values=grid.ravel()[indices]
            summary[topic]={'unique_roi_cells':len(values),'occupied':int(np.count_nonzero(values>=90)),
                'unknown':int(np.count_nonzero(values<0)),'max_cost':int(values.max()) if len(values) else None}
        except Exception as exc:faults[topic+': '+str(exc)]+=1
    result={'inspection':'static; latest TF used; positive evidence is not object identity',
        'region':vars(args)|{'output':str(args.output)},'summary':summary,'samples':dict(rows),
        'diagnostics':{k:v for k,v in recent.items() if isinstance(v,dict)},'errors':dict(faults)}
    args.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
