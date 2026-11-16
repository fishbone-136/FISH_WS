#!/usr/bin/python3
"""Capture aligned map layers and measured points without sending motion goals."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid,Odometry
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener
from check_front_obstacle import xyz,transform


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.npz').exists():
        parser.error('Refusing to overwrite a snapshot')
    rclpy.init();node=rclpy.create_node('navigation_layer_snapshot')
    buffer=Buffer();listener=TransformListener(buffer,node)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latest={};grids={};arrays={};metadata={}
    topics=('/mapping/environment_map','/traversability/local','/traversability/global',
            '/safety/navigation_local','/safety/navigation_global',
            '/local_costmap/costmap','/global_costmap/costmap')
    def full(m,t):
        latest[t]=m;grids[t]=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width).copy()
    def update(m,t):
        if t in grids:grids[t][m.y:m.y+m.height,m.x:m.x+m.width]=np.asarray(m.data).reshape(m.height,m.width)
    for topic in topics:
        node.create_subscription(OccupancyGrid,topic,lambda m,t=topic:full(m,t),latched)
        if 'costmap' in topic:
            node.create_subscription(OccupancyGridUpdate,topic+'_updates',lambda m,t=topic:update(m,t),10)
    clouds=('/cloud_registered_body','/obstacles/deskewed','/safety/remembered_obstacles')
    for topic in clouds:
        node.create_subscription(PointCloud2,topic,lambda m,t=topic:latest.update({t:m}),qos_profile_sensor_data)
    node.create_subscription(PoseStamped,'/perception/deskewed_ground_plane',
        lambda m:latest.update(ground=m),qos_profile_sensor_data)
    node.create_subscription(Odometry,'/Odometry_validated',lambda m:latest.update(pose=m),10)
    node.create_subscription(String,'/safety/obstacle_memory_status',
        lambda m:latest.update(status=json.loads(m.data)),latched)
    started=time.monotonic()
    while time.monotonic()-started<4.:rclpy.spin_once(node,timeout_sec=.02)
    for topic in topics:
        if topic not in grids:continue
        msg=latest[topic];key=topic.strip('/').replace('/','_')
        arrays[key]=grids[topic]
        yy,xx=np.indices(grids[topic].shape)
        p=np.column_stack((msg.info.origin.position.x+(xx.ravel()+.5)*msg.info.resolution,
                           msg.info.origin.position.y+(yy.ravel()+.5)*msg.info.resolution,
                           np.zeros(xx.size)))
        body=transform(p,buffer.lookup_transform('base_link',msg.header.frame_id,Time()))
        arrays[key+'_body_xy']=body[:,:2].reshape(*grids[topic].shape,2)
        roi=(body[:,0]>.5)&(body[:,0]<2.5)&(np.abs(body[:,1])<1.)
        vals=grids[topic].ravel()[roi]
        metadata[topic]={'resolution':msg.info.resolution,'frame':msg.header.frame_id,
            'origin':[msg.info.origin.position.x,msg.info.origin.position.y],
            'roi_cells':len(vals),'roi_occupied':int((vals>=90).sum()),
            'roi_free':int((vals==0).sum()),'roi_unknown':int((vals<0).sum())}
    for topic in clouds:
        if topic not in latest:continue
        msg=latest[topic];key=topic.strip('/').replace('/','_')
        points=xyz(msg)
        if msg.header.frame_id=='unavailable':
            metadata[topic]={'unavailable':True}
            continue
        if msg.header.frame_id!='base_link':
            points=transform(points,buffer.lookup_transform('base_link',msg.header.frame_id,Time()))
        arrays[key]=points
    for key in ('ground','pose'):
        if key not in latest:continue
        m=latest[key];p=m.pose if key=='ground' else m.pose.pose;q=p.orientation
        metadata[key]={'frame':m.header.frame_id,'position':[p.position.x,p.position.y,p.position.z],
                       'orientation':[q.x,q.y,q.z,q.w]}
    metadata['status']=latest.get('status')
    client=node.create_client(Trigger,'/safety/save_obstacle_memory')
    if client.wait_for_service(timeout_sec=1.):
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=4.)
        if future.done() and future.result().success:metadata['checkpoint']=future.result().message
    np.savez_compressed(args.output.with_suffix('.npz'),**arrays)
    args.output.write_text(json.dumps(metadata,indent=2))
    print(json.dumps(metadata,indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
