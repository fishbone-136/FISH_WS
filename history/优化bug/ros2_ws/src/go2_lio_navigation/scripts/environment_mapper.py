#!/usr/bin/env python3
"""Persistent environment reference map, corrected-keyframe terrain, and export."""

import collections
from array import array
import datetime
import json
import os
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import yaml

from environment_grid import (build_layers, pose_matrix, transform_points, voxel_filter,
                              cleared_point_mask)


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def pose_array(pose):
    p, q = pose.position, pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def xyz_from_cloud(cloud):
    fields = {f.name: f for f in cloud.fields}
    if not all(name in fields and fields[name].datatype == PointField.FLOAT32
               for name in ('x', 'y', 'z')) or cloud.point_step < 12:
        raise ValueError('Mapping requires float32 XYZ fields')
    if len(cloud.data) < cloud.row_step * cloud.height:
        raise ValueError('Truncated mapping cloud')
    endian = '>' if cloud.is_bigendian else '<'
    dtype = np.dtype(dict(
        names=['x', 'y', 'z'], formats=[endian+'f4']*3,
        offsets=[fields[name].offset for name in ('x', 'y', 'z')],
        itemsize=cloud.point_step))
    data = np.ndarray((cloud.height, cloud.width), dtype=dtype,
                      buffer=cloud.data, strides=(cloud.row_step, cloud.point_step))
    points = np.column_stack([data[name].ravel() for name in ('x', 'y', 'z')])
    return points[np.isfinite(points).all(axis=1)]


def cloud_message(points, stamp, frame, costs=None):
    result = PointCloud2()
    result.header.stamp, result.header.frame_id = stamp, frame
    result.height, result.width = 1, len(points)
    names = ['x', 'y', 'z'] if costs is None else ['x', 'y', 'z', 'intensity']
    result.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                     for i, name in enumerate(names)]
    values = points if costs is None else np.column_stack([points, costs])
    result.point_step = 4 * len(names)
    result.row_step = result.point_step * len(points)
    result.is_dense = True
    # Generated ROS setters otherwise validate every byte in Python. The
    # typed array takes the fast path with exactly the same PointCloud2 layout.
    result.data = array('B', np.asarray(values, dtype='<f4').tobytes())
    return result


class EnvironmentMapper(Node):
    def __init__(self):
        super().__init__('environment_mapper')

        def param(name, value):
            return self.declare_parameter(name, value).value
        self.frame = param('map_frame', 'map')
        self.odom_frame = param('odom_frame', 'odom_lio')
        self.resolution = float(param('resolution', 0.15))
        self.leaf = float(param('keyframe_voxel_size', 0.10))
        self.max_range = float(param('mapping_range', 8.0))
        self.min_z = float(param('min_sensor_height', -1.40))
        self.max_z = float(param('max_sensor_height', 1.50))
        self.max_keyframes = int(param('max_keyframes', 400))
        self.max_extent = float(param('max_map_extent', 60.0))
        self.min_points = int(param('min_points_per_cell', 2))
        self.rate = float(param('update_rate', 1.0))
        self.cloud_rate = float(param('cloud_publish_rate', 1.0))
        self.last_cloud_publish = -1.e30
        self.max_preview_age = float(param('max_preview_age', 0.8))
        self.use_graph_fallback=bool(param('use_pose_graph_fallback',False))
        self.output_directory = param('output_directory', '')
        self.parent_launch_pid = int(param('parent_launch_pid', 0))
        if min(self.resolution, self.leaf, self.rate, self.max_extent) <= 0:
            raise ValueError('Environment map limits must be positive')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.valid = False
        self.body_cache = collections.deque(maxlen=60)
        self.poses = {}
        self.scans = {}
        self.preview = None
        self.fallback = None
        self.fallback_pending = None
        self.last_layers = None
        self.grid_bounds = None
        self.last_cloud = None
        self.last_geometry = None
        self.last_status = None
        self.pose_revision = 0
        self.history_revision = -1
        self.history_points = np.empty((0, 3), dtype=np.float32)
        self.world_cache={}
        self.pending_clears=collections.deque(maxlen=64)
        self.last_build_key=None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.environment_pub = self.create_publisher(
            PointCloud2, '/mapping/environment_cloud', latched)
        self.surface_pub = self.create_publisher(PointCloud2, '/mapping/terrain_surface', latched)
        self.map_pub = self.create_publisher(OccupancyGrid, '/mapping/environment_map', latched)
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.on_body,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cloud_registered', self.on_preview,
                                 qos_profile_sensor_data)
        self.create_subscription(NavPath, '/loop_closure/corrected_path', self.on_path, latched)
        if self.use_graph_fallback:
            self.create_subscription(
                PointCloud2, '/loop_closure/global_map', self.on_fallback, latched)
        self.create_subscription(Bool, '/lio/localization_valid', self.on_valid, latched)
        self.create_subscription(PointCloud2,'/safety/cleared_obstacle_volumes',
                                 self.on_clear_volumes,10)
        self.create_service(Trigger, '/mapping/save', self.save)
        self.create_timer(1.0 / self.rate, self.publish_map)
        self.get_logger().info('Persistent environment mapping ready: %.2f m/cell, %.1f Hz; '
                               'reference layers do not overwrite live Nav2 safety maps' %
                               (self.resolution, self.rate))

    def on_valid(self, message):
        self.valid = message.data
        if not self.valid:
            self.preview = None
            self.body_cache.clear()
            # Keep the last environment for orientation, but stop integrating
            # until LIO recovers. The base motion gate remains in charge.

    def on_body(self, message):
        if self.valid:
            self.body_cache.append((stamp_ns(message.header.stamp), message))

    def on_preview(self, message):
        if self.valid:
            self.preview = message

    def on_path(self, message):
        if message.header.frame_id != self.frame:
            self.get_logger().error('Corrected path has unexpected frame')
            return
        poses = {stamp_ns(p.header.stamp): pose_array(p.pose)
                 for p in message.poses[-self.max_keyframes:]}
        if poses != self.poses:
            self.poses = poses
            self.scans = {k: v for k, v in self.scans.items() if k in poses}
            self.pose_revision += 1

    def on_fallback(self, message):
        if message.header.frame_id == self.frame:
            self.fallback_pending = message

    def on_clear_volumes(self,message):
        names=('x','y','z_min','z_max','radius')
        fields={f.name:f for f in message.fields}
        if (message.height!=1 or not all(k in fields and fields[k].datatype==7 for k in names)
                or len(message.data)<message.width*message.point_step):return
        dtype=np.dtype({'names':names,'formats':['>f4' if message.is_bigendian else '<f4']*5,
                        'offsets':[fields[k].offset for k in names],'itemsize':message.point_step})
        data=np.frombuffer(message.data,dtype=dtype,count=message.width)
        volumes=np.column_stack([data[k] for k in names])
        if (not np.isfinite(volumes).all() or (volumes[:,4]<=0).any() or
                (volumes[:,3]<volumes[:,2]).any()):return
        self.pending_clears.append((message.header,volumes))

    def world_scan(self,stamp):
        scan,pose=self.scans[stamp],self.poses[stamp]
        cached=self.world_cache.get(stamp)
        if cached is None or cached[0]!=pose or cached[1] is not scan:
            points=transform_points(scan,pose_matrix(pose))
            cached=(pose,scan,points)
            self.world_cache[stamp]=cached
        return cached[2]

    def apply_clear_volumes(self):
        # The safety memory already paid for 3-D visibility and temporal
        # confirmation. Re-use its proof; do not build another ray index here.
        changed=False
        for _ in range(len(self.pending_clears)):
            header,volumes=self.pending_clears.popleft()
            try:
                tf=self.tf.lookup_transform(header.frame_id,self.frame,rclpy.time.Time())
            except Exception:
                self.pending_clears.append((header,volumes));continue
            t,q=tf.transform.translation,tf.transform.rotation
            frame_from_map=pose_matrix((t.x,t.y,t.z,q.x,q.y,q.z,q.w))
            cleared_at=stamp_ns(header.stamp)
            for stamp in list(self.scans):
                # A later observation is allowed to put a real object back.
                if stamp>cleared_at or stamp not in self.poses:continue
                world=self.world_scan(stamp)
                if not len(world):continue
                points=transform_points(world,frame_from_map)
                # Reject far keyframes before allocating a spatial index.
                if (np.any(points[:,:2].max(axis=0)<(volumes[:,:2]-volumes[:,4,None]).min(axis=0)) or
                    np.any(points[:,:2].min(axis=0)>(volumes[:,:2]+volumes[:,4,None]).max(axis=0))):continue
                remove=cleared_point_mask(points,volumes)
                if remove.any():
                    self.scans[stamp]=self.scans[stamp][~remove]
                    self.world_cache.pop(stamp,None);changed=True
            if self.fallback is not None and len(self.fallback):
                remove=cleared_point_mask(transform_points(self.fallback,frame_from_map),volumes)
                if remove.any():self.fallback=self.fallback[~remove];changed=True
        if changed:self.pose_revision+=1

    def reconcile_keyframes(self):
        if not self.body_cache:
            return
        oldest = self.body_cache[0][0] - 50000000
        for stamp in self.poses:
            if stamp in self.scans or stamp < oldest:
                continue
            closest, cloud = min(self.body_cache, key=lambda entry: abs(entry[0] - stamp))
            difference = abs(closest - stamp)
            if difference > 50000000:
                continue
            points = xyz_from_cloud(cloud)
            radius = np.linalg.norm(points, axis=1)
            points = points[(radius > 0.25) & (radius < self.max_range) &
                            (points[:, 2] >= self.min_z) & (points[:, 2] <= self.max_z)]
            self.scans[stamp] = voxel_filter(points, self.leaf)
            self.pose_revision += 1

    def historical_cloud(self):
        if self.history_revision == self.pose_revision:
            return self.history_points
        self.world_cache={k:v for k,v in self.world_cache.items() if k in self.scans}
        chunks = [self.world_scan(stamp) for stamp in self.poses if stamp in self.scans]
        if self.fallback is not None and len(self.scans) < len(self.poses):
            # A late-starting mapper cannot recover past dense body scans.
            # Use the already-corrected coarse graph cloud for that old area.
            chunks.append(self.fallback)
        points = np.concatenate(chunks) if chunks else np.empty((0, 3), dtype=np.float32)
        self.history_points = voxel_filter(points, self.leaf)
        self.history_revision = self.pose_revision
        return self.history_points

    def publish_map(self):
        if self.parent_launch_pid:
            try:
                argv = Path('/proc/%d/cmdline' % self.parent_launch_pid).read_bytes()
                parent_alive = b'go2_lio_navigation.launch.py' in argv
            except OSError:
                parent_alive = False
            if not parent_alive:
                self.get_logger().info(
                    'Parent navigation launch stopped; closing environment view')
                rclpy.shutdown()
                return
        if not self.valid:
            return
        try:
            self.reconcile_keyframes()
            if self.fallback_pending is not None:
                self.fallback = xyz_from_cloud(self.fallback_pending)
                self.fallback_pending = None
                self.pose_revision += 1
            self.apply_clear_volumes()
            points = self.historical_cloud()
            preview_key=None
            if self.preview is not None:
                age = (self.get_clock().now().nanoseconds -
                       stamp_ns(self.preview.header.stamp)) / 1.e9
                if -.05 <= age < self.max_preview_age:
                    transform = self.tf.lookup_transform(
                        self.frame, self.odom_frame, rclpy.time.Time())
                    t, q = transform.transform.translation, transform.transform.rotation
                    matrix = pose_matrix((t.x, t.y, t.z, q.x, q.y, q.z, q.w))
                    preview_key=(stamp_ns(self.preview.header.stamp),tuple(matrix.ravel()))
                    preview = xyz_from_cloud(self.preview)
                    # Bound preview radius around the sensor, not map origin.
                    sensor = self.tf.lookup_transform(
                        self.odom_frame, 'livox_frame', rclpy.time.Time())
                    p = sensor.transform.translation
                    delta = preview - np.array([p.x, p.y, p.z])
                    preview = preview[(np.linalg.norm(delta, axis=1) < self.max_range) &
                                      (delta[:, 2] >= self.min_z) & (delta[:, 2] <= self.max_z)]
                    points = np.concatenate([points, transform_points(preview, matrix)])
            build_key=(self.pose_revision,preview_key)
            if build_key==self.last_build_key:
                # Keep latched outputs; do not sort/rasterize old geometry on
                # timer retries. A newly attached subscriber receives it too.
                if self.last_layers is not None:
                    self.publish_reference_clouds(self.last_geometry,self.last_layers)
                return
            # Classification uses dense keyframe observations; downsampling
            # the result first would erase the vertical extent of thin walls.
            layers = build_layers(points, self.resolution, self.min_points, self.max_extent,
                                  previous_bounds=self.grid_bounds)
            if layers is None:
                return
            self.grid_bounds = np.concatenate([
                layers['origin'], layers['origin'] + self.resolution *
                np.array([layers['width'], layers['height']])])
            current = self.get_clock().now()
            stamp = current.to_msg()
            self.publish_reference_clouds(points,layers)
            grid = OccupancyGrid()
            grid.header.stamp, grid.header.frame_id = stamp, self.frame
            grid.info.map_load_time = stamp
            grid.info.resolution = self.resolution
            grid.info.width, grid.info.height = layers['width'], layers['height']
            grid.info.origin.position.x = float(layers['origin'][0])
            grid.info.origin.position.y = float(layers['origin'][1])
            # This plane is only a reference overlay, below measured terrain.
            # Grey unknown cells must not look like black occupied cells or
            # be fabricated into free terrain where HAP has no floor return.
            if len(layers['surface']):
                grid.info.origin.position.z = (
                    float(np.percentile(layers['surface'][:, 2], 5)) - 0.04)
            grid.info.origin.orientation.w = 1.0
            grid.data = layers['costs'].ravel().tolist()
            self.map_pub.publish(grid)
            self.last_layers, self.last_geometry = layers, points
            self.last_build_key=build_key
            status = (len(self.scans), len(self.poses), grid.info.width, grid.info.height)
            if status != self.last_status:
                self.get_logger().info('Environment map: %d/%d dense keyframes, %dx%d cells, '
                                       '%d terrain cells' % (*status, len(layers['surface'])),
                                       throttle_duration_sec=5.0)
                self.last_status = status
        except Exception as error:
            self.get_logger().warning('Environment update skipped: %s' % error,
                                      throttle_duration_sec=5.0)

    def publish_reference_clouds(self,points,layers):
        # Late RViz subscriptions need a latched 3-D sample even when geometry
        # has not changed. Cache its voxelization for that whole build revision.
        now=self.get_clock().now();seconds=now.nanoseconds/1.e9
        if seconds-self.last_cloud_publish<1./max(.1,self.cloud_rate):return
        if self.environment_pub.get_subscription_count()>0:
            key=(id(points),id(layers))
            if getattr(self,'cloud_cache_key',None)!=key:
                self.last_cloud=voxel_filter(points,self.resolution)
                self.cloud_cache_key=key
            self.environment_pub.publish(cloud_message(self.last_cloud,now.to_msg(),self.frame))
        if self.surface_pub.get_subscription_count()>0:
            self.surface_pub.publish(cloud_message(layers['surface'],now.to_msg(),self.frame,
                                                   layers['surface_costs']))
        self.last_cloud_publish=seconds

    def save(self, request, response):
        if self.last_layers is None:
            response.success, response.message = False, 'No valid environment map yet'
            return response
        try:
            root = Path(self.output_directory) if self.output_directory else (
                Path(os.environ.get('GO2_HUMBLE_PROJECT_ROOT', os.getcwd())) / 'maps')
            directory = root / datetime.datetime.now().strftime('environment_%Y%m%d_%H%M%S_%f')
            directory.mkdir(parents=True, exist_ok=False)
            layers = self.last_layers
            costs = layers['costs']
            image = np.where(costs < 0, 205, np.where(costs >= 90, 0, 254)).astype(np.uint8)
            with (directory / 'map.pgm').open('wb') as stream:
                stream.write(('P5\n%d %d\n255\n' % (layers['width'], layers['height'])).encode())
                stream.write(np.flipud(image).tobytes())
            origin = [float(x) for x in layers['origin']] + [0.0]
            metadata = dict(image='map.pgm', mode='trinary', resolution=self.resolution,
                            origin=origin, negate=0, occupied_thresh=0.65, free_thresh=0.25)
            (directory / 'map.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
            np.savez_compressed(directory / 'elevation.npz', elevation_m=layers['elevation'],
                                terrain_cost=costs, origin_xy=layers['origin'],
                                resolution=self.resolution)
            cloud = voxel_filter(self.last_geometry, self.resolution)
            header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                      'COUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                      'POINTS %d\nDATA binary\n' % (len(cloud), len(cloud)))
            with (directory / 'environment.pcd').open('wb') as stream:
                stream.write(header.encode())
                stream.write(np.asarray(cloud, dtype='<f4').tobytes())
            info = dict(frame=self.frame, reference_map=True, relocalization_ready=False,
                        dense_keyframes=len(self.scans), graph_keyframes=len(self.poses),
                        clipped=layers['clipped'], elevation_units='metres',
                        note='Exported reference map; validate before saved-map navigation.')
            (directory / 'metadata.json').write_text(json.dumps(info, indent=2), encoding='utf-8')
            response.success, response.message = True, str(directory)
        except Exception as error:
            response.success, response.message = False, str(error)
        return response


def main():
    rclpy.init()
    node = EnvironmentMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
