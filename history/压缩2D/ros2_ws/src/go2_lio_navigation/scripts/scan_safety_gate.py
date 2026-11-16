#!/usr/bin/env python3
"""Fail-closed scan freshness gate after Humble's collision monitor."""

import math
import time
import json
import numpy as np

from geometry_msgs.msg import PolygonStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from go2_navigation.runtime_guard import acquire_process_lock

from turn_clearance import braking_speed_scale, arc_collision_time
from observation_window import ForwardObservationWindow, bound_sparse_command
from reactive_safety import ClosingApproach, SafetyRecoveryRamp, stop_zone_clear, close_clearance_scale


class ScanSafetyGate(Node):
    def __init__(self):
        super().__init__('scan_safety_gate')
        self.max_age = self.declare_parameter('max_scan_age', 0.35).value
        self.command_timeout = self.declare_parameter('command_timeout', 0.2).value
        self.footprint_max_age = self.declare_parameter('max_footprint_age', 0.5).value
        rate = self.declare_parameter('publish_rate', 50.0).value
        scan_topic = self.declare_parameter('scan_topic', '/scan_safety').value
        from go2_navigation.safety_geometry import configured_protection
        front, rear, half_width = configured_protection()
        self.stop_front = float(self.declare_parameter('stop_front', front).value)
        self.stop_rear = float(self.declare_parameter('stop_rear', rear).value)
        self.stop_width = float(self.declare_parameter('stop_half_width', half_width).value)
        self.approach = ClosingApproach(
            float(self.declare_parameter('approach_front', front).value),
            float(self.declare_parameter('approach_half_width', .50).value),
            float(self.declare_parameter('approach_time', 1.2).value))
        self.approach_observe_ms = 0.
        self.memory_required = bool(self.declare_parameter('memory_required', True).value)
        self.memory_received = self.memory_stamp = -1.e30
        self.memory_frame = None
        self.memory_supported = False
        self.memory_xy = np.empty((0, 2))
        self.ego_pose = None
        self.ego_frame = None
        self.recovery = SafetyRecoveryRamp(
            self.declare_parameter('recovery_linear_accel',.85).value,
            self.declare_parameter('recovery_angular_accel',1.80).value)
        if not all(math.isfinite(x) and x > 0. for x in (
                self.stop_front, self.stop_rear, self.stop_width,
                self.approach.stop_front, self.approach.half_width, self.approach.horizon)):
            raise ValueError('Reactive stop envelope and approach limits must be positive')
        self.turn_half_length = float(self.declare_parameter('turn_half_length', max(front, rear)).value)
        self.turn_half_width = float(self.declare_parameter('turn_half_width', half_width).value)
        self.turn_margin = float(self.declare_parameter('turn_margin', 0.0).value)
        if min(self.max_age, self.command_timeout, self.footprint_max_age, rate) <= 0:
            raise ValueError('Scan safety gate limits must be positive')
        if not all(math.isfinite(x) for x in (
                self.turn_half_length, self.turn_half_width, self.turn_margin)
        ) or min(self.turn_half_length, self.turn_half_width) <= 0 or (
                self.turn_margin < 0):
            raise ValueError('Turn envelope must be positive and margin nonnegative')
        self.command = Twist()
        self.command_received = self.scan_received = -1.e30
        self.scan_stamp = -1.e30
        self.front_observed = False
        self.forward_window = ForwardObservationWindow()
        self.last_observed_command = (0., 0., 0.)
        self.last_sent_command = (0., 0., 0.)
        self.last_valid = None
        self.footprint_received = self.footprint_stamp = -1.e30
        self.footprint_supported = False
        self.last_footprint_valid = self.last_ready = None
        self.obstacle_points = []
        self.footprint_half_length = self.turn_half_length
        self.footprint_half_width = self.turn_half_width
        self.last_turn_clear = None
        self.last_diagnostic = -1.e30
        self.ego_velocity = (0., 0., 0.)
        self.ego_received = self.ego_stamp = -1.e30
        self.diagnostics = self.create_publisher(String, '/safety/motion_diagnostics', 1)
        self.output = self.create_publisher(Twist, '/cmd_vel_guarded', 1)
        self.status = self.create_publisher(
            Bool, '/safety/scan_valid',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.footprint_status = self.create_publisher(
            Bool, '/safety/footprint_valid',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.turn_status = self.create_publisher(
            Bool, '/safety/turn_clearance_valid',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/cmd_vel_safe', self.on_command, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, 1)
        latest_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, '/safety/remembered_obstacles', self.on_memory,
                                 latest_sensor)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, latest_sensor)
        self.create_subscription(
            PolygonStamped, '/local_costmap/published_footprint', self.on_footprint,
            qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.step)

    def on_command(self, command):
        self.command, self.command_received = command, time.monotonic()
        if all(abs(x) <= 1.e-6 for x in (
                command.linear.x, command.linear.y, command.angular.z)) and any(
                abs(x) > 1.e-6 for x in self.last_sent_command):
            self.step(event_driven=True)

    def on_scan(self, scan):
        self.scan_received = time.monotonic()
        self.scan_stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1.e9
        observed = 0
        obstacle_points = []
        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                obstacle_points.append((distance * math.cos(angle), distance * math.sin(angle)))
            if abs(angle) <= 0.35 and (
                    math.isinf(distance) and distance > 0 or
                    math.isfinite(distance) and scan.range_min <= distance <= scan.range_max):
                observed += 1
        self.front_observed = observed >= 2 and scan.header.frame_id == 'base_link'
        # HAP's nonrepetitive scan can leave the narrow forward sector empty
        # for one frame. Keep the age of the last REAL forward observation;
        # sparse frames do not refresh it or extend the 0.35 s safety limit.
        self.forward_window.observe(
            observed, self.scan_stamp, self.scan_received,
            supported_frame=scan.header.frame_id == 'base_link')
        self.obstacle_points = obstacle_points if scan.header.frame_id == 'base_link' else []
        ego_fresh = (time.monotonic()-self.ego_received <= .5 and
                     abs(self.scan_stamp-self.ego_stamp) <= .5)
        observe_started = time.perf_counter()
        self.approach.observe(self.obstacle_points, self.scan_stamp,
                              self.ego_velocity if ego_fresh else (0., 0., 0.))
        self.approach_observe_ms = 1000.*(time.perf_counter()-observe_started)
        # Do not wait for the next velocity packet/timer to stop a new hazard.
        self.step(event_driven=True)

    def on_odom(self, message):
        velocity = message.twist.twist
        values = (velocity.linear.x, velocity.linear.y, velocity.angular.z)
        if message.child_frame_id == 'base_link' and all(math.isfinite(v) for v in values):
            self.ego_velocity = values
            self.ego_stamp = message.header.stamp.sec+message.header.stamp.nanosec/1.e9
            self.ego_received = time.monotonic()
            p,q = message.pose.pose.position,message.pose.pose.orientation
            if all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)) and (
                    q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w > .5):
                self.ego_pose = (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),
                                                  1-2*(q.y*q.y+q.z*q.z)))
                self.ego_frame = message.header.frame_id

    def on_memory(self,message):
        self.memory_received = time.monotonic()
        self.memory_stamp = message.header.stamp.sec+message.header.stamp.nanosec/1.e9
        self.memory_frame = message.header.frame_id
        fields = {f.name:f for f in message.fields}
        self.memory_supported = (self.memory_frame != 'unavailable' and message.height==1 and
                                 all(k in fields and fields[k].datatype==7 for k in ('x','y')))
        if self.memory_supported:
            try:
                dtype=np.dtype({'names':['x','y'],'formats':['>f4' if message.is_bigendian else '<f4']*2,
                                'offsets':[fields[k].offset for k in ('x','y')],
                                'itemsize':message.point_step})
                points=np.frombuffer(message.data,dtype=dtype,count=message.width)
                xy = np.column_stack((points['x'], points['y']))
                self.memory_xy = xy[np.isfinite(xy).all(axis=1)]
            except (ValueError,TypeError):
                self.memory_supported=False

    def on_footprint(self, message):
        self.footprint_received = time.monotonic()
        stamp = message.header.stamp
        self.footprint_stamp = stamp.sec + stamp.nanosec / 1.e9
        self.footprint_supported = (
            message.header.frame_id in ('base_link', 'odom_lio', 'odom') and
            len(message.polygon.points) >= 3 and all(
                math.isfinite(p.x) and math.isfinite(p.y) for p in message.polygon.points))
        if self.footprint_supported:
            points = message.polygon.points
            cx = sum(p.x for p in points) / len(points)
            cy = sum(p.y for p in points) / len(points)
            lengths = [math.hypot(points[(i+1) % len(points)].x-p.x,
                                  points[(i+1) % len(points)].y-p.y)
                       for i, p in enumerate(points)]
            # The Nav2 footprint is a centered rectangle, possibly expressed
            # in odom. Edge lengths are invariant to that frame's pose.
            if len(points) == 4 and abs(lengths[0]-lengths[2]) < .01 and (
                    abs(lengths[1]-lengths[3]) < .01):
                length, width = max(lengths)/2., min(lengths)/2.
            else:
                length = width = max(math.hypot(p.x-cx, p.y-cy) for p in points)
            self.footprint_half_length = max(self.turn_half_length, length)
            self.footprint_half_width = max(self.turn_half_width, width)

    def step(self, event_driven=False):
        steady = time.monotonic()
        current = self.get_clock().now().nanoseconds / 1.e9
        age = current - self.scan_stamp
        valid = (self.forward_window.fresh(current, steady, self.max_age) and
                 -0.1 <= age <= self.max_age and
                 steady - self.scan_received <= self.max_age)
        if valid != self.last_valid:
            self.status.publish(Bool(data=bool(valid)))
            self.last_valid = valid
        footprint_age = current - self.footprint_stamp
        footprint_valid = (
            self.footprint_supported and -0.1 <= footprint_age <= self.footprint_max_age and
            steady - self.footprint_received <= self.footprint_max_age)
        if footprint_valid != self.last_footprint_valid:
            self.footprint_status.publish(Bool(data=bool(footprint_valid)))
            self.last_footprint_valid = footprint_valid
        memory_valid = (not self.memory_required or (
            self.memory_supported and self.ego_pose is not None and self.memory_frame==self.ego_frame and
            steady-self.memory_received<=.75 and -.05<=current-self.memory_stamp<=.75 and
            steady-self.ego_received<=.5 and -.05<=current-self.ego_stamp<=.5))
        ready = valid and footprint_valid and memory_valid
        if ready != self.last_ready:
            self.get_logger().info('Motion gate %s (scan=%s, footprint=%s, memory=%s)' % (
                'ready' if ready else 'closed', bool(valid), bool(footprint_valid), bool(memory_valid)))
            self.last_ready = ready
            # Capture short freshness failures on the transition itself; a
            # half-second diagnostic timer can miss the entire stop pulse.
            self.last_diagnostic = -1.e30
        fresh_command = steady - self.command_received <= self.command_timeout
        finite_command = all(math.isfinite(x) for x in (
            self.command.linear.x, self.command.linear.y, self.command.angular.z))
        candidate = self.command if ready and fresh_command and finite_command else Twist()
        sparse_bounded = False
        if not self.front_observed:
            bounded = bound_sparse_command(
                (candidate.linear.x, candidate.linear.y, candidate.angular.z),
                self.last_observed_command)
            candidate = Twist()
            candidate.linear.x, candidate.linear.y, candidate.angular.z = bounded
            sparse_bounded = True
        requested = (candidate.linear.x, candidate.linear.y, candidate.angular.z)
        half_length = self.footprint_half_length+self.turn_margin
        half_width = self.footprint_half_width+self.turn_margin
        remembered = []
        if self.memory_supported and self.ego_pose is not None and self.memory_frame==self.ego_frame:
            px,py,yaw=self.ego_pose
            c,s=math.cos(yaw),math.sin(yaw)
            delta = self.memory_xy - (px, py)
            delta = delta[np.sum(delta*delta, axis=1) <= 2.25**2]
            dx,dy=delta[:,0],delta[:,1]
            remembered = np.column_stack((c*dx+s*dy, -s*dx+c*dy)).tolist()
        obstacles = self.obstacle_points+remembered
        caution = close_clearance_scale(obstacles, *requested, half_length, half_width)
        requested = tuple(caution*x for x in requested)
        braking = braking_speed_scale(
            obstacles, *requested, half_length, half_width, max(0., age))
        turn_clear = braking > 0.
        if turn_clear != self.last_turn_clear:
            self.turn_status.publish(Bool(data=turn_clear))
            if not turn_clear:
                self.get_logger().info('Turn blocked: observed obstacle in padded swept footprint')
            self.last_turn_clear = turn_clear
        hard_clear = stop_zone_clear(
            obstacles, self.stop_front, self.stop_rear, self.stop_width)
        permitted = ready and fresh_command and finite_command and turn_clear and hard_clear
        ratio = min(braking, self.approach.scale(
            requested[0], current, requested[1], requested[2], half_width)) if permitted else 0.
        memory_scale = 1.
        if remembered and math.hypot(requested[0],requested[1])>1.e-6:
            memory_pose_age=max(0.,age,current-self.ego_stamp)
            contact=arc_collision_time(remembered,*requested,half_length,half_width,
                                       memory_pose_age+.20+self.approach.horizon)
            if math.isfinite(contact):
                memory_scale=max(0.,min(1.,(contact-memory_pose_age-.20)/self.approach.horizon))
            ratio=min(ratio,memory_scale)
        limited = not permitted or ratio < 1. or caution < 1.
        safe = tuple(ratio*x for x in requested)
        output = Twist()
        recovered = self.recovery.apply(safe, steady, limited)
        # Axis-wise recovery changes curvature. Check that exact command, then
        # uniformly reduce the checked arc; never emit an unchecked transition.
        final_ratio = 1. if permitted else 0.
        if recovered != safe and any(abs(x) > 1.e-6 for x in recovered) and permitted:
            final_ratio = self.approach.braking_scale(
                *recovered,current,half_length,half_width)
            final_ratio = min(final_ratio,braking_speed_scale(
                obstacles,*recovered,half_length,half_width,max(0.,age)))
        checked = tuple(final_ratio*x for x in recovered)
        self.recovery.commit_checked(checked)
        output.linear.x, output.linear.y, output.angular.z = checked
        if (self.front_observed and valid) or all(abs(x) <= 1.e-6 for x in (
                output.linear.x, output.linear.y, output.angular.z)):
            self.last_observed_command = (
                output.linear.x, output.linear.y, output.angular.z)
        command = (output.linear.x, output.linear.y, output.angular.z)
        if steady-self.last_diagnostic >= .5:
            self.last_diagnostic = steady
            self.diagnostics.publish(String(data=json.dumps({
                'scan_valid': bool(valid), 'footprint_valid': bool(footprint_valid),
                'front_observed': bool(self.front_observed), 'sparse_bounded': sparse_bounded,
                'memory_valid': bool(memory_valid), 'remembered_points': len(remembered),
                'memory_age': round(current-self.memory_stamp, 3),
                'memory_receive_age': round(steady-self.memory_received, 3),
                'odom_age': round(current-self.ego_stamp, 3),
                'odom_receive_age': round(steady-self.ego_received, 3),
                'footprint_age': round(footprint_age, 3),
                'footprint_receive_age': round(steady-self.footprint_received, 3),
                'command_receive_age': round(steady-self.command_received, 3),
                'memory_approach_scale': round(memory_scale,3),
                'command_fresh': bool(fresh_command), 'hard_zone_clear': bool(hard_clear),
                'turn_clear': bool(turn_clear), 'scan_age': round(age, 3),
                'caution_scale': round(caution, 3), 'approach_braking_scale': round(ratio, 3),
                'recovery_check_scale': round(final_ratio, 3),
                'external_closing_speed': round(self.approach.closing, 3),
                'approach_observe_ms': round(self.approach_observe_ms, 3),
                'input': [self.command.linear.x, self.command.angular.z],
                'output': [output.linear.x, output.angular.z]})))
        # Normal commands stay on the 50 Hz clock. Publish additional packets
        # only for a safety reduction, rather than duplicating every scan and
        # incoming command onto the reliable Unitree RPC queue.
        reduction = any(abs(new) < abs(old)-1.e-6
                        for new, old in zip(command, self.last_sent_command))
        if not event_driven or reduction:
            self.output.publish(output)
            self.last_sent_command = command


def main():
    acquire_process_lock('scan_gate')
    rclpy.init()
    node = ScanSafetyGate()
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
