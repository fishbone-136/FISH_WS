#!/usr/bin/env python3
"""Settle a completed navigation session once; never send a walking command."""
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState

from go2_navigation.runtime_guard import acquire_process_lock
from idle_stand import IdleStand


class IdleStandGuard(Node):
    def __init__(self):
        super().__init__('idle_stand_guard')
        acquire_process_lock('idle_stand')
        self.settle = IdleStand()
        self.statuses = {}
        self.command_time = self.sport_time = self.move_time = -math.inf
        self.zero = self.stationary = False
        self.relative_active, self.relative_seen_at = False, -math.inf
        self.pub = self.create_publisher(Request, '/api/sport/request', 1)
        self.diagnostic = self.create_publisher(String, '/go2/idle_stand_status',
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for topic in ('/navigate_to_pose/_action/status', '/navigate_through_poses/_action/status',
                      '/spin/_action/status', '/drive_on_heading/_action/status', '/backup/_action/status'):
            self.create_subscription(GoalStatusArray, topic,
                lambda m, key=topic: self.on_status(key, m),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/cmd_vel_guarded', self.on_velocity, qos_profile_sensor_data)
        self.create_subscription(SportModeState, '/lf/sportmodestate', self.on_sport, qos_profile_sensor_data)
        self.create_subscription(Response, '/api/sport/response', self.on_response, qos_profile_sensor_data)
        self.create_subscription(Request, '/api/sport/request', self.on_request, qos_profile_sensor_data)
        self.create_subscription(String, '/go2/interactive_command',
                                 lambda _: self.settle.suspend(), 10)
        self.create_subscription(Bool, '/go2/relative_navigation_active', self.on_relative, qos_profile_sensor_data)
        self.create_timer(.05, self.tick)
        self.last_diagnostic = None
        self.get_logger().info('Idle standing guard ready; waits for an observed navigation action to finish')

    def on_status(self, key, msg):
        self.statuses[key] = any(s.status in (1, 2, 3) for s in msg.status_list)
        now = time.monotonic()
        self.settle.navigation(self.navigation_active(now), now)

    def on_relative(self, msg):
        self.relative_active, self.relative_seen_at = msg.data, time.monotonic()
        self.settle.navigation(self.navigation_active(self.relative_seen_at), self.relative_seen_at)

    def navigation_active(self, now):
        return any(self.statuses.values()) or (self.relative_active and now-self.relative_seen_at < .6)

    def on_velocity(self, msg):
        self.command_time = time.monotonic()
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        # Only actual zero, not a physical gait deadband, permits standing.
        self.zero = all(math.isfinite(v) and abs(v) <= 1.e-6 for v in values)
        self.settle.velocity(self.zero, self.command_time)

    def on_sport(self, msg):
        self.sport_time = time.monotonic()
        vx, vy, wz = msg.velocity[0], msg.velocity[1], msg.yaw_speed
        self.stationary = (all(math.isfinite(v) for v in (vx, vy, wz)) and
                           math.hypot(vx, vy) < .04 and abs(wz) < .10)

    def on_request(self, msg):
        if msg.header.identity.api_id == 1008:
            self.move_time = time.monotonic()
            self.settle.velocity(False, self.move_time)

    def on_response(self, msg):
        if self.settle.response(msg.header.identity.id, msg.header.identity.api_id,
                                msg.header.status.code, time.monotonic()):
            self.get_logger().info(f'Idle settle API {msg.header.identity.api_id}: code={msg.header.status.code}')

    def tick(self):
        now = time.monotonic()
        self.settle.navigation(self.navigation_active(now), now)
        valid = (self.zero and now-self.command_time < .20 and
                 self.stationary and now-self.sport_time < .50 and
                 now-self.move_time >= .80 and self.pub.get_subscription_count() > 0)
        identity = time.time_ns()//1000
        api = self.settle.tick(now, valid, identity)
        if api is not None:
            req = Request()
            req.header.identity.id, req.header.identity.api_id = identity, api
            req.header.policy.noreply = False
            req.parameter = '{}'
            self.pub.publish(req)
        status = json.dumps({'phase':self.settle.phase, 'active_goal':self.settle.active,
                             'armed':self.settle.armed, 'error':self.settle.error})
        if status != self.last_diagnostic:
            self.diagnostic.publish(String(data=status))
            self.last_diagnostic = status
            if self.settle.error:
                self.get_logger().error(self.settle.error)


def main():
    rclpy.init()
    node = IdleStandGuard()
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
