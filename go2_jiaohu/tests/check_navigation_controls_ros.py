#!/usr/bin/python3
"""Real UI bridge lifecycle regression, isolated localhost; no robot goals."""
import os
if os.environ.get('ROS_DOMAIN_ID') != '94' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires ROS_DOMAIN_ID=94 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
import json
from pathlib import Path
import time
import rclpy
from PyQt5.QtWidgets import QApplication
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from go2_ui.navigation.ros_bridge import RosNavigationWorker


def main():
    app = QApplication([])
    rclpy.init()
    node = rclpy.create_node('navigation_controls_fixture')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps = node.create_publisher(OccupancyGrid, '/safety/navigation_global', qos)
    health = node.create_publisher(Bool, '/lio/localization_valid', qos)
    faults = node.create_publisher(String, '/safety/navigation_fault', qos)
    tf = TransformBroadcaster(node)
    state = {'id': 3, 'healthy': True, 'fresh_tf': True, 'requests': [], 'cancel': 0, 'zeros': 0}

    def manage(request, response):
        state['requests'].append(request.command)
        state['id'] = 2 if request.command == request.PAUSE else 3
        response.success = True
        return response

    def lifecycle(request, response):
        response.current_state.id = state['id']
        return response

    def cancel(request, response):
        state['cancel'] += 1
        response.return_code = response.ERROR_NONE
        return response

    def zero(message):
        assert message.linear.x == message.linear.y == message.angular.z == 0.
        state['zeros'] += 1

    node.create_service(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', manage)
    node.create_service(GetState, '/bt_navigator/get_state', lifecycle)
    node.create_service(CancelGoal, '/navigate_to_pose/_action/cancel_goal', cancel)
    node.create_subscription(Twist, '/cmd_vel', zero, 10)
    def execute(_):
        raise AssertionError('The lifecycle regression must never send a navigation goal')
    server = ActionServer(node, NavigateToPose, '/navigate_to_pose', execute)
    grid = OccupancyGrid()
    grid.header.frame_id = 'odom_lio'
    grid.info.width = grid.info.height = 10
    grid.info.resolution = .1
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * 100

    def publish():
        grid.header.stamp = node.get_clock().now().to_msg()
        maps.publish(grid)
        health.publish(Bool(data=state['healthy']))
        transform = TransformStamped()
        stamp = node.get_clock().now()
        if not state['fresh_tf']:
            stamp -= rclpy.duration.Duration(seconds=8.)
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = 'odom_lio'
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.
        tf.sendTransform(transform)
    node.create_timer(.05, publish)
    worker = RosNavigationWorker()
    statuses, results = [], []
    worker.status_updated.connect(lambda s, m: statuses.append((s, m)))
    worker.control_finished.connect(lambda *args: results.append(args))
    worker.start()

    def until(condition, timeout=7.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            app.processEvents()
            if condition():
                return
        raise AssertionError({'statuses': statuses[-6:], 'results': results, 'state': state})

    try:
        until(lambda: statuses and statuses[-1][0] == 'ready')
        worker.enqueue('pause')
        until(lambda: results and results[-1][:2] == ('pause', True))
        until(lambda: statuses[-1][0] == 'paused')
        assert state['cancel'] > 0 and state['zeros'] > 0
        worker.enqueue('resume')
        until(lambda: results[-1][:2] == ('resume', True))
        until(lambda: statuses[-1][0] == 'ready')
        # Real transforms stop arriving while the TF cache remains populated.
        state['fresh_tf'] = False
        until(lambda: statuses[-1][0] == 'waiting', timeout=7.)
        state['healthy'] = False
        faults.publish(String(data=json.dumps({'message': 'fixture localization stopped'})))
        until(lambda: statuses[-1][0] == 'fault')
        before = len(state['requests'])
        worker.enqueue('resume')
        until(lambda: results[-1][:2] == ('resume', False))
        assert len(state['requests']) == before, 'Faulted localization was resumed'
        report = {'pause_resume_acknowledged': True, 'cancel_and_zero_on_stop': True,
                  'stale_cached_tf_rejected': True, 'fault_resume_rejected': True,
                  'lifecycle_requests': state['requests'], 'isolated_domain': 94}
        output = Path(os.environ['GO2_HUMBLE_PROJECT_ROOT']) / 'reports/ui_navigation_controls_ros_20261110.json'
        output.write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
    finally:
        worker.stop()
        assert worker.wait(5000), 'UI worker did not stop'
        server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
