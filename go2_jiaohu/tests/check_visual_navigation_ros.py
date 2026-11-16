#!/usr/bin/python3
"""Isolated localhost Nav2 fixture: real goal acceptance/cancel, no robot I/O."""
import os
if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires ROS_DOMAIN_ID=96 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import json
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from PyQt5.QtWidgets import QApplication
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from go2_ui.navigation.ros_bridge import RosNavigationWorker
from voice import config
from voice.visual_navigation import ApproachIntent, GroundedTarget, RgbdSnapshot


def main():
    measured_calibration = os.environ.get('GO2_TEST_CAMERA_CALIBRATION') == 'lidar'
    fresh_capture = os.environ.get('GO2_TEST_FRESH_CAPTURE') == '1'
    app = QApplication([])
    rclpy.init()
    node = rclpy.create_node('visual_navigation_nav2_fixture')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    group = ReentrantCallbackGroup()
    qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    maps = node.create_publisher(OccupancyGrid,'/safety/navigation_global',qos)
    health = node.create_publisher(Bool,'/lio/localization_valid',qos)
    dynamic_tf, static_tf = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    requests, canceled = [], []
    def execute(handle):
        requests.append((handle.request.pose.pose.position.x,handle.request.pose.pose.position.y))
        deadline = time.monotonic()+8.
        while time.monotonic()<deadline and rclpy.ok():
            if handle.is_cancel_requested:
                canceled.append(True)
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(.02)
        handle.succeed()
        return NavigateToPose.Result()
    server = ActionServer(node,NavigateToPose,'navigate_to_pose',execute,
        cancel_callback=lambda request:CancelResponse.ACCEPT,callback_group=group)
    def lifecycle(request,response):
        response.current_state.id=3
        return response
    node.create_service(GetState,'/bt_navigator/get_state',lifecycle,callback_group=group)
    camera=TransformStamped()
    camera.header.frame_id='base_link'
    camera.child_frame_id='livox_frame' if measured_calibration else config.VISUAL_CAMERA_FRAME
    camera.header.stamp=node.get_clock().now().to_msg()
    camera.transform.translation.x=.14 if measured_calibration else .2
    camera.transform.translation.z=.13 if measured_calibration else .3
    camera.transform.rotation.x=0. if measured_calibration else -.5
    camera.transform.rotation.y=0. if measured_calibration else .5
    camera.transform.rotation.z=0. if measured_calibration else -.5
    camera.transform.rotation.w=1. if measured_calibration else .5
    static_tf.sendTransform(camera)
    grid=OccupancyGrid()
    grid.header.frame_id='odom_lio'
    grid.info.width=grid.info.height=200
    grid.info.resolution=.1
    grid.info.origin.position.x=grid.info.origin.position.y=-10.
    grid.info.origin.orientation.w=1.
    grid.data=[0]*40000
    def publish():
        grid.header.stamp=node.get_clock().now().to_msg()
        maps.publish(grid)
        health.publish(Bool(data=True))
        pose=TransformStamped()
        pose.header.frame_id='odom_lio'
        pose.child_frame_id='base_link'
        pose.header.stamp=node.get_clock().now().to_msg()
        pose.transform.rotation.w=1.
        dynamic_tf.sendTransform(pose)
    node.create_timer(.05,publish)
    thread=threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    worker=RosNavigationWorker()
    statuses,updates=[],[]
    worker.status_updated.connect(lambda *args:statuses.append(args))
    worker.visual_goal_updated.connect(lambda *args:updates.append(args))
    worker.start()
    def until(predicate,timeout=8.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            app.processEvents()
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError({'statuses':statuses[-4:],'updates':updates[-6:],'requests':requests})
    try:
        until(lambda:statuses and statuses[-1][0]=='ready')
        worker.enqueue('visual_preflight','fixture-visual')
        until(lambda:any(item[:2]==('fixture-visual','prepared') for item in updates))
        snapshot=RgbdSnapshot(np.zeros((60,80,3),np.uint8),np.full((60,80),3000,np.uint16),
            time.time()-(0. if fresh_capture else .15),time.monotonic()-(0. if fresh_capture else .15),lambda *args:(0.,0.,3.))
        target=GroundedTarget('fixture-visual',ApproachIntent('黑色椅子',1.),snapshot,(.1,.1,.9,.9),(0.,0.,3.))
        worker.enqueue('visual_goal',target)
        until(lambda:any(item[:2]==('fixture-visual','accepted') for item in updates))
        until(lambda:bool(requests))
        expected=(2.1565497759,.0178366313) if measured_calibration else (2.2,0.)
        assert all(abs(actual-want)<.001 for actual,want in zip(requests[0],expected)), requests
        worker.enqueue('cancel_visual','fixture-visual')
        until(lambda:any(item[:2]==('fixture-visual','finished') and '取消' in item[2] for item in updates))
        shutdown_target=GroundedTarget('fixture-shutdown',ApproachIntent('黑色椅子',1.),snapshot,(.1,.1,.9,.9),(0.,0.,3.))
        worker.enqueue('visual_goal',shutdown_target)
        until(lambda:any(item[:2]==('fixture-shutdown','accepted') for item in updates))
        worker.stop()
        until(lambda:not worker.isRunning())
        assert len(canceled)==2,canceled
        report={'isolated_domain':96,'localhost_only':True,'goal_xy':requests[0],
            'static_optical_tf_used':not measured_calibration,
            'measured_optical_to_lidar_used':measured_calibration,
            'fresh_capture_waits_for_tf':fresh_capture,
            'nav2_accepted':True,'owned_goal_canceled':True,
            'owned_goal_canceled_on_shutdown':True,
            'updates':updates}
        name='visual_camera_calibration_ros_20261111.json' if measured_calibration else 'visual_navigation_ros_20261111.json'
        if fresh_capture:
            name='visual_parallel_capture_ros_20261111.json'
        path=Path(os.environ['GO2_HUMBLE_PROJECT_ROOT'])/'reports'/name
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False),flush=True)
    finally:
        worker.stop()
        assert worker.wait(5000)
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
        server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
