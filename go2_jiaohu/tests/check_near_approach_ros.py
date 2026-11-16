"""Localhost-only ROS state -> depth-feedback controller -> fake SDK integration."""
import os
if os.environ.get('ROS_DOMAIN_ID') != '97' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires ROS_DOMAIN_ID=97 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
import json
from pathlib import Path
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import SportModeState

from deploy.near_approach_controller import NearApproachController
from deploy.robot_state_monitor import RobotStateMonitor
from tests.test_near_approach import snapshot, BOX
from voice.visual_navigation import ApproachIntent


class FakeSDK:
    last_error = ''
    def __init__(self):
        self.distance = 2.
        self.velocities = []
        self.stopped = self.released = False
    def reconnect(self):
        return True
    def send_visual_velocity(self, vx, wz):
        self.velocities.append((vx,wz))
        if vx:
            self.distance = max(.87,self.distance-.06)
        return True
    def execute_action(self,*args,**kwargs):
        self.stopped = True
        return True
    def shutdown(self):
        self.released = True


def main():
    rclpy.init()
    node=rclpy.create_node('near_approach_fake_robot_state')
    pub=node.create_publisher(SportModeState,'/lf/sportmodestate',qos_profile_sensor_data)
    feedback=SportModeState()
    feedback.body_height=.25
    feedback.imu_state.rpy=[0.,0.,0.]
    feedback.position=[0.,0.,0.]
    feedback.velocity=[0.,0.,0.]
    timer=node.create_timer(.02,lambda:pub.publish(feedback))
    thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True)
    thread.start()
    robot=FakeSDK()
    statuses=[]
    people=type('Detector',(),{'detect':lambda self,image:[BOX]})()
    controller=NearApproachController(lambda:snapshot(robot.distance),lambda *args:statuses.append(args),
        robot=robot,people=people,state_factory=RobotStateMonitor,guard=lambda:True)
    try:
        controller._run(ApproachIntent('人'),'isolated-ros',True)
        assert statuses[-1][1]=='succeeded',statuses
        assert robot.stopped and robot.released
        assert robot.velocities[-1]==(0.,0.)
        report={'isolated_domain':97,'localhost_only':True,'real_ros_state_subscription':True,
            'fake_rgbd':True,'fake_sdk':True,'feedback_approach_succeeded':True,
            'stopped_and_released':robot.stopped and robot.released,
            'velocity_commands':robot.velocities,'statuses':statuses,
            'physical_robot_commands_sent':False}
        path=Path(os.environ['GO2_HUMBLE_PROJECT_ROOT'])/'reports/visual_approach_ros_20261111.json'
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False),flush=True)
    finally:
        node.destroy_timer(timer)
        rclpy.shutdown()
        thread.join(timeout=2.)
        node.destroy_node()


if __name__=='__main__':
    main()
