#!/usr/bin/python3
"""Latch a localization stop, pause Nav2, and retain a visible diagnostic."""
import json
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,DurabilityPolicy
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ManageLifecycleNodes
from std_msgs.msg import Bool,String
from visualization_msgs.msg import Marker


class RetriedRequest:
    """One bounded in-flight RPC; retry errors, rejections and lost replies."""
    def __init__(self, client, request, accepted, timeout=3., retry_delay=1.):
        self.client,self.request,self.accepted=client,request,accepted
        self.timeout,self.retry_delay=timeout,retry_delay
        self.future=None
        self.deadline=self.next_attempt=0.
        self.acknowledged=False

    def tick(self, now):
        if self.acknowledged:return
        if self.future is not None:
            if self.future.done():
                try:self.acknowledged=bool(self.accepted(self.future.result()))
                except Exception:self.acknowledged=False
                self.future=None
                self.next_attempt=now+self.retry_delay
            elif now>=self.deadline:
                self.future.cancel()
                self.future=None
                self.next_attempt=now+self.retry_delay
            else:return
        if not self.acknowledged and now>=self.next_attempt and self.client.service_is_ready():
            try:
                self.future=self.client.call_async(self.request)
                self.deadline=now+self.timeout
            except Exception:
                self.next_attempt=now+self.retry_delay


def main():
    rclpy.init();node=rclpy.create_node('navigation_fault_guard')
    code=node.declare_parameter('lio_exit_code',0).value
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    valid=node.create_publisher(Bool,'/lio/localization_valid',latched)
    status=node.create_publisher(String,'/safety/navigation_fault',latched)
    marker=node.create_publisher(Marker,'/safety/navigation_fault_marker',latched)
    cancel=node.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
    pause=node.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
    pause_request=ManageLifecycleNodes.Request();pause_request.command=ManageLifecycleNodes.Request.PAUSE
    cancel_request=RetriedRequest(cancel,CancelGoal.Request(),
        lambda reply:reply.return_code==CancelGoal.Response.ERROR_NONE)
    pause_request=RetriedRequest(pause,pause_request,lambda reply:reply.success)
    state={'warning':False}
    began=time.monotonic()
    def tick():
        steady=time.monotonic()
        cancel_request.tick(steady);pause_request.tick(steady)
        valid.publish(Bool(data=False))
        status.publish(String(data=json.dumps({'state':'localization_stopped',
            'navigation_paused':pause_request.acknowledged,
            'navigation_cancelled':cancel_request.acknowledged,'lio_exit_code':code,
            'message':f'FAST-LIO 定位已停止（退出码 {code}），导航已停车；查看定位日志确认原因，排除故障后重新启动。'})))
        m=Marker();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg()
        m.ns='navigation_fault';m.id=1;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
        m.pose.position.z=.5;m.pose.orientation.w=1.;m.scale.z=.22
        m.color.r=1.;m.color.g=.15;m.color.b=.1;m.color.a=1.
        m.text='LOCALIZATION STOPPED\nNavigation paused - check FAST-LIO error log'
        marker.publish(m)
        if steady-began>8. and not pause_request.acknowledged and not state['warning']:
            node.get_logger().error('Nav2 pause not acknowledged; localization motion gate remains closed')
            state['warning']=True
    node.get_logger().error('FAST-LIO exited: motion is latched off; RViz and diagnostics remain open. '
                            'Check the FAST-LIO error log, then restart the navigation launch.')
    tick();node.create_timer(.5,tick)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
