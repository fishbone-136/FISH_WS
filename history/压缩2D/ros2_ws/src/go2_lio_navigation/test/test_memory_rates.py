"""Changing timers must retain memory; invalid batches change nothing."""
import os
from pathlib import Path
import sys
import unittest

if os.environ.get('ROS_DOMAIN_ID')!='90' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=90 and ROS_LOCALHOST_ONLY=1')
os.environ['CYCLONEDDS_URI']=(
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
    '</Peers></Discovery></Domain></CycloneDDS>')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from rclpy.parameter import Parameter
from obstacle_memory import RememberedObstacles


class MemoryRateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init();cls.node=RememberedObstacles()

    @classmethod
    def tearDownClass(cls):
        cls.node.worker.shutdown(wait=True,cancel_futures=True)
        cls.node.destroy_node();rclpy.shutdown()

    def test_timer_changes_retain_obstacle_memory(self):
        node=self.node
        node.memory.mark_xyz([[1.,0.,.2]],(0.,0.,0.),1.)
        before=node.memory.cells.copy()
        result=node.set_parameters_atomically([
            Parameter('memory_publish_rate',value=20.),
            Parameter('navigation_publish_rate',value=12.),Parameter('clearing_rate',value=8.)])
        self.assertTrue(result.successful)
        self.assertEqual(node.memory_timer.timer_period_ns,50000000)
        self.assertEqual(node.maps_timer.timer_period_ns,int(1.e9/12.))
        self.assertEqual(node.clearing_rate,8.)
        self.assertEqual(node.memory.cells,before)

    def test_invalid_batch_changes_neither_timers_nor_rates(self):
        node=self.node;before=(node.memory_timer.timer_period_ns,node.navigation_publish_rate)
        for invalid in (0.,float('nan'),100.):
            result=node.set_parameters_atomically([
                Parameter('memory_publish_rate',value=25.),
                Parameter('navigation_publish_rate',value=invalid)])
            self.assertFalse(result.successful)
            self.assertEqual((node.memory_timer.timer_period_ns,node.navigation_publish_rate),before)


if __name__=='__main__':unittest.main()
