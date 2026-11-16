"""Worker signal handling and crash recovery, with a real spawned process."""
from concurrent.futures import Future
from concurrent.futures.process import BrokenProcessPool
import os
from pathlib import Path
import signal
import sys
import time
import unittest
from unittest.mock import Mock

if os.environ.get('ROS_DOMAIN_ID')!='90' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=90 and ROS_LOCALHOST_ONLY=1')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from obstacle_memory import RememberedObstacles


class WorkerTests(unittest.TestCase):
    def test_ctrl_c_is_owned_by_parent_and_worker_crash_withdraws_evidence(self):
        rclpy.init();node=RememberedObstacles()
        try:
            pid=node.worker.submit(os.getpid).result(timeout=8.)
            os.kill(pid,signal.SIGINT)
            self.assertEqual(node.worker.submit(os.getpid).result(timeout=3.),pid)
            node.memory.mark_xyz([[1.,0.,.3]],(0.,0.,0.),1.)
            original=node.memory.cells.copy()
            os.kill(pid,signal.SIGKILL)
            try:failed=node.worker.submit(os.getpid)
            except BrokenProcessPool as exc:
                failed=Future();failed.set_exception(exc)
            deadline=time.monotonic()+3.
            while not failed.done() and time.monotonic()<deadline:time.sleep(.01)
            self.assertTrue(failed.done())
            node.localized=True
            node.clearing=failed
            node.clearing_context=(original,1.)
            node.clearing_keys=list(original)
            node.points_pub=Mock()
            node.update_clearing()
            self.assertIn('clearing worker stopped',node.fault)
            self.assertEqual(node.memory.cells,original)
            msg=node.points_pub.publish.call_args.args[0]
            self.assertEqual(msg.header.frame_id,'unavailable')
            self.assertEqual(msg.width,0)
        finally:
            node.worker.shutdown(wait=True,cancel_futures=True)
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
