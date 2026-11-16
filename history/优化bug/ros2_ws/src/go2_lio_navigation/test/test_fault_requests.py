"""A fault handler must retry rejected/lost RPCs without flooding the server."""
from concurrent.futures import Future
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_fault_guard import RetriedRequest


class Client:
    def __init__(self):
        self.ready=False
        self.requests=[]

    def service_is_ready(self):return self.ready

    def call_async(self, request):
        future=Future();self.requests.append(future);return future


class FaultRequestTests(unittest.TestCase):
    def test_late_service_rejection_then_success(self):
        client=Client();request=RetriedRequest(client,object(),bool)
        request.tick(0.)
        self.assertEqual(client.requests,[])
        client.ready=True;request.tick(.5)
        client.requests[-1].set_result(False);request.tick(1.)
        self.assertFalse(request.acknowledged)
        request.tick(1.5);self.assertEqual(len(client.requests),1)
        request.tick(2.);self.assertEqual(len(client.requests),2)
        client.requests[-1].set_result(True);request.tick(2.5)
        self.assertTrue(request.acknowledged)
        request.tick(20.);self.assertEqual(len(client.requests),2)

    def test_unanswered_request_does_not_block_retries(self):
        client=Client();client.ready=True
        request=RetriedRequest(client,object(),bool)
        request.tick(0.)
        request.tick(1.);self.assertEqual(len(client.requests),1)
        request.tick(3.);self.assertTrue(client.requests[0].cancelled())
        request.tick(4.);self.assertEqual(len(client.requests),2)
        client.requests[-1].set_exception(RuntimeError('server restarted'))
        request.tick(4.5);self.assertFalse(request.acknowledged)
        request.tick(5.5);self.assertEqual(len(client.requests),3)


if __name__=='__main__':unittest.main()
