from dataclasses import replace
import time

import numpy as np

from tests.test_visual_navigation_ui import fixture
from voice.visual_navigation import ApproachIntent, GroundedTarget


def test_cold_start_identifies_target_in_parallel_then_remeasures_after_nav_ready(fixture):
    n=fixture
    n.page._navigation_state='waiting'
    n.session.request(ApproachIntent('白色文件柜'))
    assert n.locate.call_count==1
    assert not n.worker.commands
    intent,ticket=n.locate.call_args.args[1:]
    original=GroundedTarget(ticket,intent,n.snapshot,(.2,.2,.8,.8),(0.,0.,3.))
    n.session.target_ready(original)
    assert not n.worker.commands  # Recognition alone never authorizes motion.
    fresh=replace(n.snapshot,depth_mm=np.full((100,100),3050,np.uint16),
        captured_at=time.time(),captured_monotonic=time.monotonic(),deproject=lambda u,v,z:(0.,0.,z))
    n.session._snapshot_provider=lambda:fresh
    n.page._navigation_state='ready'
    n.session._poll()
    n.worker.visual_goal_updated.emit(ticket,'prepared','ready')
    name,target=n.worker.commands[-1]
    assert name=='visual_goal'
    assert target.snapshot is fresh and target.optical_point==(0.,0.,3.05)
    assert n.locate.call_count==1


def test_scene_change_restarts_grounding_on_fresh_frame_and_ignores_old_callback(fixture):
    n=fixture
    n.page._navigation_state='waiting'
    n.session.request(ApproachIntent('文件柜'))
    intent,ticket=n.locate.call_args.args[1:]
    n.session.target_ready(GroundedTarget(ticket,intent,n.snapshot,(.2,.2,.8,.8),(0.,0.,3.)))
    changed=replace(n.snapshot,color=np.full((100,100,3),200,np.uint8),captured_at=time.time(),captured_monotonic=time.monotonic())
    n.session._snapshot_provider=lambda:changed
    n.page._navigation_state='ready'
    n.session._poll()
    n.worker.visual_goal_updated.emit(ticket,'prepared','ready')
    assert n.locate.call_count==2
    refreshed_ticket=n.locate.call_args.args[2]
    assert refreshed_ticket != ticket
    n.session.grounding_failed(ticket,'late error')
    assert n.session.active
    n.session.target_ready(GroundedTarget(refreshed_ticket,intent,changed,(.2,.2,.8,.8),(0.,0.,3.)))
    assert n.worker.commands[-1][0]=='visual_goal'
    assert n.worker.commands[-1][1].request_id==ticket


def test_cancel_while_navigation_initializes_invalidates_parallel_model_result(fixture):
    n=fixture
    n.page._navigation_state='waiting'
    n.session.request(ApproachIntent('文件柜'))
    intent,ticket=n.locate.call_args.args[1:]
    n.session.cancel()
    n.session.target_ready(GroundedTarget(ticket,intent,n.snapshot,(.2,.2,.8,.8),(0.,0.,3.)))
    n.page._navigation_state='ready'
    n.session._poll()
    assert not n.worker.commands
