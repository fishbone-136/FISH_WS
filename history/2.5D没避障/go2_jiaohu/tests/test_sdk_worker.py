from voice import config
from voice.sdk_worker import (
    _execute_instant,
    _motion_spec,
    _result_code,
    _stop_active_motion,
)


class _FakeSportClient:
    def __init__(self, code=0):
        self.code = code
        self.called = []

    def StopMove(self):
        self.called.append("stop")
        return self.code

    def Move(self, vx, vy, vyaw):
        self.called.append((vx, vy, vyaw))
        return self.code

    def StandDown(self):
        self.called.append("sit")
        return self.code

    def StandUp(self):
        self.called.append("stand")
        return self.code

    def RecoveryStand(self):
        self.called.append("recovery")
        return self.code


def test_sdk_motion_mapping_keeps_validated_duration_unchanged():
    motion, duration = _motion_spec("turn_left", 0.5, 4.2, 90.0)
    assert motion == (0.0, 0.0, 0.5)
    assert duration == 4.2


def test_sdk_direction_signs_match_unitree_move_api():
    assert _motion_spec("forward", 0.3, 1.0, 0.0)[0] == (0.3, 0.0, 0.0)
    assert _motion_spec("backward", 0.3, 1.0, 0.0)[0] == (-0.3, 0.0, 0.0)
    assert _motion_spec("right", 0.3, 1.0, 0.0)[0] == (0.0, -0.3, 0.0)
    assert _motion_spec("turn_right", 0.5, 1.0, 0.0)[0] == (0.0, 0.0, -0.5)


def test_instant_action_propagates_sdk_return_code():
    client = _FakeSportClient(code=3104)
    assert _execute_instant(client, "stand") == 3104
    assert client.called == ["stand"]
    assert _result_code(0) == 0


def test_stopmove_is_attempted_even_if_zero_velocity_frames_fail(monkeypatch):
    monkeypatch.setattr(config, "ROBOT_SDK_MOVE_INTERVAL", 0.0)
    client = _FakeSportClient(code=3104)

    assert _stop_active_motion(client) == 3104
    assert client.called[-1] == "stop"
