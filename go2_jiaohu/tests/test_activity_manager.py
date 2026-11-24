from go2_ui.activity_manager import ExclusiveActivityManager
import pytest


def test_switch_stops_previous_before_starting_next():
    events = []
    manager = ExclusiveActivityManager()
    manager.register(
        "voice",
        lambda: events.append("start voice"),
        lambda: events.append("stop voice"),
    )
    manager.register(
        "gesture",
        lambda: events.append("start gesture"),
        lambda: events.append("stop gesture"),
    )

    assert manager.activate("voice") is True
    assert manager.activate("gesture") is True
    assert manager.active == "gesture"
    assert events == ["start voice", "stop voice", "start gesture"]


def test_repeated_activation_does_not_start_duplicate():
    starts = []
    manager = ExclusiveActivityManager()
    manager.register("voice", lambda: starts.append("voice"), lambda: None)

    assert manager.activate("voice") is True
    assert manager.activate("voice") is False
    assert starts == ["voice"]


def test_failed_start_leaves_everything_inactive():
    events = []
    manager = ExclusiveActivityManager()
    manager.register("voice", lambda: True, lambda: events.append("stop voice"))
    manager.register("vision", lambda: False, lambda: events.append("stop vision"))

    assert manager.activate("voice") is True
    assert manager.activate("vision") is False
    assert manager.active is None
    assert events == ["stop voice", "stop vision"]


def test_start_exception_cleans_up_and_allows_next_activation():
    events = []
    manager = ExclusiveActivityManager()

    def fail():
        events.append("acquire camera")
        raise RuntimeError("camera unavailable")

    manager.register("vision", fail, lambda: events.append("release camera"))
    manager.register("voice", lambda: events.append("start voice"), lambda: None)

    with pytest.raises(RuntimeError, match="camera unavailable"):
        manager.activate("vision")

    assert manager.active is None
    assert events == ["acquire camera", "release camera"]
    assert manager.activate("voice") is True


def test_async_stop_blocks_new_activity_until_resources_are_released():
    events,done = [],[False]
    manager = ExclusiveActivityManager()
    manager.register('follow',lambda:True,lambda:done[0])
    manager.register('voice',lambda:events.append('start voice'),lambda:None)
    assert manager.activate('follow')
    assert manager.activate('voice') is False
    assert manager.active == 'follow'
    assert events == []
    done[0] = True
    assert manager.activate('voice')
    assert events == ['start voice']
