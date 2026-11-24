"""Exclusive lifecycle coordinator for UI functions.

The dashboard has several entry points that own the same microphone, camera,
robot motion bridge, or ROS graph.  Keeping the exclusivity rule here avoids
each button implementing a slightly different version of "stop old, start
new".
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


StartCallback = Callable[[], bool | None]
StopCallback = Callable[[], bool | None]


@dataclass(frozen=True)
class _Activity:
    start: StartCallback
    stop: StopCallback


class ExclusiveActivityManager:
    """Run at most one registered activity at a time.

    ``start`` may return ``False`` when a feature cannot start.  In that case
    no activity is marked active.  Selecting the already-active activity is a
    no-op, which prevents duplicate threads/processes.
    """

    def __init__(self) -> None:
        self._activities: dict[str, _Activity] = {}
        self._active: str | None = None
        self._switching = False

    @property
    def active(self) -> str | None:
        return self._active

    def register(
        self,
        name: str,
        start: StartCallback,
        stop: StopCallback,
    ) -> None:
        if not name:
            raise ValueError("activity name cannot be empty")
        if name in self._activities:
            raise ValueError(f"activity already registered: {name}")
        self._activities[name] = _Activity(start=start, stop=stop)

    def activate(self, name: str) -> bool:
        if name not in self._activities:
            raise KeyError(name)
        if self._active == name or self._switching:
            return False

        self._switching = True
        try:
            if self.deactivate() is False:
                return False
            try:
                started = self._activities[name].start()
            except Exception:
                # Release partially acquired devices even when startup raises.
                self._activities[name].stop()
                raise
            if started is False:
                # A start callback may have acquired only part of its resources
                # before discovering an unavailable device.  Always run its
                # matching cleanup hook on failure.
                self._activities[name].stop()
                return False
            self._active = name
            return True
        finally:
            self._switching = False

    def toggle(self, name: str) -> bool:
        if self._active == name:
            self.deactivate()
            return False
        return self.activate(name)

    def deactivate(self) -> bool:
        active = self._active
        if active is None:
            return True
        # Clear first so callbacks cannot observe stale ownership or restart
        # the same feature through a nested signal.
        self._active = None
        if self._activities[active].stop() is False:
            self._active = active
            return False
        return True
