"""State machine used to keep unstable scans out of the persistent SLAM map."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class GateDecision:
    """Result returned for one incoming scan."""

    allow: bool
    reason: str
    scan_gap: Optional[float]


class ScanGatePolicy:
    """Hysteretic gate driven by yaw speed and scan timestamp continuity."""

    def __init__(
        self,
        block_angular_velocity: float,
        resume_angular_velocity: float,
        settle_time: float,
        max_scan_gap: float,
    ) -> None:
        if block_angular_velocity <= 0.0:
            raise ValueError("block_angular_velocity must be positive")
        if not 0.0 <= resume_angular_velocity < block_angular_velocity:
            raise ValueError(
                "resume_angular_velocity must be non-negative and below block threshold"
            )
        if settle_time < 0.0 or max_scan_gap <= 0.0:
            raise ValueError("settle_time and max_scan_gap must be valid durations")

        self.block_angular_velocity = block_angular_velocity
        self.resume_angular_velocity = resume_angular_velocity
        self.settle_time = settle_time
        self.max_scan_gap = max_scan_gap
        self.blocked = False
        self.block_reason = ""
        self.stable_since: Optional[float] = None
        self.last_scan_stamp: Optional[float] = None

    def evaluate(
        self, now: float, scan_stamp: float, angular_velocity: float
    ) -> GateDecision:
        """Return whether a scan is stable enough to be inserted into SLAM."""
        scan_gap = None
        if self.last_scan_stamp is not None:
            scan_gap = scan_stamp - self.last_scan_stamp
        self.last_scan_stamp = scan_stamp

        invalid_gap = scan_gap is not None and (
            scan_gap <= 0.0 or scan_gap > self.max_scan_gap
        )
        if invalid_gap:
            self.blocked = True
            self.block_reason = "scan_gap"
            self.stable_since = (
                now - self.settle_time
                if angular_velocity <= self.resume_angular_velocity
                else None
            )
            return GateDecision(False, self.block_reason, scan_gap)

        if angular_velocity >= self.block_angular_velocity:
            self.blocked = True
            self.block_reason = "angular_velocity"
            self.stable_since = None
            return GateDecision(False, self.block_reason, scan_gap)

        if not self.blocked:
            return GateDecision(True, "stable", scan_gap)

        if angular_velocity > self.resume_angular_velocity:
            self.stable_since = None
            return GateDecision(False, self.block_reason, scan_gap)

        if self.stable_since is None:
            self.stable_since = now
            return GateDecision(False, "settling", scan_gap)

        if now - self.stable_since < self.settle_time:
            return GateDecision(False, "settling", scan_gap)

        self.blocked = False
        self.block_reason = ""
        self.stable_since = None
        return GateDecision(True, "resumed", scan_gap)
