import pytest

from go2_mapping_f.gate_policy import ScanGatePolicy


def make_policy():
    return ScanGatePolicy(0.70, 0.50, 0.20, 0.30)


def test_stable_scans_pass_immediately():
    policy = make_policy()

    assert policy.evaluate(1.00, 10.00, 0.10).allow
    assert policy.evaluate(1.07, 10.07, 0.40).allow


def test_fast_rotation_blocks_until_hysteresis_and_settle_complete():
    policy = make_policy()

    assert policy.evaluate(1.00, 10.00, 0.71).reason == "angular_velocity"
    assert not policy.evaluate(1.10, 10.07, 0.55).allow
    assert policy.evaluate(1.20, 10.14, 0.40).reason == "settling"
    assert not policy.evaluate(1.39, 10.21, 0.30).allow
    assert policy.evaluate(1.41, 10.28, 0.30).allow


def test_large_or_reversed_scan_gap_is_rejected():
    policy = make_policy()

    assert policy.evaluate(1.00, 10.00, 0.0).allow
    decision = policy.evaluate(1.35, 10.35, 0.0)
    assert not decision.allow
    assert decision.reason == "scan_gap"
    assert decision.scan_gap == pytest.approx(0.35)

    # A gap drops only its first scan; the next normal scan resumes immediately.
    decision = policy.evaluate(1.42, 10.42, 0.0)
    assert decision.allow

    decision = policy.evaluate(1.50, 10.41, 0.0)
    assert not decision.allow
    assert decision.reason == "scan_gap"
