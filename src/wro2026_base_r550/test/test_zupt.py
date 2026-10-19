"""Gyro bias by zero-velocity update (zupt.py), with a fake clock."""
import pytest

from wro2026_base_r550.zupt import GyroBias

DT = 0.05   # 20 Hz feedback


def run(z, seconds, vx=0.0, wz=0.0, gz=0.0, t0=0.0):
    out = None
    n = int(round(seconds / DT))
    for i in range(n):
        out = z.update(t0 + i * DT, vx, wz, 0.0, 0.0, gz)
    return out, t0 + n * DT


def test_learns_bias_when_still_and_removes_it():
    z = GyroBias(still_time=1.0, time_constant=5.0)
    out, _ = run(z, 30.0, gz=0.0002)              # 30 s still, bias 0.0002 rad/s
    assert z.bias[2] == pytest.approx(0.0002, rel=0.01)
    assert abs(out[2]) < 2e-6


def test_waits_still_time_before_learning():
    z = GyroBias(still_time=1.0)
    run(z, 0.9, gz=0.001)
    assert z.bias[2] == 0.0 and not z.learning


def test_no_learning_while_wheels_turn():
    z = GyroBias()
    out, _ = run(z, 10.0, vx=0.1, gz=0.3)         # driving in a curve
    assert z.bias == [0.0, 0.0, 0.0] and out[2] == pytest.approx(0.3)


def test_rotation_while_wheels_still_is_rejected():
    """Car carried by hand: wheels still but the body turns."""
    z = GyroBias(max_rate=0.01)
    run(z, 10.0, gz=0.5)
    assert z.bias[2] == 0.0 and z.rejected > 0


def test_bias_is_clamped():
    z = GyroBias(max_rate=1.0, max_bias=0.02)
    run(z, 60.0, gz=0.009)
    run(z, 60.0, gz=0.018, t0=60.0)
    run(z, 60.0, gz=0.027, t0=120.0)
    assert z.bias[2] == pytest.approx(0.02)


def test_disabled_passes_through():
    z = GyroBias(enabled=False)
    out, _ = run(z, 10.0, gz=0.001)
    assert out[2] == 0.001 and z.bias[2] == 0.0


def test_motion_resets_the_still_timer():
    z = GyroBias(still_time=1.0)
    _, t = run(z, 0.8, gz=0.001)
    _, t = run(z, 0.2, vx=0.1, gz=0.001, t0=t)
    run(z, 0.8, gz=0.001, t0=t)
    assert z.bias[2] == 0.0
