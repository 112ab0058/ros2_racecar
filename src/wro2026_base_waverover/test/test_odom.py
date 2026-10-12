"""Dead-reckoning odometry: commanded speed x pwm_to_mps, IMU yaw rate."""
import math

import pytest

from wro2026_base_waverover.core import normalize_angle, odom_twist, OdomIntegrator

DT = 0.05  # 20 Hz like the node


def integrate(seconds, left, right, gz, pwm_to_mps, **kw):
    odom = OdomIntegrator()
    for _ in range(round(seconds / DT)):
        v, wz = odom_twist(left, right, gz, pwm_to_mps, **kw)
        odom.update(DT, v, wz)
    return odom


def test_straight_line():
    odom = integrate(2.0, 0.05, 0.05, 0.0, pwm_to_mps=1.0)
    assert odom.x == pytest.approx(0.1)
    assert odom.y == pytest.approx(0.0)
    assert odom.yaw == pytest.approx(0.0)


def test_spin_in_place_uses_imu_and_does_not_translate():
    odom = integrate(2.0, -0.05, 0.05, 0.5, pwm_to_mps=1.0)
    assert odom.yaw == pytest.approx(1.0)
    assert odom.x == pytest.approx(0.0)
    assert odom.y == pytest.approx(0.0)


def test_no_command_does_not_move_even_with_gyro_bias():
    odom = integrate(10.0, 0, 0, 0.02, pwm_to_mps=1.0)
    assert (odom.x, odom.y, odom.yaw) == (0.0, 0.0, 0.0)


def test_unknown_pwm_to_mps_never_translates():
    odom = integrate(2.0, 0.05, 0.05, 0.3, pwm_to_mps=0.0)
    assert (odom.x, odom.y) == (0.0, 0.0)
    assert odom.yaw == pytest.approx(0.6)


def test_stale_imu_gives_no_rotation():
    v, wz = odom_twist(0.05, 0.05, 1.0, 1.0, imu_fresh=False)
    assert (v, wz) == (0.05, 0.0)
    assert odom_twist(0.05, 0.05, None, 1.0) == (0.05, 0.0)


def test_yaw_rate_sign():
    assert odom_twist(-0.05, 0.05, 0.5, 1.0, yaw_rate_sign=-1.0) == (0.0, -0.5)


def test_arc_matches_circle():
    # v = 0.1 m/s, wz = 0.5 rad/s for 2*pi s -> half circle of radius 0.2 m
    odom = OdomIntegrator()
    steps = 2000
    for _ in range(steps):
        odom.update(2 * math.pi / steps, 0.1, 0.5)
    assert odom.x == pytest.approx(0.0, abs=1e-6)
    assert odom.y == pytest.approx(0.4, abs=1e-6)
    assert abs(odom.yaw) == pytest.approx(math.pi, abs=1e-6)


def test_zero_or_negative_dt_is_ignored():
    odom = OdomIntegrator()
    odom.update(0.0, 1.0, 1.0)
    odom.update(-1.0, 1.0, 1.0)
    assert (odom.x, odom.y, odom.yaw) == (0.0, 0.0, 0.0)


def test_normalize_angle():
    assert normalize_angle(3 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert normalize_angle(0.25) == pytest.approx(0.25)
