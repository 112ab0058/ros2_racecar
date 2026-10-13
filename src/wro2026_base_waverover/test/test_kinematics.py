"""Differential kinematics and the wheel speed cap."""
import math

import pytest

from wro2026_base_waverover.core import clamp, MAX_WHEEL, speed_limit, twist_to_wheels

TRACK = 0.125


def test_straight_forward_and_backward():
    assert twist_to_wheels(0.03, 0.0, TRACK, 0.05) == (0.03, 0.03)
    assert twist_to_wheels(-0.03, 0.0, TRACK, 0.05) == (-0.03, -0.03)


def test_spin_in_place_uses_half_track():
    left, right = twist_to_wheels(0.0, 0.8, TRACK, 0.05)
    assert left == pytest.approx(-0.05)
    assert right == pytest.approx(0.05)


def test_each_wheel_is_clamped_to_max_speed():
    assert twist_to_wheels(1.0, 0.0, TRACK, 0.05) == (0.05, 0.05)
    assert twist_to_wheels(-1.0, 0.0, TRACK, 0.05) == (-0.05, -0.05)
    # per-wheel clamp (same as the tested driver): right saturates, left does not
    left, right = twist_to_wheels(0.05, 1.0, TRACK, 0.05)
    assert left == pytest.approx(0.05 - 1.0 * TRACK / 2)
    assert right == 0.05


def test_max_speed_never_exceeds_firmware_limit():
    assert speed_limit(3.0) == MAX_WHEEL
    assert twist_to_wheels(2.0, 0.0, TRACK, 3.0) == (MAX_WHEEL, MAX_WHEEL)


def test_negative_max_speed_is_treated_as_magnitude():
    assert speed_limit(-0.05) == 0.05


def test_zero_max_speed_only_ever_produces_stop():
    for v, w in [(0.5, 0.0), (-0.5, 0.0), (0.0, 5.0), (0.3, -2.0)]:
        left, right = twist_to_wheels(v, w, TRACK, 0.0)
        assert left == 0 and right == 0


def test_nan_or_inf_twist_is_a_stop():
    # max(-0.05, min(0.05, nan)) would be 0.05 (full forward) without the guard
    for v, w in [(math.nan, 0.0), (0.0, math.nan), (math.inf, 0.0), (0.0, -math.inf)]:
        assert twist_to_wheels(v, w, TRACK, 0.05) == (0.0, 0.0)
    assert clamp(math.nan, 0.05) == 0.0
    assert speed_limit(math.nan) == 0.0
