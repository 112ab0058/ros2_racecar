"""cmd_vel_units: 'pwm' keeps the original behaviour, 'mps' converts with pwm_to_mps."""
import math

import pytest

from wro2026_base_waverover.core import (BaseController, check_cmd_vel_units, twist_mps_to_wheels,
                                         twist_to_wheels)

TRACK = 0.125


def test_pwm_is_the_default_and_unchanged(clock):
    c = BaseController(TRACK, 0.05, 0.3, clock=clock)
    assert c.cmd_vel_units == 'pwm'
    c.set_cmd_vel(0.03, 0.0)
    assert c.requested() == (0.03, 0.03)
    assert c.requested() == tuple(twist_to_wheels(0.03, 0.0, TRACK, 0.05))


def test_mps_converts_with_pwm_to_mps(clock):
    # 2.0 m/s per unit command: 0.06 m/s -> 0.03 PWM
    c = BaseController(TRACK, 0.05, 0.3, clock=clock, cmd_vel_units='mps', pwm_to_mps=2.0)
    c.set_cmd_vel(0.06, 0.0)
    assert c.requested() == (0.03, 0.03)
    c.set_cmd_vel(0.0, 0.8)  # wheel +-0.05 m/s -> +-0.025 PWM
    assert c.requested() == (-0.025, 0.025)


def test_mps_is_still_capped_by_max_speed():
    assert twist_mps_to_wheels(10.0, 0.0, TRACK, 0.05, 2.0) == (0.05, 0.05)
    assert twist_mps_to_wheels(-10.0, 0.0, TRACK, 0.05, 2.0) == (-0.05, -0.05)


def test_mps_non_finite_or_unknown_scale_is_a_stop():
    assert twist_mps_to_wheels(math.nan, 0.0, TRACK, 0.05, 2.0) == (0.0, 0.0)
    assert twist_mps_to_wheels(0.1, math.inf, TRACK, 0.05, 2.0) == (0.0, 0.0)
    assert twist_mps_to_wheels(0.1, 0.0, TRACK, 0.05, 0.0) == (0.0, 0.0)


@pytest.mark.parametrize('scale', [0.0, -1.0, math.nan, math.inf])
def test_mps_without_measured_pwm_to_mps_refuses_to_start(scale, clock):
    with pytest.raises(ValueError, match='pwm_to_mps'):
        check_cmd_vel_units('mps', scale)
    with pytest.raises(ValueError):
        BaseController(TRACK, 0.05, 0.3, clock=clock, cmd_vel_units='mps', pwm_to_mps=scale)


def test_unknown_units_are_rejected(clock):
    with pytest.raises(ValueError, match='cmd_vel_units'):
        BaseController(TRACK, 0.05, 0.3, clock=clock, cmd_vel_units='m/s', pwm_to_mps=1.0)
    check_cmd_vel_units('pwm', 0.0)  # pwm does not need the scale
