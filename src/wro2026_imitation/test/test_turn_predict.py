"""TurnPredictor (turn_predict.py) on ideal corners: when it locks, and why."""
import math

import numpy as np
import pytest
from wro2026_imitation.turn_predict import LASER_X, TurnPredictor


def corner(xc, sign, w=0.40, d=0.003):
    """
    Return the wall points of an ideal corner, seen like the lidar.

    Wall points (base_link) of a corner seen like the lidar (nearest point per beam): car
    centred in a w m corridor, the inner wall ends xc m ahead of the rear axle.
    """
    h = w / 2.0
    pts = [(x, sign * h) for x in np.arange(-0.6, xc + 1e-9, d)]                 # inner wall
    pts += [(xc, sign * y) for y in np.arange(h, h + 0.8, d)]                    # island face
    pts += [(x, -sign * h) for x in np.arange(-0.6, xc + w + 1e-9, d)]           # outer wall
    pts += [(xc + w, sign * y) for y in np.arange(-h, h + 0.8, d)]               # wall ahead
    p = np.array(pts)
    a = np.arctan2(p[:, 1], p[:, 0] - LASER_X)
    r = np.hypot(p[:, 0] - LASER_X, p[:, 1])
    k = np.round(a % (2 * math.pi) / (2 * math.pi / 480)).astype(int) % 480
    out = np.full(480, np.inf)
    np.minimum.at(out, k, r)
    ang = 2 * math.pi * np.arange(480) / 480
    return LASER_X + out * np.cos(ang), out * np.sin(ang)


def lock_point(sign, k0=0.0):
    """Inner wall end (m ahead of the rear axle) on the first scan the prediction locks."""
    tp = TurnPredictor()
    for xc in np.arange(0.45, -0.10, -0.005):
        ok, room, later, wait = tp.ready(sign, *corner(xc, sign), k0, k0)
        if ok:
            return xc, room
    return None, None


def test_the_right_turn_locks_later_than_the_left():
    """
    The tighter right turn locks later than the left one.

    Full lock drives 0.403 m to the left, 0.292 m to the right: the tighter right turn must
    wait until the island corner is closer (2026-11-23: left 0.215, right 0.145 m ahead of
    the rear axle; the fixed corner_ahead 0.14 of 11-20 locked at ~0.27 both ways).
    """
    left, room_l = lock_point(1.0)
    right, room_r = lock_point(-1.0)
    assert 0.18 <= left <= 0.25 and 0.11 <= right <= 0.18
    assert left - right > 0.04
    assert room_l > 0.02 and room_r > 0.03            # both sides get room


def test_far_before_the_corner_it_waits():
    """0.4 m before the end the turn would run into the island (inner side first): wait."""
    ok, room, later, wait = TurnPredictor().ready(1.0, *corner(0.40, 1.0))
    assert not ok and room < 0.0


def test_too_late_it_locks_at_once():
    """Past the window the turn meets the wall ahead (outer side first): lock now."""
    ok, room, later, wait = TurnPredictor().ready(1.0, *corner(0.05, 1.0))
    assert ok and room < 0.0


def test_a_car_still_turning_the_other_way_locks_earlier():
    """
    A car still turning the other way must lock earlier.

    Turning right at -2.68 1/m (a small right correction) when the lock comes: the steering
    needs ~0.4 s to swing to full left, so the lock must come earlier than from straight.
    """
    straight, _ = lock_point(1.0, 0.0)
    swinging, _ = lock_point(1.0, -2.68)
    assert swinging > straight + 0.01


def test_commands_on_their_way_count():
    """
    Commands still on their way to the steering are followed.

    'straight' was sent 1 cm ago while the car still turns right: it reaches the steering
    before the lock does, so the predicted path is less to the right than if the right
    correction went on.
    """
    tp = TurnPredictor()
    on_its_way = tp.path(1.0, -2.68, queued=[(0.5, -2.68), (0.01, 0.13)])
    still_right = tp.path(1.0, -2.68, queued=[(0.5, -2.68)])
    i = 3                                          # 6 cm along
    assert on_its_way[i, 1] > still_right[i, 1]


def test_no_points_no_lock():
    ok, room, later, wait = TurnPredictor().ready(1.0, np.full(480, np.nan), np.full(480, np.nan))
    assert not ok


def test_bad_settings():
    with pytest.raises(ValueError):
        TurnPredictor(k_full_left=-1.0)
    with pytest.raises(ValueError):
        TurnPredictor(wait=0.005, wait_step=0.01)
