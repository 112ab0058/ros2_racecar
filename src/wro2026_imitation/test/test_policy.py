"""Safety rules in front of the policies, and the keep-centred baseline (no ROS, no motors)."""
import math

import numpy as np
import pytest
from wro2026_imitation.features import LEFT, mirror, N_BINS, RIGHT, scan_features, STRAIGHT
from wro2026_imitation.model import MLP
from wro2026_imitation.policy import (BAD_SCAN, center_rule, check_model, Driver, inside_outline,
                                      model_policy, NO_SCAN, OK, STOP, TOO_CLOSE)

BEAMS = 480
INC = 2.0 * math.pi / BEAMS


def scan():
    return np.full(BEAMS, np.inf, dtype=np.float32)


def put(deg, dist):
    r = scan()
    r[int(round((deg % 360.0) / 0.75)) % BEAMS] = dist
    return r


def corridor(left, right, front=np.inf):
    """Walls along the car `left` / `right` m from the lidar, and one across `front` m ahead."""
    a = np.arange(BEAMS) * INC
    s, c = np.sin(a), np.cos(a)
    with np.errstate(divide='ignore', invalid='ignore'):
        to_left = np.where(s > 1e-9, left / s, np.inf)
        to_right = np.where(s < -1e-9, -right / s, np.inf)
        ahead = np.where(c > 1e-9, front / c, np.inf)
    return np.minimum.reduce([to_left, to_right, ahead]).astype(np.float32)


def always(label):
    return lambda x: (label, None)


def never(x):
    raise AssertionError('the policy must not be asked')


@pytest.mark.parametrize('deg, dist, inside', [
    (0.0, 0.14, True), (0.0, 0.17, False),       # front: 0.11 m + 0.04 margin
    (90.0, 0.12, True), (90.0, 0.15, False),     # side: 0.09 m + 0.04 margin
    (-90.0, 0.12, True), (45.0, 0.17, True), (45.0, 0.25, False)])
def test_points_near_the_car_outline(deg, dist, inside):
    assert inside_outline(put(deg, dist), 0.0, INC, 0.04) == inside


def test_no_return_is_not_an_obstacle():
    assert not inside_outline(scan(), 0.0, INC, 0.04)


def test_lane_of_60_cm_is_not_too_close():
    assert not inside_outline(corridor(0.30, 0.30), 0.0, INC, 0.04)
    assert inside_outline(corridor(0.12, 0.48), 0.0, INC, 0.04)


@pytest.mark.parametrize('left, right, front, expected', [
    (0.30, 0.30, np.inf, STRAIGHT),
    (0.40, 0.20, np.inf, LEFT),         # right wall closer: move left
    (0.20, 0.40, np.inf, RIGHT),
    (0.32, 0.28, np.inf, STRAIGHT),     # within side_margin
    (np.inf, 0.30, 0.40, LEFT),         # wall ahead, open on the left (corner)
    (0.30, np.inf, 0.40, RIGHT)])
def test_center_rule(left, right, front, expected):
    assert center_rule(scan_features(corridor(left, right, front)))[0] == expected


def test_center_rule_is_mirror_symmetric():
    x = np.random.default_rng(7).random((500, N_BINS))
    m, swapped = mirror(x, center_rule(x))
    assert np.array_equal(center_rule(m), swapped)


def test_driver_stops_before_the_first_scan():
    assert Driver(always(LEFT), 0.1, 0.5).command(0.0) == (0.0, 0.0, STOP, NO_SCAN)


def test_driver_turns_with_the_recorded_radius():
    d = Driver(always(LEFT), 0.1, 0.5)
    d.on_scan(0.0, corridor(0.3, 0.3), 0.0, INC)
    vx, wz, label, reason = d.command(0.1)
    assert (label, reason) == (LEFT, OK)
    assert (vx, wz) == pytest.approx((0.1, 0.2))        # radius vx / wz = 0.5 m
    d.decide = always(RIGHT)
    d.on_scan(0.2, corridor(0.3, 0.3), 0.0, INC)
    assert d.command(0.25)[:3] == pytest.approx((0.1, -0.2, RIGHT))


def test_driver_stops_when_scans_stop():
    d = Driver(always(STRAIGHT), 0.1, 0.5, scan_timeout=0.3)
    d.on_scan(0.0, corridor(0.3, 0.3), 0.0, INC)
    assert d.command(0.29)[3] == OK
    assert d.command(0.31) == (0.0, 0.0, STOP, NO_SCAN)


def test_too_close_stops_for_good():
    d = Driver(always(STRAIGHT), 0.1, 0.5)
    d.on_scan(0.0, put(0.0, 0.13), 0.0, INC)
    assert d.command(0.05) == (0.0, 0.0, STOP, TOO_CLOSE)
    d.decide = never
    d.on_scan(0.1, corridor(0.3, 0.3), 0.0, INC)       # obstacle gone: still stopped
    assert d.command(0.15) == (0.0, 0.0, STOP, TOO_CLOSE)


def test_bad_scan_stops_until_a_good_one():
    d = Driver(always(STRAIGHT), 0.1, 0.5)
    d.on_scan(0.0, corridor(0.3, 0.3), -math.pi, INC)   # starts at the back
    assert d.command(0.05)[2:] == (STOP, BAD_SCAN)
    d.on_scan(0.1, corridor(0.3, 0.3)[:240], 0.0, INC)  # half a turn
    assert d.command(0.15)[3] == BAD_SCAN
    d.on_scan(0.2, corridor(0.3, 0.3), 0.0, INC)
    assert d.command(0.25)[3] == OK


def test_model_policy_returns_the_most_likely_label():
    model = MLP(N_BINS, seed=3)
    x = scan_features(corridor(0.3, 0.3))
    label, p = model_policy(model)(x)
    assert label == int(np.argmax(p)) and p.sum() == pytest.approx(1.0)


def test_check_model_refuses_other_inputs():
    good = {'half_fov_deg': 110.0, 'bin_deg': 5.0, 'max_range': 3.0,
            'labels': ['left', 'straight', 'right']}
    check_model(MLP(N_BINS), good)
    with pytest.raises(ValueError, match='max_range'):
        check_model(MLP(N_BINS), dict(good, max_range=5.0))
    with pytest.raises(ValueError, match='layer sizes'):
        check_model(MLP(N_BINS - 2), good)
