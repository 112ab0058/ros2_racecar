"""Safety rules in front of the policies, and the keep-centred baseline (no ROS, no motors)."""
import math

import numpy as np
import pytest
from wro2026_imitation.features import (FULL_LEFT, LEFT, mirror, N_BINS, RIGHT,
                                        scan_features, STRAIGHT)
from wro2026_imitation.model import MLP
from wro2026_imitation.policy import (BAD_SCAN, center_rule, check_model, Driver, inside_outline,
                                      model_policy, NO_SCAN, OK, side_word, state_text, STOP,
                                      TOO_CLOSE)

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


def stand_scan():
    """
    Scan from the stand on 2026-11-12.

    A point 0.130 m away at 237.8 deg (rear right), 2.0 cm from the side, plus a wall
    1 m to the right.
    """
    r = corridor(np.inf, 1.0)
    r[int(round(237.75 / 0.75))] = 0.130
    return r


def test_remembered_point_near_the_side_does_not_latch():
    r = stand_scan()
    mem = np.zeros(BEAMS, dtype=bool)
    mem[int(round(237.75 / 0.75))] = True
    d = Driver(always(STRAIGHT), 0.1, 0.5, stop_margin=0.02)
    d.on_scan(0.0, r, 0.0, INC, remembered=mem)
    assert d.command(0.05)[3] == OK
    assert d.nearest[2] and 0.019 < d.nearest[1] < 0.021        # remembered, 2.0 cm away
    assert '右後' in state_text(OK, d) and '記憶' in state_text(OK, d)


def test_measured_point_near_the_side_latches_and_says_where():
    d = Driver(always(STRAIGHT), 0.1, 0.5, stop_margin=0.02)
    d.on_scan(0.0, stand_scan(), 0.0, INC)                       # same point, measured
    assert d.command(0.05) == (0.0, 0.0, STOP, TOO_CLOSE)
    text = state_text(TOO_CLOSE, d)
    assert '太靠近' in text and '右後' in text and '2.0 cm' in text and '記憶' not in text


def test_remembered_point_inside_the_car_latches():
    r = put(90.0, 0.085)                                         # inside the left side
    d = Driver(always(STRAIGHT), 0.1, 0.5, stop_margin=0.02)
    d.on_scan(0.0, r, 0.0, INC, remembered=np.isfinite(r))
    assert d.command(0.05)[3] == TOO_CLOSE
    assert '車身內' in state_text(TOO_CLOSE, d)


@pytest.mark.parametrize('deg, word', [(0, '前'), (20, '前'), (45, '左前'), (90, '左'),
                                       (135, '左後'), (180, '後'), (-170, '後'),
                                       (237.8, '右後'), (-90, '右'), (-40, '右前'),
                                       (350, '前')])
def test_side_words(deg, word):
    assert side_word(deg) == word


def test_state_text_for_the_other_reasons():
    d = Driver(always(STRAIGHT), 0.1, 0.5)
    assert '沒有光達' in state_text(NO_SCAN, d)
    assert state_text(OK, d) == '正常'
    assert state_text(OK, d, 'turn', 2).startswith('打滿轉彎，過了 2 個彎')
    assert '方向還沒判斷' in state_text(OK, d, 'follow', 0, 'auto')
    assert '彎道往右' in state_text(OK, d, 'follow', 1, 'cw')


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


def test_check_model_takes_full_lock_models_and_old_ones():
    """5 labels (2026-11-25, with full lock) or the first 3 (older models)."""
    five = {'half_fov_deg': 110.0, 'bin_deg': 5.0, 'max_range': 3.0,
            'labels': ['left', 'straight', 'right', 'full_left', 'full_right']}
    check_model(MLP(N_BINS, n_out=5), five)
    with pytest.raises(ValueError, match='layer sizes'):
        check_model(MLP(N_BINS, n_out=3), five)
    with pytest.raises(ValueError, match='labels'):
        check_model(MLP(N_BINS, n_out=2), dict(five, labels=['left', 'right']))


def test_network_full_lock_drives_the_full_lock_radius():
    class FullLeft:
        def predict_proba(self, x):
            return np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])

    d = Driver(model_policy(FullLeft()), 0.1, 0.5, full_turn_radius=0.35)
    d.on_scan(0.0, corridor(0.3, 0.3), 0.0, INC)
    vx, wz, decision, reason = d.command(0.05)
    assert (decision, reason) == (FULL_LEFT, OK)
    assert (vx, wz) == pytest.approx((0.1, 0.1 / 0.35))


def test_reset_unlatches_only_when_the_newest_scan_is_clear():
    """The phone's 重置: refused while something is still too close or the scan is old."""
    d = Driver(always(STRAIGHT), 0.1, 0.5)
    assert d.reset(0.0) == (False, '沒有鎖住')
    d.on_scan(0.0, put(0.0, 0.13), 0.0, INC)                   # 2 cm in front: latched
    assert d.command(0.05)[3] == TOO_CLOSE
    done, why = d.reset(0.05)
    assert not done and '還太靠近' in why and '前' in why
    assert '鎖住' in state_text(TOO_CLOSE, d) and '現在還有' in state_text(TOO_CLOSE, d)
    d.on_scan(0.1, corridor(0.3, 0.3), 0.0, INC)               # driven away by hand
    assert d.command(0.15)[3] == TOO_CLOSE                      # still latched ...
    assert '可以按「重置」' in state_text(TOO_CLOSE, d)          # ... but the page may reset
    assert d.reset(0.5) == (False, '沒有新的光達資料')           # newest scan 0.4 s old
    assert d.reset(0.15) == (True, '已解除鎖住')
    assert d.command(0.16) == (0.0, 0.0, STOP, NO_SCAN)         # waits for the next scan
    d.on_scan(0.2, corridor(0.3, 0.3), 0.0, INC)
    assert d.command(0.25)[3] == OK
    d.on_scan(0.3, put(0.0, 0.13), 0.0, INC)                   # too close again: latched again
    assert d.command(0.35)[3] == TOO_CLOSE


def test_a_policy_stop_stops_the_car_with_reason_ok():
    """The corner rule's state done says STOP itself: the car stops, the phone says why."""
    d = Driver(always(STOP), 0.1, 0.5)
    d.on_scan(0.0, corridor(0.3, 0.3), 0.0, INC)
    assert d.command(0.05) == (0.0, 0.0, STOP, OK)
    assert state_text(OK, d, 'done', 12).startswith('跑完了，停在起點，過了 12 個彎')


class Turning:
    """A policy in the middle of a corner turn: full lock, heading still to turn."""

    def __init__(self, label, plan):
        self.label, self.plan = label, plan

    def __call__(self, x):
        return self.label, None

    def planned_turn(self):
        return self.plan


@pytest.mark.parametrize('gap, predict, latched', [
    (0.005, False, True),      # the outline + 2 cm rule (until 2026-11-26)
    (0.005, True, False),      # predicted: driving along a wall 0.5 cm away touches nothing
    (-0.002, True, True)])     # inside the outline: touching now
def test_predicted_stop_drives_along_a_wall_close_beside(gap, predict, latched):
    """Ray 2026-11-26: stop when the car will touch, not at some distance."""
    r = corridor(0.09 + gap, 0.3)
    d = Driver(always(STRAIGHT), 0.1, 0.5, stop_margin=0.0 if predict else 0.02,
               predict=predict)
    d.on_scan(0.0, r, 0.0, INC)
    d.on_scan(0.1, r, 0.0, INC)                     # now with a command on its way
    assert (d.command(0.15)[3] == TOO_CLOSE) == latched


@pytest.mark.parametrize('front, latched', [(0.15, True), (0.20, False)])
def test_predicted_stop_looks_5_cm_ahead(front, latched):
    """Straight on: a wall 4 cm ahead of the bumper is within the 5 cm horizon, 9 cm is not."""
    r = corridor(0.3, 0.3, front)
    d = Driver(always(STRAIGHT), 0.1, 0.5, stop_margin=0.0, predict=True)
    d.on_scan(0.0, r, 0.0, INC)
    d.on_scan(0.1, r, 0.0, INC)
    assert (d.command(0.15)[3] == TOO_CLOSE) == latched
    if latched:
        text = state_text(TOO_CLOSE, d)
        assert text.startswith('照現在的轉法開下去會碰到') and '先用按鍵開離' in text


@pytest.mark.parametrize('front, latched', [(0.37, True), (0.47, False)])
def test_predicted_stop_follows_the_rest_of_the_turn(front, latched):
    """
    Full left lock, 80 deg still to turn: the outer front corner swings 0.548 m ahead.

    A wall 0.50 m ahead of the rear axle (0.26 m from the bumper) stops the car now; 0.60
    m does not. The outline rule saw neither.
    """
    r = corridor(np.inf, np.inf, front)
    d = Driver(Turning(FULL_LEFT, (1.0, math.radians(80.0))), 0.1, 0.5, stop_margin=0.0,
               predict=True)
    d.k_now = 2.48                                  # at full lock already
    d.on_scan(0.0, r, 0.0, INC)
    d.on_scan(0.1, r, 0.0, INC)
    assert (d.command(0.15)[3] == TOO_CLOSE) == latched
    assert d.nearest[1] > 0.2


def test_predicted_stop_while_the_policy_holds_the_car():
    """The policy says STOP (corner: blocked / done): only where the car is counts."""
    r = corridor(0.3, 0.3, 0.15)
    d = Driver(always(STOP), 0.1, 0.5, stop_margin=0.0, predict=True)
    d.on_scan(0.0, r, 0.0, INC)
    d.on_scan(0.1, r, 0.0, INC)
    assert d.command(0.15) == (0.0, 0.0, STOP, OK)
