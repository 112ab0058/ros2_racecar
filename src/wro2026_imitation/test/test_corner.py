"""Ray's corner technique (corner.py) on the 2-D field: laps, safety rules, blind lidar."""
import math

import numpy as np
import pytest
from wro2026_imitation.corner import CornerPolicy, FOLLOW, FULL_LEFT, FULL_RIGHT, TURN
from wro2026_imitation.features import LEFT, RIGHT, STRAIGHT
from wro2026_imitation.field_sim import camera as camera_view, Car, Field, lidar
from wro2026_imitation.policy import Driver, NO_ODOM, OK, STOP
from wro2026_lidar_n10p.scan_memory import ScanMemory

INC = 2.0 * math.pi / 480


def drive_laps(direction, laps=1, island=0.80, memory=True, seed=0, policy_direction=None,
               partial=False, camera=False):
    """Drive the corner policy around the field; return (ok, why, laps, min clearance, policy)."""
    field = Field(1.6, island)
    centre = (0.8 + island / 2.0) / 2.0
    car = Car(-0.35 if direction == 'ccw' else 0.35, -centre,
              0.0 if direction == 'ccw' else math.pi)
    policy = CornerPolicy(policy_direction or direction)
    driver = Driver(policy, 0.1, 0.5, stop_margin=0.02, full_turn_radius=0.35)
    mem = ScanMemory()
    rng = np.random.default_rng(seed)
    dt, t, vx, wz, turned, last, clear = 0.02, 0.0, 0.0, 0.0, 0.0, car.yaw, 9.0
    while t < laps * 50.0:
        if int(round(t / dt)) % 5 == 0:                       # odometry and lidar at 10 Hz
            driver.on_odom(t, car.x, car.y, car.yaw)
            mem.on_odom(car.x, car.y, car.yaw)
            if camera:
                mem.add_camera(t, camera_view(field, car, rng=rng), 0.0, INC)
            r = lidar(field, car, rng=rng, partial=partial)
            if memory:
                r, _ = mem.on_scan(t, r, 0.0, INC)
            driver.on_scan(t, r, 0.0, INC)
            vx, wz, decision, reason = driver.command(t)
            if decision == STOP:
                return False, reason, turned / (2 * math.pi), clear, policy
        car.step(vx, wz, dt)
        turned += abs(car.yaw - last)
        last = car.yaw
        clear = min(clear, field.clearance(car.x, car.y, car.yaw))
        if clear < 0:
            return False, 'crash', turned / (2 * math.pi), clear, policy
        if turned >= laps * 2 * math.pi:
            return True, 'done', turned / (2 * math.pi), clear, policy
        t += dt
    return False, 'too slow', turned / (2 * math.pi), clear, policy


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_one_lap_each_way_with_the_blind_black_walls(direction):
    ok, why, laps, clear, policy = drive_laps(direction)
    assert ok, (why, laps)
    assert clear > 0.02 and policy.corners == 4


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
@pytest.mark.parametrize('seed', [0, 1, 2])
def test_auto_finds_the_direction_and_drives_the_lap(direction, seed):
    ok, why, laps, clear, policy = drive_laps(direction, seed=seed, policy_direction='auto')
    assert ok, (why, laps)
    assert policy.direction == direction and policy.corners == 4 and clear > 0.02


def wall_scan(walls):
    """Lidar scan (480 beams, no noise, lidar at 0, 0) of walls (x0, y0, x1, y1)."""
    a = np.arange(480) * INC
    c, s = np.cos(a), np.sin(a)
    r = np.full(480, np.inf)
    for x0, y0, x1, y1 in walls:
        dx, dy = x1 - x0, y1 - y0
        den = c * dy - s * dx                     # ray (c, s) * t = (x0, y0) + (dx, dy) * u
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (x0 * dy - y0 * dx) / den
            u = (x0 * s - y0 * c) / den
        hit = (np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
        r = np.where(hit, np.minimum(r, t), r)
    return r


def auto_after(walls, scans=5):
    policy = CornerPolicy('auto')
    policy.on_odom(0.0, 0.0, 0.0)
    for _ in range(scans):
        label, _ = policy(wall_scan(walls), 0.0, INC)
        assert label not in (FULL_LEFT, FULL_RIGHT)
    return policy


def test_auto_a_gap_in_a_wall_is_not_a_corner():
    """Right wall missing for 10 cm (blind, no return): points after the gap are on the line."""
    walls = [(-1.0, 0.2, 2.0, 0.2), (-1.0, -0.2, 0.3, -0.2), (0.4, -0.2, 2.0, -0.2)]
    policy = auto_after(walls)
    assert policy.direction is None and policy.state == FOLLOW


@pytest.mark.parametrize('flip, expected', [(1.0, 'ccw'), (-1.0, 'cw')])
def test_auto_the_side_that_opens_is_the_inside(flip, expected):
    """Inner wall ends 0.3 m ahead, open beyond; the outer wall runs into the wall ahead."""
    walls = [(-1.0, 0.2, 0.3, 0.2),          # inner wall (island), ends at x 0.3
             (0.3, 0.2, 0.3, 1.0),           # island's next side
             (-1.0, 1.0, 1.0, 1.0),          # far outer wall across the next corridor
             (-1.0, -0.2, 0.7, -0.2),        # outer wall
             (0.7, -0.2, 0.7, 1.0)]          # wall ahead
    walls = [(x0, flip * y0, x1, flip * y1) for x0, y0, x1, y1 in walls]
    policy = auto_after(walls, scans=2)
    assert policy.direction is None                    # needs confirm_scans (3) in a row
    policy(wall_scan(walls), 0.0, INC)
    assert policy.direction == expected


def test_without_memory_the_lap_fails():
    """No memory: the blind inner wall is not seen in time; the safety stop ends the run."""
    ok, why, laps, clear, policy = drive_laps('ccw', memory=False)
    assert not ok and why in ('too_close', 'crash') and laps < 0.5


def test_open_side_with_no_returns_is_not_a_corner():
    """Blind beams (inf) are unknown, never 'open'."""
    policy = CornerPolicy('ccw')
    policy.on_odom(0.0, 0.0, 0.0)
    r = np.full(480, np.inf)
    label, _ = policy(r, 0.0, INC)
    assert label == STRAIGHT and policy.state == FOLLOW


def test_turn_lasts_until_the_heading_changed_by_turn_deg():
    policy = CornerPolicy('cw', turn_deg=80.0)
    policy.on_odom(0.0, 0.0, 0.0)
    policy._enter(TURN)
    r = np.full(480, np.inf)
    assert policy(r, 0.0, INC)[0] == FULL_RIGHT
    policy.on_odom(0.1, -0.1, math.radians(-79.0))
    assert policy(r, 0.0, INC)[0] == FULL_RIGHT
    policy.on_odom(0.12, -0.15, math.radians(-81.0))
    assert policy(r, 0.0, INC)[0] != FULL_RIGHT and policy.state != TURN


def test_driver_stops_without_fresh_odometry():
    driver = Driver(CornerPolicy('ccw'), 0.1, 0.5, full_turn_radius=0.35)
    r = np.full(480, np.inf)
    driver.on_scan(0.0, r, 0.0, INC)
    assert driver.command(0.05)[2:] == (STOP, NO_ODOM)
    driver.on_odom(0.1, 0.0, 0.0, 0.0)
    driver.on_scan(0.1, r, 0.0, INC)
    assert driver.command(0.15)[3] == OK
    assert driver.command(0.5)[2] == STOP          # odometry 0.4 s old


def test_full_lock_uses_the_full_turn_radius():
    for label, sign in ((FULL_LEFT, 1), (FULL_RIGHT, -1), (LEFT, 1), (RIGHT, -1)):
        driver = Driver(lambda x, label=label: (label, None), 0.1, 0.5, full_turn_radius=0.35)
        driver.on_scan(0.0, np.full(480, np.inf), 0.0, INC)
        vx, wz, _, _ = driver.command(0.0)
        radius = 0.35 if label in (FULL_LEFT, FULL_RIGHT) else 0.5
        assert vx == 0.1 and wz == pytest.approx(sign * 0.1 / radius)


def test_bad_direction():
    with pytest.raises(ValueError):
        CornerPolicy('left')
    assert CornerPolicy('auto').direction is None and CornerPolicy('cw').direction == 'cw'


def run4_scan(camera):
    """
    Run 4 of 2026-11-13 (cw, false full right lock 0.11 m after the start), lidar frame.

    Inner (right) wall y -0.20, outer (left) wall y +0.22, both straight on. Replayed from
    the bag (2026-11-20): the black inner wall answered only with 7 points bunched behind
    the car (x -0.25..-0.20, r 0.28-0.32) and from about 0.46 m ahead; in between the
    lidar saw nothing. camera: add what the C70 sees, the wall bottoms from x 0.28 m
    (lidar frame) to 1.0 m from the lidar.
    """
    walls = [(-1.0, -0.20, 2.0, -0.20), (-1.0, 0.22, 2.0, 0.22)]
    r = wall_scan(walls)
    x = r * np.cos(np.arange(480) * INC)
    behind = (x >= -0.25) & (x <= -0.20) & (r < 0.33) & (np.sin(np.arange(480) * INC) < 0)
    seen = behind | (r >= 0.46)
    if camera:
        seen |= (x >= 0.28) & (r <= 1.0)
    return np.where(seen, r, np.inf)


OLD_RULE = {'fit_min_span': 0.0, 'fit_max_slope_deg': None, 'wall_dist': None}   # 2026-11-13


@pytest.mark.parametrize('camera', [False, True])
@pytest.mark.parametrize('old_rule', [True, False])
def test_run4_false_corner(camera, old_rule):
    """
    The 11-13 fault: 7 points within 5 cm behind the car made a wall that 'ends' there.

    Only the old rule without the camera turns: the fit checks (span >= 0.10 m) reject the
    bunched points, and the camera's wall bottoms continue the wall ahead.
    """
    policy = CornerPolicy('cw', corner_ahead=0.15, **(OLD_RULE if old_rule else {}))
    policy.on_odom(0.0, 0.0, 0.0)
    label, _ = policy(run4_scan(camera), 0.0, INC)
    if old_rule and not camera:
        assert label == FULL_RIGHT
    else:
        assert label != FULL_RIGHT and policy.state == FOLLOW and not policy.approach


def test_fit_checks_keep_a_real_corner():
    """Inner wall seen from x -0.25 to its end at 0.10 m, open beyond: still a corner."""
    walls = [(-1.0, -0.20, 0.10, -0.20), (0.10, -0.20, 0.10, -1.0),
             (-1.0, 0.22, 0.70, 0.22), (0.70, 0.22, 0.70, -1.0), (-1.0, -0.62, 2.0, -0.62)]
    policy = CornerPolicy('cw', corner_ahead=0.15)
    policy.on_odom(0.0, 0.0, 0.0)
    assert policy(wall_scan(walls), 0.0, INC)[0] == FULL_RIGHT


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_one_lap_with_the_measured_blind_edge_and_the_camera(direction):
    ok, why, laps, clear, policy = drive_laps(direction, partial=True, camera=True)
    assert ok, (why, laps)
    assert clear > 0.02 and policy.corners == 4
