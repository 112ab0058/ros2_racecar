"""Ray's corner technique (corner.py) on the 2-D field: laps, safety rules, blind lidar."""
import math

import numpy as np
import pytest
from wro2026_imitation.corner import (BLOCKED, CornerPolicy, DONE, FOLLOW, FULL_LEFT,
                                      FULL_RIGHT, HOLD, REWAIT, STOP as CORNER_STOP, TURN)
from wro2026_imitation.features import LEFT, RIGHT, STRAIGHT
from wro2026_imitation.field_sim import camera as camera_view, Car, Field, lidar
from wro2026_imitation.policy import Driver, NO_ODOM, OK, STOP
from wro2026_lidar_n10p.scan_memory import ScanMemory

INC = 2.0 * math.pi / 480


def start_pose(direction, island=0.80, start=0.05, lat=0.0, head=0.0):
    """
    Return the rear-axle pose (x, y, yaw) on the bottom straight.

    start m past the island's corner, lat m and head deg towards the island (inner side).
    """
    y = -(0.8 + island / 2.0) / 2.0 + lat
    if direction == 'ccw':
        return -island / 2.0 + start, y, math.radians(head)
    return island / 2.0 - start, y, math.pi - math.radians(head)


def drive_laps(direction, laps=1, island=0.80, memory=True, seed=0, policy_direction=None,
               partial=False, camera=False, car=None, start=0.05, lat=0.0, head=0.0,
               cam_max=1.0, predict=False, **policy_kw):
    """
    Drive the corner policy around the field; return (ok, why, laps, min clearance, policy).

    car: Car keyword arguments (default: the measured car, 0.403 m left / 0.292 m right);
    start, lat, head: start pose (start_pose); cam_max: the camera's max_range (m);
    policy_kw: CornerPolicy keyword arguments. predict: orin_auto.sh since 2026-11-26
    (stop when the path ahead touches, margins 0); else the outline + 2 cm rule.
    """
    field = Field(1.6, island)
    car = Car(*start_pose(direction, island, start, lat, head), **(car or {}))
    policy = CornerPolicy(policy_direction or direction, **policy_kw)
    driver = Driver(policy, 0.1, 0.5, stop_margin=0.0 if predict else 0.02,
                    full_turn_radius=0.35, predict=predict)
    mem = ScanMemory()
    rng = np.random.default_rng(seed)
    dt, t, vx, wz, turned, last, clear = 0.02, 0.0, 0.0, 0.0, 0.0, car.yaw, 9.0
    while t < laps * 50.0:
        if int(round(t / dt)) % 5 == 0:                       # odometry and lidar at 10 Hz
            driver.on_odom(t, car.x, car.y, car.yaw, vx, vx * car.curv)   # EKF twist
            mem.on_odom(car.x, car.y, car.yaw)
            if camera:
                mem.add_camera(t, camera_view(field, car, rng=rng, max_range=cam_max), 0.0, INC)
            r = lidar(field, car, rng=rng, partial=partial)
            remembered = None
            if memory:
                r, remembered = mem.on_scan(t, r, 0.0, INC)
            # as orin_auto.sh: stop_margin 0.02 for measured points, memory_margin 0 for
            # remembered ones (memory_marks): the camera reads 1.8 cm short, so a
            # remembered wall can sit 2-3 cm inside the real one
            driver.on_scan(t, r, 0.0, INC, remembered)
            vx, wz, decision, reason = driver.command(t)
            if decision == STOP:
                return False, reason, abs(turned) / (2 * math.pi), clear, policy
        car.step(vx, wz, dt)
        turned += car.yaw - last          # net heading: small corrections cancel out
        last = car.yaw
        clear = min(clear, field.clearance(car.x, car.y, car.yaw))
        if clear < 0:
            return False, 'crash', abs(turned) / (2 * math.pi), clear, policy
        if abs(turned) >= laps * 2 * math.pi:
            return True, 'done', abs(turned) / (2 * math.pi), clear, policy
        t += dt
    return False, 'too slow', abs(turned) / (2 * math.pi), clear, policy


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
    # 5: the next lap's first corner can lock just before the net heading reaches 360 deg
    assert policy.direction == direction and policy.corners in (4, 5) and clear > 0.02


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


def test_without_memory_only_the_silhouette_sees_the_wall_end():
    """
    No memory: the wall's own points never show its end in time (old rule: the run ends).

    The beams past the end (crossing_end) still show it: a lap passes on them alone
    (2026-11-24). The memory stays on: the too_close stop needs the near walls.
    """
    ok, why, laps, clear, policy = drive_laps('ccw', memory=False, crossing=False)
    assert not ok and why in ('too_close', 'crash') and laps < 0.5
    ok, why, laps, clear, policy = drive_laps('ccw', memory=False)
    assert ok and clear > 0.01, (why, laps)


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


OLD_RULE = {'fit_min_span': 0.0, 'fit_max_slope_deg': None, 'wall_dist': None,     # 11-13
            'crossing': False}


@pytest.mark.parametrize('camera', [False, True])
@pytest.mark.parametrize('old_rule', [True, False])
def test_run4_false_corner(camera, old_rule):
    """
    The 11-13 fault: 7 points within 5 cm behind the car made a wall that 'ends' there.

    Only the old rule without the camera turns: the fit checks (span >= 0.10 m) reject the
    bunched points, and the camera's wall bottoms continue the wall ahead.
    """
    policy = CornerPolicy('cw', timing='ahead', corner_ahead=0.15,
                          **(OLD_RULE if old_rule else {}))
    policy.on_odom(0.0, 0.0, 0.0)
    label, _ = policy(run4_scan(camera), 0.0, INC)
    if old_rule and not camera:
        assert label == FULL_RIGHT
    else:
        assert label != FULL_RIGHT and policy.state == FOLLOW and not policy.approach


@pytest.mark.parametrize('crossing', [False, True])
def test_where_the_lidar_stops_seeing_the_wall_is_not_its_end(crossing):
    """
    2-D field 11-24 (cw, start 0.56 m, 5 cm from the outer wall, 10 deg into the island).

    The black inner wall answered only with points 0.10 m long behind the car, at the
    edge of what the lidar sees; it goes on, blind, to 0.30 m ahead. The span and
    distance checks pass; only the beams that went through the line tell: none do just
    past that "end", so with crossing it is not one (the real end is found from them).
    """
    walls = [(-1.0, -0.26, 0.30, -0.26), (0.30, -0.26, 0.30, -1.2),
             (-1.0, 0.14, 0.70, 0.14), (0.70, 0.14, 0.70, -1.2), (-1.0, -1.06, 2.0, -1.06)]
    r = wall_scan(walls)
    x, y = r * np.cos(np.arange(480) * INC), r * np.sin(np.arange(480) * INC)
    edge = (y < -0.2) & (y > -0.3) & (x >= -0.25) & (x <= -0.13)    # the edge points
    r = np.where(edge | (r >= 0.47), r, np.inf)
    ang = (np.arange(480) * INC + math.pi) % (2 * math.pi) - math.pi
    policy = CornerPolicy('cw', crossing=crossing)
    end = policy.inner_wall_end(r, ang, min_span=policy.fit_min_span,
                                max_slope=policy.fit_max_slope, wall_dist=policy.wall_dist)
    if crossing:
        assert end == pytest.approx(0.30, abs=0.02)
    else:
        assert end == pytest.approx(-0.13, abs=0.02)


def test_fit_checks_keep_a_real_corner():
    """Inner wall seen from x -0.25 to its end at 0.10 m, open beyond: still a corner."""
    walls = [(-1.0, -0.20, 0.10, -0.20), (0.10, -0.20, 0.10, -1.0),
             (-1.0, 0.22, 0.70, 0.22), (0.70, 0.22, 0.70, -1.0), (-1.0, -0.62, 2.0, -0.62)]
    policy = CornerPolicy('cw', timing='ahead', corner_ahead=0.15)
    policy.on_odom(0.0, 0.0, 0.0)
    assert policy(wall_scan(walls), 0.0, INC)[0] == FULL_RIGHT


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_one_lap_with_the_measured_blind_edge_and_the_camera(direction):
    ok, why, laps, clear, policy = drive_laps(direction, partial=True, camera=True)
    assert ok, (why, laps)
    # 5: the next lap's first corner can lock just before the net heading reaches 360 deg
    assert clear > 0.02 and policy.corners in (4, 5)


def rotate(walls, deg):
    """Walls (x0, y0, x1, y1) turned by deg about the lidar (the car yawed by -deg)."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [(c * x0 - s * y0, s * x0 + c * y0, c * x1 - s * y1, s * x1 + c * y1)
            for x0, y0, x1, y1 in walls]


@pytest.mark.parametrize('max_slope, turns', [(20.0, False), (40.0, True)])
def test_a_car_still_yawed_after_a_turn_sees_the_corner(max_slope, turns):
    """11-20 field run 3: 25 deg off the next wall; a 20 deg limit locked 6.5 cm late."""
    walls = [(-1.0, -0.20, 0.10, -0.20), (0.10, -0.20, 0.10, -1.0),
             (-1.0, 0.22, 0.70, 0.22), (0.70, 0.22, 0.70, -1.0), (-1.0, -0.62, 2.0, -0.62)]
    policy = CornerPolicy('cw', timing='ahead', corner_ahead=0.15,
                          fit_max_slope_deg=max_slope)
    policy.on_odom(0.0, 0.0, 0.0)
    label, _ = policy(wall_scan(rotate(walls, -25.0)), 0.0, INC)
    assert (label == FULL_RIGHT) == turns


@pytest.mark.parametrize('ahead', [0.10, 0.04])
def test_one_corner_ahead_cannot_serve_both_ways(ahead):
    """
    One fixed corner_ahead cannot pass both ways on the asymmetric car.

    The car turns 0.403 m to the left and 0.292 m to the right. Sweep in this field
    (2026-11-23, corner_ahead -0.04 to 0.20): a lap ccw passes with 0.08-0.12, cw with
    0.00-0.06; no value passes both. On the 2026-11-20 field 0.14 passed ten left corners
    and cut the first right one (the field's numbers differ by a few cm: wall ends are
    seen differently there). timing 'predict' passes both (next test).
    """
    passed = [drive_laps(d, timing='ahead', corner_ahead=ahead)[0] for d in ('ccw', 'cw')]
    assert passed == ([True, False] if ahead > 0.07 else [False, True])


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_predicted_turn_both_ways_on_the_asymmetric_car(direction):
    """Timing 'predict' (the default) passes both ways: each side has its own radius."""
    ok, why, laps, clear, policy = drive_laps(direction, laps=2)
    assert ok, (why, laps)
    assert clear > 0.02 and policy.corners == 8


def test_reset_starts_again_in_follow_and_keeps_direction_and_corners():
    """After the phone's 重置 the corner rule follows again; direction and count stay."""
    p = CornerPolicy('cw')
    p.on_odom(0.0, 0.0, 0.0)
    p.on_odom(0.5, 0.0, 0.0)
    p._enter(TURN)                                  # stopped half way through a corner
    p.last_label = FULL_RIGHT
    p.reset()
    assert (p.state, p.corners, p.direction, p.last_label) == (FOLLOW, 1, 'cw', STRAIGHT)
    assert p.events[-1] == (0.5, 'reset') and p.armed_odo is None and p.sent == []


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_stops_at_the_start_after_the_set_laps(direction):
    """
    Stop_after_laps 1: four corners, then STOP on the start straight near the start.

    The field's start is 5 cm past the island corner, so the ccw lap's last turn ends
    past it and the car stops right after the turn; the start point must be within
    5 cm ahead to 15 cm behind, within 8 cm of the corridor's middle.
    """
    ok, why, laps, clear, policy = drive_laps(direction, laps=1.5, stop_after_laps=1)
    assert (ok, why, policy.state, policy.corners) == (False, OK, DONE, 4), (why, laps)
    (x, y, _), (sx, sy, syaw) = policy.pose, policy.start
    ahead = (sx - x) * math.cos(syaw) + (sy - y) * math.sin(syaw)
    side = -(sx - x) * math.sin(syaw) + (sy - y) * math.cos(syaw)
    assert -0.15 <= ahead <= 0.05 and abs(side) < 0.08 and clear > 0.02


def late_start_scan(end=0.12, lat=0.20, visible=0.47):
    """
    Lidar frame scan of a start in the second half of a straight (ccw, inner wall left).

    The school field's shape (corridor 0.40 m, island 0.80 m): the island's wall (y = lat)
    ends end m ahead of the lidar, its next side runs left; the wall ahead (x = end + 0.40)
    runs on past the island as the next corridor's outer wall, up to the far wall.
    Black walls answer only from visible m (the 2026-11-13 measurement): the island's end
    and the side walls near the car are blind, only the walls far behind and ahead return
    (on 11-13 also at 1.4 m: intensity 37-47).
    """
    walls = [(end - 0.80, lat, end, lat), (end, lat, end, lat + 0.80),   # island
             (-1.2, lat - 0.40, end + 0.40, lat - 0.40),                 # outer wall
             (end + 0.40, lat - 0.40, end + 0.40, lat + 1.20),           # wall ahead
             (-1.2, lat + 1.20, end + 0.40, lat + 1.20)]                 # far wall
    r = wall_scan(walls)
    return np.where(r >= visible, r, np.inf)


@pytest.mark.parametrize('end', [0.05, 0.12, 0.25])
def test_crossing_finds_the_island_end_in_the_blind_spot(end):
    """No point of the island near its end; the beams past it give the end within 2 cm."""
    policy = CornerPolicy('ccw')
    r = late_start_scan(end)
    ang = (np.arange(480) * INC + math.pi) % (2 * math.pi) - math.pi
    assert policy.walk_end(r, ang) is None                     # the old rule sees nothing
    found = policy.inner_wall_end(r, ang)
    assert found == pytest.approx(end, abs=0.02) and policy.end_line is not None


def test_crossing_needs_beams_past_the_wall_line():
    """A wall that goes on (blind near the car, seen far behind and ahead) has no end."""
    walls = [(-1.0, 0.2, 2.0, 0.2), (-1.0, -0.2, 2.0, -0.2)]
    r = wall_scan(walls)
    r = np.where(r >= 0.47, r, np.inf)
    ang = (np.arange(480) * INC + math.pi) % (2 * math.pi) - math.pi
    policy = CornerPolicy('ccw')
    assert policy.inner_wall_end(r, ang) is None and policy.end_line is None


@pytest.mark.parametrize('flip, expected', [(1.0, 'ccw'), (-1.0, 'cw')])
def test_auto_direction_from_a_late_start(flip, expected):
    """Only the island's side has beams past its line: that side is the inside."""
    walls_r = late_start_scan(0.12)
    if flip < 0:
        walls_r = np.roll(walls_r[::-1], 1)                    # mirror left / right
    policy = CornerPolicy('auto')
    policy.on_odom(0.0, 0.0, 0.0)
    for _ in range(3):
        policy(walls_r, 0.0, INC)
    assert policy.direction == expected


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
@pytest.mark.parametrize('start', [0.45, 0.56])
def test_late_start_drives_the_corners(direction, start):
    """
    WRO 2026 lets the car start anywhere in the straight (rule 9.7, six zones).

    0.56 m: the front bumper at the straight's end. Without crossing_end the island's
    end stays in the blind spot and the car drives into the first corner (11-24 sweep).
    """
    ok, why, laps, clear, policy = drive_laps(direction, start=start, partial=True,
                                              camera=True, cam_max=0.5)
    assert ok, (why, laps, policy.events[:3])
    assert clear > 0.01
    ok, why, *_ = drive_laps(direction, start=start, partial=True, camera=True, cam_max=0.5,
                             crossing=False)
    assert not ok


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_late_start_stops_inside_the_start_straight(direction):
    """After the laps the whole car is in the start straight (WRO 9.24.2), wheels rolled."""
    ok, why, laps, clear, policy = drive_laps(direction, laps=1.5, stop_after_laps=1,
                                              start=0.56, partial=True, camera=True,
                                              cam_max=0.5)
    assert (why, policy.state, policy.corners) == (OK, DONE, 4), (why, laps)
    x, y, yaw = policy.pose
    along = x + 0.40 if direction == 'ccw' else 0.40 - x       # past the island's corner
    c = math.cos(yaw) if direction == 'ccw' else -math.cos(yaw)
    front = along + 0.24 * c + 0.05                           # + ~5 cm still rolling
    assert along - 0.031 > 0.0 and front < 0.80


def test_laps_done_needs_the_corners_the_start_heading_and_the_start_point():
    assert CORNER_STOP == STOP
    p = CornerPolicy('ccw', stop_after_laps=1)
    p.on_odom(0.0, 0.0, 0.0)                         # the start
    p.on_odom(-0.02, 0.0, 0.0)
    p.corners = 3
    assert not p.laps_done()                         # one corner missing
    p.corners = 4
    assert p.laps_done()                             # start 2 cm ahead
    p.on_odom(-0.02, 0.0, math.radians(90.0))
    assert not p.laps_done()                         # another straight's heading
    p.on_odom(-0.20, 0.0, 0.0)
    assert not p.laps_done()                         # start still 20 cm ahead
    assert not CornerPolicy('ccw').laps_done()       # 0 = never


@pytest.mark.parametrize('start', [0.39, 0.45])
def test_a_turn_that_cannot_fit_stops_instead_of_hitting(start):
    """
    2026-11-26 field run 1: 4 cm from the island late in a 0.39 m straight (ccw).

    No full lock fits (room 0.006-0.007 m): the rule stops (blocked) before the turn,
    counts no corner, and touches nothing; turning anyway (min_room < 0, the rule before)
    latches too_close.
    """
    kw = {'island': 0.82, 'start': start, 'lat': 0.065, 'partial': True, 'camera': True,
          'cam_max': 0.5}
    ok, why, laps, clear, policy = drive_laps('ccw', **kw)
    assert (ok, why, policy.state, policy.corners) == (False, OK, BLOCKED, 0), policy.events
    assert clear > 0.03 and 'does not fit' in policy.events[-1][1]
    ok, why, *_ = drive_laps('ccw', min_room=-1.0, **kw)
    assert (ok, why) == (False, 'too_close')


def test_blocked_follows_again_when_no_corner_is_ahead():
    """Moved by hand to where no wall end is ahead: follow again and drive."""
    policy = CornerPolicy('ccw')
    policy.on_odom(0.0, 0.0, 0.0)
    policy._enter(BLOCKED)
    r = wall_scan([(-1.0, 0.2, 2.0, 0.2), (-1.0, -0.2, 2.0, -0.2)])
    label, _ = policy(r, 0.0, INC)
    assert policy.state == FOLLOW and label != CORNER_STOP
    assert policy.corners == 0 and policy.events[-1][1].startswith('follow')


@pytest.mark.parametrize('direction', ['ccw', 'cw'])
def test_predicted_stop_on_the_narrow_field(direction):
    """
    The 2026-11-26 field: 0.38 m corridor, start 6 cm towards the outer wall (orin_auto.sh).

    Two laps with no stop; the car passes the walls closer than the old 2 cm margin.
    """
    ok, why, laps, clear, policy = drive_laps(direction, laps=2, island=0.84, start=0.40,
                                              lat=-0.06, partial=True, camera=True,
                                              cam_max=0.5, predict=True)
    assert ok, (why, laps, policy.events[-3:])
    assert clear > 0.005


def turn_scan(end, gap=0.08, corridor=0.38):
    """
    Return a lidar scan in a right (cw) corner.

    The island gap m right of the car, ending end m ahead of the rear axle; the outer
    wall on the left; the wall ahead corridor m past the island's end.
    """
    a = 0.09 + gap
    walls = [(-0.8, -a, end, -a), (end, -a, end, -a - 0.8),
             (-0.8, corridor - a, end + corridor + 0.3, corridor - a),
             (end + corridor, corridor - a, end + corridor, -a - 1.2)]
    return wall_scan([(x0 - 0.13, y0, x1 - 0.13, y1) for x0, y0, x1, y1 in walls])


def turning_cw():
    """Return a cw policy that has just gone to full lock (corner 1)."""
    policy = CornerPolicy('cw')
    policy.on_odom(0.0, 0.0, 0.0)
    policy._enter(TURN)
    policy.last_label = FULL_RIGHT
    policy.sent = [(0.0, FULL_RIGHT)]
    return policy


@pytest.mark.parametrize('end, state', [(0.30, REWAIT), (0.25, REWAIT), (0.20, TURN),
                                        (0.10, TURN)])
def test_a_turn_that_will_touch_the_island_straightens_first(end, state):
    """
    Ray 2026-11-26 (cw): locked a little early, the island's corner came inside the turn.

    The rest of the turn is predicted every scan: if it touches the island first, drive
    straight on (rewait) instead of turning into it; 20 cm or less ahead it fits (the car
    8 cm from the island, a 0.38 m corridor).
    """
    policy = turning_cw()
    label, _ = policy(turn_scan(end), 0.0, INC)
    assert policy.state == state and policy.corners == 1
    assert label == (STRAIGHT if state == REWAIT else FULL_RIGHT)


def test_rewait_locks_again_on_the_same_corner():
    """After ~0.14 m straight on the turn fits: full lock again, still corner 1."""
    policy = turning_cw()
    policy(turn_scan(0.30), 0.0, INC)
    assert policy.state == REWAIT
    for d in np.arange(0.01, 0.205, 0.01):           # 1 cm per scan, straight on
        policy.on_odom(d, 0.0, 0.0)
        label, _ = policy(turn_scan(0.30 - d), 0.0, INC)
        if policy.state == TURN:
            break
    assert policy.state == TURN and label == FULL_RIGHT and policy.corners == 1
    assert 'resumed' in policy.events[-1][1]


def test_rewait_holds_when_nothing_fits():
    """Driving on never fits (rewait_max reached): STOP and wait (hold)."""
    policy = turning_cw()
    policy.rewait_max = 0.0
    policy(turn_scan(0.30), 0.0, INC)
    policy.on_odom(0.01, 0.0, 0.0)
    label, _ = policy(turn_scan(0.29), 0.0, INC)
    assert policy.state == HOLD and label == CORNER_STOP


def test_the_driver_lets_the_rule_straighten_before_it_checks():
    """Predicted stop: the rule's own correction is checked, so no too_close latch."""
    policy = turning_cw()
    driver = Driver(policy, 0.1, 0.5, stop_margin=0.0, full_turn_radius=0.35, predict=True)
    driver.odom_time = 0.0
    driver.sent = [(0.0, FULL_RIGHT)]
    driver.on_scan(0.0, turn_scan(0.30), 0.0, INC)
    vx, wz, decision, reason = driver.command(0.05)
    assert (reason, decision, policy.state) == (OK, STRAIGHT, REWAIT)
