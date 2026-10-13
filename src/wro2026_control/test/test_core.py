"""Unit and closed-loop tests for the pose-free race controller.

Unit tests need only numpy and OpenCV.  The closed-loop tests drive the
controller in the offline 2D simulator (needs shapely) on the real field
SDF; they are skipped when shapely is missing (e.g. on the car).

    python3 -m pytest -q src/wro2026_control/test
"""

import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))

from wro2026_control.core import (CCW, CW, Command, Params, RaceController,  # noqa: E402
                                  Scan, State, wrap)

try:
    import sim2d  # noqa: E402
    HAVE_SIM = True
except ImportError:          # shapely not installed
    HAVE_SIM = False


def corridor_scan(left=0.285, right=0.285, ahead=1.2, n=360, rmax=3.5):
    """Scan of a straight corridor (robot frame) with a wall ahead."""
    ranges = []
    for i in range(n):
        a = wrap(2 * math.pi * i / n)
        best = math.inf
        c, s = math.cos(a), math.sin(a)
        if s > 1e-6:
            best = min(best, left / s)
        if s < -1e-6:
            best = min(best, -right / s)
        if c > 1e-6:
            best = min(best, ahead / c)
        ranges.append(best if best < rmax else math.inf)
    return Scan(angle_min=0.0, angle_increment=2 * math.pi / n, ranges=ranges,
                range_min=0.12, range_max=rmax)


# ---------------------------------------------------------------- unit tests
def test_wrap():
    assert wrap(3 * math.pi) == pytest.approx(math.pi)
    assert wrap(-0.5) == pytest.approx(-0.5)


def test_no_motion_before_start():
    c = RaceController()
    cmd = c.step(0.0, 0.0, 0.0, corridor_scan())
    assert (cmd.linear, cmd.angular) == (0.0, 0.0)
    assert c.state is State.WAIT


def test_drives_forward_centred_in_a_corridor():
    c = RaceController()
    c.start()
    s = 0.0
    for k in range(10):
        cmd = c.step(0.05 * k, 0.0, s, corridor_scan())
        s += cmd.linear * 0.05
    assert c.state is State.RUN
    assert cmd.linear > 0.05
    assert abs(cmd.angular) < 0.05


def test_steers_back_to_the_lane_centre():
    c = RaceController()
    c.start()
    cmd = None
    for k in range(4):
        # robot 8 cm left of centre: should turn right (negative angular)
        cmd = c.step(0.05 * k, 0.0, 0.0, corridor_scan(left=0.205, right=0.365))
    assert cmd.angular < -0.05


def test_direction_from_first_line_colour():
    c = RaceController()
    c.start()
    c.step(0.0, 0.0, 0.0, corridor_scan())
    c.step(0.05, 0.0, 0.0, corridor_scan(), orange=True)
    c.step(0.10, 0.0, 0.0, corridor_scan(), orange=True, blue=True)
    assert c.direction == CW
    c2 = RaceController()
    c2.start()
    c2.step(0.0, 0.0, 0.0, corridor_scan())
    c2.step(0.05, 0.0, 0.0, corridor_scan(), blue=True)
    assert c2.direction == CCW


def test_both_lines_at_once_do_not_decide():
    c = RaceController()
    c.start()
    c.step(0.0, 0.0, 0.0, corridor_scan())
    c.step(0.05, 0.0, 0.0, corridor_scan(), orange=True, blue=True)
    assert c.direction == 0


def test_never_drives_into_a_wall_ahead():
    c = RaceController()
    c.start()
    cmd = Command()
    for k in range(20):
        cmd = c.step(0.05 * k, 0.0, 0.0, corridor_scan(ahead=0.12))
    assert cmd.linear <= 0.0


def test_ackermann_curvature_limit():
    p = Params(min_turn_radius=0.3)
    c = RaceController(p)
    c.start()
    for k in range(6):
        cmd = c.step(0.05 * k, 0.0, 0.0, corridor_scan(left=0.205, right=0.365))
        if cmd.linear > 0:
            assert abs(cmd.angular) <= cmd.linear / 0.3 + 1e-9


def test_done_is_final():
    c = RaceController()
    c.start()
    c.step(0.0, 0.0, 0.0, corridor_scan())
    c.state = State.DONE
    cmd = c.step(0.05, 0.0, 0.0, corridor_scan())
    assert (cmd.linear, cmd.angular) == (0.0, 0.0)


def test_scan_points_use_the_scan_time_pose():
    c = RaceController()
    c.start()
    c.step(0.0, 0.0, 0.0, corridor_scan())
    # robot drives 0.1 m; a scan taken at t=0 must land where it was taken
    c.step(0.5, 0.0, 0.1, corridor_scan())
    scan = corridor_scan(ahead=1.0)
    scan.stamp = 0.0
    pts = c._scan_points(scan)
    ahead = pts[np.argmin(np.abs(pts[:, 1]))]
    assert ahead[0] == pytest.approx(1.0 + c.p.lidar_x, abs=0.01)


# --------------------------------------------------------- closed loop (2D)
SIM_CASES = [
    ("ccw", "none"), ("cw", "none"),
    ("ccw", "sdf"), ("cw", "sdf"),
]


@pytest.mark.skipif(not HAVE_SIM, reason="shapely not installed")
@pytest.mark.parametrize("direction,scenario", SIM_CASES)
def test_three_laps_in_the_2d_simulator(direction, scenario):
    start = (-0.05, -0.50, 0.0) if direction == "ccw" else (-0.05, -0.50, math.pi)
    world = sim2d.load_world(obstacles=sim2d.SCENARIOS[scenario])
    res = sim2d.run(world, sim2d.SimConfig(start=start))
    assert not res.collided
    assert res.done and res.corners == 12
    assert res.stop_offset < 0.08
