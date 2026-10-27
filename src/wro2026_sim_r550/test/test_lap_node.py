"""lap_node: the default path stays the same, the new path options and controllers behave."""
import math

import pytest
from wro2026_sim_r550.lap_node import (box_clearance, CONTROLLERS, curvature, island_clearance,
                                       LapController, lateral_error, make_path, nearest,
                                       obstacle_clearance, outer_clearance, plan_offsets, wrap)


def old_make_path(half_side, radius, step=0.01):
    """Return the path of lap_node before the 2026-10-27 campaign (reference)."""
    c = half_side - radius
    pts = []

    def line(x0, y0, x1, y1):
        n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
        pts.extend((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n))

    def arc(cx, cy, a0):
        n = max(1, int(round(radius * math.pi / 2 / step)))
        pts.extend((cx + radius * math.cos(a0 + math.pi / 2 * i / n),
                    cy + radius * math.sin(a0 + math.pi / 2 * i / n)) for i in range(n))

    line(-c, -half_side, c, -half_side)
    arc(c, -c, -math.pi / 2)
    line(half_side, -c, half_side, c)
    arc(c, c, 0.0)
    line(c, half_side, -c, half_side)
    arc(-c, c, math.pi / 2)
    line(-half_side, c, -half_side, -c)
    arc(-c, -c, math.pi)
    return pts


@pytest.mark.parametrize('half_side, radius', [(0.55, 0.42), (0.58, 0.42), (0.60, 0.45)])
def test_default_path_unchanged(half_side, radius):
    new, old = make_path(half_side, radius), old_make_path(half_side, radius)
    assert len(new) == len(old)
    assert max(math.dist(a, b) for a, b in zip(new, old)) < 1e-9


def heading(path, i):
    (x0, y0), (x1, y1) = path[i], path[(i + 1) % len(path)]
    return math.atan2(y1 - y0, x1 - x0)


def test_path_is_closed_and_smooth_with_options():
    for kw in ({}, {'transition': 0.15}, {'inset': 0.02}, {'half_side': 0.60, 'transition': 0.1},
               {'offsets': [0.40, 0.60, 0.55, 0.55]}):
        path = make_path(**kw)
        n = len(path)
        gaps = [math.dist(path[i], path[(i + 1) % n]) for i in range(n)]
        assert max(gaps) < 0.0105 and min(gaps) > 0.0095, kw
        turns = [wrap(heading(path, (i + 1) % n) - heading(path, i)) for i in range(n)]
        assert sum(turns) == pytest.approx(2 * math.pi, abs=1e-6), kw   # one CCW loop
        assert min(turns) > -1e-6, kw                                   # left turns only


def test_transition_ramps_the_curvature():
    plain, ramp = curvature(make_path()), curvature(make_path(transition=0.15))
    jump = max(abs(b - a) for a, b in zip(plain, plain[1:]))
    assert max(abs(b - a) for a, b in zip(ramp, ramp[1:])) < 0.5 * jump
    assert max(ramp) == pytest.approx(1 / 0.42, rel=0.02)


def test_straights_and_overlap():
    straights = []
    make_path(0.55, 0.42, straights=straights)
    assert straights == pytest.approx([0.26] * 4)
    straights = []
    make_path(0.55, 0.42, transition=0.10, straights=straights)
    assert all(s < 0.26 for s in straights)
    with pytest.raises(ValueError):
        make_path(offsets=[0.40, 0.40, 0.40, 0.40])      # corners would overlap


def test_inset_pulls_the_corner_exit_in():
    plain, inset = make_path(0.58, 0.42), make_path(0.58, 0.42, inset=0.02)

    def worst(path):
        return min(outer_clearance((x, y, heading(path, i))) for i, (x, y) in enumerate(path))
    assert worst(inset) > worst(plain) + 0.01


def test_plan_offsets():
    assert plan_offsets([], 0.55, 0.40, 0.60) == [0.55] * 4
    # outer half of the south lane -> pass inside; inner half of the east lane -> outside
    assert plan_offsets([(0.0, -0.60), (0.40, 0.0)], 0.55, 0.40, 0.60) == [0.40, 0.60, 0.55, 0.55]
    assert plan_offsets([(0.0, 0.62), (-0.38, 0.0)], 0.55, 0.40, 0.60) == [0.55, 0.55, 0.40, 0.60]


def test_clearances():
    pose = (0.0, -0.55, 0.0)                        # south lane heading east
    assert outer_clearance(pose) == pytest.approx(0.8 - 0.55 - 0.095)
    assert island_clearance(pose) == pytest.approx(0.55 - 0.095 - 0.2)
    assert box_clearance(pose, 0.0, -0.40, 0.025, 0.025) == pytest.approx(0.03)
    assert box_clearance(pose, 0.1, -0.55, 0.025, 0.025) == 0.0      # inside the car
    assert obstacle_clearance(pose, []) == float('inf')


def test_controllers_reject_unknown():
    with pytest.raises(ValueError):
        LapController(make_path(), 'magic')
    with pytest.raises(ValueError):
        LapController(make_path(), 'feedforward', k_q=1.0)


@pytest.mark.parametrize('controller', CONTROLLERS)
def test_every_controller_drives_two_laps_of_a_kinematic_car(controller):
    """
    A perfect kinematic car with a first-order steering lag must stay near the line.

    Never more than 3 cm toward the outer wall; stanley keeps the FRONT axle on the line, so
    its rear axle (base_link) cuts the corners on the island side by design.
    """
    path = make_path()
    ctl = LapController(path, controller, steer_tau=0.2)
    x, y, yaw, k_act, dt = 0.0, -0.55, 0.0, 0.0, 0.05
    ctl.start((x, y, yaw))
    lats, t = [], 0.0
    i = nearest(path, x, y)
    while ctl.progress < 2 * len(path) and t < 120.0:
        v, wz = ctl.step((x, y, yaw))
        assert abs(wz) <= v / 0.35 + 1e-9                    # minimum turn radius
        k_cmd = wz / v if v > 0.02 else 0.0
        for _ in range(10):                                  # 5 ms physics
            k_act += (k_cmd - k_act) * 0.005 / 0.2
            x += v * math.cos(yaw) * 0.005
            y += v * math.sin(yaw) * 0.005
            yaw += v * k_act * 0.005
        t += dt
        i = nearest(path, x, y, i)
        lats.append(lateral_error(path, i, x, y))
    assert ctl.progress >= 2 * len(path), controller
    assert min(lats) > -0.03, controller                     # + = island side
    assert max(lats) < (0.10 if controller == 'stanley' else 0.03), controller
