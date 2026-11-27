"""wall.py: wall bottoms, floor projection, ranges; the node stays read-only."""
import ast
import math
import os
import re

import cv2
import numpy as np
import pytest
from wro2026_camera_c70.wall import (FloorCamera, lidar_pitch_fix, rotation, to_ranges,
                                     wall_bottom)

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PKG, 'config')


def real_camera():
    return FloorCamera.from_files(os.path.join(CONFIG, 'c70_ost_1280x720.yaml'),
                                  os.path.join(CONFIG, 'c70_extrinsics.yaml'))


def plain_camera(pitch=0.0, yaw=0.0):
    """No distortion, f 800 px, centre (640, 360), lens 0.1 m high at base_link x 0.2."""
    return FloorCamera([800, 0, 640, 0, 800, 360, 0, 0, 1], [0, 0, 0, 0, 0], 0.1, pitch, 0.0,
                       yaw, 0.2)


def test_rotation_looks_forward():
    assert np.allclose(rotation(0, 0, 0) @ [0, 0, 1], [1, 0, 0])     # optical axis -> +x
    assert np.allclose(rotation(0, 0, 0) @ [1, 0, 0], [0, -1, 0])    # image right -> -y
    assert rotation(10, 0, 0) @ [0, 0, 1] @ [0, 0, 1] < 0             # pitch + = looks down
    assert rotation(0, 0, 10) @ [0, 0, 1] @ [0, 1, 0] > 0             # yaw + = turned left


def test_floor_point_straight_ahead():
    pts = plain_camera().floor_points([(640, 460), (640, 360), (640, 200)])
    assert len(pts) == 1                                  # horizon and sky are dropped
    assert pts[0] == pytest.approx([0.2 + 0.1 * 800 / 100, 0.0])


def test_floor_point_to_the_left():
    x, y = plain_camera().floor_points([(540, 460)])[0]  # 100 px left of the centre
    assert x == pytest.approx(1.0) and y == pytest.approx(0.8 * 100 / 800)


def test_calibration_files_load():
    cam = real_camera()
    assert cam.size == (1280, 720)
    assert cam.height == pytest.approx(0.1029) and cam.x == pytest.approx(0.20)
    assert cam.y == 0.0                                   # y_from_base_link_m: UNKNOWN


def floor_and_wall(rows, h=720, w=1280):
    """Bright floor below rows[u], dark wall from rows[u] upwards (per column)."""
    img = np.full((h, w), 200, dtype=np.uint8)
    v = np.arange(h)[:, None]
    img[v < np.asarray(rows)[None, :]] = 30
    return img


def test_wall_bottom_finds_the_lowest_dark_row():
    px = wall_bottom(floor_and_wall(np.full(1280, 400)))
    assert len(px) == 320 and np.all(px[:, 1] == 399)
    assert np.array_equal(px[:, 0], np.arange(2, 1280, 4))


def test_wall_bottom_skips_dark_bottom_and_bright_columns():
    rows = np.full(1280, 400)
    rows[:640] = 720           # left half: dark down to the bottom edge (wall too close)
    rows[640:960] = 0          # bright all the way up (no wall)
    px = wall_bottom(floor_and_wall(rows))
    assert len(px) and px[:, 0].min() > 960


def test_wall_bottom_ignores_thin_dark_lines():
    img = floor_and_wall(np.full(1280, 300))
    img[500:505] = 30          # a 5 px dark line on the floor is not a wall (run 12)
    assert np.all(wall_bottom(img)[:, 1] == 299)


def test_to_ranges_beams_like_scan():
    lx = 0.13
    pts = [(lx + 1.0, 0.0), (lx, 0.5), (lx + 0.3, -0.3),
           (lx - 0.2, 0.0), (lx - 0.2005, 0.0), (lx - 0.21, 0.0),   # one beam: median
           (lx + 3.0, 0.0)]                                         # beyond max_range
    r = to_ranges(pts, lx, 480, 2.0)
    assert r[0] == pytest.approx(1.0)
    assert r[120] == pytest.approx(0.5)                     # 90 deg = left
    assert r[420] == pytest.approx(math.hypot(0.3, 0.3))    # -45 deg = front right
    assert r[240] == pytest.approx(0.2005)
    assert np.isfinite(r).sum() == 4


def test_a_wall_ahead_comes_back_at_its_distance():
    """Draw the wall bottom with the real calibration (distortion too), find it again."""
    cam = real_camera()
    w, h = cam.size
    wall_x = 0.80                                          # wall across the floor, base_link
    # inside the 76 deg view: the distortion polynomial folds points far outside it back in
    y = np.linspace(-0.45, 0.45, 2001)
    base = np.c_[np.full_like(y, wall_x), y, np.full_like(y, -cam.height)]
    in_cam = (base - [cam.x, cam.y, 0.0]) @ cam.rot        # base_link -> camera axes
    uv = cv2.projectPoints(in_cam, np.zeros(3), np.zeros(3), cam.k, cam.d)[0].reshape(-1, 2)
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < w)
    rows = np.full(w, h, dtype=float)
    np.minimum.at(rows, uv[ok, 0].astype(int), uv[ok, 1])
    img = floor_and_wall(np.ceil(rows).astype(int), h, w)
    pts = cam.floor_points(wall_bottom(img))
    assert len(pts) > 200
    assert np.median(pts[:, 0]) == pytest.approx(wall_x, abs=0.01)
    assert np.all(np.abs(pts[:, 0] - wall_x) < 0.03)


def test_read_only():
    tree = ast.parse(open(os.path.join(PKG, 'wro2026_camera_c70', 'wall_node.py'),
                          encoding='utf-8').read())
    topics, subs = [], []
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [a.name for a in node.names] + [getattr(node, 'module', None) or '']
            assert not any('serial' in n for n in names)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr not in ('write', 'send')
            if node.func.attr == 'create_publisher':
                topics.append(node.args[1].value)
            if node.func.attr == 'create_subscription':     # lidar_fix: /scan only
                subs.append((node.args[0].id, node.args[1].value))
    assert sorted(topics) == ['camera/image/compressed', 'camera/wall_scan', 'diagnostics']
    assert subs == [('LaserScan', 'scan')]


def test_parameters_match_yaml():
    tree = ast.parse(open(os.path.join(PKG, 'wro2026_camera_c70', 'wall_node.py'),
                          encoding='utf-8').read())
    params = next(n.value for n in tree.body
                  if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == 'PARAMETERS')
    names = {item.elts[0].value: item.elts[1].value for item in params.elts}
    yaml_vals = {}
    for line in open(os.path.join(CONFIG, 'c70_wall.yaml'), encoding='utf-8'):
        m = re.match(r'^\s{4}(\w+):\s*([^#]*?)\s*(#.*)?$', line)
        if m and m.group(1) != 'ros__parameters':
            yaml_vals[m.group(1)] = m.group(2)
    assert set(yaml_vals) == set(names)
    for key, default in names.items():
        raw = yaml_vals[key]
        if isinstance(default, bool):
            assert raw == str(default).lower(), key
        elif isinstance(default, str):
            assert raw.strip("'") == default, key
        else:
            assert float(raw) == pytest.approx(float(default)), key


def test_beige_tape_on_the_wall_is_not_floor():
    """Columns 0-639: a bright beige patch from the bottom edge up to row 500, wall above."""
    rows = np.full(1280, 400)
    rows[:640] = 500
    img = cv2.cvtColor(floor_and_wall(rows), cv2.COLOR_GRAY2BGR)
    img[500:, :640] = (110, 160, 195)               # beige (BGR): grey ~165, saturation ~111
    px = wall_bottom(img)
    assert len(px) and px[:, 0].min() > 640 and np.all(px[:, 1] == 399)
    assert len(wall_bottom(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))) == 320   # grey: no check


def lidar_of(points, bins=480, laser_x=0.13):
    """/scan-like ranges of floor points (base_link): the nearest point per beam."""
    ranges = np.full(bins, np.inf)
    for x, y in points:
        x -= laser_x
        k = int(round((math.atan2(y, x) % (2 * math.pi)) / (2 * math.pi / bins))) % bins
        ranges[k] = min(ranges[k], math.hypot(x, y))
    return ranges


def test_lidar_pitch_fix_finds_the_tilt():
    """
    The car leans 1 deg forward (the camera looks 1 deg further down than calibrated).

    The lidar sees the far wall where it is; the fix finds the 1 deg from those points and
    the near points then land where they really are.
    """
    truth, model = plain_camera(pitch=1.0), plain_camera(pitch=0.0)
    u = np.arange(200.0, 1081.0, 8.0)
    far = np.c_[u, np.full(len(u), 470.0)]           # 0.6-0.9 m from the lidar
    near = np.c_[u, np.full(len(u), 560.0)]          # ~0.4 m
    change, pairs = lidar_pitch_fix(model, far, lidar_of(truth.floor_points(far)), 0.13, 480)
    assert pairs >= 8 and change == pytest.approx(1.0, abs=0.1)
    assert np.abs(model.floor_points(near) - truth.floor_points(near)).max() > 0.01
    assert np.abs(model.floor_points(near, change) - truth.floor_points(near)).max() < 0.005


def test_lidar_pitch_fix_needs_the_same_wall():
    """No lidar, or the lidar sees another surface 30 cm further: no correction."""
    truth, model = plain_camera(pitch=1.0), plain_camera(pitch=0.0)
    u = np.arange(200.0, 1081.0, 8.0)
    far = np.c_[u, np.full(len(u), 470.0)]
    assert lidar_pitch_fix(model, far, np.full(480, np.inf), 0.13, 480) == (0.0, 0)
    other = lidar_of(truth.floor_points(far)) + 0.30
    assert lidar_pitch_fix(model, far, other, 0.13, 480) == (0.0, 0)
