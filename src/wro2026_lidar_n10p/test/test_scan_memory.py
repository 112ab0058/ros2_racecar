"""ScanMemory: fill the near blind beams (black walls, 2026-11-11) from earlier returns."""
import math

import numpy as np
import pytest
from wro2026_lidar_n10p.scan_memory import PoseHistory, ScanMemory

N = 480
INC = 2 * math.pi / N
LASER_X = 0.13


def corridor_scan(x, half_width=0.2, blind=0.4, length=3.0):
    """Scan of a straight corridor along +x, lidar at base_link x; returns < blind are lost."""
    lx = x + LASER_X
    r = np.full(N, np.inf)
    for k in range(N):
        a = k * INC
        s = math.sin(a)
        if abs(s) > 1e-6:
            d = half_width / abs(s)
            if (lx + d * math.cos(a)) < length:
                r[k] = d
        if math.cos(a) > 1e-6:      # end wall at x = length
            d_end = (length - lx) / math.cos(a)
            r[k] = min(r[k], d_end)
    r[r < blind] = np.inf            # black wall: nothing comes back up close
    return r


def beam(deg):
    return int(round(math.radians(deg % 360) / INC)) % N


def drive(mem, xs, t0=0.0, dt=0.1):
    out = mask = None
    for i, x in enumerate(xs):
        mem.on_odom(x, 0.0, 0.0)
        out, mask = mem.on_scan(t0 + i * dt, corridor_scan(x), 0.0, INC)
    return out, mask


def test_side_walls_come_back_after_driving():
    mem = ScanMemory(laser_x=LASER_X)
    raw = corridor_scan(0.6)
    assert np.isinf(raw[beam(90)]) and np.isinf(raw[beam(-90)])     # blind beside the car
    out, mask = drive(mem, np.arange(0.0, 0.61, 0.01))
    for deg in (90, -90, 70, -110):
        assert mask[beam(deg)]
        expected = 0.2 / abs(math.sin(math.radians(deg)))
        assert out[beam(deg)] == pytest.approx(expected, abs=0.02)


def test_measured_beams_are_never_changed():
    mem = ScanMemory(laser_x=LASER_X)
    out, mask = drive(mem, np.arange(0.0, 0.61, 0.01))
    raw = corridor_scan(0.6)
    seen = np.isfinite(raw)
    assert np.array_equal(out[seen], raw[seen]) and not mask[seen].any()


def test_no_odometry_passes_the_scan_through():
    mem = ScanMemory()
    raw = corridor_scan(0.0)
    out, mask = mem.on_scan(0.0, raw, 0.0, INC)
    assert np.array_equal(out, raw) and not mask.any()


def test_points_are_forgotten_after_driving_or_time():
    mem = ScanMemory(laser_x=LASER_X, keep_dist=0.3)
    drive(mem, np.arange(0.0, 0.61, 0.01))
    assert len(mem.pts) and (mem.odo - mem.born_odo).max() <= 0.3 + 1e-9
    mem = ScanMemory(laser_x=LASER_X, keep_time=1.0)
    drive(mem, np.arange(0.0, 0.61, 0.01))           # 6 s of scans
    mem.on_odom(0.6, 0.0, 0.0)
    out, mask = mem.on_scan(100.0, np.full(N, np.inf), 0.0, INC)
    assert not mask.any() and len(mem.pts) == 0


def test_a_return_behind_a_remembered_point_clears_it():
    mem = ScanMemory(laser_x=0.0)
    mem.on_odom(0.0, 0.0, 0.0)
    r = np.full(N, np.inf)
    r[beam(0)] = 0.5                                  # an obstacle 0.5 m ahead
    mem.on_scan(0.0, r, 0.0, INC)
    assert len(mem.pts) == 1
    r[beam(0)] = 1.0                                  # it moved away: wall at 1.0 m now
    out, mask = mem.on_scan(0.1, r, 0.0, INC)
    assert out[beam(0)] == 1.0 and not mask.any()
    assert np.allclose(mem.pts, [[1.0, 0.0]])        # only the new return is remembered


def test_turning_keeps_the_wall_in_the_right_place():
    """Points stored, then the car turns 90 deg left on the spot: the wall moves to the right."""
    mem = ScanMemory(laser_x=0.0)
    mem.on_odom(0.0, 0.0, 0.0)
    r = np.full(N, np.inf)
    r[beam(0)] = 0.3                                  # wall point 0.3 m ahead
    mem.on_scan(0.0, r, 0.0, INC)
    mem.on_odom(0.0, 0.0, math.pi / 2)
    out, mask = mem.on_scan(0.1, np.full(N, np.inf), 0.0, INC)
    assert mask[beam(-90)] and out[beam(-90)] == pytest.approx(0.3, abs=1e-6)


def test_bad_parameters():
    with pytest.raises(ValueError):
        ScanMemory(fill_range=2.0, remember_range=1.0)


def test_beam_zero_wraps_around():
    """A point just right of straight ahead (angle near 360 deg) fills beam 0's neighbours."""
    mem = ScanMemory(laser_x=0.0)
    mem.on_odom(0.0, 0.0, 0.0)
    r = np.full(N, np.inf)
    r[N - 1] = 0.3                                    # last beam, 0.75 deg right of ahead
    mem.on_scan(0.0, r, 0.0, INC)
    out, mask = mem.on_scan(0.1, np.full(N, np.inf), 0.0, INC)
    assert mask[N - 1] and out[N - 1] == pytest.approx(0.3, abs=1e-6)
    assert mask[0] and out[0] == pytest.approx(0.3, abs=1e-6)   # within point_radius


def test_fill_is_not_pulled_closer_by_neighbours():
    """Wall seen at a slant: each beam keeps the distance in its own direction."""
    mem = ScanMemory(laser_x=0.0)
    mem.on_odom(0.0, 0.0, 0.0)
    r = np.full(N, np.inf)
    ks = np.arange(beam(20), beam(60))
    r[ks] = 0.2 / np.sin(ks * INC)                    # wall y = 0.2, beams 20-60 deg
    mem.on_scan(0.0, r, 0.0, INC)
    out, mask = mem.on_scan(0.1, np.full(N, np.inf), 0.0, INC)
    inner = ks[5:-5]
    assert mask[inner].all()
    assert np.allclose(out[inner], r[inner], atol=0.012)      # points are kept 1 cm apart


def test_camera_points_fill_blind_beams_and_are_marked():
    """The camera saw the wall 0.3 m to the left from where the car was 0.1 m ago."""
    mem = ScanMemory(laser_x=0.0)
    mem.on_odom(0.1, 0.0, 0.0)
    cam = np.full(N, np.inf)
    cam[beam(90)] = 0.3
    assert mem.add_camera(0.0, cam, 0.0, INC, pose=(0.0, 0.0, 0.0)) == 1
    assert np.allclose(mem.pts, [[0.0, 0.3]])                   # placed with the given pose
    out, mask = mem.on_scan(0.1, np.full(N, np.inf), 0.0, INC)
    k = int(np.argmin(out))
    assert mask[k] and mem.filled_by_camera[k]
    assert out[k] == pytest.approx(math.hypot(0.1, 0.3), abs=0.005)
    lidar = np.full(N, np.inf)
    lidar[beam(-90)] = 0.3                                     # a lidar point on the right
    mem.on_scan(0.2, lidar, 0.0, INC)
    out, mask = mem.on_scan(0.3, np.full(N, np.inf), 0.0, INC)
    assert mask[beam(-90)] and not mem.filled_by_camera[beam(-90)]


def test_camera_points_without_odometry_are_not_stored():
    mem = ScanMemory()
    cam = np.full(N, np.inf)
    cam[0] = 0.5
    assert mem.add_camera(0.0, cam, 0.0, INC) == 0 and len(mem.pts) == 0


def test_a_lidar_return_behind_a_camera_point_clears_it():
    """E.g. a dark mark on the floor: the lidar sees over it to the wall behind."""
    mem = ScanMemory(laser_x=0.0)
    mem.on_odom(0.0, 0.0, 0.0)
    cam = np.full(N, np.inf)
    cam[beam(0)] = 0.4
    mem.add_camera(0.0, cam, 0.0, INC)
    r = np.full(N, np.inf)
    r[beam(0)] = 0.9
    mem.on_scan(0.1, r, 0.0, INC)
    assert np.allclose(mem.pts, [[0.9, 0.0]])


def test_pose_history_interpolates_and_refuses_gaps():
    h = PoseHistory(keep=2.0)
    assert h.at(0.0) is None
    h.add(10.0, 0.0, 0.0, math.radians(170))
    h.add(10.1, 0.01, 0.0, math.radians(-170))               # heading crossed +-180 deg
    x, y, yaw = h.at(10.05)
    assert x == pytest.approx(0.005) and abs(abs(yaw) - math.pi) < 1e-9
    assert h.at(10.0) == pytest.approx((0.0, 0.0, math.radians(170)))
    assert h.at(10.25) == pytest.approx((0.01, 0.0, math.radians(-170)))   # newest, 0.15 s
    assert h.at(10.35) is None and h.at(9.7) is None
    h.add(10.05, 9.0, 9.0, 0.0)                               # out of order: ignored
    h.add(13.0, 1.0, 0.0, 0.0)
    assert len(h.poses) == 1                                  # older than 2 s dropped
