"""
2-D test field for the drive policies (pure numpy, no ROS, no Gazebo).

A square loop, our car, a lidar with the black-wall blind spot; for tests and quick
checks, not physics.

Field (2026-11-11 school field, docs FIELD_PLAN_160CM): outer walls 1.6 m square, island
in the middle; corridor 0.39-0.43 m measured, so the default island is 0.80 m (0.40 m).
Car (R550_HARDWARE.md): base_link = rear axle, front 0.24 m, rear 0.031 m, width 0.18 m,
lidar 0.13 m ahead. Turning (2026-11-10 floor tests): the car never turns tighter than
0.40 m (commanded 0.35 = full lock), commanded 0.5 m drives 0.524 m. The steering
follows commands with a lag (steer_tau). Lidar: 480 beams, 10 Hz, black foam returns
nothing closer than blind_range (0.4 m measured), small range noise. With `partial` the
blind spot is as measured on 2026-11-13 instead: nothing below 0.29 m, then a return
share rising to 100 % at 0.47 m, random per beam and scan (the edge that fooled run 4).
Camera (camera()): the C70's wall bottoms, as wro2026_camera_c70 publishes them.
"""
import math

import numpy as np

FRONT, REAR, HALF_WIDTH, LASER_X = 0.24, 0.031, 0.09, 0.13
MIN_RADIUS = 0.40          # driven radius at full lock
RADIUS_SCALE = 1.048       # driven / commanded radius above full lock (0.5 -> 0.524)
PARTIAL = (0.29, 0.47)     # m: black walls give no return below, always a return above
CAMERA_X = 0.20            # lens in front of base_link (c70_extrinsics.yaml)
CAMERA_HALF_VIEW = math.radians(45.0)   # floor seen within +-45 deg of the lens and
CAMERA_NEAR = 0.26                      # >= 0.26 m from it (real calibration, 2026-11-20)


def square(half):
    """Four wall segments of a square centred on the origin."""
    c = [(-half, -half), (half, -half), (half, half), (-half, half)]
    return [(c[i], c[(i + 1) % 4]) for i in range(4)]


class Field:
    """Walls as segments ((x1, y1), (x2, y2))."""

    def __init__(self, outer=1.6, island=0.80):
        """Outer and island sizes (m, inner faces of the walls)."""
        self.walls = np.array(square(outer / 2.0) + square(island / 2.0), dtype=float)
        self.outer, self.island = outer, island

    def raycast(self, ox, oy, angles):
        """Distance from (ox, oy) along each world angle to the nearest wall (inf: none)."""
        p, q = self.walls[:, 0], self.walls[:, 1]
        dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
        ex, ey = (q - p)[:, 0][None], (q - p)[:, 1][None]
        wx, wy = (p[:, 0] - ox)[None], (p[:, 1] - oy)[None]
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (wx * ey - wy * ex) / den              # along the beam
            u = (wx * dy - wy * dx) / den              # along the wall, 0..1
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        return np.where(hit, t, np.inf).min(axis=1)

    def clearance(self, x, y, yaw):
        """Smallest distance from the car outline to a wall (m); negative: they cross."""
        c, s = math.cos(yaw), math.sin(yaw)
        local = [(FRONT, HALF_WIDTH), (FRONT, -HALF_WIDTH), (-REAR, -HALF_WIDTH),
                 (-REAR, HALF_WIDTH)]
        poly = np.array([(x + c * a - s * b, y + s * a + c * b) for a, b in local])
        edges = [(poly[i], poly[(i + 1) % 4]) for i in range(4)]
        best = np.inf
        for p, q in self.walls:
            for a, b in edges:
                if _cross(a, b, p, q):
                    return -1.0
                best = min(best, _seg_dist(a, b, p, q))
        return best


def _cross(a, b, p, q):
    def orient(u, v, w):
        return (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
    return (orient(a, b, p) * orient(a, b, q) < 0) and (orient(p, q, a) * orient(p, q, b) < 0)


def _pt_seg(pt, a, b):
    ab = b - a
    t = np.clip(np.dot(pt - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return float(np.hypot(*(a + t * ab - pt)))


def _seg_dist(a, b, p, q):
    return min(_pt_seg(a, p, q), _pt_seg(b, p, q), _pt_seg(p, a, b), _pt_seg(q, a, b))


class Car:
    """Rear-axle pose; commands (vx, wz) as base_node gets them; steering lag."""

    def __init__(self, x, y, yaw, steer_tau=0.15):
        """Start pose (rear axle, m / rad)."""
        self.x, self.y, self.yaw = x, y, yaw
        self.curv = 0.0
        self.steer_tau = steer_tau

    def step(self, vx, wz, dt):
        """Drive dt seconds."""
        want = 0.0
        if abs(vx) > 1e-6 and abs(wz) > 1e-6:
            radius = max(MIN_RADIUS, RADIUS_SCALE * abs(vx / wz))
            want = math.copysign(1.0 / radius, wz * vx)
        self.curv += (want - self.curv) * min(1.0, dt / self.steer_tau)
        self.yaw += vx * self.curv * dt
        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt


def lidar(field, car, n=480, blind_range=0.4, noise=0.003, rng=None, partial=False):
    """
    One scan (ROS order: beam 0 = ahead, counter-clockwise), blind spot applied.

    partial: the measured blind edge (PARTIAL, needs rng) instead of a sharp blind_range.
    """
    lx = car.x + LASER_X * math.cos(car.yaw)
    ly = car.y + LASER_X * math.sin(car.yaw)
    a = 2.0 * math.pi * np.arange(n) / n
    r = field.raycast(lx, ly, a + car.yaw)
    if rng is not None and noise > 0:
        r = r + rng.normal(0.0, noise, n)
    if partial:
        lo, hi = PARTIAL
        r[rng.random(n) >= np.clip((r - lo) / (hi - lo), 0.0, 1.0)] = np.inf
    else:
        r[r < blind_range] = np.inf
    return r


def camera(field, car, n=480, max_range=1.0, noise=0.01, bias=-0.018, rng=None, rays=181):
    """
    Wall bottoms the C70 sees, like camera/wall_scan: per lidar beam, range from the lidar.

    Rays from the lens within CAMERA_HALF_VIEW; a wall bottom nearer to the lens than
    CAMERA_NEAR is below the image. bias: the camera reads 1.8 cm short of the corrected
    lidar (2026-11-19); noise along the range. Farther than max_range: dropped.
    """
    c, s = math.cos(car.yaw), math.sin(car.yaw)
    cx, cy = car.x + CAMERA_X * c, car.y + CAMERA_X * s
    b = np.linspace(-CAMERA_HALF_VIEW, CAMERA_HALF_VIEW, rays) + car.yaw
    d = field.raycast(cx, cy, b)
    ok = np.isfinite(d) & (d >= CAMERA_NEAR)
    # the points in the lidar frame (lidar at LASER_X, same heading)
    dx = cx + d[ok] * np.cos(b[ok]) - (car.x + LASER_X * c)
    dy = cy + d[ok] * np.sin(b[ok]) - (car.y + LASER_X * s)
    x, y = c * dx + s * dy, -s * dx + c * dy
    r = np.hypot(x, y) + bias
    if rng is not None and noise > 0:
        r = r + rng.normal(0.0, noise, len(r))
    out = np.full(n, np.inf)
    k = np.round(np.arctan2(y, x) % (2.0 * math.pi) / (2.0 * math.pi / n)).astype(int) % n
    keep = r <= max_range
    np.minimum.at(out, k[keep], r[keep])
    return out
