"""Pose-free race controller for the WRO-style loop course.

No ROS imports: this module is unit tested and driven by the offline 2D
simulator (``tools/sim2d.py``).  ``race_node.py`` only converts messages.

Inputs per control tick
-----------------------
* yaw (rad, any fixed frame: Gazebo odom, IMU, wheel odom ...)
* signed travelled path length ``s`` (m, forward positive)
* the latest LiDAR scan (robot frame, CCW angles, 0 = forward)
* orange / blue line flags from ``line_detector.py``

Not used: absolute x / y, maps, field-specific coordinates, ground truth.

How it works
------------
1. A local odometric frame is dead-reckoned from yaw and path length.  It
   only has to be good over a few metres; everything that matters is
   re-measured from the walls every scan.
2. Lane geometry is measured from the LiDAR: the lane centre line from the
   side walls, the lane direction from a line fit to the side walls (this
   also removes slow yaw drift), and the wall ahead.
3. The first line seen gives the driving direction (orange first =
   clockwise, blue first = counter-clockwise).  If no line is seen, the side
   that opens up at the first corner decides.
4. A reference path is built: the lane centre line, a quarter circle at the
   corner (placed from the measured wall ahead), and the next lane's centre
   line.  Corners therefore need no separate turn manoeuvre.
5. Scan points (plus recent points that are now inside the LiDAR's blind
   zone) are rasterised into a local grid and a distance transform gives
   the clearance of any point.  A dynamic-programming planner chooses
   lateral offsets from the reference at stations every few centimetres,
   checking the real footprint at every pose: it passes obstacles, walls
   and corners with the same rule.
6. Pure pursuit follows the planned path; a short-horizon collision check
   can only make the command safer.  If no forward motion is safe the car
   backs off a little and replans; repeated failure stops it (DONE,
   "blocked").
7. Corners are counted on the reference path.  After ``laps`` laps the car
   stops where it started, measured from the wall ahead of the start lane.

Every forward command has linear > 0 and respects ``min_turn_radius``, so
the same logic runs on a steering-wheel (Ackermann) car.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Optional, Sequence

import cv2
import numpy as np


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class Params:
    # speeds (m/s, rad/s)
    cruise_speed: float = 0.12
    avoid_speed: float = 0.07
    min_speed: float = 0.03
    backoff_speed: float = -0.05
    max_angular: float = 0.9
    lateral_accel: float = 0.06       # m/s^2 allowed on curves (v^2 * kappa)
    min_turn_radius: float = 0.0      # > 0 for Ackermann: |w| <= |v| / R

    # robot geometry, base_link frame (x forward, y left)
    lidar_x: float = -0.032           # LiDAR position
    body_front: float = 0.07          # body front ahead of the LiDAR
    body_rear: float = 0.07           # body rear behind the LiDAR
    half_width: float = 0.09

    # lane measurement
    default_lane_width: float = 0.57
    lane_width_min: float = 0.35
    lane_width_max: float = 0.95
    side_window: float = 0.12         # +- along the lane for side-wall readings
    max_side_range: float = 1.0
    centre_gain: float = 0.3          # filter gain for the lane centre line
    yaw_fit_gain: float = 0.05        # filter gain for the wall-direction fit
    yaw_fit_window: float = 0.35

    # course
    corner_radius: float = 0.22       # reference arc radius (<= half lane width)
    corners_per_lap: int = 4
    laps: int = 3
    direction_decide_dist: float = 0.25  # decide by open side this far before the corner

    # planner
    plan_horizon: float = 1.0
    plan_ds: float = 0.05
    plan_dq: float = 0.01
    plan_max_angle: float = 1.0       # rad, path angle to the reference
    plan_min_radius: float = 0.12     # m, tightest planned curve (diff drive)
    hard_clearance: float = 0.025
    soft_clearance: float = 0.07
    w_offset: float = 20.0
    w_slope: float = 10.0
    w_bend: float = 0.3
    w_soft: float = 2.0
    grid_res: float = 0.01
    grid_half: float = 1.3            # grid covers +- this around the robot
    replan_period: float = 0.1        # s

    # tracking
    stop_margin: float = 0.10         # m of free path kept in reserve ...
    stop_time: float = 2.0            # ... v <= (free path - margin) / stop_time
    pursuit_min: float = 0.10
    pursuit_gain: float = 0.6         # lookahead += gain * speed

    # safety
    safety_time: float = 1.0          # s of travel checked for collision ...
    safety_min_horizon: float = 0.04  # ... at least / at most this far
    safety_max_horizon: float = 0.15
    safety_margin: float = 0.008
    backoff_dist: float = 0.07
    backoff_timeout: float = 2.5
    backoff_turn_rate: float = 0.6
    max_backoffs_per_lane: int = 6
    stall_time: float = 6.0           # s without progress counts as blocked
    stall_dist: float = 0.03


class State(str, Enum):
    WAIT = "WAIT"
    RUN = "RUN"
    FINISH = "FINISH"
    BACKOFF = "BACKOFF"
    DONE = "DONE"


CW = -1    # right turns
CCW = 1    # left turns


@dataclass
class Scan:
    angle_min: float
    angle_increment: float
    ranges: Sequence[float]
    range_min: float = 0.05
    range_max: float = 10.0
    stamp: Optional[float] = None     # acquisition time (same clock as step's t)


@dataclass
class Command:
    linear: float = 0.0
    angular: float = 0.0


@dataclass
class Status:
    state: str
    reason: str
    direction: int
    corners: int
    laps_done: int
    lateral: Optional[float]
    lateral_ref: float
    lane_width: float
    front: Optional[float]
    seg_along: float
    clearance: Optional[float]
    backoffs: int

    def as_dict(self) -> dict:
        return asdict(self)


class RaceController:
    def __init__(self, params: Optional[Params] = None):
        self.p = params or Params()
        self._body = self._body_samples()
        self.reset()

    # ================================================================ setup
    def reset(self) -> None:
        self.state = State.WAIT
        self.reason = "reset"
        self.direction = 0
        self.corners = 0
        self.backoffs = 0
        self._started = False
        self._t = 0.0
        self._s_prev: Optional[float] = None
        self._s = 0.0
        self._x = self._y = 0.0           # local odometric pose of base_link
        self._yaw = 0.0
        self._hist: list[tuple] = []      # (t, x, y, yaw) for scan time alignment
        self.seg_yaw: Optional[float] = None
        self.lane_width = self.p.default_lane_width
        self._width_samples = 0
        self._centre_n: Optional[float] = None
        self._wall_u: Optional[float] = None
        self._wall_hist: list[float] = []
        self._mem = np.zeros((0, 2))
        self._pts = np.zeros((0, 2))
        self._grid = None                 # (distance image, x0, y0)
        self._last_scan: Optional[Scan] = None
        self._path: Optional[np.ndarray] = None   # (k, 2) planned path, odom frame
        self._path_ok = True
        self._path_len = 0.0
        self._path_clear: Optional[float] = None
        self._plan_t = -1e9
        self._q0: Optional[float] = None
        self.lateral_ref = 0.0
        self._prev_lines = (False, False)
        self._first_line: Optional[str] = None
        self._start_xy: Optional[tuple] = None
        self._start_wall_dist: Optional[float] = None
        self._lane_backoffs = 0
        self._resume = State.RUN
        self._backoff_t = 0.0
        self._backoff_xy = (0.0, 0.0)
        self._backoff_heading: Optional[float] = None
        self._progress_t = 0.0
        self._progress_s = 0.0

    def start(self) -> None:
        self._started = True

    def _set(self, state: State, reason: str) -> None:
        self.state = state
        self.reason = reason

    # ============================================================ geometry
    def _axes(self) -> tuple[np.ndarray, np.ndarray]:
        a = np.array([math.cos(self.seg_yaw), math.sin(self.seg_yaw)])
        b = np.array([-a[1], a[0]])
        return a, b

    def _robot_un(self) -> tuple[float, float]:
        a, b = self._axes()
        r = np.array([self._x, self._y])
        return float(r @ a), float(r @ b)

    def _pose_at(self, t: Optional[float]) -> tuple[float, float, float]:
        """Pose at time ``t`` from the recent pose history (latest if unknown)."""
        if t is None or len(self._hist) < 2 or t >= self._hist[-1][0]:
            return self._x, self._y, self._yaw
        h = self._hist
        if t <= h[0][0]:
            return h[0][1], h[0][2], h[0][3]
        for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(reversed(h[:-1]), reversed(h[1:])):
            if t0 <= t <= t1:
                f = (t - t0) / max(t1 - t0, 1e-9)
                return (x0 + f * (x1 - x0), y0 + f * (y1 - y0), wrap(a0 + f * wrap(a1 - a0)))
        return self._x, self._y, self._yaw

    def _scan_points(self, scan: Scan) -> np.ndarray:
        # Points are placed with the pose at the scan's acquisition time: the
        # car may have moved and turned since, which would shift every wall.
        r = np.asarray(scan.ranges, dtype=float)
        ang = scan.angle_min + scan.angle_increment * np.arange(r.size)
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        r, ang = r[ok], ang[ok]
        x, y, yaw = self._pose_at(scan.stamp)
        c, s_ = math.cos(yaw), math.sin(yaw)
        lx = x + self.p.lidar_x * c
        ly = y + self.p.lidar_x * s_
        th = ang + yaw
        return np.stack([lx + r * np.cos(th), ly + r * np.sin(th)], axis=1)

    def _update_memory(self, pts: np.ndarray, range_min: float) -> np.ndarray:
        # Points closer than the LiDAR's minimum range vanish from the scan.
        # Keep recent nearby points so the planner still sees them.
        c, s_ = math.cos(self._yaw), math.sin(self._yaw)
        lx, ly = self._x + self.p.lidar_x * c, self._y + self.p.lidar_x * s_
        near = np.hypot(pts[:, 0] - lx, pts[:, 1] - ly) < 0.4
        keep = np.zeros((0, 2))
        if self._mem.size:
            d = np.hypot(self._mem[:, 0] - lx, self._mem[:, 1] - ly)
            keep = self._mem[d < range_min + 0.03]
        self._mem = np.vstack([keep, pts[near]])[-2000:]
        return np.vstack([pts, keep]) if keep.size else pts

    def _build_grid(self, pts: np.ndarray) -> None:
        p = self.p
        n = int(round(2 * p.grid_half / p.grid_res))
        x0, y0 = self._x - p.grid_half, self._y - p.grid_half
        img = np.full((n, n), 255, dtype=np.uint8)
        j = np.floor((pts[:, 0] - x0) / p.grid_res).astype(int)
        i = np.floor((pts[:, 1] - y0) / p.grid_res).astype(int)
        ok = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        img[i[ok], j[ok]] = 0
        if np.any(ok):
            dist = cv2.distanceTransform(img, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
            dist = dist.astype(np.float32) * p.grid_res
        else:
            dist = np.full((n, n), 5.0, dtype=np.float32)
        self._grid = (dist, x0, y0)

    def _clearance_at(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        dist, x0, y0 = self._grid
        n = dist.shape[0]
        j = np.floor((xs - x0) / self.p.grid_res).astype(int)
        i = np.floor((ys - y0) / self.p.grid_res).astype(int)
        inside = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        out = np.full(np.shape(xs), 5.0, dtype=float)
        out[inside] = dist[i[inside], j[inside]]
        return out - 0.5 * self.p.grid_res

    def _body_samples(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Footprint samples (x, y) and each sample's depth inside the body."""
        p = self.p
        x0, x1 = p.lidar_x - p.body_rear, p.lidar_x + p.body_front
        hw = p.half_width
        xs, ys, depth = [], [], []
        for x in np.linspace(x0, x1, 5):
            for y in (-hw, hw):
                xs.append(x)
                ys.append(y)
                depth.append(0.0)
        for y in np.linspace(-hw, hw, 5)[1:-1]:
            for x in (x0, x1):
                xs.append(x)
                ys.append(y)
                depth.append(0.0)
        for x in np.linspace(x0, x1, 5)[1:-1]:
            for y in np.linspace(-hw, hw, 5)[1:-1]:
                xs.append(x)
                ys.append(y)
                depth.append(min(x - x0, x1 - x, hw - abs(y)))
        return np.array(xs), np.array(ys), np.array(depth)

    def _pose_clearance(self, px, py, th) -> np.ndarray:
        """Clearance of the footprint at poses (any shape), metres."""
        bx, by, depth = self._body
        px, py, th = np.asarray(px, float), np.asarray(py, float), np.asarray(th, float)
        c, s_ = np.cos(th)[..., None], np.sin(th)[..., None]
        wx = px[..., None] + c * bx - s_ * by
        wy = py[..., None] + s_ * bx + c * by
        return (self._clearance_at(wx, wy) - depth).min(axis=-1)

    # ===================================================== lane measurement
    @staticmethod
    def _wall_distance(values: np.ndarray) -> Optional[float]:
        if values.size < 3:
            return None
        q25, q50, q75 = np.percentile(values, [25, 50, 75])
        return float(q50) if q75 - q25 <= 0.02 else None

    def _measure_lane(self, pts: np.ndarray) -> None:
        p = self.p
        a, b = self._axes()
        ur, nr = self._robot_un()
        rel_u = pts @ a - ur
        rel_n = pts @ b - nr
        half = self.lane_width / 2.0

        if abs(wrap(self._yaw - self.seg_yaw)) < 0.3:
            # --- side walls -> centre line and lane width
            beside = np.abs(rel_u) < p.side_window
            left = self._wall_distance(rel_n[beside & (rel_n > 0) & (rel_n < p.max_side_range)])
            right = self._wall_distance(-rel_n[beside & (rel_n < 0) & (rel_n > -p.max_side_range)])
            cands = []
            if left is not None and right is not None:
                width = left + right
                gate = 0.06 if self._width_samples >= 5 else 1.0
                if p.lane_width_min <= width <= p.lane_width_max and abs(width - self.lane_width) < gate:
                    self.lane_width += 0.15 * (width - self.lane_width)
                    self._width_samples += 1
                    half = self.lane_width / 2.0
                    cands.append(nr + (left - right) / 2.0)
            if not cands:
                if left is not None and left < self.lane_width:
                    cands.append(nr + left - half)
                if right is not None and right < self.lane_width:
                    cands.append(nr - right + half)
            if cands:
                ref = self._centre_n if self._centre_n is not None else nr
                c = min(cands, key=lambda v: abs(v - ref))
                if self._centre_n is None:
                    self._centre_n = c
                elif abs(c - self._centre_n) < 0.08:
                    self._centre_n += p.centre_gain * (c - self._centre_n)

            # --- wall direction fit (removes slow yaw drift)
            if self._centre_n is not None and self.state is State.RUN:
                for sign in (1, -1):
                    wall_n = self._centre_n + sign * half
                    sel = (np.abs(rel_u) < p.yaw_fit_window) & (np.abs(rel_n + nr - wall_n) < 0.04)
                    if np.count_nonzero(sel) < 15:
                        continue
                    u, n = rel_u[sel], rel_n[sel]
                    if np.ptp(u) < 0.4:
                        continue
                    k, c0 = np.polyfit(u, n, 1)
                    good = np.abs(n - (k * u + c0)) < 0.008
                    if np.count_nonzero(good) < 12:
                        continue
                    k, _ = np.polyfit(u[good], n[good], 1)
                    if abs(k) < 0.15:
                        self.seg_yaw = wrap(self.seg_yaw + p.yaw_fit_gain * math.atan(k))

        # --- wall ahead (places the corner)
        if self._centre_n is not None:
            band = (np.abs(rel_n + nr - self._centre_n) < half - 0.06) & (rel_u > 0.15)
            u = rel_u[band] + ur
            nn = rel_n[band] + nr
            if u.size >= 5:
                bins = np.floor(u / 0.02).astype(int)
                vals, counts = np.unique(bins, return_counts=True)
                for idx in np.argsort(-counts)[:4]:
                    if counts[idx] < 5:
                        break
                    centre = (vals[idx] + 0.5) * 0.02
                    sel = np.abs(u - centre) < 0.03
                    if np.ptp(nn[sel]) >= 0.25:          # wide: a wall, not an obstacle
                        w = float(np.median(u[sel]))
                        self._wall_hist = (self._wall_hist + [w])[-7:]
                        self._wall_u = float(np.median(self._wall_hist))
                        break

    # ============================================================ direction
    def _track_lines(self, orange: bool, blue: bool) -> None:
        prev_o, prev_b = self._prev_lines
        if not orange and not blue:
            self._first_line = None
        elif self._first_line is None:
            rose_o, rose_b = orange and not prev_o, blue and not prev_b
            if rose_o and rose_b:
                self._first_line = "both"
            elif orange and not blue:
                self._first_line = "orange"
            elif blue and not orange:
                self._first_line = "blue"
        self._prev_lines = (orange, blue)
        if self.direction == 0 and self._first_line in ("orange", "blue"):
            self.direction = CW if self._first_line == "orange" else CCW
            self.reason = "direction from first line: " + ("CW" if self.direction == CW else "CCW")

    def _side_open(self, pts: np.ndarray, side: int) -> bool:
        """Is the lane open to ``side`` just before the wall ahead?"""
        a, b = self._axes()
        u, n = pts @ a, (pts @ b - self._centre_n) * side
        half = self.lane_width / 2.0
        region = ((u > self._wall_u - self.lane_width + 0.05) & (u < self._wall_u - 0.05)
                  & (n > half - 0.05) & (n < half + 0.35))
        return np.count_nonzero(region) < 3

    def _check_direction(self, pts: np.ndarray) -> None:
        if self._wall_u is None or self.corners > 0:
            return
        ur, _ = self._robot_un()
        corner_start = self._wall_u - self.lane_width / 2.0 - self.p.corner_radius
        if ur < corner_start - self.p.direction_decide_dist:
            return
        left, right = self._side_open(pts, CCW), self._side_open(pts, CW)
        if left == right:
            return
        opened = CCW if left else CW
        if self.direction != opened:
            self.reason = ("direction from open side" if self.direction == 0
                           else "direction corrected by open side")
            self.direction = opened

    # ============================================================ reference
    def _reference(self):
        """Reference polyline in the odom frame.

        Returns ``(points (k, 2), headings (k,), arc length (k,), corner_end)``
        where ``corner_end`` is the arc length at the end of the corner arc,
        or None while no corner is known.
        """
        p = self.p
        a, b = self._axes()
        ur, _ = self._robot_un()
        cn = self._centre_n
        step = 0.02
        u_start = ur - 0.3
        half = self.lane_width / 2.0
        corner_index = None
        if self.direction != 0 and self._wall_u is not None:
            d = self.direction
            rc = min(max(p.corner_radius, p.min_turn_radius), half)
            u_next = self._wall_u - half             # next lane's centre line
            u_c = u_next - rc
            us, ns = [], []
            if u_c > u_start:
                k = int(math.ceil((u_c - u_start) / step))
                us += list(np.linspace(u_start, u_c, k + 1))
                ns += [cn] * (k + 1)
            th = np.linspace(-d * math.pi / 2, 0.0, 24)
            us += list(u_c + rc * np.cos(th))
            ns += list(cn + d * rc + rc * np.sin(th))
            corner_index = len(us) - 1
            extra = p.plan_horizon + 0.6
            k = int(math.ceil(extra / step))
            tail = np.linspace(0.0, extra, k + 1)[1:]
            us += [u_next] * k
            ns += list(cn + d * rc + d * tail)
        else:
            extra = p.plan_horizon + 0.9
            k = int(math.ceil(extra / step))
            us = list(np.linspace(u_start, u_start + extra, k + 1))
            ns = [cn] * (k + 1)
        us, ns = np.asarray(us), np.asarray(ns)
        pts = us[:, None] * a + ns[:, None] * b
        seg = np.diff(pts, axis=0)
        keep = np.concatenate([[True], np.hypot(seg[:, 0], seg[:, 1]) > 1e-6])
        pts = pts[keep]
        seg = np.diff(pts, axis=0)
        s = np.concatenate([[0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
        head = np.arctan2(seg[:, 1], seg[:, 0])
        head = np.concatenate([head, head[-1:]])
        corner_end = None
        if corner_index is not None:
            corner_end = float(s[int(np.count_nonzero(keep[:corner_index + 1])) - 1])
        return pts, head, s, corner_end

    @staticmethod
    def _project(pts, s, x, y) -> tuple[float, float]:
        """Arc length of the nearest reference point and signed offset (left +)."""
        seg = np.diff(pts, axis=0)
        l2 = np.maximum((seg ** 2).sum(axis=1), 1e-12)
        t = np.clip(((x - pts[:-1, 0]) * seg[:, 0] + (y - pts[:-1, 1]) * seg[:, 1]) / l2, 0, 1)
        cx = pts[:-1, 0] + t * seg[:, 0]
        cy = pts[:-1, 1] + t * seg[:, 1]
        d2 = (cx - x) ** 2 + (cy - y) ** 2
        i = int(np.argmin(d2))
        s0 = s[i] + t[i] * math.sqrt(l2[i])
        cross = seg[i, 0] * (y - cy[i]) - seg[i, 1] * (x - cx[i])
        q = math.copysign(math.sqrt(d2[i]), cross) if d2[i] > 1e-16 else 0.0
        return float(s0), q

    # ============================================================== planner
    def _plan(self, ref, free_start: bool = False) -> Optional[float]:
        """Second-order dynamic programming over lateral offsets.

        Stations every ``plan_ds`` along the reference; at each station the
        lateral offset is one of a fixed grid.  The state is the pair of
        offsets at two consecutive stations, so every transition knows the
        pose heading (first difference) and the path curvature (second
        difference plus the reference's own curvature).  Poses are checked
        with the real footprint against the distance transform; curvature
        above ``1 / plan_min_radius`` (or the Ackermann limit) is forbidden,
        so the path is drivable from the current heading.

        With ``free_start`` the initial heading is not fixed; the return
        value is then the heading the best path starts with (used to steer
        while backing off).  Otherwise the result is stored in ``_path``.
        """
        p = self.p
        pts, head, s, _ = ref
        s0, q0 = self._project(pts, s, self._x, self._y)
        self._q0 = q0
        ds = p.plan_ds
        n = int(round(p.plan_horizon / ds))
        sk = s0 + ds * np.arange(n + 1)
        hu = np.unwrap(head)
        px = np.interp(sk, s, pts[:, 0])
        py = np.interp(sk, s, pts[:, 1])
        hk = np.interp(sk, s, hu)
        kref = np.gradient(hk, ds)                            # reference curvature
        nx, ny = -np.sin(hk), np.cos(hk)
        qmax = max(self.lane_width / 2.0 - p.half_width, 0.02)
        grid = np.arange(-qmax, qmax + 1e-9, p.plan_dq)
        m = grid.size
        kmax = 1.0 / max(p.min_turn_radius, p.plan_min_radius)
        big = 1e6
        now = float(self._pose_clearance(self._x, self._y, self._yaw))
        hard_lim = min(p.hard_clearance, now - 0.004)

        def pose_terms(k, slope):
            """Cost and feasibility of poses at station k for given slopes."""
            ang = np.arctan(slope)
            X = px[k] + grid * nx[k]
            Y = py[k] + grid * ny[k]
            X, Y = np.broadcast_to(X, slope.shape), np.broadcast_to(Y, slope.shape)
            clear = self._pose_clearance(X, Y, hk[k] + ang)
            c = (p.w_offset * (grid / qmax) ** 2 + p.w_slope * slope ** 2) * ds
            c = c + np.where(clear < p.soft_clearance,
                             p.w_soft * ((p.soft_clearance - clear) / 0.02) ** 2 * ds * 20, 0.0)
            c = c + np.where((clear < hard_lim) | (np.abs(ang) > p.plan_max_angle), big, 0.0)
            return c, clear

        head_err = clamp(wrap(self._yaw - hk[0]), -1.3, 1.3)
        s_init = math.tan(head_err)
        # station 1: state = offset at station 1 (station 0 is the robot)
        slope1 = (grid - q0) / ds
        c1, clear1 = pose_terms(1, slope1)
        if not free_start:
            curv = (slope1 - s_init) / ds + kref[1]
            c1 = c1 + p.w_bend * ((slope1 - s_init) / ds) ** 2 * ds
            c1 = c1 + np.where(np.abs(curv) > kmax * 1.5, big, 0.0)
        if free_start:
            j = int(np.argmin(c1))
            if c1[j] >= big / 2:
                return None
            # look one step further so the start heading is not myopic
            return float(hk[0] + math.atan(slope1[j]))
        dq = grid[None, :] - grid[:, None]                     # (i, j): q_j - q_i
        slope = dq / ds
        d2 = grid[None, None, :] - 2 * grid[None, :, None] + grid[:, None, None]   # (i, j, l)
        bend_extra = d2 / ds ** 2
        bend_cost = p.w_bend * bend_extra ** 2 * ds
        # station 2
        C = c1[:, None] + p.w_bend * ((slope - slope1[:, None]) / ds) ** 2 * ds
        C = C + np.where(np.abs((slope - slope1[:, None]) / ds + kref[2]) > kmax, big, 0.0)
        e2, clr = pose_terms(2, slope)
        C = C + e2
        clears = {1: np.broadcast_to(clear1, (m,)), 2: clr}
        backs = []
        reach, endC = (2, C) if C.min() < big / 2 else (1, None)
        if reach == 1 and c1.min() >= big / 2:
            reach = 0
        for k in range(3, n + 1):
            if reach < k - 1:
                break
            curv_bad = np.abs(bend_extra + kref[k]) > kmax
            total = C[:, :, None] + bend_cost + np.where(curv_bad, big, 0.0)   # (i, j, l)
            bp = np.argmin(total, axis=0)                                       # (j, l)
            best = np.take_along_axis(total, bp[None], axis=0)[0]
            ek, clr = pose_terms(k, slope)
            C = best + ek
            backs.append(bp)
            clears[k] = clr
            if C.min() < big / 2:
                reach, endC = k, C
            else:
                break
        # backtrack from the last reachable station
        idx = [0] * (reach + 1)
        if reach >= 2:
            j, l = np.unravel_index(int(np.argmin(endC)), endC.shape)
            idx[reach], idx[reach - 1] = int(l), int(j)
            for k in range(reach, 2, -1):
                i = int(backs[k - 3][idx[k - 1], idx[k]])
                idx[k - 2] = i
        elif reach == 1:
            idx[1] = int(np.argmin(c1))
        q = np.array([q0] + [grid[t] for t in idx[1:]], dtype=float)
        K = reach + 1
        self._path = np.stack([px[:K] + q * nx[:K], py[:K] + q * ny[:K]], axis=1)
        self._path_len = reach * ds
        self._path_ok = reach >= 2
        cl = []
        for k in range(1, min(reach, 8) + 1):
            c_k = clears[k]
            cl.append(float(c_k[idx[k]] if k == 1 else c_k[idx[k - 1], idx[k]]))
        self._path_clear = min(cl) if cl else 0.0
        self.lateral_ref = float(q[min(int(round(0.25 / ds)), reach)])
        self._plan_t = self._t
        return None

    # ============================================================= tracking
    def _track(self) -> Command:
        """Pure pursuit on the planned path with a curvature speed limit."""
        p = self.p
        path = self._path
        c, s_ = math.cos(self._yaw), math.sin(self._yaw)
        rel = path - np.array([self._x, self._y])
        fwd = rel[:, 0] * c + rel[:, 1] * s_
        lat = -rel[:, 0] * s_ + rel[:, 1] * c
        dist = np.hypot(rel[:, 0], rel[:, 1])
        v = p.cruise_speed
        seg = np.diff(path[:10], axis=0)
        if seg.shape[0] >= 2:
            hd = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
            ln = np.maximum(np.hypot(seg[:, 0], seg[:, 1]), 1e-6)
            kappa = float((np.abs(np.diff(hd)) / ln[1:]).max())
            v = min(v, math.sqrt(p.lateral_accel / max(kappa, 1e-6)))
        if self._path_clear is not None and self._path_clear < p.soft_clearance:
            v = min(v, p.avoid_speed)
        free = self._path_len - p.stop_margin
        v = min(v, max(free, 0.0) / p.stop_time)
        if not self._path_ok:
            v = p.min_speed
        v = max(v, p.min_speed)
        look = p.pursuit_min + p.pursuit_gain * v
        ahead = np.nonzero((fwd > 0.02) & (dist >= look))[0]
        i = int(ahead[0]) if ahead.size else path.shape[0] - 1
        tx, ty = fwd[i], lat[i]
        kappa = 2.0 * ty / max(tx * tx + ty * ty, 1e-6)
        if abs(math.atan2(ty, tx)) > 0.9:
            v = p.min_speed
        w = clamp(v * kappa, -p.max_angular, p.max_angular)
        return self._limit(Command(v, w))

    def _limit(self, cmd: Command) -> Command:
        if self.p.min_turn_radius > 0.0:
            w_max = abs(cmd.linear) / self.p.min_turn_radius
            cmd.angular = clamp(cmd.angular, -w_max, w_max)
        return cmd

    def _arc_clearance(self, V: np.ndarray, W: np.ndarray, horizon: float) -> np.ndarray:
        """Minimum footprint clearance along constant (v, w) arcs."""
        steps = np.linspace(0.0, horizon, 8)[1:]
        sgn = np.where(V < 0, -1.0, 1.0)[:, None]
        k = (W / np.maximum(np.abs(V), 1e-6))[:, None]     # yaw change per metre
        dist = sgn * steps[None, :] * np.ones_like(k)
        th = k * steps[None, :]
        small = np.abs(k) < 1e-6
        sk = np.where(small, 1.0, k)
        lx = np.where(small, dist, sgn * np.sin(th) / sk)
        ly = np.where(small, 0.0, sgn * (1.0 - np.cos(th)) / sk)
        c, s_ = math.cos(self._yaw), math.sin(self._yaw)
        X = self._x + c * lx - s_ * ly
        Y = self._y + s_ * lx + c * ly
        # x = sgn*sin(k d)/k, y = sgn*(1-cos(k d))/k, yaw change = k d for
        # both driving directions (k = w / |v|)
        return self._pose_clearance(X, Y, self._yaw + th).min(axis=1)

    def _safety(self, cmd: Command) -> Optional[Command]:
        """Closest safe command to ``cmd``; None if no forward motion is safe."""
        p = self.p
        now = float(self._pose_clearance(self._x, self._y, self._yaw))
        need = min(p.safety_margin, now - 0.003)
        ws = np.concatenate([[cmd.angular], np.linspace(-p.max_angular, p.max_angular, 19)])
        speeds = [cmd.linear, max(min(cmd.linear, p.min_speed), 0.01)]
        V = np.repeat(speeds, ws.size)
        W = np.tile(ws, len(speeds))
        if p.min_turn_radius > 0:
            W = np.clip(W, -np.abs(V) / p.min_turn_radius, np.abs(V) / p.min_turn_radius)
        horizon = clamp(abs(cmd.linear) * p.safety_time + 0.02,
                        p.safety_min_horizon, p.safety_max_horizon)
        clear = self._arc_clearance(V, W, horizon)
        ok = (clear >= need) & (clear > 0.002)
        if ok[0]:
            return cmd
        if not np.any(ok):
            return None
        cost = (W - cmd.angular) ** 2 + 4.0 * (V - cmd.linear) ** 2
        i = int(np.argmin(np.where(ok, cost, np.inf)))
        return self._limit(Command(float(V[i]), float(W[i])))

    # ================================================================ step
    def status(self) -> Status:
        front = None
        along = 0.0
        if self.seg_yaw is not None:
            along, _ = self._robot_un()
            if self._wall_u is not None:
                front = self._wall_u - along
        return Status(state=self.state.value, reason=self.reason, direction=self.direction,
                      corners=self.corners,
                      laps_done=self.corners // max(self.p.corners_per_lap, 1),
                      lateral=self._q0, lateral_ref=self.lateral_ref,
                      lane_width=self.lane_width, front=front, seg_along=along,
                      clearance=self._path_clear, backoffs=self.backoffs)

    def step(self, t: float, yaw: float, s: float, scan: Optional[Scan],
             orange: bool = False, blue: bool = False) -> Command:
        """Advance one control tick and return the velocity command."""
        self._t = t
        if self._s_prev is not None:
            ds = s - self._s_prev
            mid = wrap(self._yaw + 0.5 * wrap(yaw - self._yaw))
            self._x += ds * math.cos(mid)
            self._y += ds * math.sin(mid)
            self._s += ds
        self._s_prev = s
        self._yaw = yaw
        self._hist.append((t, self._x, self._y, yaw))
        if len(self._hist) > 40:
            del self._hist[:-40]
        if self.seg_yaw is None:
            self.seg_yaw = yaw
            self._progress_t = t
        if self.state is State.DONE or scan is None:
            return Command()

        new_scan = scan is not self._last_scan
        self._last_scan = scan
        if new_scan:
            pts = self._scan_points(scan)
            self._build_grid(self._update_memory(pts, scan.range_min))
            self._measure_lane(pts)
            self._pts = pts
        if self._grid is None or self._centre_n is None:
            return Command()

        if self.state is State.WAIT:
            if not self._started:
                return Command()
            self._start_xy = (self._x, self._y)
            self._progress_t = t
            self._set(State.RUN, "start")

        self._track_lines(orange, blue)
        if new_scan:
            self._check_direction(self._pts)
        if self.state is State.BACKOFF:
            return self._backoff()

        if self._start_wall_dist is None and self._wall_u is not None and self.corners == 0:
            a, _ = self._axes()
            self._start_wall_dist = self._wall_u - float(np.array(self._start_xy) @ a)

        ref = self._reference()
        self._corner_progress(ref)
        if self.state is State.FINISH and self._finished():
            self._set(State.DONE, "laps complete")
        if self.state is State.DONE:
            return Command()

        if self._path is None or new_scan or self._t - self._plan_t > self.p.replan_period:
            self._plan(self._reference())
        if not self._path_ok:
            return self._start_backoff("no collision-free path")
        if self._path.shape[0] < 2:
            return self._start_backoff("no path")
        cmd = self._track()
        safe = self._safety(cmd)
        stalled = self._stalled()
        if safe is None or stalled:
            return self._start_backoff("stalled" if stalled else "no safe forward motion")
        return safe

    # ======================================================= laps / corners
    def _corner_progress(self, ref) -> None:
        pts, _, s, corner_end = ref
        if corner_end is None:
            return
        s0, _ = self._project(pts, s, self._x, self._y)
        if s0 < corner_end:
            return
        # passed the corner arc: switch to the next lane's frame
        d = self.direction
        u_next = self._wall_u - self.lane_width / 2.0
        self.seg_yaw = wrap(self.seg_yaw + d * math.pi / 2.0)
        self._centre_n = -d * u_next
        self._wall_u = None
        self._wall_hist = []
        self._path = None
        self.corners += 1
        self._lane_backoffs = 0
        if self.corners >= self.p.laps * self.p.corners_per_lap:
            self._set(State.FINISH, "final corner")
        else:
            self.reason = f"corner {self.corners}"

    def _finished(self) -> bool:
        if self._wall_u is None or self._start_wall_dist is None:
            return False
        ur, _ = self._robot_un()
        return self._wall_u - ur <= self._start_wall_dist

    # ============================================================ recovery
    def _stalled(self) -> bool:
        if abs(self._s - self._progress_s) > self.p.stall_dist:
            self._progress_s = self._s
            self._progress_t = self._t
            return False
        if self._t - self._progress_t > self.p.stall_time:
            self._progress_t = self._t
            return True
        return False

    def _start_backoff(self, why: str) -> Command:
        if self._lane_backoffs >= self.p.max_backoffs_per_lane:
            self._set(State.DONE, f"blocked: {why}")
            return Command()
        self._lane_backoffs += 1
        self.backoffs += 1
        self._resume = self.state
        self._backoff_t = self._t
        self._backoff_xy = (self._x, self._y)
        # heading a free path would start with: reversing while turning the
        # nose toward it makes the next forward attempt drivable
        self._backoff_heading = self._plan(self._reference(), free_start=True)
        self._path = None
        self._set(State.BACKOFF, why)
        return self._backoff()

    def _backoff(self) -> Command:
        p = self.p
        moved = math.hypot(self._x - self._backoff_xy[0], self._y - self._backoff_xy[1])
        if moved < p.backoff_dist and self._t - self._backoff_t < p.backoff_timeout:
            # reverse; turn the nose toward the heading a free path needs
            w_des = 0.0
            if self._backoff_heading is not None:
                err = wrap(self._backoff_heading - self._yaw)
                if abs(err) > 0.08:
                    w_des = math.copysign(p.backoff_turn_rate, err)
            ws = np.concatenate([[w_des], np.linspace(-0.6, 0.6, 13)])
            if p.min_turn_radius > 0:
                lim = abs(p.backoff_speed) / p.min_turn_radius
                ws = np.clip(ws, -lim, lim)
            V = np.full(ws.size, p.backoff_speed)
            clear = self._arc_clearance(V, ws, 0.08)
            ok = clear > 0.006
            if np.any(ok):
                i = int(np.argmin(np.where(ok, (ws - w_des) ** 2, np.inf)))
                return Command(float(V[i]), float(ws[i]))
        self._set(self._resume, "resume after backoff")
        self._progress_t = self._t
        self._progress_s = self._s
        return Command()
