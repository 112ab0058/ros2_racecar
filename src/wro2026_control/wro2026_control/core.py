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
    corner_radius: float = 0.30       # reference arc radius (capped at half the lane width)
    corners_per_lap: int = 4
    laps: int = 3
    direction_decide_dist: float = 0.25  # decide by open side this far before the corner

    # planner
    plan_horizon: float = 1.0
    plan_ds: float = 0.05
    plan_dq: float = 0.015
    plan_max_angle: float = 1.0       # rad, path angle to the reference
    plan_min_radius: float = 0.12     # m, tightest planned curve (diff drive)
    plan_heading_bins: int = 13       # footprint clearance precomputed at these headings
    plan_arcs: int = 15               # first move: arcs of this many curvatures
    hard_clearance: float = 0.018
    soft_clearance: float = 0.07
    w_offset: float = 20.0
    w_slope: float = 4.0
    w_bend: float = 0.3
    w_soft: float = 2.0
    grid_res: float = 0.01            # m; clearances are exact, not quantised
    grid_half: float = 1.3            # grid covers +- this around the robot
    replan_period: float = 0.1        # s

    # tracking
    stop_margin: float = 0.10         # m of free path kept in reserve ...
    stop_time: float = 2.0            # ... v <= (free path - margin) / stop_time
    pursuit_min: float = 0.07
    pursuit_gain: float = 0.4         # lookahead += gain * speed

    # safety
    safety_time: float = 1.0          # s of travel checked for collision ...
    safety_min_horizon: float = 0.04  # ... at least / at most this far
    safety_max_horizon: float = 0.15
    safety_margin: float = 0.008
    backoff_dist: float = 0.07        # first back-off; later ones go further
    backoff_dist_max: float = 0.25
    trap_weight: float = 3.0          # planner penalty near poses where the car got stuck
    trap_radius: float = 0.08
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
        self._grid = None                 # (labels, label points, x0, y0)
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
        self._backoff_len = 0.0
        self._trail: list[tuple] = []     # (x, y) driven in this lane, for backing off
        self._traps: list[tuple] = []     # (x, y) where the car got stuck in this lane
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
        if r.size >= 3:
            # 3-beam median where the neighbours lie on the same surface:
            # removes range noise without rounding off edges
            stack = np.stack([np.roll(r, 1), r, np.roll(r, -1)])
            med = np.median(stack, axis=0)
            with np.errstate(invalid="ignore"):
                same = np.all(np.abs(stack - r) < 0.03, axis=0)
            r = np.where(same & np.isfinite(med), med, r)
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
        """Nearest-obstacle lookup for the planner and the safety check.

        Points are rasterised into a local grid and OpenCV's distance
        transform labels every cell with its nearest occupied cell.  The
        clearance of a query is then the exact distance to the point stored
        in that cell, so it is not quantised to the grid resolution.
        """
        p = self.p
        n = int(round(2 * p.grid_half / p.grid_res))
        x0, y0 = self._x - p.grid_half, self._y - p.grid_half
        j = np.floor((pts[:, 0] - x0) / p.grid_res).astype(int)
        i = np.floor((pts[:, 1] - y0) / p.grid_res).astype(int)
        ok = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        if not np.any(ok):
            self._grid = None
            self._grid_empty = True
            return
        img = np.full((n, n), 255, dtype=np.uint8)
        img[i[ok], j[ok]] = 0
        _, labels = cv2.distanceTransformWithLabels(
            img, cv2.DIST_L2, cv2.DIST_MASK_5, labelType=cv2.DIST_LABEL_PIXEL)
        lab_pts = np.zeros((int(labels.max()) + 1, 2))
        lab_pts[labels[i[ok], j[ok]]] = pts[ok]
        self._grid = (labels, lab_pts, x0, y0)
        self._grid_empty = False

    def _nearest(self, xs: np.ndarray, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Nearest obstacle point to each query (far away if none known)."""
        xs, ys = np.asarray(xs, float), np.asarray(ys, float)
        qx = xs + 5.0
        qy = ys.copy()
        if self._grid is None:
            return qx, qy
        labels, lab_pts, x0, y0 = self._grid
        n = labels.shape[0]
        j = np.floor((xs - x0) / self.p.grid_res).astype(int)
        i = np.floor((ys - y0) / self.p.grid_res).astype(int)
        inside = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        near = lab_pts[labels[i[inside], j[inside]]]
        qx[inside], qy[inside] = near[:, 0], near[:, 1]
        return qx, qy

    def _clearance_at(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        qx, qy = self._nearest(xs, ys)
        return np.hypot(np.asarray(xs) - qx, np.asarray(ys) - qy)

    def _body_samples(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Footprint sample points (base frame) used to find nearby obstacles."""
        p = self.p
        x0, x1 = p.lidar_x - p.body_rear, p.lidar_x + p.body_front
        xs, ys = np.meshgrid(np.linspace(x0, x1, 4),
                             np.linspace(-p.half_width, p.half_width, 4), indexing="ij")
        return xs.ravel(), ys.ravel(), np.zeros(xs.size)

    def _pose_clearance(self, px, py, th) -> np.ndarray:
        """Exact footprint clearance at poses (any shape), metres.

        Each footprint sample looks up its nearest obstacle point; the
        distance from those candidate points to the footprint rectangle is
        computed exactly (negative = inside the body).
        """
        bx, by, _ = self._body
        p = self.p
        px, py, th = np.asarray(px, float), np.asarray(py, float), np.asarray(th, float)
        c, s_ = np.cos(th)[..., None], np.sin(th)[..., None]
        wx = px[..., None] + c * bx - s_ * by
        wy = py[..., None] + s_ * bx + c * by
        qx, qy = self._nearest(wx, wy)
        dx, dy = qx - px[..., None], qy - py[..., None]
        lx = c * dx + s_ * dy
        ly = -s_ * dx + c * dy
        x0, x1, hw = p.lidar_x - p.body_rear, p.lidar_x + p.body_front, p.half_width
        ddx = np.maximum(np.maximum(x0 - lx, lx - x1), 0.0)
        ddy = np.maximum(np.abs(ly) - hw, 0.0)
        outside = np.hypot(ddx, ddy)
        inside = -np.minimum(np.minimum(lx - x0, x1 - lx), hw - np.abs(ly))
        d = np.where((ddx > 0) | (ddy > 0), outside, inside)
        return d.min(axis=-1)

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
        lateral offset is one of a fixed grid.  All geometry is Cartesian:
        a move between two stations is a straight chord, the pose heading is
        the chord direction, and the curvature is the turn between two
        chords divided by their mean length.  The state is the pair of
        offsets at two consecutive stations, so every transition knows its
        heading and curvature.  Each pose's footprint clearance comes from
        the obstacle map; curvature above ``1 / plan_min_radius`` (or the
        Ackermann limit) is forbidden, so the path is drivable from the
        current heading.  Cost: offset from the reference, heading away from
        the reference, curvature, low clearance, places where the car got
        stuck before.  The path ends where free space ends.

        With ``free_start`` the initial heading is not fixed and the return
        value is the heading the best first move takes (used to steer while
        backing off).  Otherwise the result is stored in ``_path``.
        """
        p = self.p
        pts, head, s, _ = ref
        s0, q0 = self._project(pts, s, self._x, self._y)
        self._q0 = q0
        ds = p.plan_ds
        n = int(round(p.plan_horizon / ds))
        sk = s0 + ds * np.arange(n + 1)
        px = np.interp(sk, s, pts[:, 0])
        py = np.interp(sk, s, pts[:, 1])
        hk = np.interp(sk, s, np.unwrap(head))
        nx, ny = -np.sin(hk), np.cos(hk)
        qmax = max(self.lane_width / 2.0 - p.half_width, 0.02)
        grid = np.arange(-qmax, qmax + 1e-9, p.plan_dq)
        m = grid.size
        PX = px[:, None] + grid[None, :] * nx[:, None]          # (station, offset)
        PY = py[:, None] + grid[None, :] * ny[:, None]
        kmax = 1.0 / max(p.min_turn_radius, p.plan_min_radius)
        big = 1e6
        now = float(self._pose_clearance(self._x, self._y, self._yaw))
        hard_lim = min(p.hard_clearance, now - 0.004)
        cols = np.arange(m)

        # footprint clearance for every station, offset and a set of headings
        # around the reference (one vectorised lookup per plan)
        hb = np.linspace(-p.plan_max_angle, p.plan_max_angle, p.plan_heading_bins)
        hstep = hb[1] - hb[0]
        table = self._pose_clearance(
            np.broadcast_to(PX[:, :, None], (n + 1, m, hb.size)),
            np.broadcast_to(PY[:, :, None], (n + 1, m, hb.size)),
            hk[:, None, None] + hb[None, None, :])           # (station, offset, heading)

        def pose_terms(k, theta, X, Y):
            """Cost of poses at station k (positions X, Y, headings theta)."""
            dev = np.arctan2(np.sin(theta - hk[k]), np.cos(theta - hk[k]))
            b = np.clip(np.rint((dev + p.plan_max_angle) / hstep).astype(int), 0, hb.size - 1)
            clear = table[k, np.broadcast_to(cols, dev.shape), b]
            c = (p.w_offset * (grid / qmax) ** 2 + p.w_slope * dev ** 2) * ds
            c = c + np.where(clear < p.soft_clearance,
                             p.w_soft * ((p.soft_clearance - clear) / 0.02) ** 2, 0.0)
            c = c + np.where((clear < hard_lim) | (np.abs(dev) > p.plan_max_angle), big, 0.0)
            for tx, ty in self._traps:
                c = c + p.trap_weight * np.exp(-((X - tx) ** 2 + (Y - ty) ** 2) / p.trap_radius ** 2)
            return c, clear

        if free_start:
            # any heading allowed: which first move is cheapest?
            dx1, dy1 = PX[1] - self._x, PY[1] - self._y
            th1 = np.arctan2(dy1, dx1)
            c1, _ = pose_terms(1, th1, PX[1], PY[1])
            j = int(np.argmin(c1))
            return None if c1[j] >= big / 2 else float(th1[j])

        # station 1: circular arcs from the robot's pose (curvature samples),
        # so the first move always matches the current heading exactly
        yaw = self._yaw
        K1 = np.linspace(-kmax, kmax, p.plan_arcs)
        the = yaw + K1 * ds
        safe_k = np.where(np.abs(K1) < 1e-9, 1.0, K1)
        X1 = self._x + np.where(np.abs(K1) < 1e-9, ds * math.cos(yaw), (np.sin(the) - math.sin(yaw)) / safe_k)
        Y1 = self._y + np.where(np.abs(K1) < 1e-9, ds * math.sin(yaw), (math.cos(yaw) - np.cos(the)) / safe_k)
        q1 = (X1 - px[1]) * nx[1] + (Y1 - py[1]) * ny[1]
        clear1 = self._pose_clearance(X1, Y1, the)
        dev1 = np.arctan2(np.sin(the - hk[1]), np.cos(the - hk[1]))
        c1 = ((p.w_offset * (q1 / qmax) ** 2 + p.w_slope * dev1 ** 2 + p.w_bend * K1 ** 2) * ds
              + np.where(clear1 < p.soft_clearance,
                         p.w_soft * ((p.soft_clearance - clear1) / 0.02) ** 2, 0.0)
              + np.where((clear1 < hard_lim) | (np.abs(dev1) > p.plan_max_angle), big, 0.0))
        for tx, ty in self._traps:
            c1 = c1 + p.trap_weight * np.exp(-((X1 - tx) ** 2 + (Y1 - ty) ** 2) / p.trap_radius ** 2)

        # chords between grid stations: TH[k-1] is the chord into station k
        DX = PX[1:, None, :] - PX[:-1, :, None]
        DY = PY[1:, None, :] - PY[:-1, :, None]
        TH = np.arctan2(DY, DX)
        LEN = np.maximum(np.hypot(DX, DY), 1e-6)
        # station 2: chords from the arc ends to the grid
        DX12, DY12 = PX[2][None, :] - X1[:, None], PY[2][None, :] - Y1[:, None]
        TH12 = np.arctan2(DY12, DX12)                             # (arc, j)
        LEN12 = np.maximum(np.hypot(DX12, DY12), 1e-6)
        turn = np.arctan2(np.sin(TH12 - the[:, None]), np.cos(TH12 - the[:, None]))
        kap = 2.0 * np.sin(turn) / LEN12
        e2, clr2 = pose_terms(2, TH12, PX[2][None, :], PY[2][None, :])
        C = (c1[:, None] + e2 + p.w_bend * kap ** 2 * ds
             + np.where(np.abs(kap) > kmax, big, 0.0))
        clears = {1: clear1, 2: clr2}
        backs = []
        if C.min() < big / 2:
            reach, endC = 2, C
        else:
            reach, endC = (1 if c1.min() < big / 2 else 0), None
        for k in range(3, n + 1):
            if reach < k - 1:
                break
            tin = TH12 if k == 3 else TH[k - 2]                   # chord into station k-1
            lin = LEN12 if k == 3 else LEN[k - 2]
            tout = TH[k - 1]                                      # (i, j) chord into station k
            turn = np.arctan2(np.sin(tout[None, :, :] - tin[:, :, None]),
                              np.cos(tout[None, :, :] - tin[:, :, None]))   # (h, i, j)
            kap = turn / (0.5 * (lin[:, :, None] + LEN[k - 1][None, :, :]))
            total = (C[:, :, None] + p.w_bend * kap ** 2 * ds
                     + np.where(np.abs(kap) > kmax, big, 0.0))
            bp = np.argmin(total, axis=0)                         # (i, j)
            best = np.take_along_axis(total, bp[None], axis=0)[0]
            ek, clr = pose_terms(k, tout, PX[k][None, :], PY[k][None, :])
            C = best + ek
            backs.append(bp)
            clears[k] = clr
            if C.min() < big / 2:
                reach, endC = k, C
            else:
                break
        idx = [0] * (reach + 1)                                   # idx[1] is an arc index
        if reach >= 2:
            i, j = np.unravel_index(int(np.argmin(endC)), endC.shape)
            idx[reach], idx[reach - 1] = int(j), int(i)
            for k in range(reach, 2, -1):
                idx[k - 2] = int(backs[k - 3][idx[k - 1], idx[k]])
        elif reach == 1:
            idx[1] = int(np.argmin(c1))
        path = [(self._x, self._y)]
        if reach >= 1:
            path.append((X1[idx[1]], Y1[idx[1]]))
        path += [(PX[k, idx[k]], PY[k, idx[k]]) for k in range(2, reach + 1)]
        self._path = np.array(path, dtype=float)
        self._path_len = reach * ds
        self._path_ok = reach >= 2
        cl = [float(clear1[idx[1]])] if reach >= 1 else []
        for k in range(2, min(reach, 8) + 1):
            cl.append(float(clears[k][idx[k - 1], idx[k]]))
        self._path_clear = min(cl) if cl else 0.0
        q = [q0] + ([float(q1[idx[1]])] if reach >= 1 else []) + [grid[idx[k]] for k in range(2, reach + 1)]
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
        if self.state in (State.RUN, State.FINISH) and (
                not self._trail or math.hypot(self._x - self._trail[-1][0],
                                              self._y - self._trail[-1][1]) > 0.01):
            self._trail.append((self._x, self._y))
            if len(self._trail) > 300:
                del self._trail[:-300]
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
        if self._centre_n is None or not hasattr(self, "_grid_empty"):
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
        self._traps = []
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
        self._backoff_len = min(self.p.backoff_dist * self._lane_backoffs, self.p.backoff_dist_max)
        c, s_ = math.cos(self._yaw), math.sin(self._yaw)
        # remember where it got stuck (a little ahead of the base): the next
        # plans avoid this spot and so try a different way through
        self._traps.append((self._x + 0.05 * c, self._y + 0.05 * s_))
        self._backoff_heading = self._plan(self._reference(), free_start=True)
        self._path = None
        self._set(State.BACKOFF, why)
        return self._backoff()

    def _backoff(self) -> Command:
        """Reverse along the trail just driven (known to be free), then resume."""
        p = self.p
        moved = math.hypot(self._x - self._backoff_xy[0], self._y - self._backoff_xy[1])
        if moved < self._backoff_len and self._t - self._backoff_t < p.backoff_timeout + self._backoff_len / 0.05:
            w_des = 0.0
            c, s_ = math.cos(self._yaw), math.sin(self._yaw)
            target = None
            for tx, ty in reversed(self._trail):
                dx, dy = tx - self._x, ty - self._y
                if dx * c + dy * s_ < -0.02 and math.hypot(dx, dy) > 0.08:
                    target = (dx, dy)
                    break
            if target is not None:
                fx = target[0] * c + target[1] * s_
                ly = -target[0] * s_ + target[1] * c
                # pure pursuit for reverse driving (mirror the robot frame)
                w_des = abs(p.backoff_speed) * 2.0 * (-ly) / max(fx * fx + ly * ly, 1e-6)
                w_des = clamp(w_des, -0.8, 0.8)
            elif self._backoff_heading is not None:
                err = wrap(self._backoff_heading - self._yaw)
                if abs(err) > 0.08:
                    w_des = math.copysign(0.6, err)
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
        # drop the trail part that was reversed over
        while self._trail and math.hypot(self._trail[-1][0] - self._x,
                                         self._trail[-1][1] - self._y) < 0.03:
            self._trail.pop()
        self._set(self._resume, "resume after backoff")
        self._progress_t = self._t
        self._progress_s = self._s
        return Command()
