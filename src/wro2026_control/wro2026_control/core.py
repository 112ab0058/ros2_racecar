"""Pose-free race controller core for the WRO-style loop course.

This module has no ROS imports so it can be unit tested and run in the
offline 2D simulator (`tools/sim2d.py`).  The ROS 2 node in `race_node.py`
only converts messages into the plain inputs below.

What the controller uses
------------------------
* yaw (rad, any fixed frame: Gazebo odom, IMU, wheel odom ...)
* signed travelled path length ``s`` (m, forward positive)
* one LiDAR scan (robot frame, angles CCW positive, 0 = forward)
* the orange / blue line trigger flags from ``line_detector.py``

What it deliberately does NOT use
---------------------------------
* absolute x / y position, map coordinates or field-specific zones
* ground truth
* in-place rotation (every forward state keeps linear > 0, so the same
  logic works on the final steering-wheel car; set ``min_turn_radius``)

Behaviour
---------
WAIT -> FOLLOW -> APPROACH -> TURN -> FOLLOW ... -> FINISH -> DONE

* Direction (CW / CCW) is locked from the colour of the first line seen:
  orange first -> clockwise (right turns), blue first -> counter-clockwise.
  After that only the leading colour starts a corner, so the second line of
  the same corner never triggers a wrong turn.
* A corner is also detected without a line (front wall close and the turn
  side open), so one missed line does not end the run.
* Turns start at a wall distance computed from the measured lane width and
  the turn radius, not from field coordinates.
* Obstacles are LiDAR clusters inside the lane that are not connected to a
  wall.  The car shifts its lateral target to the side with the larger gap,
  keeps that side until the obstacle is behind it, then returns to centre.
* After ``laps * corners_per_lap`` corners the car drives back to the start
  distance and stops (DONE).
* Every state has a bounded exit.  The only stopped state is DONE: laps
  complete, or boxed in after repeated back-offs (reported as "blocked").
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Optional, Sequence

import numpy as np


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class Params:
    # speeds (m/s, rad/s)
    cruise_speed: float = 0.12
    avoid_speed: float = 0.09
    min_avoid_speed: float = 0.03
    approach_speed: float = 0.08
    turn_speed: float = 0.06
    turn_rate: float = 0.55          # nominal |angular| while turning
    max_angular: float = 0.60
    backoff_speed: float = -0.05
    min_turn_radius: float = 0.0     # >0 for Ackermann: |w| <= |v| / R

    # lane keeping
    heading_kp: float = 1.6
    lateral_kp: float = 4.0          # rad of heading offset per metre
    lateral_max_angle: float = 0.80  # rad
    avoid_max_angle: float = 1.10    # rad, steepest approach angle when avoiding
    max_side_range: float = 1.0      # side wall readings beyond this are ignored
    side_window: float = 0.12        # +- along-track window for side walls (m)
    default_lane_width: float = 0.57
    lane_width_min: float = 0.35
    lane_width_max: float = 0.95
    lateral_dead_reckon_max: float = 0.6   # m of travel without a wall reading

    # robot geometry (relative to the LiDAR)
    lidar_x: float = -0.032          # LiDAR position in base frame (m)
    body_front: float = 0.07         # body front ahead of the LiDAR (m)
    body_rear: float = 0.07          # body rear behind the LiDAR (m)
    half_width: float = 0.09
    wall_margin: float = 0.035

    # corners
    corners_per_lap: int = 4
    laps: int = 3
    line_lockout: float = 0.35       # m after a turn during which lines are ignored
    approach_max_dist: float = 0.55  # turn anyway after this much travel past the line
    approach_timeout: float = 8.0
    fallback_margin: float = 0.03    # fallback corner if front < turn distance - margin
    open_side_factor: float = 1.35   # turn side counts as open beyond lane_width * factor
    turn_tolerance: float = math.radians(4.0)
    turn_timeout: float = 12.0
    min_turn_angular: float = 0.18

    # finish
    finish_min_dist: float = 0.05
    finish_extra: float = 0.0

    # safety
    safety_horizon: float = 0.20     # m of each candidate arc checked for collision
    safety_margin: float = 0.015     # footprint inflation for the check (m)
    plan_horizon: float = 0.25       # m rolled out by the lane planner
    plan_clearance: float = 0.06     # clearance below this is penalised
    plan_lateral_scale: float = 0.04
    plan_heading_scale: float = 0.35
    backoff_dist: float = 0.08
    backoff_timeout: float = 2.5
    backoff_check: float = 0.10      # m of reverse arc checked for collision
    backoff_turn_rate: float = 0.5
    backoff_max_angle: float = 0.9
    max_backoffs_per_state: int = 6

    # obstacles
    obstacle_lookahead: float = 0.70
    obstacle_edge_reject: float = 0.05   # points this close to a wall are walls
    obstacle_max_size: float = 0.20
    obstacle_cluster_gap: float = 0.05
    obstacle_min_points: int = 2
    obstacle_clearance: float = 0.05     # body-to-obstacle lateral clearance
    obstacle_pass_after: float = 0.12    # keep the offset this far past the obstacle
    obstacle_merge_dist: float = 0.12

    @property
    def turn_radius(self) -> float:
        return self.turn_speed / max(self.turn_rate, 1e-3)


class State(str, Enum):
    WAIT = "WAIT"
    FOLLOW = "FOLLOW"
    APPROACH = "APPROACH"
    TURN = "TURN"
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


@dataclass
class Obstacle:
    along: float        # segment progress of the near face (m)
    lateral: float      # lane-centred lateral position, left positive (m)
    half_size: float
    side: int = 0       # pass side chosen once: +1 left, -1 right
    hits: int = 1


@dataclass
class Command:
    linear: float = 0.0
    angular: float = 0.0
    checked: bool = False   # already verified collision-free by the planner


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
    obstacles: int
    backoffs: int

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass
class _Geometry:
    """Per-tick scan interpretation in the corridor frame."""

    along: np.ndarray       # forward along the current segment heading (m)
    lat: np.ndarray         # left of the robot, segment-aligned (m)
    x: np.ndarray           # robot frame forward (m, from LiDAR)
    y: np.ndarray           # robot frame left (m)
    left: Optional[float]
    right: Optional[float]
    front: Optional[float]


class RaceController:
    def __init__(self, params: Optional[Params] = None):
        self.p = params or Params()
        self.reset()

    # ------------------------------------------------------------------ setup
    def reset(self) -> None:
        self.state = State.WAIT
        self.reason = "reset"
        self.direction = 0
        self.corners = 0
        self.seg_yaw: Optional[float] = None
        self.lane_width = self.p.default_lane_width
        self._lane_width_samples = 0
        self.lateral: Optional[float] = None
        self.lateral_ref = 0.0
        self.obstacles: list[Obstacle] = []
        self.backoffs = 0
        self._started = False
        self._t = 0.0
        self._s_prev: Optional[float] = None
        self._along = 0.0              # progress along current segment
        self._total_along = 0.0
        self._state_t = 0.0
        self._state_along = 0.0
        self._seg_start_along = 0.0    # along at which this segment started
        self._start_to_first_turn: Optional[float] = None
        self._segment_lengths: list[float] = []
        self._turn_target: Optional[float] = None
        self._resume_state = State.FOLLOW
        self._resume_clock = (0.0, 0.0)
        self._prev_lines = (False, False)
        self._first_line: Optional[str] = None   # colour that appeared first
        self._turn_clusters: list = []
        self._next_lateral: Optional[float] = None
        self._ox = 0.0                 # local dead-reckoned pose, only used to
        self._oy = 0.0                 # carry recent scan points into the
        self._mem = np.zeros((0, 2))   # LiDAR's blind zone (< range_min)
        self._lat_unmeasured = 0.0     # travel since the last wall measurement
        self._last_scan: Optional[Scan] = None
        self._backoff_from_along = 0.0
        self._backoffs_in_state = 0
        self._front: Optional[float] = None

    def start(self) -> None:
        self._started = True

    # --------------------------------------------------------------- helpers
    def _enter(self, state: State, reason: str) -> None:
        if state is not State.BACKOFF and state is not self.state:
            self._backoffs_in_state = 0
        self.state = state
        self.reason = reason
        self._state_t = self._t
        self._state_along = self._along

    def _elapsed(self) -> float:
        return self._t - self._state_t

    def _travel(self) -> float:
        return self._along - self._state_along

    def status(self) -> Status:
        return Status(
            state=self.state.value, reason=self.reason, direction=self.direction,
            corners=self.corners,
            laps_done=self.corners // max(self.p.corners_per_lap, 1),
            lateral=self.lateral, lateral_ref=self.lateral_ref,
            lane_width=self.lane_width, front=self._front,
            seg_along=self._along - self._seg_start_along,
            obstacles=len(self.obstacles), backoffs=self.backoffs)

    def _geometry(self, scan: Scan, yaw: float) -> _Geometry:
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + scan.angle_increment * np.arange(r.size)
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        r, a = r[ok], a[ok]
        x, y = r * np.cos(a), r * np.sin(a)
        e = wrap(yaw - self.seg_yaw) if self.seg_yaw is not None else 0.0
        along = r * np.cos(a + e)
        lat = r * np.sin(a + e)

        beside = np.abs(along) < self.p.side_window
        left = self._wall_distance(lat[beside & (lat > 0) & (lat < self.p.max_side_range)])
        right = self._wall_distance(-lat[beside & (lat < 0) & (lat > -self.p.max_side_range)])

        ahead = (along > 0.0) & (np.abs(lat) < 0.10)
        front = float(np.min(along[ahead])) if np.any(ahead) else None
        return _Geometry(along, lat, x, y, left, right, front)

    def _lidar_origin(self) -> tuple[float, float]:
        c, s_ = math.cos(self._yaw), math.sin(self._yaw)
        return self._ox + self.p.lidar_x * c, self._oy + self.p.lidar_x * s_

    def _update_memory(self, g: _Geometry, range_min: float) -> None:
        # Keep nearby points from recent scans in a local frame.  When the car
        # gets closer than the LiDAR's minimum range they vanish from the
        # live scan; the collision check still sees them through this memory.
        ox, oy = self._lidar_origin()
        c, s_ = math.cos(self._yaw), math.sin(self._yaw)
        near = np.hypot(g.x, g.y) < 0.45
        pts = np.stack([ox + c * g.x[near] - s_ * g.y[near],
                        oy + s_ * g.x[near] + c * g.y[near]], axis=1)
        if self._mem.size:
            d = np.hypot(self._mem[:, 0] - ox, self._mem[:, 1] - oy)
            keep = d < range_min + 0.04          # only what is now in the blind zone
            pts = np.vstack([self._mem[keep], pts])
        self._mem = pts[-3000:]

    def _memory_points(self) -> tuple[np.ndarray, np.ndarray]:
        if not self._mem.size:
            return np.zeros(0), np.zeros(0)
        ox, oy = self._lidar_origin()
        c, s_ = math.cos(self._yaw), math.sin(self._yaw)
        dx, dy = self._mem[:, 0] - ox, self._mem[:, 1] - oy
        return c * dx + s_ * dy, -s_ * dx + c * dy

    @staticmethod
    def _wall_distance(values: np.ndarray) -> Optional[float]:
        # A lane wall beside the car gives many readings at one distance.
        # Wall ends, gaps and far structures give a spread -> rejected.
        if values.size < 3:
            return None
        q25, q50, q75 = np.percentile(values, [25, 50, 75])
        if q75 - q25 > 0.02:
            return None
        return float(q50)

    def _update_lateral(self, g: _Geometry) -> None:
        """Lateral position in the lane, left of centre positive."""
        p = self.p
        aligned = (self.state in (State.FOLLOW, State.APPROACH, State.FINISH)
                   and abs(wrap(self._yaw - self.seg_yaw)) < 0.15)
        half = self.lane_width / 2.0
        if g.left is not None and g.right is not None:
            width = g.left + g.right
            # first trusted measurements may move the estimate freely inside
            # the limits; afterwards only consistent readings update it
            gate = 0.05 if self._lane_width_samples >= 5 else p.lane_width_max
            if (p.lane_width_min <= width <= p.lane_width_max
                    and abs(width - self.lane_width) < gate):
                if aligned:
                    self.lane_width += 0.15 * (width - self.lane_width)
                    self._lane_width_samples += 1
                self.lateral = (g.right - g.left) / 2.0
                self._lat_unmeasured = 0.0
                return
        candidates = []
        if g.right is not None and g.right < self.lane_width:
            candidates.append(g.right - half)
        if g.left is not None and g.left < self.lane_width:
            candidates.append(half - g.left)
        if not candidates:
            if self._lat_unmeasured > p.lateral_dead_reckon_max:
                self.lateral = None
            return
        ref = self.lateral if self.lateral is not None else 0.0
        self.lateral = min(candidates, key=lambda c: abs(c - ref))
        self._lat_unmeasured = 0.0

    def _turn_front_dist(self) -> float:
        # LiDAR range to the wall ahead at which a turn of radius R ends on
        # the next lane's centre line.
        return self.p.turn_radius + self.lane_width / 2.0 - self.p.lidar_x

    def _side_open(self, g: _Geometry, side: int) -> bool:
        dist = g.left if side > 0 else g.right
        return dist is None or dist > self.lane_width * self.p.open_side_factor

    # ------------------------------------------------------------- obstacles
    def _obstacle_clusters(self, g: _Geometry) -> tuple[list, np.ndarray]:
        """Find in-lane clusters that are not connected to a wall.

        Returns ``(clusters, mask)`` where each cluster is
        ``(near_along, lateral, half_size)`` relative to the robot and
        ``mask`` marks the scan points that belong to accepted clusters.
        """
        p = self.p
        mask = np.zeros(g.along.size, dtype=bool)
        if self.lateral is None or self.seg_yaw is None:
            return [], mask
        half_lane = self.lane_width / 2.0
        abs_lat = self.lateral + g.lat
        inside = ((g.along > 0.02) & (g.along < p.obstacle_lookahead)
                  & (np.abs(abs_lat) < half_lane - p.obstacle_edge_reject))
        n = inside.size
        clusters = []
        if n == 0 or not np.any(inside):
            return clusters, mask
        pts = np.stack([g.along, abs_lat], axis=1)
        i = 0
        while i < n:
            if not inside[i]:
                i += 1
                continue
            j = i
            while (j + 1 < n and inside[j + 1]
                   and np.hypot(*(pts[j + 1] - pts[j])) < p.obstacle_cluster_gap):
                j += 1
            found = self._accept_cluster(pts, inside, i, j, n)
            if found is not None:
                clusters.append(found)
                mask[i:j + 1] = True
            i = j + 1
        # a cluster that wraps around index 0 is split; that only happens
        # directly behind the robot, which is outside the lookahead anyway.
        return clusters, mask

    def _accept_cluster(self, pts, inside, i, j, n):
        p = self.p
        if j - i + 1 < p.obstacle_min_points:
            return None
        # connected to a non-lane structure (wall) on either end -> not an obstacle
        for k, end in ((i - 1, i), (j + 1, j)):
            if 0 <= k < n and not inside[k]:
                if np.hypot(*(pts[k] - pts[end])) < p.obstacle_cluster_gap:
                    return None
        seg = pts[i:j + 1]
        size_along = float(np.ptp(seg[:, 0]))
        size_lat = float(np.ptp(seg[:, 1]))
        if size_along > p.obstacle_max_size or size_lat > p.obstacle_max_size:
            return None
        return (float(np.min(seg[:, 0])), float(np.mean(seg[:, 1])),
                max(size_lat / 2.0, 0.03))

    def _remember_obstacles(self, clusters: list) -> None:
        p = self.p
        for near, lat, half in clusters:
            along = self._along + near
            for ob in self.obstacles:
                if (abs(ob.along - along) < p.obstacle_merge_dist
                        and abs(ob.lateral - lat) < p.obstacle_merge_dist):
                    w = 1.0 / (ob.hits + 1)
                    ob.along += w * (along - ob.along)
                    ob.lateral += w * (lat - ob.lateral)
                    ob.half_size = max(ob.half_size, half)
                    ob.hits += 1
                    break
            else:
                self.obstacles.append(Obstacle(along=along, lateral=lat, half_size=half))

    def _look_into_next_lane(self, scan: Scan, yaw: float) -> None:
        # Late in a turn, look for obstacles in the lane we are turning into
        # so the lateral shift can start as soon as the turn ends.
        saved = (self.seg_yaw, self.lateral, self._lat_unmeasured)
        self.seg_yaw = self._turn_target
        self.lateral = self._next_lateral
        g = self._geometry(scan, yaw)
        self._update_lateral(g)
        self._turn_clusters, _ = self._obstacle_clusters(g)
        self._next_lateral = self.lateral
        self.seg_yaw, self.lateral, self._lat_unmeasured = saved

    def _obstacle_from_blockage(self, g: _Geometry) -> None:
        # Something unplanned is in front of the body (e.g. a wall end seen
        # through a gap).  Treat it as an obstacle and pass on the freer side.
        p = self.p
        band = (g.x > 0) & (np.abs(g.y) < p.half_width + 0.01) & (g.x < p.body_front + 0.15)
        if not np.any(band) or self.seg_yaw is None:
            return
        if self.lateral is None:
            self.lateral = 0.0
        near = float(np.min(g.along[band]))
        lat = float(np.mean(g.lat[band])) + self.lateral
        def free(sign):
            sel = (g.x > 0) & (g.y * sign > 0) & (np.abs(np.arctan2(g.y, g.x)) < 1.2)
            return float(np.min(np.hypot(g.x[sel], g.y[sel]))) if np.any(sel) else 9.0
        side = 1 if free(1) > free(-1) else -1
        self.obstacles.append(Obstacle(along=self._along + near, lateral=lat,
                                       half_size=0.04, side=side, hits=2))

    def _known_obstacle_mask(self, g: _Geometry) -> np.ndarray:
        mask = np.zeros(g.along.size, dtype=bool)
        if self.lateral is None:
            return mask
        abs_lat = self.lateral + g.lat
        for ob in self.obstacles:
            rel = ob.along - self._along
            mask |= ((g.along > rel - 0.03) & (g.along < rel + 2 * ob.half_size + 0.05)
                     & (np.abs(abs_lat - ob.lateral) < ob.half_size + 0.05))
        return mask

    def _flip_nearest_obstacle_side(self) -> None:
        pending = [ob for ob in self.obstacles if ob.side != 0
                   and ob.along + 2 * ob.half_size + self.p.obstacle_pass_after > self._along]
        if pending:
            ob = min(pending, key=lambda o: o.along)
            ob.side = -ob.side

    def _next_obstacle(self) -> Optional[Obstacle]:
        p = self.p
        pending = [ob for ob in self.obstacles
                   if ob.hits >= 2
                   and ob.along + 2 * ob.half_size + p.obstacle_pass_after > self._along]
        return min(pending, key=lambda o: o.along) if pending else None

    def _plan_lateral_ref(self) -> float:
        p = self.p
        limit = max(self.lane_width / 2.0 - p.half_width - p.wall_margin, 0.0)
        ob = self._next_obstacle()
        if ob is None:
            return 0.0
        if ob.side == 0:
            half_lane = self.lane_width / 2.0
            gap_left = half_lane - (ob.lateral + ob.half_size)
            gap_right = (ob.lateral - ob.half_size) + half_lane
            ob.side = 1 if gap_left > gap_right else -1
        need = ob.lateral + ob.side * (ob.half_size + p.half_width + p.obstacle_clearance)
        return clamp(need, -limit, limit)

    # ---------------------------------------------------------------- motion
    def _lane_command(self, speed: float, g: _Geometry) -> Command:
        """Lane following + obstacle passing as a small local planner.

        A proportional heading/lateral law gives the preferred command.  A
        set of (v, w) arcs is then scored by where each arc ends (lateral
        error to ``lateral_ref`` and heading error in the lane frame), by
        clearance and by speed; only collision-free arcs are eligible.
        """
        p = self.p
        offset = 0.0
        if self.lateral is not None:
            err = self.lateral - self.lateral_ref
            offset = clamp(-p.lateral_kp * err, -p.lateral_max_angle, p.lateral_max_angle)
        preferred = self._heading_command(speed, wrap(self.seg_yaw + offset))
        V, W = self._candidates([preferred.linear, 0.6 * speed, p.min_avoid_speed],
                                [preferred.angular])
        V[0], W[0] = preferred.linear, preferred.angular
        ok, clear, xe, ye, the = self._rollout(V, W, g, p.plan_horizon)
        if not np.any(ok):
            return preferred                      # the safety filter decides
        e = wrap(self._yaw - self.seg_yaw)
        head_end = np.arctan2(np.sin(e + the), np.cos(e + the))
        cost = (head_end / p.plan_heading_scale) ** 2
        cost += 0.5 * ((V - speed) / 0.04) ** 2
        cost += 0.3 * ((W - preferred.angular) / p.max_angular) ** 2
        cost += np.where(clear < p.plan_clearance,
                         ((p.plan_clearance - clear) / 0.015) ** 2, 0.0)
        if self.lateral is not None:
            lat_end = self.lateral + xe * np.sin(e) + ye * np.cos(e)
            cost += ((lat_end - self.lateral_ref) / p.plan_lateral_scale) ** 2
        i = int(np.argmin(np.where(ok, cost, np.inf)))
        return self._limit(Command(float(V[i]), float(W[i]), checked=True))

    def _heading_command(self, speed: float, target: float) -> Command:
        p = self.p
        err = wrap(target - self._yaw)
        w = clamp(p.heading_kp * err, -p.max_angular, p.max_angular)
        if abs(w) > 0.75 * p.max_angular:
            speed *= 0.7
        return self._limit(Command(speed, w))

    def _limit(self, cmd: Command) -> Command:
        if self.p.min_turn_radius > 0.0:
            w_max = abs(cmd.linear) / self.p.min_turn_radius
            cmd.angular = clamp(cmd.angular, -w_max, w_max)
        return cmd

    # ------------------------------------------------------------------ step
    def step(self, t: float, yaw: float, s: float, scan: Optional[Scan],
             orange: bool = False, blue: bool = False) -> Command:
        """Advance one control tick and return the velocity command."""
        self._t = t
        self._yaw = yaw
        if self.seg_yaw is None:
            self.seg_yaw = yaw
        if self._s_prev is not None:
            ds = s - self._s_prev
            self._ox += ds * math.cos(yaw)
            self._oy += ds * math.sin(yaw)
            e = wrap(yaw - self.seg_yaw)
            self._along += ds * math.cos(e)
            if self.lateral is not None:
                # dead-reckon the lateral position through wall gaps
                self.lateral += ds * math.sin(e)
                self._lat_unmeasured += abs(ds)
        self._s_prev = s

        if self.state is State.DONE:
            return Command()
        if scan is None:
            return Command()
        g = self._geometry(scan, yaw)
        self._front = g.front
        self._update_lateral(g)
        if scan is not self._last_scan:
            self._update_memory(g, scan.range_min)

        if self.state is State.WAIT:
            if not self._started:
                return Command()
            self._enter(State.FOLLOW, "start")

        if self.state is State.BACKOFF:
            return self._backoff(g)

        new_scan = scan is not self._last_scan
        self._last_scan = scan
        if self.state in (State.FOLLOW, State.APPROACH, State.FINISH):
            if abs(wrap(yaw - self.seg_yaw)) < 0.5:
                clusters, mask = self._obstacle_clusters(g)
                if new_scan:
                    self._remember_obstacles(clusters)
                # corner timing must see the wall, not an obstacle in front of it
                mask |= self._known_obstacle_mask(g)
                ahead = (g.along > 0.0) & (np.abs(g.lat) < 0.10) & ~mask
                g.front = float(np.min(g.along[ahead])) if np.any(ahead) else None
                self._front = g.front
            self.lateral_ref = self._plan_lateral_ref()

        if (self.state is State.TURN and new_scan
                and abs(wrap(yaw - self._turn_target)) < 0.5):
            self._look_into_next_lane(scan, yaw)
        self._track_line_order(orange, blue)
        if self.state is State.FOLLOW:
            cmd = self._follow(g, orange, blue)
        elif self.state is State.APPROACH:
            cmd = self._approach(g)
        elif self.state is State.TURN:
            cmd = self._turn()
        elif self.state is State.FINISH:
            cmd = self._finish(g)
        else:
            return Command()
        if self.state in (State.DONE, State.BACKOFF) or cmd.linear <= 0.0:
            return cmd
        safe = self._collision_filter(cmd, g)
        if safe is not None:
            return safe
        return self._blocked(g)

    # -------------------------------------------------------- collision check
    def _candidates(self, speeds, w_extra=()) -> tuple[np.ndarray, np.ndarray]:
        p = self.p
        ws = np.concatenate([list(w_extra), np.linspace(-p.max_angular, p.max_angular, 21)])
        V, W = np.meshgrid(np.asarray(speeds, dtype=float), ws, indexing="ij")
        V, W = V.ravel(), W.ravel()
        if p.min_turn_radius > 0.0:
            W = np.clip(W, -np.abs(V) / p.min_turn_radius, np.abs(V) / p.min_turn_radius)
        return V, W

    def _rollout(self, V: np.ndarray, W: np.ndarray, g: _Geometry, horizon: float):
        """Roll constant (v, w) arcs out ``horizon`` metres.

        Returns ``(ok, clearance, x_end, y_end, th_end)``: ``ok`` marks arcs
        that keep the footprint clear of every scan / memory point.
        """
        p = self.p
        mx, my = self._memory_points()
        px = np.concatenate([g.x, mx]) + p.lidar_x   # points in the base frame
        py = np.concatenate([g.y, my])
        sel = np.hypot(px, py) < horizon + 0.25
        px, py = px[sel], py[sel]
        steps = np.linspace(0.0, horizon, 10)
        sgn = np.where(V < 0, -1.0, 1.0)[:, None]            # reverse arcs too
        k = (W / np.maximum(np.abs(V), 1e-6))[:, None]      # yaw change per metre
        th = k * steps[None, :]
        small = np.abs(k) < 1e-6
        safe_k = np.where(small, 1.0, k)
        X = sgn * np.where(small, steps[None, :], np.sin(th) / safe_k)
        Y = sgn * np.where(small, 0.0, (1.0 - np.cos(th)) / safe_k)
        if px.size == 0:
            return (np.ones(V.size, bool), np.full(V.size, 9.0), X[:, -1], Y[:, -1], th[:, -1])
        dx = px[None, None, :] - X[:, :, None]
        dy = py[None, None, :] - Y[:, :, None]
        c, s_ = np.cos(th)[:, :, None], np.sin(th)[:, :, None]
        lx = c * dx + s_ * dy
        ly = -s_ * dx + c * dy
        x0 = p.lidar_x - p.body_rear
        x1 = p.lidar_x + p.body_front
        # distance from every point to the footprint rectangle at every step
        ddx = np.maximum(np.maximum(x0 - lx, lx - x1), 0.0)
        ddy = np.maximum(np.abs(ly) - p.half_width, 0.0)
        clear = np.hypot(ddx, ddy).min(axis=2)              # (candidates, steps)
        now = float(clear[0, 0])
        path_clear = clear[:, 1:].min(axis=1)
        # Required clearance: the margin, or (if already closer than the
        # margin) "do not get closer than now", so the car can always move
        # away from something it is touching.
        need = min(p.safety_margin, now - 0.002)
        ok = (path_clear >= need) & (path_clear > 0.003)
        return ok, path_clear, X[:, -1], Y[:, -1], th[:, -1]

    def _collision_filter(self, cmd: Command, g: _Geometry) -> Optional[Command]:
        """Return the command closest to ``cmd`` whose short arc is free.

        ``None`` means no forward motion is safe.
        """
        p = self.p
        if cmd.checked:
            return cmd
        speeds = [cmd.linear, max(min(cmd.linear, p.min_avoid_speed), 0.01)]
        V, W = self._candidates(speeds, [cmd.angular])
        V[0], W[0] = cmd.linear, cmd.angular
        ok, _, _, _, _ = self._rollout(V, W, g, p.safety_horizon)
        if ok[0]:
            return cmd
        if np.any(ok):
            cost = (W - cmd.angular) ** 2 + 2.0 * (V - cmd.linear) ** 2
            i = int(np.argmin(np.where(ok, cost, np.inf)))
            return self._limit(Command(float(V[i]), float(W[i])))
        return None

    def _blocked(self, g: _Geometry) -> Command:
        # No safe forward arc: back off.  Repeated blocks in the same state
        # flip the pass side of the nearest obstacle; if the car is still
        # boxed in after max_backoffs_per_state it stops (DONE, "blocked").
        if self._backoffs_in_state >= self.p.max_backoffs_per_state:
            self._enter(State.DONE, f"blocked in {self.state.value}")
            return Command()
        self._backoffs_in_state += 1
        self.backoffs += 1
        if self.state is not State.TURN and self._next_obstacle() is None:
            self._obstacle_from_blockage(g)
        elif self._backoffs_in_state >= 2:
            self._flip_nearest_obstacle_side()
        self._resume_state = self.state
        self._resume_clock = (self._state_t, self._state_along)
        self._backoff_from_along = self._along
        self._enter(State.BACKOFF, f"no safe forward arc in {self.state.value}")
        return self._backoff(g)

    def _track_line_order(self, orange: bool, blue: bool) -> None:
        # Near the inner corner both lines can be in view.  The colour that
        # appeared first is the leading one; equal onset stays undecided.
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

    def _follow(self, g: _Geometry, orange: bool, blue: bool) -> Command:
        p = self.p
        since_turn = self._along - self._seg_start_along
        line_ok = self.corners == 0 or since_turn > p.line_lockout
        trigger = None
        if line_ok and (orange or blue):
            if self.direction == 0:
                if self._first_line in ("orange", "blue"):
                    self.direction = CW if self._first_line == "orange" else CCW
                    trigger = "first line sets " + ("CW" if self.direction == CW else "CCW")
            elif (self.direction == CW and orange) or (self.direction == CCW and blue):
                trigger = "line"
        if trigger is None and g.front is not None and line_ok:
            # missed-line fallback: wall ahead and the turn side opens up
            if g.front < self._turn_front_dist() - p.fallback_margin:
                if self.direction != 0 and self._side_open(g, self.direction):
                    trigger = "fallback wall ahead"
                elif self.direction == 0:
                    left_open, right_open = self._side_open(g, CCW), self._side_open(g, CW)
                    if left_open != right_open:
                        self.direction = CCW if left_open else CW
                        trigger = "fallback sets direction"
        if trigger is not None:
            self._enter(State.APPROACH, trigger)
            return self._approach(g)

        speed = p.cruise_speed
        ob = self._next_obstacle()
        if ob is not None and ob.along - self._along < 0.6:
            speed = p.avoid_speed
            if self.lateral is not None and abs(self.lateral - self.lateral_ref) > 0.03:
                # not shifted yet: slow down so the heading change happens
                # in less distance (sharper path, also valid for Ackermann)
                gap = ob.along - self._along
                speed = clamp(0.3 * (gap - 0.10), p.min_avoid_speed, p.approach_speed)
        if g.front is not None and g.front < self._turn_front_dist() + 0.25:
            speed = min(speed, p.approach_speed)
        return self._lane_command(speed, g)

    def _approach(self, g: _Geometry) -> Command:
        p = self.p
        ready = (g.front is not None and g.front <= self._turn_front_dist())
        ob = self._next_obstacle()
        blocking = (ob is not None and g.front is not None
                    and ob.along - self._along < g.front)
        if blocking and self._elapsed() <= p.approach_timeout:
            # pass the obstacle first, then turn
            return self._lane_command(p.approach_speed, g)
        if ready or self._travel() >= p.approach_max_dist or self._elapsed() > p.approach_timeout:
            why = "front distance" if ready else "approach limit"
            if (self.corners == 0 and not self._side_open(g, self.direction)
                    and self._side_open(g, -self.direction)):
                # the corner opens to the other side: the line was misread
                self.direction = -self.direction
                why += "; direction corrected by open side"
            return self._start_turn(why)
        return self._lane_command(p.approach_speed, g)

    def _start_turn(self, why: str) -> Command:
        if self._start_to_first_turn is None:
            self._start_to_first_turn = self._along
        else:
            self._segment_lengths.append(self._along - self._seg_start_along)
        self._turn_target = wrap(self.seg_yaw + self.direction * math.pi / 2.0)
        self._enter(State.TURN, why)
        return self._turn()

    def _turn(self) -> Command:
        p = self.p
        err = wrap(self._turn_target - self._yaw)
        if abs(err) <= p.turn_tolerance:
            return self._finish_turn("yaw reached")
        if self._elapsed() > p.turn_timeout:
            return self._finish_turn("turn timeout")
        w = clamp(p.heading_kp * err, -p.turn_rate, p.turn_rate)
        if abs(w) < p.min_turn_angular:
            w = math.copysign(p.min_turn_angular, err)
        return self._limit(Command(p.turn_speed, w))

    def _finish_turn(self, why: str) -> Command:
        self.corners += 1
        self.seg_yaw = self._turn_target
        self._turn_target = None
        self._along = 0.0
        self._seg_start_along = 0.0
        self.obstacles.clear()
        for _ in range(2):          # seen during the turn: count as confirmed
            self._remember_obstacles(self._turn_clusters)
        self._turn_clusters = []
        self.lateral = self._next_lateral
        self._next_lateral = None
        self._lat_unmeasured = 0.0
        self.lateral_ref = 0.0
        if self.corners >= self.p.laps * self.p.corners_per_lap:
            self._enter(State.FINISH, why + "; final corner")
        else:
            self._enter(State.FOLLOW, why)
        return Command(self.p.turn_speed, 0.0)

    def _finish_dist(self) -> float:
        p = self.p
        if self._segment_lengths:
            seg = float(np.median(self._segment_lengths))
        else:
            seg = self._start_to_first_turn or 0.0
        start = self._start_to_first_turn or 0.0
        return max(seg - start + p.finish_extra, p.finish_min_dist)

    def _finish(self, g: _Geometry) -> Command:
        if self._travel() >= self._finish_dist():
            self._enter(State.DONE, "laps complete")
            return Command()
        return self._lane_command(self.p.approach_speed, g)

    def _backoff(self, g: _Geometry) -> Command:
        p = self.p
        backed = self._backoff_from_along - self._along
        cmd = None
        if backed < p.backoff_dist and self._elapsed() <= p.backoff_timeout:
            # Reverse while turning the nose further toward the planned side,
            # so the next forward attempt starts at a steeper angle.
            w_des = 0.0
            ob = self._next_obstacle()
            e = wrap(self._yaw - self.seg_yaw)
            if ob is not None and ob.side != 0 and self._resume_state is not State.TURN:
                if abs(e) < p.backoff_max_angle or e * ob.side < 0:
                    w_des = ob.side * p.backoff_turn_rate
            V, W = self._candidates([p.backoff_speed], [w_des])
            V[0], W[0] = p.backoff_speed, w_des
            ok, _, _, _, _ = self._rollout(V, W, g, p.backoff_check)
            if np.any(ok):
                i = int(np.argmin(np.where(ok, (W - w_des) ** 2, np.inf)))
                cmd = self._limit(Command(float(V[i]), float(W[i]), checked=True))
        if cmd is not None:
            return cmd
        # done, timed out, or reversing is not safe: resume the interrupted
        # state with its original clock so its own limits still apply
        resume = self._resume_state
        self.state = resume
        self.reason = "resume after backoff"
        self._state_t, self._state_along = self._resume_clock
        return Command()
