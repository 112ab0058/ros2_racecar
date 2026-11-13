"""
Ray's corner technique as a rule policy (2026-11-11, school field). Pure numpy, no ROS.

    "Keep the car in the centre. When the front wheels have fully passed the inner wall,
     go full lock. After the turn, small turns to get back to the centre. Every corner
     is the same."

States:
  follow    keep centred with small turns (aim at a point lookahead m ahead on the
            centre line; walls found by fitting a line to each side's points); when the
            inner wall ends less than entry_dist ahead, aim entry_offset m towards the
            outer wall instead (racing line: a wide entry widens the turn-in window)
  delay     the inner wall ends corner_ahead m in front of the lidar: drive straight
            another turn_delay m (default 0)
  turn      full lock to the inner side until the heading changed by turn_deg
  settle    like follow, but corners are ignored for settle_dist m (just after a turn
            the inner side can still look open)

Defaults corner_ahead 0.11 m, entry_offset 0.05 m: best of a 2026-11-12 sweep over 72
field / car variations (corridor 0.39-0.43 m, driven full-lock radius 0.38-0.42 m,
steering lag 0.15-0.3 s, heading error 0-2 %): 62 of 72 laps without stopping, median
closest approach 3.6 cm; the failures were all in the 0.39 m corridor with a 0.40-0.42 m
radius and slow steering. (0.07 m and no wide entry: 46 of 72.)

Where to turn (2-D field check, field_sim.py): at full lock the car drives a 0.40 m
radius, the front outer corner swings out 0.546 m from the turn centre and the inner side
passes 0.31 m from it. In a 0.40 m corridor the rear axle must start the turn 0.15-0.24 m
before the island corner, so the trigger is geometric: the end of the inner wall (last
point of the fitted inner wall line, walking forward until a gap) has come to
corner_ahead m in front of the lidar, and beams just past that end really measured
something farther than open_dist (seen open, not blind).

direction 'auto' (Ray, 2026-11-13: "the car should see which side is the wall and pick
the turn itself"): until the first corner it only keeps centred; the first side whose
wall ends within entry_dist with open space beyond it (beams > 0.10 m past the wall's
line, so a gap in a wall does not count; the line fitted to the points up to 0.8 m ahead,
spread over >= 0.10 m and within 20 deg of parallel) on confirm_scans scans in a row
while the other
side shows no end is the inner side, and every corner turns that way. The outer wall
never "ends" like that: at a corner it runs into the wall ahead.

It needs the scan with the blind beams filled (scan_memory_node): the black walls give
no return closer than about 0.4 m, and a blind inner side would look "open" everywhere.

decide(ranges, angle_min, angle_increment) -> (label, None); on_odom(x, y, yaw) feeds
the heading and driven distance (EKF odometry).
"""
import math

import numpy as np

from .features import LEFT, RIGHT, STRAIGHT

FULL_LEFT, FULL_RIGHT = 3, 4
FOLLOW, DELAY, TURN, SETTLE = 'follow', 'delay', 'turn', 'settle'


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def parallel_line(x, y, use, max_slope, band=0.02, cell=0.02):
    """
    Find the wall that runs nearly parallel to the car among the points x[use], y[use].

    Tries directions within atan(max_slope) of parallel (2 deg steps) and offsets, and
    keeps the line whose points (within band of it) cover the most cell-long pieces along
    the car, so a wall across the car's path (the wall ahead at a corner) cannot pull the
    line round as it does a least-squares fit, and a short object close to the car (dense
    returns: a hand 1-2 cm from the side in Ray's 11-11 cw bag) does not beat a long wall.
    Return (a, b, inliers) with y = a x + b, inliers a mask like use; None if < 6 points.
    """
    xs, ys = x[use], y[use]
    if len(xs) < 6:
        return None
    lim = math.degrees(math.atan(max_slope))
    cells = np.floor(xs / cell).astype(int)
    best = (0, 0, 0.0, 0.0)
    for deg in np.arange(-lim, lim + 1e-9, 2.0):
        th = math.radians(deg)
        d = ys * math.cos(th) - xs * math.sin(th)              # offset across direction th
        order = np.argsort(d)
        d, c = d[order], cells[order]
        hi = np.searchsorted(d, d + band)                      # points in [d[i], d[i] + band]
        starts = np.unique(np.searchsorted(d, np.round(d / 0.01) * 0.01))   # 1 cm steps
        for i in starts[starts < len(d)]:
            score = (len(np.unique(c[i:hi[i]])), hi[i] - i)
            if score > best[:2]:
                best = (score[0], score[1], th, d[i] + band / 2.0)
    _, _, th, d0 = best
    inl = use & (np.abs(y * math.cos(th) - x * math.sin(th) - d0) <= band)
    if inl.sum() < 6:
        return None
    a, b = np.polyfit(x[inl], y[inl], 1)
    return a, b, inl


class CornerPolicy:
    """See the module docstring. direction 'ccw' turns left, 'cw' right, 'auto' finds out."""

    takes_ranges = True

    def __init__(self, direction='ccw', corner_ahead=0.11, open_dist=0.5, min_open_beams=3,
                 turn_delay=0.0, turn_deg=80.0, settle_dist=0.3, lookahead=0.3,
                 deadband_deg=4.0, fit_window=(-0.25, 0.30), max_side=0.6, gap=0.06,
                 entry_offset=0.05, entry_dist=0.6, confirm_scans=3, off_line=0.10):
        """Set the rule's distances (m) and angles (deg); see the module docstring."""
        if direction not in ('ccw', 'cw', 'auto'):
            raise ValueError("direction must be 'ccw', 'cw' or 'auto'")
        # + = inner side is the left; 0 = auto, not decided yet
        self.sign = {'ccw': 1.0, 'cw': -1.0, 'auto': 0.0}[direction]
        self.confirm_scans = confirm_scans
        self.off_line = off_line
        self.seen = {1.0: 0, -1.0: 0}    # auto: scans in a row on which that side ended
        self.auto_fit_hi = 0.8           # auto: fit the wall up to this far ahead (m)
        self.auto_min_span = 0.10        # auto: fitted points spread over at least (m)
        self.auto_max_slope_deg = 20.0   # auto: wall line within this of parallel
        self.corner_ahead = corner_ahead
        self.open_dist = open_dist
        self.min_open_beams = min_open_beams
        self.gap = gap
        self.entry_offset = entry_offset
        self.entry_dist = entry_dist
        self.approach = False        # inner wall ends within entry_dist: take the wide line
        self.turn_delay = turn_delay
        self.turn_rad = math.radians(turn_deg)
        self.settle_dist = settle_dist
        self.lookahead = lookahead
        self.deadband = math.radians(deadband_deg)
        self.fit_window = fit_window
        self.max_side = max_side
        self.state = FOLLOW
        self.pose = None
        self.odo = 0.0
        self.mark_odo = 0.0          # driven distance when the state started
        self.mark_yaw = 0.0          # heading when the turn started
        self.corners = 0
        self.events = []             # (driven distance, new state) for logs

    @property
    def direction(self):
        """'ccw' (corners to the left), 'cw' (to the right), or None: auto, not decided yet."""
        return {1.0: 'ccw', -1.0: 'cw'}.get(self.sign)

    def on_odom(self, x, y, yaw):
        """Newest base_link pose in the odom frame (heading from the gyro)."""
        if self.pose is not None:
            self.odo += math.hypot(x - self.pose[0], y - self.pose[1])
        self.pose = (x, y, yaw)

    def _enter(self, state):
        self.state = state
        self.mark_odo = self.odo
        if state == TURN:
            self.mark_yaw = self.pose[2] if self.pose else 0.0
            self.corners += 1
        self.events.append((round(self.odo, 3), state))

    def inner_wall_end(self, r, angles, sign=None, off_line=0.0, min_span=0.0, max_slope=None,
                       fit_hi=None):
        """
        Return x (lidar frame, m) where the inner wall ends, or None if no wall / not open.

        The inner wall line is fitted near the car (fit_window); its points are walked
        forward until a gap wider than gap. The end counts only if at least
        min_open_beams beams pass just beyond it and measure farther than open_dist
        (and, with off_line > 0, land that far beyond the wall's line, away from the car).
        sign: side to look at (+1 left, -1 right), default the inner side.
        min_span, max_slope (auto): the fitted points must spread over min_span m and the
        line must be within atan(max_slope) of parallel; 6 points bunched at the end of
        the window gave a slope of -0.8 and a wrong 'open' side (2026-11-13 sim).
        fit_hi (auto): use the points up to this far ahead instead (wall seen directly,
        beyond the blind range), find the line with parallel_line, not least squares, and
        only a line 0.12-0.40 m from the lidar counts (a corridor wall).
        """
        sign = self.sign if sign is None else sign
        ok = np.isfinite(r)
        rr = np.where(ok, r, 0.0)
        x, y = rr * np.cos(angles), rr * np.sin(angles)
        side = sign * y
        lo, hi = self.fit_window
        if fit_hi is not None:
            hi = fit_hi
        cand = ok & (side > 0.03) & (side < self.max_side) & (x > lo)
        near = cand & (x <= hi)
        if near.sum() < 6:
            return None
        if fit_hi is None:
            a, b = np.polyfit(x[near], y[near], 1)
        else:
            line = parallel_line(x, y, near, max_slope if max_slope is not None else 0.36)
            if line is None:
                return None
            a, b, near = line
            # a corridor wall, not something against the car: 0.12-0.40 m from the lidar
            if not 0.12 <= abs(b) / math.hypot(1.0, a) <= 0.40:
                return None
        if np.ptp(x[near]) < min_span or (max_slope is not None and abs(a) > max_slope):
            return None
        res = np.abs(a * x + b - y) / math.hypot(1.0, a)
        on = cand & (res < 0.03)
        if on.sum() < 6:
            return None
        xs = np.sort(x[on])
        # walk from the last point inside fit_window (not fit_hi: a few points of the wall
        # ahead lie on the inner wall's line and would make the walk start beyond the end)
        start = np.searchsorted(xs, self.fit_window[1], side='right') - 1
        steps = np.diff(xs[max(start, 0):])
        jump = np.nonzero(steps > self.gap)[0]
        end = xs[max(start, 0) + jump[0]] if len(jump) else xs[-1]
        beyond = ok & (x > end + 0.03) & (side > 0.0) & (r > self.open_dist)
        if off_line > 0.0:
            beyond &= sign * (y - (a * x + b)) > off_line
        if beyond.sum() < self.min_open_beams:
            return None
        return float(end)

    def walls(self, r, angles):
        """
        Fit a line to each side's points near the car (lidar frame).

        Return {'left': (distance, angle) or None, 'right': ...}; angle is the wall's
        direction (rad) seen from the car, 0 = parallel to the car.
        """
        ok = np.isfinite(r) & (r < 2.0)
        x, y = r[ok] * np.cos(angles[ok]), r[ok] * np.sin(angles[ok])
        lo, hi = self.fit_window
        out = {}
        for name, side in (('left', 1.0), ('right', -1.0)):
            m = (x >= lo) & (x <= hi) & (side * y > 0.03) & (side * y < self.max_side)
            out[name] = None
            for _ in range(2):                    # fit, drop points > 3 cm off, fit again
                if m.sum() < 6:
                    break
                a, b = np.polyfit(x[m], y[m], 1)
                res = np.abs(a * x + b - y) / math.hypot(1.0, a)
                out[name] = (abs(b) / math.hypot(1.0, a), math.atan(a))
                m = m & (res < 0.03)
        return out

    def _find_direction(self, r, angles):
        """Auto: count, per side, scans in a row on which that side's wall ends (see top)."""
        for s in (1.0, -1.0):
            end = self.inner_wall_end(r, angles, sign=s, off_line=self.off_line,
                                      min_span=self.auto_min_span,
                                      max_slope=math.tan(math.radians(self.auto_max_slope_deg)),
                                      fit_hi=self.auto_fit_hi)
            self.seen[s] = self.seen[s] + 1 if end is not None and end <= self.entry_dist else 0
        left = self.seen[1.0] >= self.confirm_scans
        right = self.seen[-1.0] >= self.confirm_scans
        if left != right:
            self.sign = 1.0 if left else -1.0
            self.events.append((round(self.odo, 3), 'direction ' + self.direction))

    def center_label(self, r, angles):
        """Small-turn label that steers to the centre line (follow / settle)."""
        w = self.walls(r, angles)
        left, right = w['left'], w['right']
        if left and right:
            offset = (left[0] - right[0]) / 2.0               # + = car right of the centre
            heading = -(left[1] + right[1]) / 2.0             # + = car points left
        elif left or right:
            d, ang = left or right
            half = 0.2                                        # field corridor, about 0.4 m
            offset = (d - half) if left else (half - d)
            heading = -ang
        else:
            return STRAIGHT
        want = self.sign * self.entry_offset if self.approach else 0.0
        u = math.atan2(offset - want, self.lookahead) - heading
        if u > self.deadband:
            return LEFT
        if u < -self.deadband:
            return RIGHT
        return STRAIGHT

    def __call__(self, ranges, angle_min, angle_increment):
        """Decide from one (filled) scan; see the module docstring."""
        r = np.asarray(ranges, dtype=float)
        angles = angle_min + angle_increment * np.arange(len(r))
        angles = (angles + math.pi) % (2.0 * math.pi) - math.pi
        self.approach = False
        if self.sign == 0.0:
            self._find_direction(r, angles)
            if self.sign == 0.0:
                return self.center_label(r, angles), None
        full = FULL_LEFT if self.sign > 0 else FULL_RIGHT
        if self.state == FOLLOW:
            end = self.inner_wall_end(r, angles)
            self.approach = end is not None and end <= self.entry_dist
            if end is not None and end <= self.corner_ahead:
                self._enter(DELAY)
        if self.state == DELAY and self.odo - self.mark_odo >= self.turn_delay:
            self._enter(TURN)
        if self.state == TURN:
            turned = abs(_wrap(self.pose[2] - self.mark_yaw)) if self.pose else 0.0
            if turned < self.turn_rad:
                return full, None
            self._enter(SETTLE)
        if self.state == SETTLE and self.odo - self.mark_odo >= self.settle_dist:
            self._enter(FOLLOW)
        if self.state == DELAY:
            return STRAIGHT, None
        return self.center_label(r, angles), None
