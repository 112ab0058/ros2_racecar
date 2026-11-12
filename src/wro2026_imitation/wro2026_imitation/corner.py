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


class CornerPolicy:
    """See the module docstring. direction 'ccw' turns left at corners, 'cw' right."""

    takes_ranges = True

    def __init__(self, direction='ccw', corner_ahead=0.11, open_dist=0.5, min_open_beams=3,
                 turn_delay=0.0, turn_deg=80.0, settle_dist=0.3, lookahead=0.3,
                 deadband_deg=4.0, fit_window=(-0.25, 0.30), max_side=0.6, gap=0.06,
                 entry_offset=0.05, entry_dist=0.6):
        """Set the rule's distances (m) and angles (deg); see the module docstring."""
        if direction not in ('ccw', 'cw'):
            raise ValueError("direction must be 'ccw' or 'cw'")
        self.sign = 1.0 if direction == 'ccw' else -1.0     # + = inner side is the left
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

    def inner_wall_end(self, r, angles):
        """
        Return x (lidar frame, m) where the inner wall ends, or None if no wall / not open.

        The inner wall line is fitted near the car (fit_window); its points are walked
        forward until a gap wider than gap. The end counts only if at least
        min_open_beams beams pass just beyond it and measure farther than open_dist.
        """
        ok = np.isfinite(r)
        rr = np.where(ok, r, 0.0)
        x, y = rr * np.cos(angles), rr * np.sin(angles)
        side = self.sign * y
        lo, hi = self.fit_window
        cand = ok & (side > 0.03) & (side < self.max_side) & (x > lo)
        near = cand & (x <= hi)
        if near.sum() < 6:
            return None
        a, b = np.polyfit(x[near], y[near], 1)
        res = np.abs(a * x + b - y) / math.hypot(1.0, a)
        on = cand & (res < 0.03)
        if on.sum() < 6:
            return None
        xs = np.sort(x[on])
        start = np.searchsorted(xs, hi, side='right') - 1      # last point inside the window
        steps = np.diff(xs[max(start, 0):])
        jump = np.nonzero(steps > self.gap)[0]
        end = xs[max(start, 0) + jump[0]] if len(jump) else xs[-1]
        beyond = ok & (x > end + 0.03) & (side > 0.0) & (r > self.open_dist)
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
        full = FULL_LEFT if self.sign > 0 else FULL_RIGHT
        self.approach = False
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
