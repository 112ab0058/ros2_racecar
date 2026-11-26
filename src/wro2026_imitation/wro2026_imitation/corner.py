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
  delay     (timing 'ahead' only) the inner wall ends corner_ahead m in front of the
            lidar: drive straight another turn_delay m (default 0)
  turn      full lock to the inner side until the heading changed by turn_deg
  settle    like follow, but corners are ignored for settle_dist m (just after a turn
            the inner side can still look open)
  done      (stop_after_laps > 0) after 4 x stop_after_laps corners, back on the start
            straight (heading within 45 deg of the start's) with the start point at most
            finish_ahead m ahead along the start heading: STOP for good (2026-11-24, Ray:
            three laps, then stop where it started). finish_ahead 0.05 m: the wheels
            still turn ~5 cm after the stop (11-24 air test)
  blocked   (timing 'predict') the best full lock from here would pass a wall closer
            than min_room: STOP instead of turning into it. Each scan predicts
            again; once the turn fits (the car was driven back or put in the middle by
            hand) or no corner is ahead any more, follow again. 2026-11-26 field run 1: a
            start 4 cm from the island in the second half of a 0.39 m straight left
            room 0.002 m and touched the wall ahead (the prediction was 3 mm off); after
            each 重置 the prediction said room -0.09 m and the car locked into the wall
            anyway, three more times, counting each as a new corner.

When to go full lock (timing):
  'predict' (default, 2026-11-23) once the inner wall's end has been seen within
            entry_dist, every scan predicts the full-lock turn from where the car is now
            (turn_predict.py: this side's measured radius, the steering lag, the car's
            curvature, the outline) against the scan's points, and locks on the first
            scan where no later lock would leave more room. Nothing to tune: on 11-20 a
            fixed corner_ahead of 0.14 m passed ten left corners and cut the first right
            one, because the car turns 0.293 m to the right and 0.40 m to the left.
            If the end is not seen again for entry_dist m of driving, it was not a corner.
  'ahead'   (until 2026-11-22) lock when the inner wall ends corner_ahead m in front of
            the lidar

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

Start anywhere in the straight (2026-11-24, Ray: like WRO 2026, whose rules 9.4-9.8 put
the car in one of six zones of the start straight, facing the driving direction): from
the second half of the straight the island's end is in both sensors' blind spots and
was never in the memory either, so the car drove into the first corner (2-D field: 0/5
ccw from 0.45 m past the island's corner). crossing_end finds that end from the beams
that went past it to the far walls (the island's silhouette), see there.

decide(ranges, angle_min, angle_increment) -> (label, None); on_odom(x, y, yaw) feeds
the heading and driven distance (EKF odometry).
"""
import math

import numpy as np

from .features import FULL_LEFT, FULL_RIGHT, LEFT, RIGHT, STRAIGHT
from .turn_predict import FRONT, LASER_X, TurnPredictor

STOP = -1        # = policy.STOP (policy imports this module, so it cannot come from there)
FOLLOW, DELAY, TURN, SETTLE, DONE = 'follow', 'delay', 'turn', 'settle', 'done'
BLOCKED = 'blocked'

# Curvature (1/m, + = left) the R550 really drives for each decision, with bc_node's
# turn_radius 0.5 m and full_turn_radius 0.35 m at 0.1 m/s: steady gyro z / speed while
# the command was held >= 0.6 s, median over the 11-13 / 11-20 field bags
# (tools/real/steer_lag.py --steering). The steering is not symmetric: the same small
# turn drives 0.496 m to the left but 0.373 m to the right, full lock 0.403 / 0.292 m,
# and "straight" drifts slightly left (7.5 m radius).
DRIVEN = {STRAIGHT: 0.13, LEFT: 2.02, RIGHT: -2.68, FULL_LEFT: 2.48, FULL_RIGHT: -3.43}


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
    if inl.sum() < 6 or np.ptp(x[inl]) < 0.02:      # not a wall along the car
        return None
    a, b = np.polyfit(x[inl], y[inl], 1)
    return a, b, inl


class CornerPolicy:
    """See the module docstring. direction 'ccw' turns left, 'cw' right, 'auto' finds out."""

    takes_ranges = True

    def __init__(self, direction='ccw', corner_ahead=0.11, open_dist=0.5, min_open_beams=3,
                 turn_delay=0.0, turn_deg=80.0, settle_dist=0.3, lookahead=0.3,
                 deadband_deg=4.0, fit_window=(-0.25, 0.30), max_side=0.6, gap=0.06,
                 entry_offset=0.05, entry_dist=0.6, confirm_scans=3, off_line=0.10,
                 fit_min_span=0.10, fit_max_slope_deg=40.0, wall_dist=(0.12, 0.40),
                 timing='predict', driven=None, speed=0.1, steer_dead=0.27, steer_rate=13.0,
                 stop_after_laps=0, finish_ahead=0.05, crossing=True, cross_back=0.9,
                 min_room=0.01):
        """
        Set the rule's distances (m) and angles (deg); see the module docstring.

        crossing: also find the inner wall's end from the beams that went past it
        (crossing_end, for starts in the second half of a straight), and accept an end of
        the wall's own points only with such beams just past it (walk_end); False = the
        rule before 2026-11-24. cross_back: how far behind the lidar crossing_end's wall
        line is fitted (m).

        stop_after_laps: stop at the start after this many laps (0 = never); finish_ahead:
        stop when the start point is this close ahead (m), see state done.

        min_room: timing 'predict', the least room (m) the predicted full lock must leave,
        else the corner cannot be made from here: state blocked (STOP). 0.01: on the
        2-D field with a 0.39 m corridor every lock that made its corner left >= 0.020 m
        (12 laps from 0.20-0.45 m past the island's corner, 0-6.5 cm off the middle);
        the two that touched (6.5 cm towards the island, 0.39 / 0.45 m) left 0.006-0.007
        m, the field's run 1 0.002 m.

        timing 'predict' / 'ahead': see the module docstring. driven: curvature (1/m) the
        car really drives for each label (default DRIVEN, measured), speed (m/s),
        steer_dead (s) / steer_rate (1/m/s): the measured physics of the prediction, see
        turn_predict.py.

        fit_min_span, fit_max_slope_deg, wall_dist: a fitted inner wall counts only if its
        points spread over fit_min_span m, it is within fit_max_slope_deg of parallel and
        wall_dist m from the lidar (0 / None / None = no check, the 2026-11-13 rule).
        Replaying the 11-13 bags (LOGBOOK 2026-11-20): the false full locks of run 4 and of
        run 3 with the corrected lidar came from 6-7 points bunched within 3-5 cm behind
        the car (a line 11-25 deg off, 0.05 m from the lidar); the span and distance checks
        reject them. The slope limit is loose (40, not 20 deg): on the 11-20 field run 3
        the car came out of a turn still 21-25 deg off the next wall, a 20 deg limit
        rejected that real wall and the lock came 6.5 cm late.
        """
        if direction not in ('ccw', 'cw', 'auto'):
            raise ValueError("direction must be 'ccw', 'cw' or 'auto'")
        if timing not in ('predict', 'ahead'):
            raise ValueError("timing must be 'predict' or 'ahead'")
        if stop_after_laps < 0:
            raise ValueError('stop_after_laps must be >= 0')
        self.stop_after_laps = stop_after_laps
        self.finish_ahead = finish_ahead
        self.finish_end = (FRONT - LASER_X) + 0.05 + 0.03   # see laps_done
        self.crossing = crossing
        self.cross_back = cross_back
        self.end_line = None         # last inner_wall_end: (end, a, b) if from crossing_end
        self.wall_line = None        # last inner_wall_end: (end, a, b) by either way
        self.walk_line = None        # last walk_end: (a, b)
        self.island = None           # the inner wall's end last found, odom (_keep_island)
        self.end_seen = None         # predict: (driven distance, end ahead of the lidar) last seen
        # predict: no lock while the end is farther ahead of the lidar than this (m). The
        # right lock comes with the end -0.03..0.11 m ahead (left, 0.40 m radius; right
        # later). A car put down pointing 10 deg into the island 0.5 m before its end
        # (2-D field, cw) locked at once: the bumper's first contact was the island's wall
        # right ahead, which ready() counted as the wall ahead, "too late". The fix is
        # island_mask (ready() is told which points are the island); this stays as a
        # backstop.
        self.max_lock_end = 0.30
        self.finish_heading = math.radians(45.0)
        self.start = None            # (x, y, yaw) of the first odometry: where the run began
        self.driven = dict(DRIVEN if driven is None else driven)
        self.predictor = (TurnPredictor(self.driven[FULL_LEFT], self.driven[FULL_RIGHT],
                                        steer_dead, steer_rate, speed)
                          if timing == 'predict' else None)
        self.armed_odo = None        # predict: driven distance when the wall end was last seen
        self.min_room = min_room
        self.prediction = None       # predict: (room now, best room later, its wait) (m)
        self.trail = []              # (driven distance, heading) of the last odometry
        self.k_now = None            # curvature from the odometry's twist (1/m), if given
        self.last_label = STRAIGHT   # what the car is driving on (the last decision)
        self.sent = []               # (driven distance, label) of the recent decisions
        # + = inner side is the left; 0 = auto, not decided yet
        self.sign = {'ccw': 1.0, 'cw': -1.0, 'auto': 0.0}[direction]
        self.confirm_scans = confirm_scans
        self.off_line = off_line
        self.fit_min_span = fit_min_span
        self.fit_max_slope = (None if fit_max_slope_deg is None
                              else math.tan(math.radians(fit_max_slope_deg)))
        self.wall_dist = wall_dist
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

    def on_odom(self, x, y, yaw, vx=None, wz=None):
        """
        Take the newest odometry.

        Newest base_link pose in the odom frame (heading from the gyro); vx (m/s) and wz
        (rad/s): the odometry's twist, if there is one (the EKF's: gyro yaw rate).
        """
        if self.pose is not None:
            self.odo += math.hypot(x - self.pose[0], y - self.pose[1])
        if self.start is None:
            self.start = (x, y, yaw)
        self.pose = (x, y, yaw)
        self.k_now = wz / vx if vx is not None and wz is not None and vx > 0.05 else None
        self.trail.append((self.odo, yaw))
        while len(self.trail) > 2 and self.odo - self.trail[1][0] >= 0.02:
            self.trail.pop(0)

    def curvature(self):
        """
        Return the car's curvature now (1/m, + = left).

        Yaw rate / speed from the odometry's twist, else the heading change over the last
        ~2 cm driven.
        """
        if self.k_now is not None:
            return self.k_now
        if len(self.trail) < 2 or self.trail[-1][0] - self.trail[0][0] < 0.01:
            return 0.0
        (o0, y0), (o1, y1) = self.trail[0], self.trail[-1]
        return _wrap(y1 - y0) / (o1 - o0)

    def reset(self):
        """
        Start again in follow after the phone's 重置 unlatched a too_close stop.

        Keeps the direction (also auto's, once decided) and the corner count. A corner
        stopped half way is not finished: drive it by hand first, the rule only sees
        corners ahead. The last decision counts as straight (the steering was moved by
        hand or stopped).
        """
        self.state = FOLLOW
        self.mark_odo = self.odo
        self.armed_odo = None
        self.prediction = None
        self.island = None
        self.end_seen = None
        self.approach = False
        self.last_label = STRAIGHT
        self.sent = []
        self.seen = {1.0: 0, -1.0: 0}
        self.events.append((round(self.odo, 3), 'reset'))

    def _enter(self, state, note=''):
        self.state = state
        self.mark_odo = self.odo
        if state == TURN:
            self.mark_yaw = self.pose[2] if self.pose else 0.0
            self.corners += 1
            self.armed_odo = None
            self.island = None
            self.end_seen = None
        self.events.append((round(self.odo, 3), state + note))

    def inner_wall_end(self, r, angles, sign=None, off_line=0.0, min_span=0.0, max_slope=None,
                       fit_hi=None, wall_dist=None):
        """
        Return x (lidar frame, m) where the inner wall ends, or None if no wall / not open.

        From the wall's own points (walk_end); if they do not show it, from the beams that
        went past it (crossing_end, the island's silhouette). self.wall_line: (end, a, b)
        of the wall found (lidar frame, y = a x + b) or None; self.end_line: the same if it
        came from crossing_end, else None.
        """
        self.end_line = None
        self.wall_line = None
        self.walk_line = None
        end = self.walk_end(r, angles, sign, off_line, min_span, max_slope, fit_hi, wall_dist)
        if end is not None:
            self.wall_line = (end,) + self.walk_line
        if end is not None or not self.crossing:
            return end
        sign = self.sign if sign is None else sign
        slope = max_slope if max_slope is not None else math.tan(math.radians(40.0))
        found = self.crossing_end(r, angles, sign, slope, max(off_line, 0.05))
        if found is None:
            return None
        self.end_line = self.wall_line = found
        return found[0]

    def crossing_end(self, r, angles, sign, max_slope, beyond_line=0.05):
        """
        Return (end, a, b): the inner wall's end, from the beams that went past it; or None.

        The late start (2026-11-24, WRO 2026 rule 9.4/9.7: the car may be put anywhere in
        the start straight): from the second half of a straight the island's end is in both
        sensors' blind spots, the black wall returns nothing to the lidar closer than about
        0.3-0.47 m and the camera sees only +-45 deg. The lidar does see the walls far
        behind that end: the wall ahead runs on past the island as the next corridor's outer
        wall. A beam that returns from beyond the inner wall's line went through that line,
        so the island is not there; the island ends where the first such beams cross the
        line (its silhouette against the far wall). end is the first crossing ahead of the
        wall's points seen (behind them is the island's start) with min_open_beams
        crossings within 0.05 m (one stray return cannot make an end; taking the third
        crossing instead put far ends ~5 cm late, one beam is ~1.6 cm at 0.5 m), and the
        wall's own points must not go on within 0.15 m past it.

        The line is fitted to this side's points from cross_back m behind the lidar to
        entry_dist ahead (behind the car the black wall is seen from >= 0.47 m, or
        remembered), with parallel_line (within atan(max_slope) of parallel), spread over
        fit_min_span, 0.12-0.40 m from the lidar. Only beams landing beyond_line m or more
        beyond it count.
        """
        ok = np.isfinite(r)
        rr = np.where(ok, r, 0.0)
        x, y = rr * np.cos(angles), rr * np.sin(angles)
        side = sign * y
        cand = (ok & (side > 0.03) & (side < self.max_side) & (x > -self.cross_back)
                & (x < self.entry_dist))
        line = parallel_line(x, y, cand, max_slope)
        if line is None:
            return None
        # the wall is the longest run of line points without a 0.10 m gap: a point of
        # the wall ahead that happens to lie on the line (one point 0.9 m away) tilted
        # the fit by 2 deg and put the end 3 cm late (test, end 0.05 m)
        xi = np.sort(x[line[2]])
        runs = np.split(xi, np.nonzero(np.diff(xi) > 0.10)[0] + 1)
        run = max(runs, key=len)
        inl = line[2] & (x >= run[0]) & (x <= run[-1])
        if inl.sum() < 6 or np.ptp(x[inl]) < max(self.fit_min_span, 0.10):
            return None
        a, b = np.polyfit(x[inl], y[inl], 1)
        if abs(a) > max_slope or not 0.12 <= abs(b) / math.hypot(1.0, a) <= 0.40:
            return None
        xc = self._crossings(x, y, rr, angles, sign, a, b, beyond_line)
        # ahead of the wall seen: behind it the beams pass the island's START (the corner
        # before, 2-D field start 0.05 m past it: a full lock at once into the island)
        xc = xc[xc >= x[inl].max() - 0.03]
        xi = x[cand & (np.abs(a * x + b - y) < 0.03)]          # all points on the line
        n_in = np.searchsorted(xc, xc + 0.05, side='right') - np.arange(len(xc))
        for end in xc[n_in >= self.min_open_beams]:
            if not ((xi > end + 0.01) & (xi < end + 0.15)).any():
                return float(end), float(a), float(b)
        return None

    def walk_end(self, r, angles, sign=None, off_line=0.0, min_span=0.0, max_slope=None,
                 fit_hi=None, wall_dist=None):
        """
        Return x (lidar frame, m) where the wall's own points end, or None (no wall / not open).

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
        wall_dist (lo, hi): the same distance check for the least-squares line (follow).
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
        if wall_dist is not None and not (
                wall_dist[0] <= abs(b) / math.hypot(1.0, a) <= wall_dist[1]):
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
        # and beams that went through the line just past the end: the wall really ends
        # there. The last point seen can be where the lidar stops seeing a wall that goes
        # on (2-D field 11-24, cw start 0.56 m, 5 cm from the outer wall, 10 deg: six
        # points behind the car made an end 0.3 m before the island's, full lock at once
        # into the island; the 11-13 run 4 was the same kind)
        if self.crossing:
            xc = self._crossings(x, y, rr, angles, sign, a, b, 0.05)
            if ((xc >= end - 0.03) & (xc <= end + 0.15)).sum() < self.min_open_beams:
                return None
        self.walk_line = (float(a), float(b))
        return float(end)

    def _crossings(self, x, y, rr, angles, sign, a, b, beyond_line):
        """
        Return where beams that went through the line y = a x + b cross it (x, sorted).

        Beams that measured farther than open_dist and landed beyond_line m or more beyond
        the line (on the sign side), lidar frame.
        """
        ok = rr > 0.0
        beyond = ok & (rr > self.open_dist) & (sign * (y - (a * x + b)) > beyond_line)
        den = np.sin(angles) - a * np.cos(angles)
        with np.errstate(divide='ignore', invalid='ignore'):
            t = b / den                                   # along the beam to the line
        cross = beyond & np.isfinite(t) & (t > 0.0) & (t < rr)
        return np.sort((t * np.cos(angles))[cross])

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

    def _keep_island(self):
        """
        Store the inner wall's end just found (self.wall_line) in the odom frame.

        self.island = (end x, end y, wall heading, driven distance, from crossing_end?).
        """
        if self.wall_line is None or self.pose is None:
            return
        end, a, b = self.wall_line
        x, y, yaw = self.pose
        bx, by = end + LASER_X, a * end + b                  # base_link frame
        c, s = math.cos(yaw), math.sin(yaw)
        self.island = (x + c * bx - s * by, y + s * bx + c * by, yaw + math.atan(a), self.odo,
                       self.end_line is not None)

    def _island_now(self):
        """Return the island (end, wall direction) in base_link now, or None if not kept."""
        if self.island is not None and self.odo - self.island[3] > self.entry_dist:
            self.island = None
        if self.island is None or self.pose is None:
            return None
        ex, ey, wyaw = self.island[:3]
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * (ex - x) + s * (ey - y), -s * (ex - x) + c * (ey - y), wyaw - yaw)

    def island_points(self, length=0.6, step=0.01):
        """
        Return the island's wall up to the end crossing_end found: (x, y) in base_link (m).

        The end itself is in the blind spot and in no scan, so the turn prediction would
        see no island there and lock too early; these points (the wall's line, length m
        back from the end) stand in for it. Forgotten after entry_dist m of driving, and
        as soon as a scan shows the wall's own end (walk_end): a stand-in kept from a far,
        4.5 cm late crossing then made the lock late (2-D field, start 0.10 m, seed 1).
        """
        now = self._island_now()
        if now is None or not self.island[4]:
            return np.zeros(0), np.zeros(0)
        ex, ey, wyaw = now
        d = np.arange(0.0, length + 1e-9, step)
        return ex - d * math.cos(wyaw), ey - d * math.sin(wyaw)

    def island_mask(self, px, py, tol=0.03):
        """
        Return which points (base_link, m) are the island, or None if no island is kept.

        The island: on the inner wall's line or beyond it (towards the inner side, tol m
        slack) and not past its end along the wall. The turn prediction uses it to tell
        "touches the island first: too early" from "touches the wall ahead or the outer
        wall first: too late" (TurnPredictor._curves).
        """
        now = self._island_now()
        if now is None:
            return None
        ex, ey, wyaw = now
        c, s = math.cos(wyaw), math.sin(wyaw)
        along = (px - ex) * c + (py - ey) * s
        across = -(px - ex) * s + (py - ey) * c                # + = left of the wall's line
        with np.errstate(invalid='ignore'):
            return (self.sign * across > -tol) & (along < tol)

    def _predict(self, r, angles):
        """Arm on a wall end in follow (timing 'predict'); lock when the prediction says so."""
        if self.approach:
            self.armed_odo = self.odo
        elif self.armed_odo is not None and self.odo - self.armed_odo > self.entry_dist:
            self.armed_odo = None                 # the end was not seen again: no corner
        if self.armed_odo is None:
            self.prediction = None
            self._unblock(' (no corner ahead)')
            return
        self.approach = True                      # keep the wide entry until the lock
        ok = np.isfinite(r)
        rr = np.where(ok, r, 0.0)
        px = np.where(ok, rr * np.cos(angles) + LASER_X, np.nan)      # base_link frame
        py = np.where(ok, rr * np.sin(angles), np.nan)
        ix, iy = self.island_points()
        px, py = np.concatenate([px, ix]), np.concatenate([py, iy])
        queued = [(self.odo - o, self.driven[lab]) for o, lab in self.sent] or None
        lock, now, later, wait = self.predictor.ready(self.sign, px, py, self.curvature(),
                                                      self.driven[self.last_label],
                                                      queued=queued,
                                                      island=self.island_mask(px, py))
        self.prediction = (now, later, wait)
        if lock and self.end_seen is not None:
            # never lock while the end is still far ahead (see max_lock_end)
            lock = self.end_seen[1] - (self.odo - self.end_seen[0]) <= self.max_lock_end
        if lock and now < self.min_room:
            # no later lock does better and this one touches: do not turn into the wall
            if self.state != BLOCKED:
                self._enter(BLOCKED, ' (room %.3f m: the turn does not fit from here)' % now)
            return
        self._unblock(' (the turn fits again)')
        if lock:
            self._enter(TURN, ' (room %.3f m; best later %.3f m%s)' % (
                now, later,
                '; island end from the beams past it' if self.island and self.island[4] else ''))

    def planned_turn(self):
        """
        Return (sign, heading still to turn, rad) while turning a corner, else None.

        Driver's predicted stop follows the rest of the full-lock turn (state turn, until
        the heading changed by turn_deg) with it.
        """
        if self.state != TURN or self.pose is None:
            return None
        turned = abs(_wrap(self.pose[2] - self.mark_yaw))
        return self.sign, max(self.turn_rad - turned, 0.0)

    def _unblock(self, note):
        """Leave state blocked (back to follow), if in it."""
        if self.state == BLOCKED:
            self._enter(FOLLOW, note)

    def __call__(self, ranges, angle_min, angle_increment):
        """Decide from one (filled) scan; see the module docstring."""
        label = self._decide(ranges, angle_min, angle_increment)
        if label == STOP:                 # done: not a steering command to remember
            return label, None
        self.last_label = label
        if not self.sent or self.sent[-1][1] != label:
            self.sent.append((self.odo, label))
        # keep what can still be on its way to the steering, and the one before
        while len(self.sent) > 1 and self.odo - self.sent[1][0] > 0.1:
            self.sent.pop(0)
        return label, None

    def laps_done(self, end=None):
        """
        Return True when the set laps are driven and the car is back at the start (done).

        If the start was placed close to a corner, the last turn can end past it: then the
        car stops right after that turn (up to ~10 cm past the start, 2-D field 11-24).
        end: the inner wall's end ahead (lidar frame, m), if seen. A start in the second
        half of the straight lies where the car already turns for the next corner; WRO
        2026 (9.24.2) wants the car to stop completely inside the start straight, so it
        also stops once the front bumper, plus the ~5 cm the wheels still roll, would
        come within 3 cm of the straight's end (end <= finish_end).
        """
        if (not self.stop_after_laps or self.corners < 4 * self.stop_after_laps
                or self.pose is None or self.start is None):
            return False
        x, y, yaw = self.pose
        sx, sy, syaw = self.start
        if abs(_wrap(yaw - syaw)) > self.finish_heading:
            return False                              # not on the start straight's heading
        ahead = (sx - x) * math.cos(syaw) + (sy - y) * math.sin(syaw)
        return ahead <= self.finish_ahead or (end is not None and end <= self.finish_end)

    def _decide(self, ranges, angle_min, angle_increment):
        if self.state in (FOLLOW, SETTLE) and self.laps_done():
            self._enter(DONE)
        if self.state == DONE:
            return STOP
        r = np.asarray(ranges, dtype=float)
        angles = angle_min + angle_increment * np.arange(len(r))
        angles = (angles + math.pi) % (2.0 * math.pi) - math.pi
        self.approach = False
        if self.sign == 0.0:
            self._find_direction(r, angles)
            if self.sign == 0.0:
                return self.center_label(r, angles)
        full = FULL_LEFT if self.sign > 0 else FULL_RIGHT
        if self.state in (FOLLOW, BLOCKED):
            end = self.inner_wall_end(r, angles, min_span=self.fit_min_span,
                                      max_slope=self.fit_max_slope, wall_dist=self.wall_dist)
            if self.laps_done(end):
                self._enter(DONE)
                return STOP
            self.approach = end is not None and end <= self.entry_dist
            if self.approach:
                self.end_seen = (self.odo, end)
                self._keep_island()       # from walk_end: no stand-in points (see there)
            if self.predictor is None:
                if end is not None and end <= self.corner_ahead:
                    self._enter(DELAY)
            else:
                self._predict(r, angles)
        if self.state == DELAY and self.odo - self.mark_odo >= self.turn_delay:
            self._enter(TURN)
        if self.state == TURN:
            turned = abs(_wrap(self.pose[2] - self.mark_yaw)) if self.pose else 0.0
            if turned < self.turn_rad:
                return full
            self._enter(SETTLE)
        if self.state == SETTLE and self.odo - self.mark_odo >= self.settle_dist:
            self._enter(FOLLOW)
        if self.state == BLOCKED:
            return STOP
        if self.state == DELAY:
            return STRAIGHT
        return self.center_label(r, angles)
