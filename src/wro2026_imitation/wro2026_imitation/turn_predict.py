"""
When to go full lock at a corner, by predicting the turn (2026-11-23). Pure numpy, no ROS.

Ray (2026-11-20): stop re-tuning corner_ahead; a fixed "lock this far before the end of
the inner wall" ignores where the car is in the corridor, its heading, the radius it
really drives on that side and the steering lag. On 11-20 the same 0.14 m passed ten
left corners and cut the first right corner: the car turns 0.292 m to the right but
0.403 m to the left.

So at every scan near a corner, predict the turn. The path uses only measured physics
(tools/real/steer_lag.py --steering on the 11-13 / 11-20 bags, 266 command changes):
  steering   a rate-limited servo: the curvature moves towards the commanded one at
             rate 13 (1/m)/s (median; p25 10.5, p75 14.6), starting dead 0.27 s after the
             scan = 0.18 s from base_node's command (curvature moved 0.3 1/m after 0.20 s)
             + 0.09 s from the lidar measuring to that command (half a 0.1 s turn of the
             lidar, scan_memory_node 0.018 s, bc_node publishing every 0.05 s)
  full lock  driven curvature 2.48 1/m to the left (radius 0.403 m), -3.43 to the right
             (0.292 m); the 2026-11-20 circles gave 0.394 / 0.293 m
  now        the car's curvature now (gyro / speed) and the curvature its current command
             drives (CornerPolicy's table): until the lock acts the steering keeps moving
             to the current command. A car just told to straighten after a right
             correction is still turning right at the scan (11-20 run 6, corner 2: -1.2
             at the command, -2.8 0.3 s before).
The obstacles are the filled scan's points (measured and remembered walls). The clearance
of a turn is the smallest gap between any point and the car's outline anywhere along the
path, measured like the too_close stop (rectangle grown on every side).

ready() decides on every scan. If full lock now would touch something, the side touched
first tells which way is wrong: the inner side (the island) = too early, wait; the outer
side or the bumper (the wall ahead) = too late, lock at once. If it touches nothing, it
compares the clearance of locking now with locking after driving on with the current
command for another d = 0.01, 0.02, ... m: too early, waiting gives more room on the
island side; too late, less on the wall-ahead side. The turn starts on the first scan
where locking now leaves the most room: both sides then get the same room (the top of the
curve), and every scan of waiting would only take room away.

Checked (2026-11-23): 2-D field with the measured car, both ways pass (closest 2.9 cm
ccw, 4.7 cm cw; no single corner_ahead passes both); replaying the 11-20 field runs, it
says the cw lock of run 7 came while the turn would touch the inner side first (the car
did) and the run-1 lock 1 cm early (it latched at the inner corner). Camera wall points
that are 5-15 cm off make it see walls that are not there (run 6 corners 6 and 10).
"""
import math

import numpy as np

# Car outline from the rear axle (base_link) (R550_HARDWARE.md); lidar 0.13 m ahead (urdf).
FRONT, REAR, HALF_WIDTH, LASER_X = 0.24, 0.031, 0.09, 0.13


class TurnPredictor:
    """Predicted full-lock turn and its clearance; see the module docstring."""

    def __init__(self, k_full_left=2.48, k_full_right=-3.43, dead=0.27, rate=13.0, speed=0.1,
                 sweep_deg=90.0, reach=1.2, step=0.02, wait=0.25, wait_step=0.01, margin=0.2):
        """
        Set the measured steering and how the turn is searched.

        k_full_left / k_full_right: driven curvature at full lock (1/m); dead (s), rate
        (1/m/s): the steering, see the module docstring; speed (m/s) turns times into
        distances; sweep_deg: how far round the turn is followed; reach: points farther
        than this from the rear axle are ignored (m); step: path resolution (m; 0.02 m of
        path moves the outline's corners about 3 cm, the gap error between samples is
        < 0.3 mm); wait, wait_step: longest wait looked at and its resolution (m; 0.01 =
        one scan at 0.1 m/s); margin: points farther than this from the swept area are
        skipped (m; a clearance above it is not told apart from 'no point', the field's
        corridors give < 0.12 m).
        """
        if (k_full_left <= 0 or k_full_right >= 0 or min(speed, step, wait, wait_step, rate) <= 0
                or dead < 0 or wait < wait_step):
            raise ValueError('k_full_left > 0 > k_full_right; speed, steps, wait, rate > 0; '
                             'wait >= wait_step; dead >= 0')
        self.k_full = {1.0: k_full_left, -1.0: k_full_right}
        self.dead, self.rate = dead, rate
        self.speed = speed
        self.sweep = math.radians(sweep_deg)
        self.reach = reach
        self.step = step
        self.substep = 0.0025            # m: steering integration (0.025 s at 0.1 m/s)
        self.margin = margin
        self.waits = np.arange(0.0, wait + 1e-9, wait_step)

    def schedule(self, k0, k_cmd=None, queued=None):
        """
        Return where along the path each command already sent starts to act.

        [(path distance from which it acts, m; driven curvature, 1/m)] of the commands
        already sent. queued: [(how far the car has driven since it was sent, m; driven
        curvature)] oldest first, the last one is the current command: a command acts
        dead s after it was sent, so the ones sent within the last dead s are still on
        their way (11-20 run 6 corner 2: "straight" sent 0.15 s before the lock while the
        car was still turning right). k_cmd alone: one command sent long ago. Neither: the
        car keeps its curvature k0.
        """
        if queued is None:
            queued = [(1e9, k0 if k_cmd is None else k_cmd)]
        dead_m = self.dead * self.speed
        return [(dead_m - age, k) for age, k in queued]

    def path(self, sign, k0=0.0, k_cmd=None, wait=0.0, queued=None):
        """
        Return the rear-axle poses of the predicted turn.

        Rear-axle poses (n, 3: x, y, yaw) in today's base_link frame: drive on with the
        current command for wait m, then full lock to the left (sign +1) or right (-1),
        until the heading has turned sweep_deg from where the lock was commanded.
        k0: curvature now (1/m); k_cmd / queued: the commands already sent (schedule).
        """
        sched = self.schedule(k0, k_cmd, queued)
        sched.append((wait + self.dead * self.speed, self.k_full[sign]))   # the lock
        starts = [st for st, _ in sched]
        targets = [k0] + [k for _, k in sched]
        h = self.substep                 # the steering is integrated finely: with the 2 cm
        dk = self.rate * h / self.speed  # pose step, the 2.7 cm dead distance became 4 cm
        every = max(1, int(round(self.step / h)))
        x = y = yaw = s = 0.0
        k = k0
        yaw_lock = None
        poses = [(0.0, 0.0, 0.0)]
        n = 0
        while s < 6.0:
            if yaw_lock is None and s >= wait - 1e-9:
                yaw_lock = yaw
            if yaw_lock is not None and sign * (yaw - yaw_lock) >= self.sweep:
                break
            i = 0
            while i < len(starts) and starts[i] <= s + 1e-9:
                i += 1
            target = targets[i]
            k1 = k + min(max(target - k, -dk), dk)
            mid = yaw + 0.25 * (k + k1) * h
            x += h * math.cos(mid)
            y += h * math.sin(mid)
            yaw += 0.5 * (k + k1) * h
            k = k1
            s += h
            n += 1
            if n % every == 0:
                poses.append((x, y, yaw))
        poses.append((x, y, yaw))
        return np.array(poses)

    def _near(self, px, py, sign, k0, k_cmd, queued):
        """Points that can be the closest one (box round the longest-wait path, + margin)."""
        p = np.vstack([self.path(sign, k0, k_cmd, 0.0, queued),
                       self.path(sign, k0, k_cmd, self.waits[-1], queued)])
        ext = math.hypot(FRONT, HALF_WIDTH) + self.margin
        return (np.isfinite(px) & np.isfinite(py) & (np.hypot(px, py) <= self.reach)
                & (px >= p[:, 0].min() - ext) & (px <= p[:, 0].max() + ext)
                & (py >= p[:, 1].min() - ext) & (py <= p[:, 1].max() + ext))

    def clearance_curve(self, sign, px, py, k0=0.0, k_cmd=None, waits=None, queued=None):
        """
        Return the clearance of the turn for each wait.

        Clearance (m) of the turn locked after driving on another waits[i] m (default
        self.waits), for each i (inf: no points near the swept area). px, py: points in
        the base_link frame (m).
        """
        return self._curves(sign, px, py, k0, k_cmd, waits, queued)[0]

    def _curves(self, sign, px, py, k0, k_cmd, waits, queued=None, island=None):
        """
        Return the clearance and the side of the first contact, per wait.

        Per wait: (clearance, first contact on the inner side?). The side is that of the
        point the outline touches first: with island (a mask like px: the points that are
        the island, from where the corner rule found its wall and end) whether that point
        is the island; without, the sign of its lateral offset in the car frame at that
        pose (a point right ahead of the bumper counts as outer; wrong for a car pointing
        into the island, whose bumper then touches the island's wall right ahead). At the
        first contact the point only just touches the outline, so its side is clear;
        deeper in, a point the path runs straight through has none.
        """
        waits = self.waits if waits is None else np.asarray(waits, float)
        px, py = np.asarray(px, float), np.asarray(py, float)
        keep = self._near(px, py, sign, k0, k_cmd, queued)
        room = np.full(len(waits), np.inf)
        inner = np.zeros(len(waits), bool)
        if not keep.any():
            return room, inner
        isl = None if island is None else np.asarray(island, bool)[keep]
        qx, qy = px[keep][None, :], py[keep][None, :]
        for i, d in enumerate(waits):
            p = self.path(sign, k0, k_cmd, d, queued)
            dx, dy = qx - p[:, :1], qy - p[:, 1:2]
            c, s = np.cos(p[:, 2:3]), np.sin(p[:, 2:3])
            u, w = c * dx + s * dy, -s * dx + c * dy                 # points in the car frame
            g = np.maximum(np.maximum(u - FRONT, -REAR - u), np.abs(w) - HALF_WIDTH)
            per_pose = g.min(axis=1)
            room[i] = per_pose.min()
            if room[i] < 0.0:
                h = int(np.argmax(per_pose < 0.0))
                j = int(np.argmin(g[h]))
                inner[i] = (isl[j] if isl is not None
                            else sign * w[h, j] > 0.0 and u[h, j] <= FRONT)
        return room, inner

    def ready(self, sign, px, py, k0=0.0, k_cmd=None, queued=None, island=None):
        """
        Decide whether to lock now.

        (lock now?, clearance now, best clearance of a later lock, its wait in m).
        k0, k_cmd, queued: the steering now, see path. island: which points are the
        island (see _curves), if known.

        If full lock now would touch something: the inner side first (the island) means
        too early, wait; the outer side or the bumper first (the wall ahead) means too
        late, lock at once (waiting only brings the wall nearer). If it touches nothing:
        lock when no wait looked at leaves more room. (Scores that mixed both cases did
        not work: the depth of the deepest point stops growing at half the car's width,
        so far too early and a little too early looked the same and the car locked 0.2 m
        early in the 2-D field; "distance before the first contact" ties along a long
        inner wall, or prefers driving straight into the wall ahead.)
        """
        room0, inner0 = self._curves(sign, px, py, k0, k_cmd, self.waits[:1], queued, island)
        if not np.isfinite(room0[0]):
            return False, float(room0[0]), float(room0[0]), 0.0
        if room0[0] < 0.0:
            return (not inner0[0]), float(room0[0]), float(room0[0]), 0.0
        # a few waits first: before the window one of them already leaves more room
        n = len(self.waits)
        first = sorted({1, n // 5, n // 2, n - 1})
        room = np.full(n, -np.inf)
        room[0] = room0[0]
        room[first] = self._curves(sign, px, py, k0, k_cmd, self.waits[first], queued)[0]
        if not room[first].max() > room[0] + 1e-4:
            rest = [i for i in range(2, n) if i not in first]
            room[rest] = self._curves(sign, px, py, k0, k_cmd, self.waits[rest], queued)[0]
        best = int(np.argmax(room[1:])) + 1
        now_ok = bool(room[0] >= room[best] - 1e-4)
        return now_ok, float(room[0]), float(room[best]), float(self.waits[best])
