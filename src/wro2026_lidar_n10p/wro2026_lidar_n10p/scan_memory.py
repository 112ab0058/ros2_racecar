"""
Fill the lidar's near blind spot from memory. Pure numpy, no ROS (scan_memory_node wraps it).

Measured on the school field 2026-11-11 (raw N10P bytes): the black foam walls give no
return at all at 0.19-0.4 m, but the same walls return fine at 0.4-1.0 m. So every wall
the car drives past was seen earlier, further ahead. ScanMemory keeps those points in
the odometry frame and, while they are still close, puts them back into the beams where
the lidar now reports nothing.

Rules (all distances in metres):
  remember  valid returns closer than remember_range are stored in the odom frame
  forget    a point is dropped after keep_time seconds or keep_dist metres of driving
  clear     a point is dropped when a new return in its beam lies clear_margin or more
            behind it (the lidar sees through where the point was, so it is gone)
  fill      a beam with no return gets the closest remembered point in it, if that
            point is within fill_range; beams with a return are never changed
Filled beams are marked in the returned mask, so users can tell memory from measurement.
"""
import math

import numpy as np


class ScanMemory:
    """Remember near returns in the odom frame and fill blind beams with them."""

    def __init__(self, laser_x=0.13, remember_range=1.2, fill_range=0.6, keep_time=30.0,
                 keep_dist=1.0, clear_margin=0.05, cell=0.01, point_radius=0.012,
                 max_points=20000):
        """Set the rules (see the module docstring); cell = spacing of stored points (m)."""
        if not 0 < fill_range <= remember_range or keep_time <= 0 or keep_dist <= 0:
            raise ValueError('need 0 < fill_range <= remember_range, keep_time, keep_dist > 0')
        self.point_radius = point_radius
        self.laser_x = laser_x
        self.remember_range = remember_range
        self.fill_range = fill_range
        self.keep_time = keep_time
        self.keep_dist = keep_dist
        self.clear_margin = clear_margin
        self.cell = cell
        self.max_points = max_points
        self.pts = np.zeros((0, 2))      # x, y in the odom frame
        self.born = np.zeros(0)          # time the point was stored
        self.born_odo = np.zeros(0)      # driven distance when it was stored
        self.pose = None                 # x, y, yaw of base_link in odom
        self.odo = 0.0                   # driven distance so far

    def on_odom(self, x, y, yaw):
        """Store the newest base_link pose in the odom frame; adds to the driven distance."""
        if self.pose is not None:
            self.odo += math.hypot(x - self.pose[0], y - self.pose[1])
        self.pose = (x, y, yaw)

    def _laser_pose(self):
        x, y, yaw = self.pose
        return x + self.laser_x * math.cos(yaw), y + self.laser_x * math.sin(yaw), yaw

    def on_scan(self, t, ranges, angle_min, angle_increment):
        """
        Use one scan taken at time t; return (filled ranges, mask of filled beams).

        Before the first odometry pose the scan is returned unchanged.
        """
        r = np.asarray(ranges, dtype=float)
        if self.pose is None:
            return r.copy(), np.zeros(len(r), dtype=bool)
        lx, ly, yaw = self._laser_pose()
        n = len(r)
        valid = np.isfinite(r) & (r > 0)
        self._forget(t)
        # where the remembered points are, seen from the lidar now
        dx, dy = self.pts[:, 0] - lx, self.pts[:, 1] - ly
        dist = np.hypot(dx, dy)
        frac = ((np.arctan2(dy, dx) - yaw - angle_min) / angle_increment) % n
        beam = np.round(frac).astype(int) % n
        # clear: a new return clearly behind a remembered point means the point is gone
        seen_behind = valid[beam] & (r[beam] >= dist + self.clear_margin)
        keep = ~seen_behind
        self.pts, self.born, self.born_odo = self.pts[keep], self.born[keep], self.born_odo[keep]
        dist, beam, frac = dist[keep], beam[keep], frac[keep]
        # fill: a beam without a return gets a remembered point within fill_range.
        # A point stands for a patch point_radius wide (points are stored about one cell
        # apart, much wider than one beam close to the car), so it may fill a few beams.
        # Each beam takes the point whose direction is closest to the beam's own (taking the
        # closest distance instead made the filled walls 2 cm too near on 2026-11-11 data).
        out = r.copy()
        near = dist <= self.fill_range
        best = np.full(n, np.inf)
        if near.any():
            d, f = dist[near], frac[near]
            b = np.round(f).astype(int)                     # not wrapped: f - b stays small
            spread = np.ceil(np.arctan2(self.point_radius, d) / angle_increment).astype(int)
            idx, err, val = [], [], []
            for k in range(-spread.max(), spread.max() + 1):
                use = np.abs(k) <= spread
                idx.append((b[use] + k) % n)
                err.append(np.abs(f[use] - (b[use] + k)))
                val.append(d[use])
            idx, err, val = np.concatenate(idx), np.concatenate(err), np.concatenate(val)
            order = np.lexsort((err, idx))                  # by beam, then by direction error
            first = np.unique(idx[order], return_index=True)[1]
            best[idx[order][first]] = val[order][first]
        mask = np.isfinite(best) & ~valid
        out[mask] = best[mask]
        # remember the new near returns
        new = valid & (r < self.remember_range)
        if new.any():
            a = angle_min + angle_increment * np.nonzero(new)[0] + yaw
            p = np.stack([lx + r[new] * np.cos(a), ly + r[new] * np.sin(a)], axis=1)
            self._add(p, t)
        return out, mask

    def _forget(self, t):
        keep = (t - self.born <= self.keep_time) & (self.odo - self.born_odo <= self.keep_dist)
        self.pts, self.born, self.born_odo = self.pts[keep], self.born[keep], self.born_odo[keep]

    def _add(self, p, t):
        """Add points, one per cell (the newest wins), and cap the total."""
        pts = np.vstack([self.pts, p])
        born = np.concatenate([self.born, np.full(len(p), t)])
        odo = np.concatenate([self.born_odo, np.full(len(p), self.odo)])
        key = np.floor(pts / self.cell).astype(np.int64)
        # keep the last occurrence of each cell: reverse, unique (first), reverse back
        _, idx = np.unique(key[::-1], axis=0, return_index=True)
        idx = np.sort(len(pts) - 1 - idx)[-self.max_points:]
        self.pts, self.born, self.born_odo = pts[idx], born[idx], odo[idx]
