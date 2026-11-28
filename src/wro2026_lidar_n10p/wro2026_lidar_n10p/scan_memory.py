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

The camera (wro2026_camera_c70, camera/wall_scan) sees the black wall bottoms from about
0.3 m, where the lidar sees them only sometimes (2026-11-13). add_camera stores those
points like lidar returns, placed with the pose of the car when the frame was taken
(PoseHistory); after on_scan, filled_by_camera tells which filled beams came from them.
"""
import collections
import math

import numpy as np

LIDAR, CAMERA = 0, 1


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
        self.src = np.zeros(0, dtype=np.int8)    # LIDAR or CAMERA
        self.pose = None                 # x, y, yaw of base_link in odom
        self.odo = 0.0                   # driven distance so far
        self.filled_by_camera = None     # after on_scan: filled beams that came from the camera

    def on_odom(self, x, y, yaw):
        """Store the newest base_link pose in the odom frame; adds to the driven distance."""
        if self.pose is not None:
            self.odo += math.hypot(x - self.pose[0], y - self.pose[1])
        self.pose = (x, y, yaw)

    def _laser_pose(self, pose=None):
        x, y, yaw = pose or self.pose
        return x + self.laser_x * math.cos(yaw), y + self.laser_x * math.sin(yaw), yaw

    def _keep(self, keep):
        self.pts, self.born = self.pts[keep], self.born[keep]
        self.born_odo, self.src = self.born_odo[keep], self.src[keep]

    def on_scan(self, t, ranges, angle_min, angle_increment):
        """
        Use one scan taken at time t; return (filled ranges, mask of filled beams).

        Before the first odometry pose the scan is returned unchanged.
        """
        r = np.asarray(ranges, dtype=float)
        self.filled_by_camera = np.zeros(len(r), dtype=bool)
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
        self._keep(keep)
        dist, beam, frac = dist[keep], beam[keep], frac[keep]
        # fill: a beam without a return gets a remembered point within fill_range.
        # A point stands for a patch point_radius wide (points are stored about one cell
        # apart, much wider than one beam close to the car), so it may fill a few beams.
        # Each beam takes the point whose direction is closest to the beam's own (taking the
        # closest distance instead made the filled walls 2 cm too near on 2026-11-11 data).
        out = r.copy()
        near = dist <= self.fill_range
        best = np.full(n, np.inf)
        best_src = np.full(n, LIDAR, dtype=np.int8)
        if near.any():
            d, f, s = dist[near], frac[near], self.src[near]
            b = np.round(f).astype(int)                     # not wrapped: f - b stays small
            spread = np.ceil(np.arctan2(self.point_radius, d) / angle_increment).astype(int)
            idx, err, val, src = [], [], [], []
            for k in range(-spread.max(), spread.max() + 1):
                use = np.abs(k) <= spread
                idx.append((b[use] + k) % n)
                err.append(np.abs(f[use] - (b[use] + k)))
                val.append(d[use])
                src.append(s[use])
            idx, err = np.concatenate(idx), np.concatenate(err)
            val, src = np.concatenate(val), np.concatenate(src)
            order = np.lexsort((err, idx))                  # by beam, then by direction error
            first = np.unique(idx[order], return_index=True)[1]
            best[idx[order][first]] = val[order][first]
            best_src[idx[order][first]] = src[order][first]
        mask = np.isfinite(best) & ~valid
        out[mask] = best[mask]
        self.filled_by_camera = mask & (best_src == CAMERA)
        # remember the new near returns
        new = valid & (r < self.remember_range)
        if new.any():
            a = angle_min + angle_increment * np.nonzero(new)[0] + yaw
            p = np.stack([lx + r[new] * np.cos(a), ly + r[new] * np.sin(a)], axis=1)
            self._add(p, t, LIDAR)
        return out, mask

    def add_camera(self, t, ranges, angle_min, angle_increment, pose=None):
        """
        Remember the wall points of one camera frame (ranges seen from the lidar).

        pose: (x, y, yaw) of base_link in odom when the frame was taken; None = the newest
        pose. Returns how many points were stored (0 before the first pose).
        """
        pose = pose or self.pose
        r = np.asarray(ranges, dtype=float)
        ok = np.isfinite(r) & (r > 0)
        if pose is None or not ok.any():
            return 0
        lx, ly, yaw = self._laser_pose(pose)
        a = angle_min + angle_increment * np.nonzero(ok)[0] + yaw
        self._add(np.stack([lx + r[ok] * np.cos(a), ly + r[ok] * np.sin(a)], axis=1), t, CAMERA)
        return int(ok.sum())

    def _forget(self, t):
        self._keep((t - self.born <= self.keep_time)
                   & (self.odo - self.born_odo <= self.keep_dist))

    def _add(self, p, t, source):
        """Add points, one per cell (the newest wins), and cap the total."""
        pts = np.vstack([self.pts, p])
        born = np.concatenate([self.born, np.full(len(p), t)])
        odo = np.concatenate([self.born_odo, np.full(len(p), self.odo)])
        src = np.concatenate([self.src, np.full(len(p), source, dtype=np.int8)])
        key = np.floor(pts / self.cell).astype(np.int64)
        # keep the last occurrence of each cell: reverse, unique (first), reverse back
        _, idx = np.unique(key[::-1], axis=0, return_index=True)
        idx = np.sort(len(pts) - 1 - idx)[-self.max_points:]
        self.pts, self.born, self.born_odo, self.src = pts[idx], born[idx], odo[idx], src[idx]


class PoseHistory:
    """The last few seconds of odometry poses, to find where the car was at a given time."""

    def __init__(self, keep=2.0):
        """keep: seconds of poses to keep."""
        self.keep = keep
        self.poses = collections.deque()     # (t, x, y, yaw), t increasing

    def add(self, t, x, y, yaw):
        """Add a pose; a time not after the newest one is ignored."""
        if self.poses and t <= self.poses[-1][0]:
            return
        self.poses.append((t, x, y, yaw))
        while self.poses[0][0] < t - self.keep:
            self.poses.popleft()

    def at(self, t, max_gap=0.2):
        """Pose (x, y, yaw) at time t, interpolated; None if t is max_gap past either end."""
        if not self.poses or t < self.poses[0][0] - max_gap or t > self.poses[-1][0] + max_gap:
            return None
        prev = self.poses[0]
        if t <= prev[0]:
            return prev[1:]
        for cur in self.poses:
            if cur[0] >= t:
                f = (t - prev[0]) / (cur[0] - prev[0])
                da = math.atan2(math.sin(cur[3] - prev[3]), math.cos(cur[3] - prev[3]))
                return (prev[1] + f * (cur[1] - prev[1]), prev[2] + f * (cur[2] - prev[2]),
                        prev[3] + f * da)
            prev = cur
        return prev[1:]
