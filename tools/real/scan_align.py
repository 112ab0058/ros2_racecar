#!/usr/bin/env python3
"""
Lidar scan alignment (2-D point-to-line ICP) for E1: a position truth without a tape.

The car stands still at the start of every orin_auto.sh run. Any later scan taken near
the start is aligned to those start scans; the result is where the car really was,
relative to its start pose (forward / left along the start heading, heading change).
field_report.py uses it for the end pose and for every pass of the start (E1).

Checked 2026-11-28 (FIELD_REVIEW A5): three runs where the car did not move (air tests,
the EKF moved 3-6 m) give 0.2-0.3 cm and <= 0.07 deg; on the field runs, 7 start guesses
up to 5 cm / 3 deg off converge to the same answer. Only points 0.45-3.0 m from the lidar
(the black walls are seen from ~0.47 m). Moving scans are not de-skewed (1 cm per scan at
0.1 m/s).
"""
import math

import numpy as np

from scipy.spatial import cKDTree

LASER_X = 0.13     # m, urdf laser_link in front of base_link
STARTS = [(0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (-0.05, 0.0, 0.0), (0.0, 0.05, 0.0),
          (0.0, -0.05, 0.0), (0.0, 0.0, 3.0), (0.0, 0.0, -3.0)]   # m, m, deg off the guess


def scan_xy(ranges, angle_min, angle_increment, near=0.45, far=3.0):
    """Points (x, y) of one scan in the lidar frame, near..far m."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > near) & (r < far)
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])


def compose(a, b):
    """Pose b given in frame a -> in a's parent frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    """Inverse of a 2-D pose."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def wall_normals(q, tree, radius=0.05, min_pts=6, line=0.25):
    """
    Wall normal at every point, and which points lie on a line.

    All points within radius are used: still scans repeat every beam, so the k nearest
    points would be one beam only.
    """
    n = np.zeros_like(q)
    good = np.zeros(len(q), dtype=bool)
    for i, nb in enumerate(tree.query_ball_point(q, radius)):
        if len(nb) < min_pts:
            continue
        p = q[nb] - q[nb].mean(0)
        _, s, vt = np.linalg.svd(p, full_matrices=False)
        if s[1] < line * s[0]:
            n[i], good[i] = vt[1], True
    return n, good


def icp(src, dst, init, iters=40, tree=None, nrm=None):
    """Pose (x, y, yaw) of the src points' frame in the dst frame: (pose, rms m, pairs)."""
    if tree is None:
        tree = cKDTree(dst)
    if nrm is None:
        nrm = wall_normals(dst, tree)
    normal, good = nrm
    x, y, th = init
    for it in range(iters):
        c, s = math.cos(th), math.sin(th)
        p = src @ np.array([[c, s], [-s, c]]) + [x, y]
        d, j = tree.query(p)
        gate = 0.10 if it < 10 else 0.05 if it < 20 else 0.03
        ok = (d < gate) & good[j]
        if ok.sum() < 30:
            return None, float('nan'), int(ok.sum())
        n, q, pp = normal[j[ok]], dst[j[ok]], p[ok]
        sx, sy = src[ok, 0], src[ok, 1]
        jth = n[:, 0] * (-s * sx - c * sy) + n[:, 1] * (c * sx - s * sy)
        a = np.column_stack([n[:, 0], n[:, 1], jth])
        b = -np.sum((pp - q) * n, axis=1)
        dx, dy, dth = np.linalg.lstsq(a, b, rcond=None)[0]
        x, y, th = x + dx, y + dy, th + dth
        if it >= 20 and abs(dx) < 1e-5 and abs(dy) < 1e-5 and abs(dth) < 1e-5:
            break
    c, s = math.cos(th), math.sin(th)
    p = src @ np.array([[c, s], [-s, c]]) + [x, y]
    d, j = tree.query(p)
    ok = (d < 0.03) & good[j]
    res = np.sum((p[ok] - dst[j[ok]]) * normal[j[ok]], axis=1)
    return (x, y, th), float(np.sqrt(np.mean(res ** 2))), int(ok.sum())


class StartReference:
    """Still scans at the start, to align later scans to."""

    def __init__(self, scans, laser_x=LASER_X):
        """Keep the still start scans, each (ranges, angle_min, angle_increment)."""
        self.dst = np.vstack([scan_xy(*s) for s in scans])
        self.tree = cKDTree(self.dst)
        self.nrm = wall_normals(self.dst, self.tree)
        self.laser = (laser_x, 0.0, 0.0)

    def align(self, src_lidar, guess, also_still=False):
        """
        Find the base pose relative to the start that fits the lidar points src_lidar.

        Started from the guess (base pose relative to the start, e.g. the EKF's) and 7
        starts around it (+ no motion). Returns (pose, rms m, pairs, starts that agree
        within 5 mm, starts tried), or None.
        """
        lidar_guess = compose(compose(inverse(self.laser), guess), self.laser)
        starts = [(lidar_guess[0] + dx, lidar_guess[1] + dy, lidar_guess[2] + math.radians(dt))
                  for dx, dy, dt in STARTS]
        if also_still:
            starts.append((0.0, 0.0, 0.0))
        sols = []
        for st in starts:
            sol, rms, n = icp(src_lidar, self.dst, st, tree=self.tree, nrm=self.nrm)
            if sol is not None:
                b = compose(compose(self.laser, sol), inverse(self.laser))
                sols.append((b[0], b[1], math.atan2(math.sin(b[2]), math.cos(b[2])), rms, n))
        if not sols:
            return None
        s = np.array(sols)
        best = s[np.lexsort((s[:, 3], -s[:, 4]))[0]]     # most pairs, then lowest rms
        agree = int((np.hypot(s[:, 0] - best[0], s[:, 1] - best[1]) < 0.005).sum())
        return (float(best[0]), float(best[1]), float(best[2])), float(best[3]), int(best[4]), \
            agree, len(starts)


def merged(scans, poses, laser_x=LASER_X):
    """Points of several scans in the lidar frame of the middle one; poses: base poses."""
    laser = (laser_x, 0.0, 0.0)
    mid = poses[len(poses) // 2]
    out = []
    for s, p in zip(scans, poses):
        rel = compose(inverse(mid), p)
        lp = compose(compose(inverse(laser), rel), laser)
        c, sn = math.cos(lp[2]), math.sin(lp[2])
        out.append(scan_xy(*s) @ np.array([[c, sn], [-sn, c]]) + [lp[0], lp[1]])
    return np.vstack(out)
