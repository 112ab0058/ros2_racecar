"""
C70 camera: where the black walls meet the white floor. numpy + OpenCV, no ROS.

Why: the N10P lidar gets no return from the field's matte black walls closer than about
0.32 m and only some returns up to about 0.45 m (2026-11-13, docs/research/LOGBOOK.md).
The camera sees those wall bottoms: the floor is bright, the walls are dark.

How (checked on the 10 field photos of 2026-11-13 against the raw lidar, LOGBOOK 11-19):
  1. wall_bottom: grey, 5x5 blur; every `step` columns, from the bottom row upwards,
     the first `run` pixels in a row darker than `threshold` are the wall. A column that
     is already dark at the bottom (wall closer than the image edge, or a dark floor) or
     never turns dark gives nothing.
  2. FloorCamera.floor_points: undistort the pixel (intrinsics), turn the ray into the
     base_link frame (extrinsics) and cut it with the floor (z = -height).
  3. to_ranges: one range per beam from the lidar (laser_link), like a LaserScan, so the
     scan memory can use them like lidar returns.
Accuracy: against the tape-corrected lidar the camera reads 1.8 cm short (median) with a
mean error of 4 cm; one pixel of wall bottom is 1 mm at 0.25 m and 1 cm at 0.9 m
(docs/real/calib/README.md). The camera sees the floor from about 0.25 m in front of the
lens and 76 deg wide, so nothing right beside the car.
"""
import math

import cv2
import numpy as np
import yaml


def rotation(pitch_down_deg, roll_deg, yaw_deg):
    """
    Camera axes (x right, y down, z forward) -> base_link axes (x forward, y left, z up).

    Signs as tools/real/calibrate_camera.py floor writes them: pitch + = looking down,
    yaw + = turned left.
    """
    m = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    p = math.radians(pitch_down_deg)
    r = -math.radians(roll_deg)
    y = math.radians(yaw_deg)
    ry = np.array([[math.cos(p), 0.0, math.sin(p)], [0.0, 1.0, 0.0],
                   [-math.sin(p), 0.0, math.cos(p)]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(r), -math.sin(r)],
                   [0.0, math.sin(r), math.cos(r)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0.0], [math.sin(y), math.cos(y), 0.0],
                   [0.0, 0.0, 1.0]])
    return rz @ ry @ rx @ m


class FloorCamera:
    """A calibrated camera that looks at a flat floor."""

    def __init__(self, k, d, height, pitch_down_deg, roll_deg, yaw_deg, x, y=0.0,
                 size=(1280, 720)):
        """Intrinsics k (3x3) and d (plumb_bob); lens height and x, y in base_link (m)."""
        self.k = np.asarray(k, dtype=float).reshape(3, 3)
        self.d = np.asarray(d, dtype=float)
        self.height = float(height)
        self.x, self.y = float(x), float(y)
        self.angles = (float(pitch_down_deg), float(roll_deg), float(yaw_deg))
        self.rot = rotation(pitch_down_deg, roll_deg, yaw_deg)
        self.size = tuple(size)

    @classmethod
    def from_files(cls, intrinsics, extrinsics):
        """Load the ROS camera_info yaml and the extrinsics yaml (config/ of this package)."""
        with open(intrinsics, encoding='utf-8') as f:
            c = yaml.safe_load(f)
        with open(extrinsics, encoding='utf-8') as f:
            e = yaml.safe_load(f)
        y = e.get('y_from_base_link_m')
        return cls(c['camera_matrix']['data'], c['distortion_coefficients']['data'],
                   e['height_m'], e['pitch_down_deg'], e['roll_deg'], e['yaw_deg'],
                   e['x_from_base_link_m'], y if isinstance(y, (int, float)) else 0.0,
                   (c['image_width'], c['image_height']))

    def floor_points(self, pixels, pitch_change_deg=0.0):
        """
        Pixels (u, v) -> floor points (x, y) in base_link; pixels above the horizon dropped.

        pitch_change_deg: project as if the camera looked that much further down than its
        calibration (lidar_pitch_fix).
        """
        px = np.asarray(pixels, dtype=float).reshape(-1, 2)
        if len(px) == 0:
            return np.zeros((0, 2))
        return self._floor(self._rays(px), pitch_change_deg)[0]

    def _rays(self, px):
        """Pixels -> rays in the camera frame (undistorted, z = 1)."""
        n = cv2.undistortPoints(px.reshape(-1, 1, 2), self.k, self.d).reshape(-1, 2)
        return np.c_[n, np.ones(len(n))]

    def _floor(self, rays, pitch_change_deg=0.0):
        """Camera rays -> (floor points of the rays that go down, which rays those are)."""
        rot = self.rot
        if pitch_change_deg:
            p, r, y = self.angles
            rot = rotation(p + pitch_change_deg, r, y)
        ray = rays @ rot.T
        down = ray[:, 2] < -1e-3
        t = self.height / -ray[down, 2]
        return np.c_[self.x + t * ray[down, 0], self.y + t * ray[down, 1]], down


def lidar_pitch_fix(cam, pixels, lidar_ranges, laser_x, bins, far=(0.5, 1.0), tolerance=0.15,
                    min_pairs=8, search_deg=3.0, step_deg=0.05):
    """
    Pitch change (deg) that makes the camera's far wall points agree with the lidar.

    The camera is only ~10 cm above the floor, so a car tilted by the floor, or a wall
    standing a few mm higher or lower than the floor under the car, moves its near points
    by centimetres (1 deg of pitch: about 2 cm at 0.4 m; 2026-11-27, FIELD_REVIEW B2). The
    lidar sees the black walls well from about 0.47 m. Pairs: wall points the camera puts
    `far` (m, from the lidar) away whose beam of the same lidar scan (bins beams, like
    /scan) has a range within `tolerance` of theirs (the same wall). The change, searched
    within +-search_deg, makes the median of (camera - lidar) over the pairs closest to 0.
    Returns (change, pairs); change 0.0 when there are fewer than min_pairs pairs.
    Replayed on four 2026-11-26 runs: near points (0.3-0.5 m) more than 2 cm off the
    lidar's wall: 43 / 57 / 37 / 33 % -> 38 / 55 / 15 / 18 %.
    """
    px = np.asarray(pixels, dtype=float).reshape(-1, 2)
    lidar = np.asarray(lidar_ranges, dtype=float)
    if len(px) == 0 or len(lidar) != bins:
        return 0.0, 0
    rays = cam._rays(px)

    def ranges(change, rays):
        pts, down = cam._floor(rays, change)
        r = np.full(len(rays), np.nan)
        x, y = pts[:, 0] - laser_x, pts[:, 1]
        r[down] = np.hypot(x, y)
        a = np.full(len(rays), np.nan)
        a[down] = np.arctan2(y, x)
        return r, a

    r0, a0 = ranges(0.0, rays)
    ok = np.isfinite(r0)
    beam = np.zeros(len(px), dtype=int)
    beam[ok] = np.round((a0[ok] % (2.0 * math.pi)) / (2.0 * math.pi / bins)).astype(int) % bins
    rl = lidar[beam]
    with np.errstate(invalid='ignore'):
        pair = (ok & (r0 >= far[0]) & (r0 <= far[1]) & np.isfinite(rl)
                & (np.abs(r0 - rl) < tolerance))
    n = int(pair.sum())
    if n < min_pairs:
        return 0.0, n
    cost0 = abs(float(np.median(r0[pair] - rl[pair])))
    # all changes at once (a loop over them took ~24 ms a frame on the Orin, 2026-11-27)
    changes = np.arange(-search_deg, search_deg + step_deg / 2.0, step_deg)
    p, roll, yaw = cam.angles
    rots = np.stack([rotation(p + c, roll, yaw) for c in changes])     # changes x 3 x 3
    ray = np.einsum('kij,nj->kni', rots, rays[pair])                    # changes x pairs x 3
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.where(ray[..., 2] < -1e-3, cam.height / -ray[..., 2], np.nan)
        r = np.hypot(cam.x + t * ray[..., 0] - laser_x, cam.y + t * ray[..., 1])
    rows = np.isfinite(r).any(axis=1)
    cost = np.full(len(changes), np.inf)
    cost[rows] = np.abs(np.nanmedian(r[rows] - rl[pair], axis=1))
    i = int(np.argmin(cost))
    best = float(changes[i]) if cost[i] < cost0 - 1e-9 else 0.0
    return round(best, 3), n


def saturation(bgr):
    """HSV saturation 0-255 as OpenCV computes it: 255 * (max - min) / max."""
    hi = bgr.max(axis=-1).astype(np.int32)
    lo = bgr.min(axis=-1).astype(np.int32)
    return np.where(hi > 0, 255 * (hi - lo) // np.maximum(hi, 1), 0)


def wall_bottom(img, threshold=110, run=12, step=4, max_saturation=60, floor_rows=20):
    """
    Lowest wall pixel (u, v) in every `step`-th column (see module doc).

    v is the bottom row of the first `run` dark rows above the floor. The bottom 3 rows
    are not used (the C70's edge rows), as in the 2026-11-19 field check.
    img: BGR or grey. With BGR, the `floor_rows` rows below the wall must also look like
    the white floor (median saturation <= max_saturation): the beige masking tape on the
    field's walls is bright too (saturation 93-103, floor 9-27, 2026-11-13 photos), and
    without this a tape patch reaching the image bottom was taken for floor and its top
    edge for a wall bottom behind the real wall.
    """
    colour = img.ndim == 3
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if colour else img
    g = cv2.GaussianBlur(grey, (5, 5), 0)
    h, w = g.shape
    cols = np.arange(step // 2, w, step)
    dark = g[:, cols] < threshold
    s = np.vstack([np.zeros((1, len(cols)), dtype=np.int32), np.cumsum(dark, axis=0)])
    v = np.arange(run + 1, h - 3)                    # window = rows v - run .. v - 1
    full = (s[v] - s[v - run]) == run
    found = full.any(axis=0) & ~dark[h - 4]
    lowest = v[len(v) - 1 - np.argmax(full[::-1], axis=0)]
    if colour and found.any():
        idx = np.nonzero(found)[0]
        rows = np.clip(lowest[idx, None] + 2 + np.arange(floor_rows), 0, h - 4)
        sat = saturation(img[rows, cols[idx, None]])
        found[idx[np.median(sat, axis=1) > max_saturation]] = False
    return np.stack([cols[found], lowest[found] - 1], axis=1).astype(float)


def to_ranges(points, laser_x, bins, max_range):
    """
    Floor points (base_link) -> one range per beam seen from the lidar (inf = nothing).

    Beam k points at k * 360 / bins degrees, counter-clockwise from the front, like /scan
    (angle_min 0). Several points in one beam: their median. Points farther than
    max_range are dropped (the camera gets worse with distance; the lidar sees there).
    """
    ranges = np.full(bins, np.inf)
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y = p[:, 0] - laser_x, p[:, 1]
    r = np.hypot(x, y)
    keep = r <= max_range
    if not keep.any():
        return ranges
    r = r[keep]
    k = np.round(np.arctan2(y[keep], x[keep]) % (2.0 * math.pi) / (2.0 * math.pi / bins))
    k = k.astype(int) % bins
    order = np.lexsort((r, k))
    k, r = k[order], r[order]
    beams, start, count = np.unique(k, return_index=True, return_counts=True)
    ranges[beams] = r[start + (count - 1) // 2]
    return ranges
