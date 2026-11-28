#!/usr/bin/env python3
"""
Orin: rebuild occupancy maps from a run's bag, one per pose source. READ ONLY (bag in, maps out).

    python3 ~/racecar_ws/tools/rebuild_map.py RUN_DIR [--pose ekf,gyro,wheel,slam]
            [--gyro-scale K] [--scan /scan] [--res 0.01] [--range-offset M] [--out DIR]

Why (E2, 2026-11-28): compare the maps the different position estimates give (wheels
only, wheels + gyro, EKF, slam_toolbox), and rebuild the maps of runs where none was saved
(11-20: 184108, 184635, 191155, the runs where ROS stalled).

How: every scan is placed with the pose at its header stamp (interpolated; the lidar is
0.13 m in front of base_link), but only once the car moved >= 2 cm or 1 deg since the last
scan used. Log-odds occupancy grid: the cells along each ray become freer, the cell of the
return more occupied (returns 0.12-3.0 m). Beams without a return change nothing (the black
walls give none up close). No scan matching: the map is exactly as good as the poses.

Pose sources:
  ekf    /odometry/filtered (robot_localization: wheel speed + gyro turn rate)
  gyro   /odom speed + /imu/data turn rate, integrated here
  wheel  /odom pose (the STM32's encoders only)
  slam   map -> odom (/tf, slam_toolbox) on top of the EKF: what slam_toolbox estimated
--gyro-scale K: the ekf and gyro headings re-integrated with the turned angle times K
(bags before 2026-11-28 were recorded without r550_feedback gyro_scale; today's value is
1.0032); written as ekf_k / gyro_k beside the others.

Out (default RUN_DIR/rebuilt/): map_<pose>.pgm + .yaml in map_saver's format (0 occupied,
254 free, 205 unknown), then the E2 numbers of map_measure.py for each map. Cells are 1 cm
(slam_toolbox: 2 cm): with 2 cm cells a field lying square to the grid measured exactly
162.0 / 78.0 / 42.0 cm (every wall in one row of cells). Fewer occupied cells from the same
scans = thinner walls = poses that agree better (11-26 21:43: EKF 3185, with gyro scale 2867).
Bags recorded before 2026-11-19 have the raw lidar distance (+3.6 cm, README of
wro2026_lidar_n10p): subtracted automatically (--range-offset overrides).
"""
import argparse
import math
import os
import re
import sys

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_measure  # noqa: E402  (same folder: tools/real, ~/racecar_ws/tools)

LASER_X = 0.13             # m, urdf laser_link
RANGE_FIX_DATE = 20261119  # the driver subtracts range_offset_m from this day on
L_HIT, L_FREE, L_MAX = 0.85, -0.4, 5.0
L_OCC, L_EMPTY = math.log(0.65 / 0.35), math.log(0.196 / 0.804)   # map_saver thresholds


def stamp(m):
    """Return the seconds of a message's header stamp."""
    return m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec


def yaw_of(q):
    """Return the yaw (rad) of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read(run, scan_topic):
    """Read the scans and pose messages of the run's bag (header stamps)."""
    rd = SequentialReader()
    rd.open(StorageOptions(uri=os.path.join(run, 'bag'), storage_id=''), ConverterOptions('', ''))
    have = {t.name for t in rd.get_all_topics_and_types()}
    topics = [t for t in (scan_topic, '/odom', '/imu/data', '/odometry/filtered', '/tf')
              if t in have]
    rd.set_filter(StorageFilter(topics=topics))
    out = {'scans': [], 'ekf': [], 'wheel': [], 'imu': [], 'map_odom': []}
    while rd.has_next():
        topic, raw, _ = rd.read_next()
        if topic == scan_topic:
            m = deserialize_message(raw, LaserScan)
            out['scans'].append((stamp(m), np.asarray(m.ranges, dtype=np.float32),
                                 m.angle_min, m.angle_increment))
        elif topic in ('/odometry/filtered', '/odom'):
            m = deserialize_message(raw, Odometry)
            p = m.pose.pose
            row = (stamp(m), p.position.x, p.position.y, yaw_of(p.orientation),
                   m.twist.twist.linear.x)
            out['ekf' if topic == '/odometry/filtered' else 'wheel'].append(row)
        elif topic == '/imu/data':
            m = deserialize_message(raw, Imu)
            out['imu'].append((stamp(m), m.angular_velocity.z))
        else:
            for tr in deserialize_message(raw, TFMessage).transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    q = tr.transform.translation
                    out['map_odom'].append((stamp(tr), q.x, q.y, yaw_of(tr.transform.rotation)))
    return {k: (v if k == 'scans' else np.array(v, dtype=float)) for k, v in out.items()}


def gyro_track(wheel, imu):
    """Integrate /odom speed + /imu/data turn rate at the IMU times: t, x, y, yaw."""
    if len(wheel) < 2 or len(imu) < 2:
        return np.zeros((0, 4))
    v = np.interp(imu[:, 0], wheel[:, 0], wheel[:, 4])
    dt = np.clip(np.diff(imu[:, 0]), 0.0, 0.1)
    yaw = np.concatenate([[0.0], np.cumsum(imu[:-1, 1] * dt)])
    x = np.concatenate([[0.0], np.cumsum(v[:-1] * np.cos(yaw[:-1]) * dt)])
    y = np.concatenate([[0.0], np.cumsum(v[:-1] * np.sin(yaw[:-1]) * dt)])
    return np.column_stack([imu[:, 0], x, y, yaw])


def slam_track(ekf, map_odom):
    """Compose map -> odom (the newest before each EKF pose) with the EKF: t, x, y, yaw."""
    if len(map_odom) == 0:
        return np.zeros((0, 4))
    k = np.searchsorted(map_odom[:, 0], ekf[:, 0], side='right') - 1
    use = k >= 0
    e, mo = ekf[use], map_odom[k[use]]
    c, s = np.cos(mo[:, 3]), np.sin(mo[:, 3])
    return np.column_stack([e[:, 0], mo[:, 1] + c * e[:, 1] - s * e[:, 2],
                            mo[:, 2] + s * e[:, 1] + c * e[:, 2], mo[:, 3] + e[:, 3]])


def scaled(track, k):
    """Return the track with its turned heading times k, positions re-integrated."""
    t = np.asarray(track, dtype=float)[:, :4].copy()
    uy = np.unwrap(t[:, 3])
    yc = uy[0] + k * (uy - uy[0])
    d = np.diff(t[:, 1:3], axis=0)
    c, s = np.cos(uy[:-1]), np.sin(uy[:-1])
    fwd, left = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
    c2, s2 = np.cos(yc[:-1]), np.sin(yc[:-1])
    t[:, 1] = t[0, 1] + np.concatenate([[0.0], np.cumsum(c2 * fwd - s2 * left)])
    t[:, 2] = t[0, 2] + np.concatenate([[0.0], np.cumsum(s2 * fwd + c2 * left)])
    t[:, 3] = yc
    return t


def placed_scans(scans, track, range_offset, near=0.12, far=3.0, move=0.02, turn_deg=1.0,
                 lidar_yaw=0.0):
    """Place the scans used (after a move or turn): (lidar x, y, returns N x 2 in the map)."""
    tr = np.asarray(track, dtype=float)
    uy = np.unwrap(tr[:, 3])
    out, last = [], None
    for t, r, amin, ainc in scans:
        if t < tr[0, 0] or t > tr[-1, 0]:
            continue
        x, y, yaw = (float(np.interp(t, tr[:, 0], tr[:, 1])),
                     float(np.interp(t, tr[:, 0], tr[:, 2])), float(np.interp(t, tr[:, 0], uy)))
        if last is not None and math.hypot(x - last[0], y - last[1]) < move \
                and abs(yaw - last[2]) < math.radians(turn_deg):
            continue
        last = (x, y, yaw)
        rr = np.asarray(r, dtype=float) - range_offset
        a = amin + ainc * np.arange(len(rr)) + yaw + lidar_yaw
        ok = np.isfinite(rr) & (rr > near) & (rr < far)
        lx, ly = x + LASER_X * math.cos(yaw), y + LASER_X * math.sin(yaw)
        out.append((lx, ly, np.column_stack([lx + rr[ok] * np.cos(a[ok]),
                                             ly + rr[ok] * np.sin(a[ok])])))
    return out


def build(placed, res):
    """Build the log-odds grid from the placed scans: (grid, origin x, origin y)."""
    allp = np.vstack([p for _, _, p in placed] + [np.array([[lx, ly] for lx, ly, _ in placed])])
    x0, y0 = allp.min(axis=0) - 0.1
    x1, y1 = allp.max(axis=0) + 0.1
    w, h = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    grid = np.zeros(h * w)
    step = res / 2.0
    for lx, ly, p in placed:
        if not len(p):
            continue
        d = p - [lx, ly]
        length = np.hypot(d[:, 0], d[:, 1])
        u = d / length[:, None]
        s = np.arange(0.0, length.max(), step)
        inside = s[None, :] < (length[:, None] - res)              # stop a cell short
        fx = lx + u[:, 0, None] * s[None, :]
        fy = ly + u[:, 1, None] * s[None, :]
        free = ((fy[inside] - y0) // res).astype(int) * w + ((fx[inside] - x0) // res).astype(int)
        hit = ((p[:, 1] - y0) // res).astype(int) * w + ((p[:, 0] - x0) // res).astype(int)
        hit = np.unique(hit)
        free = np.setdiff1d(np.unique(free), hit, assume_unique=True)
        grid[free] += L_FREE
        grid[hit] += L_HIT
        np.clip(grid, -L_MAX, L_MAX, out=grid)
    return grid.reshape(h, w), x0, y0


def save(grid, x0, y0, res, out_dir, name):
    """Write a map_saver-style PGM + YAML; return the YAML path."""
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[grid > L_OCC] = 0
    img[grid < L_EMPTY] = 254
    img = img[::-1]                                    # row 0 = top (largest y)
    pgm = os.path.join(out_dir, 'map_%s.pgm' % name)
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())
    path = os.path.join(out_dir, 'map_%s.yaml' % name)
    with open(path, 'w', encoding='utf-8') as f:
        f.write('image: map_%s.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n' % (name, res, x0, y0))
    return path


def main(argv):
    """Entry point."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('run')
    ap.add_argument('--pose', default='ekf,gyro,wheel,slam')
    ap.add_argument('--gyro-scale', type=float, default=None)
    ap.add_argument('--scan', default='/scan')
    ap.add_argument('--res', type=float, default=0.01)
    ap.add_argument('--range-offset', type=float, default=None)
    ap.add_argument('--lidar-yaw', type=float, default=0.0,
                    help='deg added to every beam (lidar mounting check)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args(argv[1:])
    run = a.run.rstrip('/')
    date = re.search(r'(\d{8})_\d{6}', os.path.basename(run))
    offset = a.range_offset
    if offset is None:
        offset = 0.036 if date and int(date.group(1)) < RANGE_FIX_DATE else 0.0
    out_dir = a.out or os.path.join(run, 'rebuilt')
    os.makedirs(out_dir, exist_ok=True)
    bag = read(run, a.scan)
    if len(bag['scans']) == 0:
        print('no %s in %s' % (a.scan, run))
        return 1
    tracks = {}
    if len(bag['ekf']):
        tracks['ekf'] = bag['ekf'][:, :4]
    tracks['gyro'] = gyro_track(bag['wheel'], bag['imu'])
    if len(bag['wheel']):
        tracks['wheel'] = bag['wheel'][:, :4]
    if len(bag['ekf']):
        tracks['slam'] = slam_track(bag['ekf'], bag['map_odom'])
    names = [n for n in a.pose.split(',') if n]
    if a.gyro_scale:
        for n in [m for m in ('ekf', 'gyro') if m in names]:
            if n in tracks and len(tracks[n]) > 1:
                tracks[n + '_k'] = scaled(tracks[n], a.gyro_scale)
                names.append(n + '_k')
    print('%s: %d scans (%s), range offset %.3f m%s'
          % (os.path.basename(run), len(bag['scans']), a.scan, offset,
             ', gyro scale %.4f for ekf_k / gyro_k' % a.gyro_scale if a.gyro_scale else ''))
    written = []
    for n in names:
        tr = tracks.get(n)
        if tr is None or len(tr) < 2:
            print('   %-6s no data' % n)
            continue
        placed = placed_scans(bag['scans'], tr, offset, lidar_yaw=math.radians(a.lidar_yaw))
        if not placed:
            print('   %-6s no scans inside the pose track' % n)
            continue
        grid, x0, y0 = build(placed, a.res)
        path = save(grid, x0, y0, a.res, out_dir, n)
        written.append(path)
        print('   %-6s %4d scans used -> %s' % (n, len(placed), path))
    if written:
        print()
        map_measure.main(['map_measure'] + written)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
