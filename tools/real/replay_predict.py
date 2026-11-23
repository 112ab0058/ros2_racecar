#!/usr/bin/env python3
"""
Compare a recorded run's full-lock points with the predicted ones (Orin).

Orin: on a recorded orin_auto.sh run, compare where the car went to full lock (the fixed
corner_ahead of that day) with where the turn prediction (turn_predict.py) would have.

    source /opt/ros/jazzy/setup.bash; source ~/racecar_ws/install/setup.bash
    python3 ~/racecar_ws/tools/replay_predict.py BAG_DIR DIRECTION CORNER_AHEAD [SLOPE_DEG] \
        [--why] [--no-camera] [--rebuild] [--cam-max=M]

DIRECTION / CORNER_AHEAD: the orin_auto.sh arguments of that run (ccw|cw, m); SLOPE_DEG:
CornerPolicy fit_max_slope_deg of that day's code (20 before commit a3e33ba, 2026-11-20
~18:30; default 40).

Feeds /odometry/filtered and /scan_filled (what bc_node got) to CornerPolicy(timing
'ahead') with that day's settings, so its lock events must match the run's bc.log (they
are printed next to each other). --rebuild builds /scan_filled again from /scan and
camera/wall_scan with ScanMemory as scan_memory_node does (bags with the camera, whose
/scan already has the range offset); --cam-max=M (implies --rebuild) first drops camera
wall points farther than M from the lidar, as wall_node max_range would.
On every scan from the moment the inner wall's end was seen until the real lock, it also
asks TurnPredictor: lock now? For each corner it prints
  real lock     driven distance (m) and where the inner wall end was (lidar x, m)
  predicted     the first scan on which the prediction said "lock", if that came before
                the real lock: how much earlier (cm)
  at the real lock: room now (m) = the smallest gap the predicted turn would leave, and
                the best room of a later lock with its wait (m): wait > 0 means the
                prediction would have locked that much later
Only the path up to the real lock is the car's own; after it the car turned, so a later
predicted lock can only be read from the prediction made at the real lock.
READ ONLY (bag in, text out).
"""
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from wro2026_imitation.corner import CornerPolicy, DELAY, FOLLOW, TURN
from wro2026_imitation.turn_predict import FRONT, HALF_WIDTH, LASER_X, REAR, TurnPredictor
from wro2026_lidar_n10p.scan_memory import PoseHistory, ScanMemory


def point_gaps(pred, sign, px, py, k0, k_cmd, queued):
    """Each point's smallest gap (m) to the outline along the lock-now path (inf: no return)."""
    p = pred.path(sign, k0, k_cmd, 0.0, queued)
    ok = np.isfinite(px)
    gap = np.full(len(px), np.inf)
    dx, dy = px[ok][None, :] - p[:, :1], py[ok][None, :] - p[:, 1:2]
    c, s = np.cos(p[:, 2:3]), np.sin(p[:, 2:3])
    u, w = c * dx + s * dy, -s * dx + c * dy
    gap[ok] = np.maximum(np.maximum(u - FRONT, -REAR - u), np.abs(w) - HALF_WIDTH).min(axis=0)
    return gap


def main():
    why = '--why' in sys.argv
    no_camera = '--no-camera' in sys.argv
    cam_max = [float(a.split('=', 1)[1]) for a in sys.argv[1:] if a.startswith('--cam-max=')]
    cam_max = cam_max[0] if cam_max else None
    rebuild = '--rebuild' in sys.argv or cam_max is not None
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    bag, direction, ahead = args[0], args[1], float(args[2])
    slope = float(args[3]) if len(args) > 3 else 40.0
    uri = bag.rstrip('/') + ('' if bag.rstrip('/').endswith('bag') else '/bag')
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    pol = CornerPolicy(direction, timing='ahead', corner_ahead=ahead, fit_max_slope_deg=slope)
    pred = TurnPredictor()
    armed, first, n_events = None, None, 0
    memory, history, frames = ScanMemory(), PoseHistory(), 0
    wanted = (('/odometry/filtered', '/scan', '/camera/wall_scan') if rebuild
              else ('/odometry/filtered', '/scan_filled'))
    while reader.has_next():
        topic, raw, t = reader.read_next()
        if topic not in wanted:
            continue
        t *= 1e-9
        m = deserialize_message(raw, get_message(types[topic]))
        if topic == '/odometry/filtered':
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            x, y = m.pose.pose.position.x, m.pose.pose.position.y
            pol.on_odom(x, y, yaw, m.twist.twist.linear.x, m.twist.twist.angular.z)
            memory.on_odom(x, y, yaw)
            history.add(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, x, y, yaw)
            continue
        if topic == '/camera/wall_scan':
            pose = history.at(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
            if pose is not None:
                cam = np.asarray(m.ranges, float)
                if cam_max is not None:
                    cam[cam > cam_max] = np.inf
                memory.add_camera(t, cam, m.angle_min, m.angle_increment, pose)
                frames += 1
            continue
        if rebuild:                       # /scan: what scan_memory_node would publish
            scan = np.asarray(m.ranges, float)
            scan[scan < m.range_min] = np.inf
            r, filled = memory.on_scan(t, scan, m.angle_min, m.angle_increment)
            kind = np.where(memory.filled_by_camera, 2.0, np.where(filled, 1.0, 0.0))
        else:
            r = np.asarray(m.ranges, float)
            kind = (np.asarray(m.intensities, float) if len(m.intensities) == len(r)
                    else np.zeros(len(r)))
        angles = m.angle_min + m.angle_increment * np.arange(len(r))
        angles = (angles + math.pi) % (2 * math.pi) - math.pi
        before = pol.state
        # the steering as the prediction sees it before this scan's decision
        k0, k_cmd = pol.curvature(), pol.driven[pol.last_label]
        queued = [(pol.odo - o, pol.driven[lab]) for o, lab in pol.sent] or None
        pol(r, m.angle_min, m.angle_increment)
        for e in pol.events[n_events:]:
            print('   event', e)
        n_events = len(pol.events)
        if before != FOLLOW:
            continue
        if pol.approach:
            armed = pol.odo
        elif armed is not None and pol.odo - armed > pol.entry_dist:
            armed, first = None, None
        if armed is None:
            continue
        ok = np.isfinite(r)
        if no_camera:                                     # drop camera memory (intensity 2)
            ok &= kind < 1.5
        rr = np.where(ok, r, 0.0)
        px = np.where(ok, rr * np.cos(angles) + LASER_X, np.nan)
        py = np.where(ok, rr * np.sin(angles), np.nan)
        lock, now, later, wait = pred.ready(pol.sign, px, py, k0, k_cmd, queued)
        if lock and first is None:
            first = (pol.odo, now)
        if pol.state in (DELAY, TURN):                    # the real lock, on this scan
            end = pol.inner_wall_end(r, angles, min_span=pol.fit_min_span,
                                     max_slope=pol.fit_max_slope, wall_dist=pol.wall_dist)
            print('corner %d: real lock at %.3f m, inner wall end %s (lidar x), curvature %+.2f'
                  ' (command %+.2f)' % (pol.corners, pol.odo,
                                        '%+.3f' % end if end is not None else '-', k0, k_cmd))
            if first is not None and first[0] < pol.odo - 1e-6:
                print('   predicted lock %.1f cm EARLIER (at %.3f m, room then %.3f m)'
                      % ((pol.odo - first[0]) * 100, first[0], first[1]))
            print('   at the real lock: room now %.3f m; best later %.3f m after %.0f cm more%s'
                  % (now, later, wait * 100,
                     ' -> prediction says LOCK NOW' if lock else ' -> prediction says WAIT'))
            if why:                                       # the points closest to that turn
                gap = point_gaps(pred, pol.sign, px, py, k0, k_cmd, queued)
                for i in np.argsort(gap)[:6]:
                    print('      point base_link (%+.3f, %+.3f) gap %+.3f %s' % (
                        px[i], py[i], gap[i], ('measured', 'lidar memory', 'camera memory')[
                            int(min(max(kind[i], 0), 2))]))
            armed, first = None, None
    if rebuild:
        print('memory rebuilt from /scan, camera frames used: %d%s' % (
            frames, ', camera points beyond %.2f m dropped' % cam_max if cam_max else ''))
    print('ALL EVENTS', pol.events)


if __name__ == '__main__':
    main()
