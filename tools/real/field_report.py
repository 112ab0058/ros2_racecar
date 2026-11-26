#!/usr/bin/env python3
"""
Orin: field results of orin_auto.sh runs, for the report. READ ONLY (bags, logs in; text out).

    bash ~/racecar_ws/tools/orin_field_report.sh [RUN_DIR ...]   # default: today's runs

Per run:
  driver     corner rule (direction, timing) or network (bc.log); 自動 presses and hand
             keys (web.log)
  moving     time and distance the car moved (EKF speed > 2 cm/s)
  laps       turned heading / 360 deg (EKF), so the same count for the rule and the network;
             corners: the rule's 'corner policy: turn' lines; done: stopped at the start
  latches    too_close stops (bc.log)
  clearance  closest point to the car outline while moving (policy.outline_gap, the same
             outline bc_node stops on), from /scan (lidar only) and /scan_filled (lidar,
             memory, camera: the lidar misses black walls up close), m
  E1         end pose minus start pose (x forward, y left along the start heading, heading)
             wheel  /odom: the STM32's encoder odometry (speed and turn rate from the wheels)
             gyro   /odom speed + /imu/data turn rate (heading from the gyro only)
             ekf    /odometry/filtered (robot_localization: wheel speed + gyro turn rate)
             slam   slam_toolbox map -> odom on top of the EKF (stale if slam stopped)
             walls  heading change mod 90 deg from the walls in the first and last scans
                    (all walls are parallel or square): a truth for the heading in every
                    run, without a tape; 'vs walls' = each estimate's heading error mod 90.
                    2026-11-13 / 11-20 ccw runs (2.5 laps): ekf -4.5 deg, slam -26 / -36.
             With RUN_DIR/tape.txt: the tape-measured truth and every estimate's error.

tape.txt (docs/real/tasks/2026-11-26_field_night.md), cm, four numbers:
    left rear wheel: forward along the start tape, out (away from the car)
    left front wheel: forward, out
"""
import glob
import math
import os
import re
import sys
import time

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message
from wro2026_imitation.policy import outline_gap

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_measure import straightest  # noqa: E402  (same folder: tools/real, ~/racecar_ws/tools)

WHEELBASE = 0.144          # m, firmware; the tape marks sit on the left wheel centres
HALF_TRACK = 0.085         # m, rear track 0.170 (R550_HARDWARE.md)
MOVING = 0.02              # m/s
TOPICS = ['/odom', '/imu/data', '/odometry/filtered', '/tf', '/scan', '/scan_filled']


def yaw_of(q):
    """Yaw (rad) of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Angle in -pi..pi."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def relative(start, end):
    """End pose (x, y, yaw) seen from the start pose: (forward, left, heading change)."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(end[2] - start[2])


def tape_truth(rear_fwd, rear_out, front_fwd, front_out):
    """Rear-axle-centre end pose from the two left-wheel tape readings (cm) -> m, m, rad."""
    rf, ro, ff, fo = (v / 100.0 for v in (rear_fwd, rear_out, front_fwd, front_out))
    dth = math.atan2(fo - ro, WHEELBASE + ff - rf)
    # the axle centre is HALF_TRACK to the right of the left rear wheel
    return rf + HALF_TRACK * math.sin(dth), ro + HALF_TRACK * (1.0 - math.cos(dth)), dth


def scan_points(msg, far=2.5):
    """Lidar points (x, y in the lidar frame) between 0.12 m and far."""
    r = np.asarray(msg.ranges, dtype=float)
    a = msg.angle_min + msg.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.12) & (r < far)
    return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])


def wall_turn(first, last):
    """
    Heading change mod 90 deg (rad, -45..45 deg) from the walls: every wall of the field
    is parallel or square to the others, so the angle that makes the scan's walls
    straightest moves by minus the car's turn. 5 scans at each end; about 0.5 deg.
    """
    if len(first) < 5 or len(last) < 5:
        return None
    a0 = straightest(np.vstack([scan_points(m) for m in first]), step_deg=0.1)
    a1 = straightest(np.vstack([scan_points(m) for m in last]), step_deg=0.1)
    return mod90(a0 - a1)


def mod90(a):
    """Angle wrapped to -45..45 deg (rad)."""
    q = math.pi / 2.0
    return (a + q / 2.0) % q - q / 2.0


def read_tape(run):
    """The four tape numbers of a run, or None."""
    try:
        with open(os.path.join(run, 'tape.txt'), encoding='utf-8') as f:
            vals = [float(v) for v in re.findall(r'-?\d+(?:\.\d+)?', f.read())]
    except OSError:
        return None
    return vals[:4] if len(vals) >= 4 else None


def read_bag(path):
    """Walk the bag once; return the pose tracks, clearances and timing."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id='mcap'), ConverterOptions('', ''))
    have = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = [t for t in TOPICS if t in have]
    reader.set_filter(StorageFilter(topics=topics))
    types = {t: get_message(have[t]) for t in topics}

    out = {'wheel': [], 'gyro': [], 'ekf': [], 'slam': [], 'clear_scan': [],
           'clear_filled': [], 'map_last': None, 't0': None, 't1': None,
           'first_scans': [], 'last_scans': []}
    gyro_pose = [0.0, 0.0, 0.0]
    v_wheel, v_ekf, wz_imu, t_imu = 0.0, 0.0, None, None
    ekf_pose, map_odom = None, None
    moving_s, dist, yaw_turned, last_ekf = 0.0, 0.0, 0.0, None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp * 1e-9
        out['t0'] = out['t0'] or t
        out['t1'] = t
        msg = deserialize_message(data, types[topic])
        if topic == '/odom':
            p = msg.pose.pose
            out['wheel'].append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
            v_wheel = msg.twist.twist.linear.x
        elif topic == '/imu/data':
            if t_imu is not None:
                dt = min(t - t_imu, 0.1)
                gyro_pose[2] += wz_imu * dt
                gyro_pose[0] += v_wheel * math.cos(gyro_pose[2]) * dt
                gyro_pose[1] += v_wheel * math.sin(gyro_pose[2]) * dt
            t_imu, wz_imu = t, msg.angular_velocity.z
            out['gyro'].append((t, gyro_pose[0], gyro_pose[1], gyro_pose[2]))
        elif topic == '/odometry/filtered':
            p = msg.pose.pose
            pose = (p.position.x, p.position.y, yaw_of(p.orientation))
            v_ekf = msg.twist.twist.linear.x
            if last_ekf is not None:
                dt = min(t - last_ekf[0], 0.2)
                if abs(v_ekf) > MOVING:
                    moving_s += dt
                dist += abs(v_ekf) * dt
                yaw_turned += wrap(pose[2] - last_ekf[1][2])
            last_ekf = (t, pose)
            ekf_pose = pose
            out['ekf'].append((t,) + pose)
            if map_odom is not None:
                c, s = math.cos(map_odom[2]), math.sin(map_odom[2])
                out['slam'].append((t, map_odom[0] + c * pose[0] - s * pose[1],
                                    map_odom[1] + s * pose[0] + c * pose[1],
                                    wrap(map_odom[2] + pose[2])))
        elif topic == '/tf':
            for tr in msg.transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    q = tr.transform.translation
                    map_odom = (q.x, q.y, yaw_of(tr.transform.rotation))
                    out['map_last'] = t
        if topic == '/scan':
            if len(out['first_scans']) < 10:          # the 5 after the first 5
                out['first_scans'].append(msg)
            out['last_scans'] = (out['last_scans'] + [msg])[-5:]
        if topic in ('/scan', '/scan_filled') and abs(v_ekf) > MOVING:
            gap = outline_gap(np.asarray(msg.ranges, dtype=float), msg.angle_min,
                              msg.angle_increment)
            gap = gap[np.isfinite(gap)]
            if len(gap):
                key = 'clear_scan' if topic == '/scan' else 'clear_filled'
                out[key].append((t, float(gap.min())))
    out.update(moving_s=moving_s, dist=dist, laps=yaw_turned / (2.0 * math.pi))
    return out


def read_logs(run):
    """Driver, presses, corners, latches from bc.log and web.log."""
    def read(name):
        try:
            with open(os.path.join(run, name), encoding='utf-8', errors='replace') as f:
                return f.read()
        except OSError:
            return ''
    bc, web = read('bc.log'), read('web.log')
    mode = re.search(r'policy corner \((\w+), timing (\w+)', bc)
    if re.search(r'policy model\b', bc):
        driver = 'network'
    elif mode:
        driver = 'rule %s %s' % mode.groups()
    elif 'policy corner' in bc:
        driver = 'rule (log before 11-23)'
    else:
        driver = '?'
    picked = re.search(r'corner policy: direction (ccw|cw) at ([\d.]+) m', bc)
    if picked:
        driver += ' -> %s at %s m' % picked.groups()
    phone = re.findall(r'phone: (\S+)', web)
    return {
        'driver': driver,
        'auto': phone.count('自動'),
        'hand': sum(1 for p in phone if p not in ('自動', '停')),
        'corners': len(re.findall(r'corner policy: turn\b', bc)),
        'done': 'corner policy: done' in bc,
        'latches': len(re.findall(r'state: too_close', bc)),
        'map': os.path.exists(os.path.join(run, 'map.yaml')),
    }


def end_poses(bag):
    """E1: (forward, left, heading) end minus start for every estimate that has data."""
    poses = {}
    for name in ('wheel', 'gyro', 'ekf', 'slam'):
        track = bag[name]
        if len(track) >= 2:
            poses[name] = relative(track[0][1:], track[-1][1:])
    return poses


def lowest(track):
    """(m, seconds into the run) of the smallest clearance, or None."""
    if not track:
        return None
    t, v = min(track, key=lambda a: a[1])
    return v, t


def report(run):
    """Print one run's block; return a summary row."""
    name = os.path.basename(run.rstrip('/'))
    bag_dir = os.path.join(run, 'bag')
    if not os.path.isdir(bag_dir):
        print('%s: no bag' % name)
        return None
    logs = read_logs(run)
    bag = read_bag(bag_dir)
    t0 = bag['t0'] or 0.0
    print('== %s  (%s)' % (name, logs['driver']))
    print('   bag %.0f s; moving %.0f s, %.2f m; laps %.2f (heading); rule corners %d%s; '
          '自動 presses %d, hand keys %d; too_close latches %d; map saved %s'
          % ((bag['t1'] or t0) - t0, bag['moving_s'], bag['dist'], bag['laps'],
             logs['corners'], ', stopped at the start' if logs['done'] else '',
             logs['auto'], logs['hand'], logs['latches'], 'yes' if logs['map'] else 'NO'))
    if bag['map_last'] is not None and bag['t1'] - bag['map_last'] > 2.0:
        print('   slam stopped %.0f s into the bag (map -> odom stale after that)'
              % (bag['map_last'] - t0))
    for key, label in (('clear_scan', 'lidar'), ('clear_filled', 'filled')):
        low = lowest(bag[key])
        if low:
            print('   closest to the car outline while moving (%s): %.3f m at %.0f s'
                  % (label, low[0], low[1] - t0))
    poses = end_poses(bag)
    tape = read_tape(run)
    truth = tape_truth(*tape) if tape else None
    walls = wall_turn(bag['first_scans'][5:], bag['last_scans'])
    if poses:
        print('   E1 end - start      forward m   left m   heading deg'
              + ('   vs walls deg' if walls is not None else '')
              + ('   error m   heading err deg' if truth else ''))
    if walls is not None:
        print('      walls (heading mod 90 deg)                %5.1f' % math.degrees(walls))
    if truth:
        print('      tape          %9.3f %8.3f %11.1f' % (truth[0], truth[1],
                                                         math.degrees(truth[2])))
    for k, (fx, fy, fh) in poses.items():
        line = '      %-6s        %9.3f %8.3f %11.1f' % (k, fx, fy, math.degrees(fh))
        if walls is not None:
            line += '   %+12.1f' % math.degrees(mod90(fh - walls))
        if truth:
            line += '   %7.3f %13.1f' % (math.hypot(fx - truth[0], fy - truth[1]),
                                         math.degrees(wrap(fh - truth[2])))
        print(line)
    low_s, low_f = lowest(bag['clear_scan']), lowest(bag['clear_filled'])
    return (name, logs['driver'], bag['laps'], logs['latches'],
            low_s[0] if low_s else float('nan'), low_f[0] if low_f else float('nan'),
            'yes' if logs['map'] else 'NO')


def main(argv):
    """Entry point."""
    runs = argv[1:]
    if not runs:
        today = time.strftime('%Y%m%d')
        runs = sorted(glob.glob(os.path.expanduser('~/r550_map/%s_*_auto' % today)))
        if not runs:
            print('no runs today in ~/r550_map; give run folders as arguments')
            return 1
    rows = [r for r in (report(run) for run in runs) if r]
    if len(rows) > 1:
        print('\nsummary (E4): run, driver, laps, latches, closest lidar / filled m, map')
        for r in rows:
            print('  %-22s %-28s %5.2f %3d %7.3f %7.3f  %s' % r)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
