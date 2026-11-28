#!/usr/bin/env python3
"""
Orin: replay CornerPolicy on a recorded orin_auto.sh bag and explain each wall-end decision.

    source /opt/ros/jazzy/setup.bash; source ~/racecar_ws/install/setup.bash
    python3 ~/racecar_ws/tools/replay_corner.py BAG DIRECTION CORNER_AHEAD T_PRESS [T_STOP] \
        [--rebuild=OFFSET]

DIRECTION / CORNER_AHEAD: the orin_auto.sh arguments of that run (auto|ccw|cw, m).
T_PRESS / T_STOP: unix times when 自動 was pressed / released (web.log, "phone: 自動").

Feeds /odometry/filtered and /scan_filled (exactly what bc_node got) to the same
CornerPolicy, so with the code of that day the corner events must match the run's bc.log
(check ALL EVENTS). --rebuild=OFFSET instead builds /scan_filled again with today's
ScanMemory from /scan minus OFFSET m (0.036 for bags before the 2026-11-19 lidar
correction, 0 after) and, if the bag has it, camera/wall_scan placed with the pose at
each frame's stamp: what today's scan_memory_node would have given.
Prints, every 0.5 s for the first 6 s after T_PRESS and at every event, for the inner
side (both sides while auto has not decided):
  fit m/M     wall points in fit_window: measured / memory (scan_memory intensity 1 lidar, 2 camera)
  wall        fitted wall line's distance from the lidar (m)
  REJECTED    fixed direction: the fit fails CornerPolicy's checks (span, slope, distance)
              and cannot end a wall, whatever end / beyond say
  end, next   where the wall "ends" (lidar x, m) and the next point on the line after the gap
  beyond m/M  beams counted as open beyond the end, measured / memory, with up to six
              (x, y, offset from the wall line, m|M); offset ~0 = the same wall seen again
READ ONLY (bag in, text out). Used for the 2026-11-13 field report (LOGBOOK).
"""
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from wro2026_imitation.corner import CornerPolicy, parallel_line
from wro2026_lidar_n10p.scan_memory import PoseHistory, ScanMemory


def detail(p, rr, mem, angles, sign, auto):
    """Same steps as CornerPolicy.inner_wall_end, keeping the intermediate values."""
    ok = np.isfinite(rr)
    r0 = np.where(ok, rr, 0.0)
    x, y = r0 * np.cos(angles), r0 * np.sin(angles)
    side = sign * y
    lo, hi = p.fit_window
    if auto:
        hi = p.auto_fit_hi
    cand = ok & (side > 0.03) & (side < p.max_side) & (x > lo)
    near = cand & (x <= hi)
    out = {'fit': (int((near & ~mem).sum()), int((near & mem).sum()))}
    if near.sum() < 6:
        return out
    if not auto:
        a, b = np.polyfit(x[near], y[near], 1)
    else:
        line = parallel_line(x, y, near, math.tan(math.radians(p.auto_max_slope_deg)))
        if line is None:
            return out
        a, b, near = line
    out['wall'] = abs(b) / math.hypot(1.0, a)
    if not auto:                                   # the fit checks of follow (2026-11-20)
        bad = []
        if np.ptp(x[near]) < p.fit_min_span:
            bad.append('span %.2f' % np.ptp(x[near]))
        if p.fit_max_slope is not None and abs(a) > p.fit_max_slope:
            bad.append('slope %.0f deg' % math.degrees(math.atan(a)))
        if p.wall_dist is not None and not p.wall_dist[0] <= out['wall'] <= p.wall_dist[1]:
            bad.append('distance')
        if bad:
            out['rejected'] = ', '.join(bad)
    res = np.abs(a * x + b - y) / math.hypot(1.0, a)
    on = cand & (res < 0.03)
    if on.sum() < 6:
        return out
    xs = np.sort(x[on])
    start = max(np.searchsorted(xs, p.fit_window[1], side='right') - 1, 0)
    jump = np.nonzero(np.diff(xs[start:]) > p.gap)[0]
    end = xs[start + jump[0]] if len(jump) else xs[-1]
    out['end'] = float(end)
    out['next'] = float(xs[start + jump[0] + 1]) if len(jump) else None
    beyond = ok & (x > end + 0.03) & (side > 0.0) & (rr > p.open_dist)
    off = sign * (y - (a * x + b))
    if auto:
        beyond &= off > p.off_line
    idx = np.nonzero(beyond)[0]
    out['beyond'] = (int((beyond & ~mem).sum()), int((beyond & mem).sum()))
    out['beyond_pts'] = [(round(x[i], 2), round(y[i], 2), round(off[i], 2), 'M' if mem[i] else 'm')
                         for i in idx[:6]]
    return out


def fmt(d):
    s = 'fit m/M %2d/%2d' % d['fit']
    if 'wall' in d:
        s += ' wall %.2f' % d['wall']
    if 'rejected' in d:
        s += ' REJECTED (%s)' % d['rejected']
    if 'end' in d:
        s += ' end %+.2f next %s beyond m/M %d/%d %s' % (
            d['end'], '%+.2f' % d['next'] if d['next'] is not None else '-',
            d['beyond'][0], d['beyond'][1], d['beyond_pts'])
    return s


def main():
    flags = [a for a in sys.argv[1:] if a.startswith('--rebuild=')]
    args = [a for a in sys.argv[1:] if a not in flags]
    offset = float(flags[0].split('=', 1)[1]) if flags else None
    bag, direction, ahead = args[0], args[1], float(args[2])
    t_press = float(args[3])
    t_stop = float(args[4]) if len(args) > 4 else float('inf')
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    pol = CornerPolicy(direction, corner_ahead=ahead)
    memory, history, frames = ScanMemory(), PoseHistory(), 0
    wanted = (('/odometry/filtered', '/scan', '/camera/wall_scan') if offset is not None
              else ('/odometry/filtered', '/scan_filled'))
    last_print, n_events = -1.0, 0
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
            pol.on_odom(x, y, yaw)
            memory.on_odom(x, y, yaw)
            history.add(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, x, y, yaw)
            continue
        if topic == '/camera/wall_scan':
            pose = history.at(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
            if pose is not None:
                memory.add_camera(t, m.ranges, m.angle_min, m.angle_increment, pose)
                frames += 1
            continue
        if offset is not None:             # /scan: rebuild what scan_memory_node gives today
            scan = np.asarray(m.ranges, float) - offset
            scan[scan < m.range_min] = np.inf
            rr, mem = memory.on_scan(t, scan, m.angle_min, m.angle_increment)
        else:
            rr = np.asarray(m.ranges, float)
            mem = (np.asarray(m.intensities, float) >= 1.0 if len(m.intensities) == len(rr)
                   else np.zeros(len(rr), bool))
        angles = m.angle_min + m.angle_increment * np.arange(len(rr))
        angles = (angles + math.pi) % (2 * math.pi) - math.pi
        sign_before, state_before = pol.sign, pol.state
        pol(rr, m.angle_min, m.angle_increment)
        new = pol.events[n_events:]
        n_events = len(pol.events)
        if t < t_press - 0.5 or t > t_stop:
            continue
        if not (new or (t - last_print >= 0.5 and t < t_press + 6)):
            continue
        last_print = t
        for s in ([1.0, -1.0] if sign_before == 0.0 else [sign_before]):
            d = detail(pol, rr, mem, angles, s, sign_before == 0.0)
            print('%+6.1f s odo %.2f %-6s %s %s' % (t - t_press, pol.odo, state_before,
                                                    'L' if s > 0 else 'R', fmt(d)))
        for e in new:
            print('   EVENT', e)
    if offset is not None:
        print('rebuilt from /scan - %.3f m, camera frames used: %d' % (offset, frames))
    print('ALL EVENTS', pol.events[:12], '... total', len(pol.events))


if __name__ == '__main__':
    main()
