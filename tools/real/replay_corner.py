#!/usr/bin/env python3
"""
Orin: replay CornerPolicy on a recorded orin_auto.sh bag and explain each wall-end decision.

    source /opt/ros/jazzy/setup.bash; source ~/racecar_ws/install/setup.bash
    python3 ~/racecar_ws/tools/replay_corner.py BAG DIRECTION CORNER_AHEAD T_PRESS [T_STOP]

DIRECTION / CORNER_AHEAD: the orin_auto.sh arguments of that run (auto|ccw|cw, m).
T_PRESS / T_STOP: unix times when 自動 was pressed / released (web.log, "phone: 自動").

Feeds /odometry/filtered and /scan_filled (exactly what bc_node got) to the same
CornerPolicy, so the corner events must match the run's bc.log (check ALL EVENTS).
Prints, every 0.5 s for the first 6 s after T_PRESS and at every event, for the inner
side (both sides while auto has not decided):
  fit m/M     wall points in fit_window: measured / memory (scan_memory intensity 1.0)
  wall        fitted wall line's distance from the lidar (m)
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
    if 'end' in d:
        s += ' end %+.2f next %s beyond m/M %d/%d %s' % (
            d['end'], '%+.2f' % d['next'] if d['next'] is not None else '-',
            d['beyond'][0], d['beyond'][1], d['beyond_pts'])
    return s


def main():
    bag, direction, ahead = sys.argv[1], sys.argv[2], float(sys.argv[3])
    t_press = float(sys.argv[4])
    t_stop = float(sys.argv[5]) if len(sys.argv) > 5 else float('inf')
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    pol = CornerPolicy(direction, corner_ahead=ahead)
    last_print, n_events = -1.0, 0
    while reader.has_next():
        topic, raw, t = reader.read_next()
        if topic not in ('/odometry/filtered', '/scan_filled'):
            continue
        t *= 1e-9
        m = deserialize_message(raw, get_message(types[topic]))
        if topic == '/odometry/filtered':
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            pol.on_odom(m.pose.pose.position.x, m.pose.pose.position.y, yaw)
            continue
        rr = np.asarray(m.ranges, float)
        mem = (np.asarray(m.intensities, float) == 1.0 if len(m.intensities) == len(rr)
               else np.zeros(len(rr), bool))
        angles = m.angle_min + m.angle_increment * np.arange(len(rr))
        angles = (angles + math.pi) % (2 * math.pi) - math.pi
        sign_before, state_before = pol.sign, pol.state
        pol(m.ranges, m.angle_min, m.angle_increment)
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
    print('ALL EVENTS', pol.events[:12], '... total', len(pol.events))


if __name__ == '__main__':
    main()
