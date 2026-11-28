#!/usr/bin/env python3
"""
Orin: how fast the R550 reaches full steering lock, and the radius it then drives, per side.

    source /opt/ros/jazzy/setup.bash
    python3 ~/racecar_ws/tools/steer_lag.py BAG_DIR [BAG_DIR ...]

For every full-lock command in the bags (/cmd_vel_out, what base_node sent: |vx / wz| close
to the 0.35 m full-lock radius, held >= 1.5 s, after something that was not full lock in
that direction) it takes the gyro z (/imu/data, IMU stamps, bias from the still start) and
measures when the yaw rate has covered 50 % and 90 % of the way from its value before the
command to its steady value (median 1.5-3 s after). With a dead time + first-order lag
y = 1 - exp(-(t - td) / tau):  tau = (t90 - t50) / ln 5,  td = t50 - tau ln 2.
The car drives about v (td + tau) metres before its path matches an instant full lock
(the "lag distance" used by the corner policy's turn prediction).
Radius = odom vx / (steady gyro z - bias), as the 2026-11-20 radius check.
READ ONLY (bags in, text out).
"""
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

FULL_RADIUS = 0.35      # commanded radius of full lock (bc_node full_turn_radius)


def read(bag):
    """Arrays: cmd (t, vx, wz), gyro (stamp, gz), odom (t, vx)."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cmd, gyro, odom = [], [], []
    while reader.has_next():
        topic, raw, t = reader.read_next()
        if topic not in ('/cmd_vel_out', '/imu/data', '/odom'):
            continue
        m = deserialize_message(raw, get_message(types[topic]))
        t *= 1e-9
        if topic == '/cmd_vel_out':
            cmd.append((t, m.linear.x, m.angular.z))
        elif topic == '/imu/data':
            gyro.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec,
                         m.angular_velocity.z))
        else:
            odom.append((t, m.twist.twist.linear.x))
    return np.array(cmd), np.array(gyro), np.array(odom)


def crossing(t, y, level):
    """First time y reaches level (linear interpolation), None if never."""
    i = np.nonzero(y >= level)[0]
    if not len(i):
        return None
    i = i[0]
    if i == 0:
        return float(t[0])
    return float(t[i - 1] + (level - y[i - 1]) / (y[i] - y[i - 1]) * (t[i] - t[i - 1]))


def locks(cmd):
    """(onset time, sign, end time) of each full-lock command held >= 1.5 s."""
    t, vx, wz = cmd[:, 0], cmd[:, 1], cmd[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        full = (vx > 0.05) & (np.abs(vx / wz) < FULL_RADIUS * 1.1)
    sign = np.sign(wz) * full
    out, i = [], 1
    while i < len(t):
        if sign[i] != 0 and sign[i - 1] != sign[i]:
            j = i
            while j + 1 < len(t) and sign[j + 1] == sign[i]:
                j += 1
            if t[j] - t[i] >= 1.5:
                out.append((t[i], int(sign[i]), t[j]))
            i = j + 1
        else:
            i += 1
    return out


def steering(bags):
    """
    Report the steering as a rate-limited servo.

    The servo as a rate-limited actuator (2026-11-23): per commanded curvature (wz / vx of
    /cmd_vel_out, 1/m) the steady driven curvature (gyro z / odom vx, held >= 0.6 s,
    median after 0.4 s); per command change the dead time (until the curvature has moved
    0.3 1/m towards the new value) and the rate (fastest 0.1 s slope within 0.8 s, 1/m/s).
    """
    steady, moves = {}, []
    for bag in bags:
        uri = bag.rstrip('/') + ('' if bag.rstrip('/').endswith('bag') else '/bag')
        cmd, gyro, odom = read(uri)
        if not len(cmd) or not len(gyro) or not len(odom):
            continue
        still = np.interp(gyro[:, 0], odom[:, 0], np.abs(odom[:, 1])) < 0.002
        bias = float(np.median(gyro[still, 1])) if still.sum() > 20 else 0.0
        v = np.interp(gyro[:, 0], odom[:, 0], odom[:, 1])
        tk = gyro[:, 0]
        k = np.where(v > 0.05, (gyro[:, 1] - bias) / np.maximum(v, 0.05), np.nan)
        ok = cmd[:, 1] > 0.05
        kc = np.where(ok, cmd[:, 2] / np.where(ok, cmd[:, 1], 1.0), np.nan)
        kc = np.round(kc / 0.05) * 0.05
        i = 0
        while i < len(cmd):
            j = i
            while j + 1 < len(cmd) and kc[j + 1] == kc[i]:
                j += 1
            t0, t1 = cmd[i, 0], cmd[j, 0]
            if np.isfinite(kc[i]) and t1 - t0 >= 0.6:
                seg = k[(tk > t0 + 0.4) & (tk <= t1)]
                seg = seg[np.isfinite(seg)]
                if len(seg) >= 4:
                    steady.setdefault(float(kc[i]), []).append(float(np.median(seg)))
            if i > 0 and np.isfinite(kc[i]) and np.isfinite(kc[i - 1]):
                w = (tk >= t0 - 0.05) & (tk <= t0 + 0.8)
                tt, kk = tk[w], k[w]
                good = np.isfinite(kk)
                tt, kk = tt[good], kk[good]
                if len(kk) > 6:
                    target = 1.0 if kc[i] > kc[i - 1] else -1.0
                    start = kk[0]
                    moved = np.nonzero(target * (kk - start) >= 0.3)[0]
                    slope = np.diff(kk) / np.maximum(np.diff(tt), 1e-3)
                    if len(moved):
                        moves.append((float(kc[i - 1]), float(kc[i]), tt[moved[0]] - t0,
                                      float(np.max(target * slope)), float(start)))
            i = j + 1
    print('steady driven curvature per commanded curvature (1/m): n, median, p10-p90')
    for c in sorted(steady):
        s = np.array(steady[c])
        print('  cmd %+5.2f  n %3d  driven %+5.2f  (%+5.2f .. %+5.2f)  radius %.3f m'
              % (c, len(s), np.median(s), np.percentile(s, 10), np.percentile(s, 90),
                 1.0 / abs(np.median(s)) if abs(np.median(s)) > 0.05 else float('inf')))
    print('command changes: dead time (s) until moved 0.3 1/m, fastest rate (1/m/s)')
    groups = {}
    for a, b, dead, rate, start in moves:
        groups.setdefault((a, b), []).append((dead, rate))
    for (a, b), rows in sorted(groups.items()):
        r = np.array(rows)
        if len(r) >= 3:
            print('  %+5.2f -> %+5.2f  n %3d  dead %.3f s  rate %5.1f /m/s'
                  % (a, b, len(r), np.median(r[:, 0]), np.median(r[:, 1])))
    r = np.array([(d, rt) for a, b, d, rt, s in moves])
    if len(r):
        print('  all  n %d  dead median %.3f s  rate median %.1f /m/s (p25 %.1f, p75 %.1f)'
              % (len(r), np.median(r[:, 0]), np.median(r[:, 1]), np.percentile(r[:, 1], 25),
                 np.percentile(r[:, 1], 75)))


def main():
    if sys.argv[1] == '--steering':
        steering(sys.argv[2:])
        return
    rows = []
    for bag in sys.argv[1:]:
        uri = bag.rstrip('/') + ('' if bag.rstrip('/').endswith('bag') else '/bag')
        cmd, gyro, odom = read(uri)
        if not len(cmd) or not len(gyro):
            print(bag, 'no /cmd_vel_out or /imu/data')
            continue
        still = np.interp(gyro[:, 0], odom[:, 0], np.abs(odom[:, 1])) < 0.002
        bias = float(np.median(gyro[still, 1])) if still.sum() > 20 else 0.0
        for t0, s, t1 in locks(cmd):
            g = gyro[(gyro[:, 0] > t0 - 0.4) & (gyro[:, 0] < min(t0 + 3.0, t1))]
            before = g[g[:, 0] < t0, 1]
            steady = g[g[:, 0] > t0 + 1.5, 1]
            if len(before) < 3 or len(steady) < 10:
                continue
            g0, g1 = float(np.mean(before)), float(np.median(steady))
            if abs(g1 - g0) < 0.05:
                continue
            y = (g[:, 1] - g0) / (g1 - g0)
            after = g[:, 0] >= t0
            t50 = crossing(g[after, 0], y[after], 0.5)
            t90 = crossing(g[after, 0], y[after], 0.9)
            if t50 is None or t90 is None:
                continue
            tau = (t90 - t50) / math.log(5.0)
            td = (t50 - t0) - tau * math.log(2.0)
            v = float(np.median(odom[(odom[:, 0] > t0 + 1.5) & (odom[:, 0] < t1), 1]))
            radius = v / abs(g1 - bias)
            start = (g0 - bias) / (g1 - bias)    # 0 = straight before, 1 = already at full lock
            rows.append((s, td, tau, v, radius, start))
            print('%s  %s  from %+.2f  td %.3f s  tau %.3f s  t90 %.2f s  v %.3f  lag %.3f m'
                  '  radius %.3f m' % (bag.rstrip('/').split('/')[-1], 'L' if s > 0 else 'R',
                                       start, td, tau, t90 - t0, v, v * (td + tau), radius))
    print('summary: locks that started from about straight (|from| < 0.3) and at speed (v > 0.09)')
    for s, name in ((1, 'left'), (-1, 'right')):
        r = np.array([row[1:5] for row in rows
                      if row[0] == s and abs(row[5]) < 0.3 and row[3] > 0.09])
        if len(r):
            md = np.median(r, axis=0)
            print('%-5s n %2d  median td %.3f s  tau %.3f s  v %.3f  lag %.3f m  radius %.3f m'
                  ' (radius range %.3f-%.3f)' % (name, len(r), md[0], md[1], md[2],
                                                 md[2] * (md[0] + md[1]), md[3],
                                                 r[:, 3].min(), r[:, 3].max()))


if __name__ == '__main__':
    main()
