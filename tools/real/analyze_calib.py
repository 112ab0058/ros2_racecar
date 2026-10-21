#!/usr/bin/env python3
"""
Calibration analysis for a folder written by sensor_log_node (odom.csv, imu.csv).
Standard library only: runs on Windows, WSL or the Orin.

  python analyze_calib.py static   DIR
      car still: gyro bias and noise, accel noise, which axis is up, odom drift
  python analyze_calib.py straight DIR --distance 2.0 [--vx-scale 0.814]
      car pushed straight a measured distance: new vx_scale
  python analyze_calib.py turn     DIR [--turns 1] [--wz-scale 0.814]
      car turned LEFT by hand (counter-clockwise seen from above): signs of
      gyro z / odom wz, wz_scale (IMU as reference), gyro scale (vs --turns)

Each push/turn recording must start with >= 3 s of the car standing still
(--still): that part gives the gyro bias, which is removed before integrating.
"""
import argparse
import csv
import math
import os
import statistics as st
import sys


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    return [{k: float(v) for k, v in r.items()} for r in rows]


def mean_std(v):
    return (st.fmean(v), st.pstdev(v)) if v else (float('nan'), float('nan'))


def unwrap_total(yaws):
    total = 0.0
    for a, b in zip(yaws, yaws[1:]):
        total += math.atan2(math.sin(b - a), math.cos(b - a))
    return total


def integrate(rows, key, bias=0.0):
    total = 0.0
    for a, b in zip(rows, rows[1:]):
        dt = b['t'] - a['t']
        if 0 < dt < 0.5:                 # skip gaps (same idea as odom_max_gap)
            total += 0.5 * (a[key] + b[key] - 2 * bias) * dt
    return total


def still_part(rows, seconds):
    if not rows:
        return []
    t0 = rows[0]['t']
    return [r for r in rows if r['t'] - t0 <= seconds]


def rate(rows):
    if len(rows) < 2:
        return 0.0
    return (len(rows) - 1) / (rows[-1]['t'] - rows[0]['t'])


def static(odom, imu):
    span = imu[-1]['t'] - imu[0]['t'] if imu else 0.0
    out = ['IMU %d rows, %.1f Hz, %.1f s' % (len(imu), rate(imu), span)]
    out.append('axis  gyro mean (rad/s)  std (rad/s)  variance     | accel mean  std (m/s^2)')
    gvar, avar = [], []
    for ax in 'xyz':
        gm, gs = mean_std([r['g' + ax] for r in imu])
        am, as_ = mean_std([r['a' + ax] for r in imu])
        gvar.append(gs ** 2)
        avar.append(as_ ** 2)
        out.append('  %s   %+.5f          %.5f      %.2e     | %+7.3f    %.4f'
                   % (ax, gm, gs, gs ** 2, am, as_))
    acc = [st.fmean([r['a' + ax] for r in imu]) for ax in 'xyz']
    up = max(range(3), key=lambda i: abs(acc[i]))
    g_norm = math.sqrt(sum(a * a for a in acc))
    sign = '-' if acc[up] < 0 else '+'
    out.append('gravity |a| = %.3f m/s^2 (expect 9.81); largest on %s%s'
               % (g_norm, sign, 'xyz'[up]))
    out.append('  -> REP-103 imu_link with z up should read about +9.8 on z')
    gz = st.fmean([r['gz'] for r in imu])
    out.append('uncorrected gyro z bias would turn the heading %.2f deg per minute'
               % math.degrees(gz * 60))
    out.append('suggested imu_gyro_covariance  %.1e  (largest axis variance)' % max(gvar))
    out.append('suggested imu_accel_covariance %.1e' % max(avar))
    if odom:
        d = math.hypot(odom[-1]['x'] - odom[0]['x'], odom[-1]['y'] - odom[0]['y'])
        out.append('odom while still: moved %.4f m, yaw %.3f deg (expect 0)'
                   % (d, math.degrees(unwrap_total([r['yaw'] for r in odom]))))
    return out


def straight(odom, imu, distance, vx_scale, still):
    d_end = math.hypot(odom[-1]['x'] - odom[0]['x'], odom[-1]['y'] - odom[0]['y'])
    d_path = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(odom, odom[1:]))
    out = ['odom: path %.4f m, start->end %.4f m, yaw change %.2f deg'
           % (d_path, d_end, math.degrees(unwrap_total([r['yaw'] for r in odom])))]
    if imu:
        bias = st.fmean([r['gz'] for r in still_part(imu, still)])
        out.append('imu:  heading change %.2f deg (bias %.5f rad/s removed)'
                   % (math.degrees(integrate(imu, 'gz', bias)), bias))
    if d_path > 0:
        out.append('measured %.3f m -> odom error %+.1f %%'
                   % (distance, 100 * (d_path - distance) / distance))
        out.append('new vx_scale = %.3f * %.3f / %.4f = %.4f'
                   % (vx_scale, distance, d_path, vx_scale * distance / d_path))
    return out


def turn(odom, imu, turns, wz_scale, still):
    bias = st.fmean([r['gz'] for r in still_part(imu, still)])
    still_rows = still_part(imu, still)
    imu_yaw = {}
    for ax in 'xyz':
        imu_yaw[ax] = integrate(imu, 'g' + ax, st.fmean([r['g' + ax] for r in still_rows]))
    odom_yaw = unwrap_total([r['yaw'] for r in odom])
    out = ['gyro integrated (bias from the first %.0f s removed):' % still]
    for ax in 'xyz':
        out.append('  %s  %+8.1f deg' % (ax, math.degrees(imu_yaw[ax])))
    big = max('xyz', key=lambda a: abs(imu_yaw[a]))
    out.append('rotation shows on gyro %s; LEFT turn must be +z (REP-103)' % big)
    out.append('  -> %s' % ('OK: imu_link axes as assumed' if big == 'z' and imu_yaw['z'] > 0
                            else 'CHECK: imu_link orientation in r550.urdf is wrong'))
    out.append('odom yaw total %+8.1f deg  (%s)' % (
        math.degrees(odom_yaw), 'sign OK' if odom_yaw > 0 else 'CHECK sign'))
    out.append('gyro z bias %.5f rad/s' % bias)
    if turns:
        true = 2 * math.pi * turns
        out.append('true %+.0f deg -> gyro z error %+.1f %%, odom error %+.1f %%'
                   % (math.degrees(true), 100 * (imu_yaw['z'] - true) / true,
                      100 * (odom_yaw - true) / true))
    if abs(odom_yaw) > 0.1:
        ref = imu_yaw['z']
        out.append('new wz_scale (IMU as reference) = %.3f * %.1f / %.1f = %.4f'
                   % (wz_scale, math.degrees(ref), math.degrees(odom_yaw),
                      wz_scale * ref / odom_yaw))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('test', choices=['static', 'straight', 'turn'])
    ap.add_argument('dir')
    ap.add_argument('--distance', type=float, help='measured distance (m), straight test')
    ap.add_argument('--turns', type=float, default=0.0, help='full LEFT turns made, turn test')
    ap.add_argument('--vx-scale', type=float, default=0.814)
    ap.add_argument('--wz-scale', type=float, default=0.814)
    ap.add_argument('--still', type=float, default=3.0, help='seconds still at the start')
    a = ap.parse_args(argv)
    odom = load(os.path.join(a.dir, 'odom.csv'))
    imu = load(os.path.join(a.dir, 'imu.csv'))
    if a.test == 'static':
        lines = static(odom, imu)
    elif a.test == 'straight':
        if not a.distance:
            ap.error('straight needs --distance')
        lines = straight(odom, imu, a.distance, a.vx_scale, a.still)
    else:
        lines = turn(odom, imu, a.turns, a.wz_scale, a.still)
    text = '\n'.join(['== %s %s ==' % (a.test, a.dir)] + lines)
    print(text)
    with open(os.path.join(a.dir, 'analysis_%s.txt' % a.test), 'w') as f:
        f.write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
