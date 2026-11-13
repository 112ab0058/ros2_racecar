#!/usr/bin/env python3
"""
Analyse a soak-test folder (orin_soak.sh / soak_log_node). Standard library only.

  python analyze_soak.py DIR

Checks (each line ends with PASS / WARN / INFO):
  rates and gaps      imu_raw, odom, ekf (message stamps), scans per minute
  gyro                Allan deviation of raw gyro z (angle random walk, bias
                      instability), raw bias drift per 10 min
  ZUPT                bias learned (diagnostics) vs an offline replay of zupt.py on
                      the raw gyro; corrected gyro mean
  EKF / odom drift    yaw and position change over the whole run, car still
  battery             start, end, slope
  resources           CPU % and memory growth per node (leak check), temperatures
  lidar               per-beam noise (std) and valid fraction
Writes DIR/report_soak.txt and DIR/allan.csv.
"""
import csv
import math
import os
import statistics as st
import sys

# thresholds (car standing still)
MAX_GAP_S = 0.5
EKF_YAW_DEG_PER_H = 3.0
EKF_POS_M = 0.01
LEAK_MB_PER_H = 5.0
LEAK_MIN_GROWTH_MB = 20.0
MIN_BATTERY_V = 11.0
LIDAR_STD_MM = 20.0
LIDAR_NEAR_M = 1.5        # judge noise only on beams this close (ToF noise grows far away)


def load(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        try:
            out.append({k: (float(v) if k not in ('name', 'zone', 'zupt_learning') else v)
                        for k, v in r.items()})
        except (TypeError, ValueError):
            continue
    return out


def col(rows, key):
    return [r[key] for r in rows]


def gaps(ts):
    d = [b - a for a, b in zip(ts, ts[1:])]
    return d


def slope(xs, ys):
    """Least-squares slope dy/dx."""
    if len(xs) < 2:
        return 0.0
    mx, my = st.fmean(xs), st.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else 0.0


def unwrap(angles):
    out, total = [], 0.0
    for i, a in enumerate(angles):
        if i:
            d = a - angles[i - 1]
            total += math.atan2(math.sin(d), math.cos(d))
        out.append(total)
    return out


def allan(rate, tau0):
    """Overlapping Allan deviation of a rate signal sampled every tau0 s."""
    n = len(rate)
    theta = [0.0]
    for r in rate:
        theta.append(theta[-1] + r * tau0)
    ms, m = [], 1
    while m <= (n - 1) // 3:
        ms.append(m)
        m = max(m + 1, int(m * (10 ** (1.0 / 10))))   # ~10 points per decade
    out = []
    for m in ms:
        tau = m * tau0
        k_max = n + 1 - 2 * m
        if k_max < 2:
            break
        acc = 0.0
        for k in range(k_max):
            d = theta[k + 2 * m] - 2 * theta[k + m] + theta[k]
            acc += d * d
        out.append((tau, math.sqrt(acc / (2 * tau * tau * k_max))))
    return out


def adev_at(curve, tau):
    """Log-log interpolation of the Allan curve at tau."""
    for (t1, a1), (t2, a2) in zip(curve, curve[1:]):
        if t1 <= tau <= t2:
            f = (math.log(tau) - math.log(t1)) / (math.log(t2) - math.log(t1))
            return math.exp(math.log(a1) + f * (math.log(a2) - math.log(a1)))
    return float('nan')


def replay_zupt(ts, raw):
    """Run zupt.GyroBias on the raw gyro z (wheels still): [(t, bias)], or None."""
    here = os.path.dirname(os.path.abspath(__file__))
    for p in (os.path.join(here, '..', 'src', 'wro2026_base_r550'),
              os.path.join(here, '..', '..', 'src', 'wro2026_base_r550')):
        if os.path.isdir(p) and p not in sys.path:
            sys.path.append(p)
    try:
        from wro2026_base_r550.zupt import GyroBias
    except ImportError:
        return None
    z = GyroBias()
    out = []
    for t, g in zip(ts, raw):
        z.update(t, 0.0, 0.0, 0.0, 0.0, g)
        out.append((t, z.bias[2]))
    return out


def nearest(track, t):
    """Value of a time-sorted [(t, v)] track at the sample nearest to t."""
    lo, hi = 0, len(track) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if track[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return track[lo][1]


def verdict(ok):
    return 'PASS' if ok else 'WARN'


def analyse(d):
    out = []
    say = out.append
    imu_raw = load(os.path.join(d, 'imu_raw.csv'))
    imu = load(os.path.join(d, 'imu.csv'))
    odom = load(os.path.join(d, 'odom.csv'))
    ekf = load(os.path.join(d, 'ekf.csv'))
    battery = load(os.path.join(d, 'battery.csv'))
    bias = load(os.path.join(d, 'bias.csv'))
    scan_min = load(os.path.join(d, 'scan_minute.csv'))
    bins = load(os.path.join(d, 'scan_bins.csv'))
    res = load(os.path.join(d, 'resources.csv'))
    thermal = load(os.path.join(d, 'thermal.csv'))
    allan_rows = []

    say('== rates and gaps ==')
    for name, rows in (('imu_raw', imu_raw), ('odom', odom), ('ekf', ekf)):
        if len(rows) < 2:
            say('%-8s no data  WARN' % name)
            continue
        ts = col(rows, 't')
        g = gaps(ts)
        span = ts[-1] - ts[0]
        big = sum(1 for x in g if x > MAX_GAP_S)
        say('%-8s %7d msgs  %6.1f min  %5.2f Hz  max gap %.3f s  gaps>%.1fs: %d  %s'
            % (name, len(rows), span / 60, (len(rows) - 1) / span, max(g), MAX_GAP_S, big,
               verdict(big == 0)))
    if scan_min:
        s = col(scan_min, 'scans')[1:] or col(scan_min, 'scans')   # first minute partial
        say('scan     %d full minutes, scans/min min %d mean %.0f (expect 600)  %s'
            % (len(s), min(s), st.fmean(s), verdict(min(s) >= 570)))

    if len(imu_raw) > 1000:
        ts = col(imu_raw, 't')
        dt = st.median(gaps(ts))
        gz = col(imu_raw, 'gz')
        say('')
        say('== gyro z (raw, as the STM32 sends it) ==')
        say('mean %+.6f rad/s, std %.6f rad/s, tau0 %.4f s' % (st.fmean(gz), st.pstdev(gz), dt))
        curve = allan(gz, dt)
        allan_rows = curve
        a1 = adev_at(curve, 1.0)
        amin = min(curve, key=lambda c: c[1])
        say('Allan deviation at 1 s: %.2e rad/s -> angle random walk %.3f deg/sqrt(h)'
            % (a1, a1 * 180 / math.pi * 60))
        say('minimum %.2e rad/s at tau %.0f s -> bias instability %.2f deg/h  INFO'
            % (amin[1], amin[0], amin[1] / 0.664 * 180 / math.pi * 3600))
        win = int(600 / dt)
        means = [st.fmean(gz[i:i + win]) for i in range(0, len(gz) - win + 1, win)]
        if len(means) > 1:
            say('raw bias per 10 min: ' + ' '.join('%+.6f' % m for m in means))
            say('raw bias change first -> last: %+.6f rad/s (%+.2f deg/min if uncorrected)  INFO'
                % (means[-1] - means[0], math.degrees(means[-1] - means[0]) * 60))
        say('')
        say('== ZUPT ==')
        if bias:
            b = col(bias, 'gyro_bias_z')
            learning = sum(1 for r in bias if str(r.get('zupt_learning')) == 'True')
            say('diagnostics: bias %+.6f -> %+.6f rad/s, learning %d / %d samples'
                % (b[0], b[-1], learning, len(bias)))
            rep = replay_zupt(ts, gz)
            if rep is not None:
                # compare where both have converged (after 2 minutes of the replay)
                pts = [r for r in bias if r['t'] > ts[0] + 120]
                diffs = [abs(nearest(rep, r['t']) - r['gyro_bias_z']) for r in pts]
                if diffs:
                    med = st.median(diffs)
                    say('offline replay of zupt.py vs live bias: median diff %.1e rad/s '
                        '(%d points)  %s' % (med, len(diffs), verdict(med < 5e-6)))
        if imu:
            c = col(imu, 'gz')
            tail = c[len(c) // 10:]
            say('corrected gyro z mean (after the first 10%%): %+.7f rad/s (%+.3f deg/min)  %s'
                % (st.fmean(tail), math.degrees(st.fmean(tail)) * 60,
                   verdict(abs(math.degrees(st.fmean(tail)) * 60) < 0.05)))

    for name, rows in (('EKF', ekf), ('odom', odom)):
        if len(rows) < 2:
            continue
        say('')
        say('== %s drift (car still) ==' % name)
        yaw = unwrap(col(rows, 'yaw'))
        hours = (rows[-1]['t'] - rows[0]['t']) / 3600
        dyaw = math.degrees(yaw[-1] - yaw[0])
        maxyaw = max(abs(math.degrees(y - yaw[0])) for y in yaw)
        pos = max(math.hypot(r['x'] - rows[0]['x'], r['y'] - rows[0]['y']) for r in rows)
        ok = abs(dyaw) / max(hours, 1e-6) < EKF_YAW_DEG_PER_H and pos < EKF_POS_M
        say('yaw %+.3f deg over %.2f h (%+.2f deg/h), max |yaw| %.3f deg; max distance %.4f m  %s'
            % (dyaw, hours, dyaw / max(hours, 1e-6), maxyaw, pos, verdict(ok)))

    if battery:
        say('')
        say('== battery ==')
        v, t = col(battery, 'v'), col(battery, 't')
        say('%.2f V -> %.2f V, min %.2f V, slope %+.3f V/h  %s'
            % (v[0], v[-1], min(v), slope(t, v) * 3600, verdict(min(v) >= MIN_BATTERY_V)))

    if res:
        say('')
        say('== resources per node ==')
        for name in sorted({r['name'] for r in res}):
            rows = [r for r in res if r['name'] == name and r['rss_kb'] > 0]
            if len(rows) < 3:
                continue
            t = col(rows, 't')
            cpu = 100 * (rows[-1]['cpu_s'] - rows[0]['cpu_s']) / (t[-1] - t[0])
            rss = [x / 1024 for x in col(rows, 'rss_kb')]
            growth = rss[-1] - rss[0]
            per_h = slope(t, rss) * 3600
            leak = per_h > LEAK_MB_PER_H and growth > LEAK_MIN_GROWTH_MB
            say('%-24s CPU %5.1f %%  RSS %6.1f -> %6.1f MB (%+.1f MB/h)  %s'
                % (name, cpu, rss[0], rss[-1], per_h, verdict(not leak)))
    if thermal:
        say('')
        say('== temperatures (C) ==')
        for zone in sorted({r['zone'] for r in thermal}):
            v = [r['temp_c'] for r in thermal if r['zone'] == zone]
            say('%-16s %5.1f -> %5.1f, max %5.1f  INFO' % (zone, v[0], v[-1], max(v)))

    if bins:
        say('')
        say('== lidar per beam (car still) ==')
        good = [b for b in bins if b['valid_fraction'] > 0.9 and math.isfinite(b['std'])]
        for label, part, judge in (
                ('near (<= %.1f m)' % LIDAR_NEAR_M,
                 [b for b in good if b['mean'] <= LIDAR_NEAR_M], True),
                ('far  (> %.1f m)' % LIDAR_NEAR_M,
                 [b for b in good if b['mean'] > LIDAR_NEAR_M], False)):
            if not part:
                continue
            stds = sorted(b['std'] * 1000 for b in part)
            noisy = sum(1 for s in stds if s > LIDAR_STD_MM)
            tag = verdict(noisy <= len(part) * 0.05) if judge else 'INFO'
            say('%s beams valid > 90%%: %d; std median %.1f mm, 95%% %.1f mm, > %.0f mm: %d  %s'
                % (label, len(part), stds[len(stds) // 2], stds[int(len(stds) * 0.95)],
                   LIDAR_STD_MM, noisy, tag))
        say('beams valid > 90%%: %d / %d (single noisy beams near object edges = mixed '
            'pixels)  INFO' % (len(good), len(bins)))
        say('overall valid fraction %.3f  INFO' % st.fmean(col(bins, 'valid_fraction')))
    return out, allan_rows


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__)
        return 1
    d = argv[0]
    lines, curve = analyse(d)
    text = '\n'.join(['== soak analysis %s ==' % d] + lines)
    print(text)
    with open(os.path.join(d, 'report_soak.txt'), 'w') as f:
        f.write(text + '\n')
    with open(os.path.join(d, 'allan.csv'), 'w') as f:
        f.write('tau_s,adev_rad_s\n')
        for tau, a in curve:
            f.write('%.4f,%.4e\n' % (tau, a))
    return 0


if __name__ == '__main__':
    sys.exit(main())
