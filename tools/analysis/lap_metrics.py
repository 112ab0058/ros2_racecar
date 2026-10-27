#!/usr/bin/env python3
"""
Score lap runs (lap_node or lap_sim.py output folders) with one set of numbers.

    python tools/analysis/lap_metrics.py RUN_DIR [RUN_DIR ...]            # one line per run
    python tools/analysis/lap_metrics.py --group NAME RUN_DIR RUN_DIR ...  # + the set row

Per run (lap.csv, lap.txt; nav_lap.csv for Nav2 runs):
  outer    closest footprint corner to an OUTER wall (m)       <- main number, PASS >= 0.03
  island   closest footprint point to the island (m)
  obst     closest footprint point to an obstacle box (m), if any
  lat p95 / max   |lateral error| of the TRUE pose to the run's own path (m), from the
           moment the car is on the line (< 3 cm); in / out = extremes island (+) / wall (-)
  ekf p95  EKF position error (m), truth moved to the EKF message time with its speed
           (Nav2 runs: Nav2's own pose, TF map -> base_link = AMCL + EKF)
  lap      time per lap (s); cpu: the node's CPU share of one core (lap.txt)
Set row: clearances = the MINIMUM of the runs, errors = the MAXIMUM, lap / cpu = the mean.
"""
import argparse
import csv
import math
import os
import re

MIN_CLEARANCE = 0.03


def pct(values, q):
    v = sorted(values)
    return v[int(q * (len(v) - 1))] if v else float('nan')


def score(run):
    """Return a dict of numbers for one run folder."""
    nav = os.path.exists(os.path.join(run, 'nav_lap.csv'))
    rows = list(csv.DictReader(open(os.path.join(run, 'nav_lap.csv' if nav else 'lap.csv'))))
    rows = [{k: float(v) for k, v in r.items()} for r in rows]
    text = ''
    for name in ('lap.txt', 'nav_lap.txt'):
        if os.path.exists(os.path.join(run, name)):
            text = open(os.path.join(run, name)).read()
    out = {'run': os.path.basename(os.path.normpath(run)), 'n': len(rows)}
    if not rows:
        return out
    key_lat = 'lat' if 'lat' in rows[0] else None
    t_line = next((r['t'] for r in rows if r['xte'] < 0.03), None)
    on = [r for r in rows if t_line is not None and r['t'] >= t_line]
    if key_lat:
        lat = [r['lat'] for r in on]
    else:                                   # old csv: unsigned distance only
        lat = [r['xte'] for r in on]
    out['lat_p95'] = pct([abs(v) for v in lat], 0.95)
    out['lat_max'] = max((abs(v) for v in lat), default=float('nan'))
    out['lat_in'] = max(lat, default=float('nan'))
    out['lat_out'] = min(lat, default=float('nan'))
    if 'outer' in rows[0]:
        out['outer'] = min(r['outer'] for r in rows)
        out['island'] = min(r['island'] for r in rows)
    else:
        out['outer'] = min(r['clearance'] for r in rows)
        out['island'] = float('nan')
    if 'obstacle' in rows[0]:
        o = min(r['obstacle'] for r in rows)
        out['obst'] = o if o < 9.0 else float('nan')
    if 'ekf_x' in rows[0]:
        err = []
        for r in rows:
            dt = r.get('ekf_stamp', 0.0) - r.get('truth_stamp', 0.0)
            if abs(dt) > 0.2:
                dt = 0.0
            tx = r['truth_x'] + r['truth_v'] * dt * math.cos(r['truth_yaw'])
            ty = r['truth_y'] + r['truth_v'] * dt * math.sin(r['truth_yaw'])
            err.append(math.hypot(r['ekf_x'] - tx, r['ekf_y'] - ty))
        out['ekf_p95'] = pct(err, 0.95)
        out['ekf_end'] = err[-1]
    elif 'loc_x' in rows[0]:                # Nav2: TF map -> base_link (AMCL + EKF)
        err = [math.hypot(r['loc_x'] - r['truth_x'], r['loc_y'] - r['truth_y']) for r in rows
               if not math.isnan(r['loc_x'])]
        out['ekf_p95'] = pct(err, 0.95)
    m = re.search(r'laps: (\d+) x ([\d.]+) m, finished (\S+) in ([\d.]+) s', text)
    if m:
        out['laps'], out['length'] = int(m.group(1)), float(m.group(2))
        out['finished'] = m.group(3) == 'yes'
        out['lap'] = float(m.group(4)) / int(m.group(1))
    m = re.search(r'Nav2 through \d+ goals \((\d+) lap.*?: (\S+) in ([\d.]+) s', text)
    if m:
        out['laps'] = int(m.group(1))
        out['finished'] = 'SUCCEEDED' in m.group(2)
        out['lap'] = float(m.group(3)) / int(m.group(1))
    m = re.search(r'straights ([\d. /]+) m \(total ([\d.]+) m per lap\)', text)
    if m:
        out['straights'] = float(m.group(2))
    m = re.search(r'CPU ([\d.]+) % of one core, control tick mean ([\d.]+) ms, max ([\d.]+)', text)
    if m:
        out['cpu'], out['tick'], out['tick_max'] = (float(m.group(i)) for i in (1, 2, 3))
    return out


def group(name, scores):
    """Aggregate a set of runs: worst clearance, worst error, mean time."""
    g = {'run': name, 'runs': len(scores)}

    def vals(k):
        return [s[k] for s in scores if k in s and not math.isnan(s[k])]

    for k in ('outer', 'island', 'obst', 'lat_out'):
        if vals(k):
            g[k] = min(vals(k))
    for k in ('lat_p95', 'lat_max', 'lat_in', 'ekf_p95', 'tick_max'):
        if vals(k):
            g[k] = max(vals(k))
    for k in ('lap', 'cpu', 'tick', 'straights'):
        if vals(k):
            g[k] = sum(vals(k)) / len(vals(k))
    g['finished'] = all(s.get('finished', False) for s in scores)
    return g


def fmt(s, cm=True):
    def f(k, scale=100.0, d=1):
        v = s.get(k)
        return '—' if v is None or (isinstance(v, float) and math.isnan(v)) else \
            ('%.*f' % (d, v * scale))
    verdict = 'PASS' if s.get('outer', -1) >= MIN_CLEARANCE and s.get('finished') else 'FAIL'
    return ('| %s | %s | %s | %s | %s / %s | %s / %s | %s | %s | %s | %s | %s |'
            % (s['run'], f('outer'), f('island'), f('obst'), f('lat_p95'), f('lat_max'),
               f('lat_in'), f('lat_out'), f('ekf_p95'), f('lap', 1.0), f('straights', 1.0, 2),
               f('cpu', 1.0), verdict))


HEADER = ('| run | outer wall cm | island cm | obstacle cm | lat p95 / max cm | '
          'lat in / out cm | EKF p95 cm | lap s | straights m | CPU % | result |\n'
          '|---|---|---|---|---|---|---|---|---|---|---|')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--group', default=None, help='name of the set row')
    ap.add_argument('--quiet', action='store_true', help='only the set row')
    a = ap.parse_args()
    scores = [score(r) for r in a.runs]
    if not a.quiet:
        print(HEADER)
        for s in scores:
            print(fmt(s))
    if a.group:
        if a.quiet:
            print(HEADER)
        print(fmt(group(a.group, scores)))


if __name__ == '__main__':
    main()
