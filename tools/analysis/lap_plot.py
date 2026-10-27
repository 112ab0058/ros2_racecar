#!/usr/bin/env python3
"""
Top-down SVG of lap runs: field walls, island, each run's path (dashed) and TRUE trajectory,
with the point closest to an outer wall marked. Pure Python (no matplotlib).

    python tools/analysis/lap_plot.py OUT.svg RUN_DIR[=label] [RUN_DIR[=label] ...]
    python tools/analysis/lap_plot.py OUT.svg A/base055_s1=baseline B/combo_058_s1=best --zoom se
"""
import argparse
import csv
import os

COLORS = ['#1f6fd1', '#d1541f', '#2a9d4b', '#8e44ad', '#c0392b', '#7f8c8d']
WALL, ISLAND = 0.8, 0.2
VIEWS = {'all': (-0.85, -0.85, 1.7), 'se': (0.05, -0.85, 0.8)}   # x0, y0, size (m)


def read(path):
    with open(path) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('out')
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--zoom', default='all', choices=sorted(VIEWS))
    a = ap.parse_args()
    x0, y0, size = VIEWS[a.zoom]
    px = 600.0 / size

    def p(x, y):
        return (x - x0) * px, (y0 + size - y) * px

    def poly(points, **style):
        attrs = ' '.join('%s="%s"' % (k.replace('_', '-'), v) for k, v in style.items())
        return '<polyline points="%s" fill="none" %s/>' % (
            ' '.join('%.1f,%.1f' % p(x, y) for x, y in points), attrs)

    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="600" height="%d" '
           'font-family="sans-serif" font-size="13">' % (600 + 22 * len(a.runs)),
           '<rect width="600" height="600" fill="#fafafa"/>']
    w = [(-WALL, -WALL), (WALL, -WALL), (WALL, WALL), (-WALL, WALL), (-WALL, -WALL)]
    out.append(poly(w, stroke='#333', stroke_width=3))
    isl = [(-ISLAND, -ISLAND), (ISLAND, -ISLAND), (ISLAND, ISLAND), (-ISLAND, ISLAND),
           (-ISLAND, -ISLAND)]
    out.append(poly(isl, stroke='#333', stroke_width=3))
    for k, spec in enumerate(a.runs):
        run, _, label = spec.partition('=')
        label = label or os.path.basename(os.path.normpath(run))
        color = COLORS[k % len(COLORS)]
        if os.path.exists(os.path.join(run, 'path.csv')):
            path = read(os.path.join(run, 'path.csv'))
            out.append(poly([(r['x'], r['y']) for r in path] + [(path[0]['x'], path[0]['y'])],
                            stroke=color, stroke_width=1, stroke_dasharray='4 3', opacity=0.8))
        name = 'nav_lap.csv' if os.path.exists(os.path.join(run, 'nav_lap.csv')) else 'lap.csv'
        rows = read(os.path.join(run, name))
        out.append(poly([(r['truth_x'], r['truth_y']) for r in rows], stroke=color,
                        stroke_width=1.6))
        key = 'outer' if 'outer' in rows[0] else 'clearance'
        worst = min(rows, key=lambda r: r[key])
        cx, cy = p(worst['truth_x'], worst['truth_y'])
        out.append('<circle cx="%.1f" cy="%.1f" r="5" fill="%s"/>' % (cx, cy, color))
        out.append('<text x="10" y="%d" fill="%s">%s: closest outer wall %.1f cm (dot)</text>'
                   % (620 + 22 * k, color, label, 100 * worst[key]))
    out.append('</svg>')
    with open(a.out, 'w', encoding='utf-8') as f:
        f.write('\n'.join(out) + '\n')


if __name__ == '__main__':
    main()
