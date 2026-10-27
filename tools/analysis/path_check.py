#!/usr/bin/env python3
"""
Zero-error check of lap_node paths: drive the measured footprint EXACTLY along the path
(rear axle on the line, heading = path heading) and report the clearances. Shows how much
of the wall margin the geometry alone (front overhang swinging out in the corners) costs,
before any tracking error.

    python tools/analysis/path_check.py                       # the campaign table
    python tools/analysis/path_check.py --half-side 0.58 --transition 0.15 --inset 0.02
    python tools/analysis/path_check.py --offsets 0.40 0.55 0.55 0.55 --obstacles 0 -0.6
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lap_sim import import_lap_node  # noqa: E402


def check(L, half_side=0.55, radius=0.42, transition=0.0, inset=0.0, offsets=None,
          obstacles=()):
    """Return a dict: zero-error clearances, where the outer one is smallest, straights."""
    straights = []
    path = L.make_path(half_side, radius, transition=transition, offsets=offsets, inset=inset,
                       straights=straights)
    n = len(path)
    best = (float('inf'), None)
    island = obst = float('inf')
    for i in range(n):
        (x, y), (qx, qy) = path[i], path[(i + 1) % n]
        pose = (x, y, math.atan2(qy - y, qx - x))
        o = L.outer_clearance(pose)
        if o < best[0]:
            best = (o, pose)
        island = min(island, L.island_clearance(pose))
        obst = min(obst, L.obstacle_clearance(pose, obstacles))
    k = L.curvature(path)
    length = sum(math.dist(path[i], path[(i + 1) % n]) for i in range(n))
    # heading relative to the side the car is coming from, at the worst point
    rel = math.degrees(best[1][2]) % 90.0
    return {'outer': best[0], 'worst_heading': rel, 'island': island, 'obstacle': obst,
            'straights': straights, 'length': length, 'max_curv': max(k)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--half-side', type=float, default=None)
    ap.add_argument('--radius', type=float, default=0.42)
    ap.add_argument('--transition', type=float, default=0.0)
    ap.add_argument('--inset', type=float, default=0.0)
    ap.add_argument('--offsets', type=float, nargs=4, default=None)
    ap.add_argument('--obstacles', type=float, nargs='*', default=[])
    a = ap.parse_args()
    L, _ = import_lap_node()
    obstacles = list(zip(a.obstacles[0::2], a.obstacles[1::2]))
    if a.half_side is None and a.offsets is None:
        cases = [(h, r, tr, ins) for h in (0.55, 0.57, 0.58, 0.60) for r in (0.42, 0.45)
                 for tr in (0.0, 0.10, 0.20) for ins in (0.0, 0.02)]
    else:
        cases = [(a.half_side or 0.55, a.radius, a.transition, a.inset)]
    print('| half side | radius | transition | inset | outer wall cm (straight) | worst at '
          'heading | island cm | straights m | lap m | max curvature |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    for h, r, tr, ins in cases:
        try:
            c = check(L, h, r, tr, ins, a.offsets, obstacles)
        except ValueError as e:
            print('| %.2f | %.2f | %.2f | %.2f | infeasible: %s |' % (h, r, tr, ins, e))
            continue
        on_straight = 100 * (L.WALL - h - 0.095)          # nominal line, no inset
        print('| %.2f | %.2f | %.2f | %.2f | %.1f (%.1f) | %.0f deg | %.1f | %s | %.2f | %.2f |%s'
              % (h, r, tr, ins, 100 * c['outer'], on_straight, c['worst_heading'],
                 100 * c['island'], ' / '.join('%.2f' % s for s in c['straights']),
                 c['length'], c['max_curv'],
                 (' obstacle %.1f cm' % (100 * c['obstacle'])) if obstacles else ''))


if __name__ == '__main__':
    main()
