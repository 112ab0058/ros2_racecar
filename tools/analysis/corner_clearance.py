#!/usr/bin/env python3
"""
Check whether the R550 can turn a corner of the 160 cm field (60 cm lane).

Geometry only (no simulation): the car footprint (relative to the rear-axle centre,
measured 2026-10-20) drives straight along the south lane, turns left with rear-axle
radius R around the island corner, then goes north. Searches the lane positions with
the largest minimum clearance to the outer wall and the island.

  python corner_clearance.py
"""
import math

XR, XF, HW = -0.031, 0.24, 0.09     # rear end, front end, half width (m) from rear axle
OUT, ISL = 0.8, 0.2                 # outer wall and island half sizes (m)


def footprint_points(x, y, th, n=12):
    c, s = math.cos(th), math.sin(th)
    local = []
    for i in range(n + 1):
        ax = XR + (XF - XR) * i / n
        local += [(ax, -HW), (ax, HW)]
    for i in range(1, n):
        ay = -HW + 2 * HW * i / n
        local += [(XR, ay), (XF, ay)]
    return [(x + c * ax - s * ay, y + s * ax + c * ay) for ax, ay in local], (c, s)


def clearance(x, y, th):
    """Smallest gap (m) between the footprint and any wall; negative = collision."""
    pts, (c, s) = footprint_points(x, y, th)
    cl = min(OUT - max(abs(px), abs(py)) for px, py in pts)
    for px, py in pts:
        if abs(px) < ISL and abs(py) < ISL:
            return -0.01
        cl = min(cl, math.hypot(max(abs(px) - ISL, 0.0), max(abs(py) - ISL, 0.0)))
    for qx, qy in ((ISL, ISL), (ISL, -ISL), (-ISL, ISL), (-ISL, -ISL)):
        lx = c * (qx - x) + s * (qy - y)
        ly = -s * (qx - x) + c * (qy - y)
        if XR <= lx <= XF and -HW <= ly <= HW:
            return -0.01
    return cl


def path_clearance(y0, x1, radius):
    """South lane at y=y0 heading +x, left turn of the given radius, north at x=x1."""
    cx, cy = x1 - radius, y0 + radius
    worst = math.inf
    for i in range(41):
        worst = min(worst, clearance(cx - 0.4 + 0.4 * i / 40, y0, 0.0))
    for i in range(91):
        a = math.radians(i)
        worst = min(worst, clearance(cx + radius * math.sin(a), cy - radius * math.cos(a), a))
    for i in range(41):
        worst = min(worst, clearance(x1, cy + 0.4 * i / 40, math.pi / 2))
    return worst


def main():
    for radius in (0.35, 0.40, 0.45, 0.50):
        best = max((path_clearance(y0 / 1000, x1 / 1000, radius), y0, x1)
                   for y0 in range(-700, -299, 10) for x1 in range(300, 701, 10))
        print('R %.2f m: best min clearance %.3f m at south-lane y=%.2f, east-lane x=%.2f'
              % (radius, best[0], best[1] / 1000, best[2] / 1000))


if __name__ == '__main__':
    main()
