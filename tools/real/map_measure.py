#!/usr/bin/env python3
"""
E2: measure the field on a saved slam_toolbox map, to compare with the tape measure.
READ ONLY (map in, text out). Needs numpy only (Windows or Orin).

    python3 map_measure.py ~/r550_map/<run>/map.yaml [more maps]

The map's frame is the car's start pose, so the field sits at an angle. Steps:
  1. occupied cells (value < occupied_thresh) -> points (m)
  2. angle: the rotation that makes the walls straightest (the 2 cm histograms along x and
     y are most peaked; 0.25 deg steps over 90 deg)
  3. outer walls: median position of the points beyond 0.6 m from the centre, taking
     only the middle 0.5 m of each side (no corners); island faces: the same, 0.2-0.55 m
     from the centre, re-centred on the island (field 2026-11: island ~0.8 m,
     corridors ~0.4 m). 2026-11-28: started from several centres; the island must be
     square (sides 0.4-1.1 m, within 0.1 m of each other) and the start whose island
     faces and outer walls hold the most cells wins (things outside the field, seen
     over the 10 cm walls, had moved the single start off the island in rebuilt maps).
  4. field inside size, island size, the four corridors (island face to outer wall);
     INCOMPLETE when an outer wall is missing (runs of a few corners).
The lidar sees the walls' inner faces and the island's outer faces, as a tape measured
from inside the field does. A map cell is 2 cm, so expect about +-2 cm. Also reads the
maps of rebuild_map.py (1 cm cells).
"""
import math
import os
import sys

import numpy as np


def read_pgm(path):
    """Return a PGM (P5 binary or P2 text) as a uint8 / uint16 array."""
    with open(path, 'rb') as f:
        data = f.read()
    fields, pos = [], 0
    while len(fields) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[pos:end])
        pos = end
    magic, w, h, maxval = fields[0], int(fields[1]), int(fields[2]), int(fields[3])
    pos += 1
    if magic == b'P5':
        dtype = np.uint8 if maxval < 256 else np.dtype('>u2')
        return np.frombuffer(data, dtype=dtype, count=w * h, offset=pos).reshape(h, w)
    return np.array(data[pos:].split(), dtype=int)[:w * h].reshape(h, w)


def read_map(yaml_path):
    """Return (occupied points N x 2 in m, resolution m)."""
    meta = {}
    with open(yaml_path, encoding='utf-8') as f:
        for line in f:
            if ':' in line:
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    image = meta['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    res = float(meta['resolution'])
    origin = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    negate = int(meta.get('negate', '0'))
    occ = float(meta.get('occupied_thresh', '0.65'))
    img = read_pgm(image).astype(float)
    p = img / img.max() if negate else (img.max() - img) / img.max()
    rows, cols = np.nonzero(p > occ)
    h = img.shape[0]
    x = origin[0] + (cols + 0.5) * res
    y = origin[1] + (h - 1 - rows + 0.5) * res       # image row 0 is the top (max y)
    return np.column_stack([x, y]), res


def rotate(pts, ang):
    """Rotate points by -ang (rad)."""
    c, s = math.cos(ang), math.sin(ang)
    return np.column_stack([c * pts[:, 0] + s * pts[:, 1], -s * pts[:, 0] + c * pts[:, 1]])


def straightest(pts, step_deg=0.25, binw=0.02):
    """Rotation (rad) in 0..90 deg that makes the x and y histograms most peaked."""
    best, best_ang = -1.0, 0.0
    for deg in np.arange(0.0, 90.0, step_deg):
        r = rotate(pts, math.radians(deg))
        score = 0.0
        for k in (0, 1):
            h, _ = np.histogram(r[:, k], bins=np.arange(r[:, k].min(), r[:, k].max() + binw,
                                                        binw))
            score += float((h.astype(float) ** 2).sum())
        if score > best:
            best, best_ang = score, math.radians(deg)
    return best_ang


def measure(pts):
    """Field and island edges in the straightened frame: dict of positions (m)."""
    def face(d, k, sign, near, far):
        """Median position of one side's points (middle 0.5 m of the side), or nan."""
        other = 1 - k
        pick = (np.abs(d[:, other]) < 0.25) & (sign * d[:, k] > near) & (sign * d[:, k] < far)
        side = d[pick, k]
        return (float(np.median(side)) if len(side) >= 10 else float('nan')), int(len(side))

    def on_island(centre):
        """Re-centre on the island's faces: (island, centre, cells on the faces and walls)."""
        island = None
        for _ in range(3):
            d = pts - centre
            f = [face(d, k, s, 0.2, 0.55) for k in (0, 1) for s in (-1, 1)]
            if any(math.isnan(v) for v, _ in f):
                return None, centre, 0
            island = (np.array([f[0][0], f[2][0]]) + centre,
                      np.array([f[1][0], f[3][0]]) + centre)
            centre = (island[0] + island[1]) / 2.0
        w, h = island[1] - island[0]
        if not (0.4 < w < 1.1 and 0.4 < h < 1.1 and abs(w - h) < 0.1):
            return None, centre, 0          # the island is square; an outer wall taken for a face
        d = pts - centre
        walls = [face(d, k, s, 0.6, 1.2)[1] for k in (0, 1) for s in (-1, 1)]
        return island, centre, sum(n for _, n in f) + sum(walls)

    # Several starts, the one whose island faces and outer walls hold the most cells wins.
    # (Before 2026-11-28 only the middle of the 1-99 % range: things outside the field, seen
    # over the 10 cm walls, are > 1 % of the cells in some rebuilt maps and moved it off.)
    lo, hi = np.percentile(pts, 1, axis=0), np.percentile(pts, 99, axis=0)
    med = np.median(pts, axis=0)
    starts = [(lo + hi) / 2.0, med] + [med + [dx, dy] for dx in np.arange(-0.3, 0.31, 0.15)
                                       for dy in np.arange(-0.3, 0.31, 0.15)]
    island, centre, best = None, (lo + hi) / 2.0, 0
    for st in starts:
        isl, c, n = on_island(np.asarray(st, dtype=float))
        if isl is not None and n > best:
            island, centre, best = isl, c, n
    d = pts - centre
    out = {'island': (island[0] - centre, island[1] - centre) if island else None,
           'centre': centre}
    for k, names in ((0, ('left', 'right')), (1, ('bottom', 'top'))):
        for sign, name in ((-1, names[0]), (1, names[1])):
            out[name], out[name + '_n'] = face(d, k, sign, 0.6, 1.2)
    return out


def main(argv):
    """Entry point."""
    if len(argv) < 2:
        print(__doc__)
        return 1
    for path in argv[1:]:
        pts, res = read_map(path)
        ang = straightest(pts)
        m = measure(rotate(pts, ang))
        print('== %s  (%d occupied cells, %.0f mm, field turned %.2f deg)'
              % (path, len(pts), res * 1000, math.degrees(ang)))
        cm = lambda v: 100.0 * v
        print('   field inside: %.1f x %.1f cm  (walls left/right from %d/%d cells, '
              'bottom/top %d/%d)' % (cm(m['right'] - m['left']), cm(m['top'] - m['bottom']),
                                     m['left_n'], m['right_n'], m['bottom_n'], m['top_n']))
        if m['island'] is None:
            print('   island: not found')
            continue
        (ix0, iy0), (ix1, iy1) = m['island']
        print('   island: %.1f x %.1f cm' % (cm(ix1 - ix0), cm(iy1 - iy0)))
        print('   corridors: left %.1f, right %.1f, bottom %.1f, top %.1f cm'
              % (cm(ix0 - m['left']), cm(m['right'] - ix1), cm(iy0 - m['bottom']),
                 cm(m['top'] - iy1)))
        if any(math.isnan(m[k]) for k in ('left', 'right', 'bottom', 'top')):
            print('   INCOMPLETE map (an outer wall is missing): island and corridors not usable')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
