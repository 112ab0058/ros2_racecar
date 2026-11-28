#!/usr/bin/env python3
"""
Windows: check the camera wall bottoms on photos (READ ONLY, files only).

    python tools/real/cam_wall_check.py OUT_DIR PHOTO.jpg [PHOTO.jpg ...]
    python tools/real/cam_wall_check.py OUT_DIR --field DIR     # DIR/r550_camfield/*_1.jpg

Uses the same code as the node (src/wro2026_camera_c70/wro2026_camera_c70/wall.py, its
config/ calibration). For each photo writes OUT_DIR/<name>.png: left the photo with the
wall bottoms found (blue), right a top view from the lidar (car up). With --field, the
raw lidar saved by lidar_snap.py at the same moment (DIR/r550_lidartest/<label>.npy) is
drawn too (black, corrected by range_offset_m like the driver) and the camera is
compared with it: median and 90 % of the range difference where both see a wall >= 0.45 m.
Needs numpy, opencv-python, pyyaml, pillow (Windows Python has them).
"""
import glob
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PKG = os.path.join(ROOT, 'src', 'wro2026_camera_c70')
sys.path.insert(0, PKG)
from wro2026_camera_c70.wall import FloorCamera, to_ranges, wall_bottom  # noqa: E402

LASER_X = 0.13
RANGE_OFFSET = 0.036          # src/wro2026_lidar_n10p/config/n10p.yaml range_offset_m
FONT = 'C:/Windows/Fonts/msjh.ttc'


def lidar_xy(path):
    """lidar_snap.py rows (ROS deg, mm, intensity) -> points in the lidar frame (m)."""
    a = np.load(path)
    a = a[a[:, 1] > 0]
    r = a[:, 1] / 1000.0 - RANGE_OFFSET
    th = np.radians(a[:, 0])
    return np.c_[r * np.cos(th), r * np.sin(th)]


def compare(cam_ranges, lid):
    """Camera beam range minus the lidar median in the same beam (+-1 deg), lidar >= 0.45 m."""
    inc = 2.0 * math.pi / len(cam_ranges)
    lb = np.arctan2(lid[:, 1], lid[:, 0]) % (2.0 * math.pi)
    lr = np.hypot(lid[:, 0], lid[:, 1])
    out = []
    for k in np.nonzero(np.isfinite(cam_ranges))[0]:
        d = np.abs((lb - k * inc + math.pi) % (2.0 * math.pi) - math.pi)
        m = d < math.radians(1.0)
        if m.sum() >= 3:
            rl = np.median(lr[m])
            if rl >= 0.45 and abs(rl - cam_ranges[k]) < 0.25:
                out.append(cam_ranges[k] - rl)
    return np.array(out)


def draw(img, px, cam_ranges, lid, title, line, out):
    s = 320                                            # px per metre in the top view
    top = Image.new('RGB', (520, 720), '#fcfcfb')
    g = ImageDraw.Draw(top)
    cx, cy = 260, 600                                  # lidar here, forward = up

    def p(x, y):
        return cx - y * s, cy - x * s
    for rad in (0.3, 0.45, 1.0):
        g.ellipse([cx - rad * s, cy - rad * s, cx + rad * s, cy + rad * s], outline='#e4e3df')
    g.rectangle([*p(0.11, 0.09), *p(-0.161, -0.09)], outline='#0b0b0b', width=2)
    if lid is not None:
        for x, y in lid:
            if -0.4 < x < 1.8 and abs(y) < 0.8:
                a, b = p(x, y)
                g.ellipse([a - 2, b - 2, a + 2, b + 2], fill='#3a3936')
    inc = 2.0 * math.pi / len(cam_ranges)
    for k in np.nonzero(np.isfinite(cam_ranges))[0]:
        a, b = p(cam_ranges[k] * math.cos(k * inc), cam_ranges[k] * math.sin(k * inc))
        g.ellipse([a - 2, b - 2, a + 2, b + 2], fill='#2a78d6')
    f = ImageFont.truetype(FONT, 15)
    g.text((10, 10), title, font=f, fill='#0b0b0b')
    g.text((10, 32), line, font=f, fill='#52514e')
    g.text((10, 690), '藍 = 相機牆腳　黑 = 光達（已扣偏差）　虛圓 30/45/100 cm', font=f,
           fill='#52514e')
    im = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(im)
    for u, v in px:
        d.ellipse([u - 3, v - 3, u + 3, v + 3], fill='#2a78d6')
    canvas = Image.new('RGB', (640 + 520, 720), '#fcfcfb')
    canvas.paste(im.resize((640, 360)), (0, 0))
    canvas.paste(top, (640, 0))
    canvas.save(out)


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    outdir = sys.argv[1]
    os.makedirs(outdir, exist_ok=True)
    if sys.argv[2] == '--field':
        field = sys.argv[3]
        photos = sorted(glob.glob(os.path.join(field, 'r550_camfield', '*_1.jpg')))
    else:
        field, photos = None, sys.argv[2:]
    cfg = os.path.join(PKG, 'config')
    cam = FloorCamera.from_files(os.path.join(cfg, 'c70_ost_1280x720.yaml'),
                                 os.path.join(cfg, 'c70_extrinsics.yaml'))
    diffs = []
    for path in photos:
        img = cv2.imread(path)
        name = os.path.splitext(os.path.basename(path))[0]
        px = wall_bottom(img)
        ranges = to_ranges(cam.floor_points(px), LASER_X, 480, 1.0)
        seen = ranges[np.isfinite(ranges)]
        line = '相機：%d 個牆腳像素，%d 束，最近 %s' % (
            len(px), len(seen), '%.2f m' % seen.min() if len(seen) else '-')
        lid = None
        label = name.rsplit('_', 1)[0]
        npy = os.path.join(field, 'r550_lidartest', label + '.npy') if field else ''
        if npy and os.path.exists(npy):
            lid = lidar_xy(npy)
            d = compare(ranges, lid)
            if len(d):
                diffs.append(d)
                line += '；相機-光達 中位數 %+.1f cm（%d 束）' % (100 * np.median(d), len(d))
        print(name, '|', line)
        draw(img, px, ranges, lid, name, line, os.path.join(outdir, name + '.png'))
    if diffs:
        d = np.concatenate(diffs)
        print('全部：相機-光達 中位數 %+.1f cm，|差| 平均 %.1f cm、90%% %.1f cm（%d 束）' % (
            100 * np.median(d), 100 * np.mean(np.abs(d)), 100 * np.percentile(np.abs(d), 90),
            len(d)))


if __name__ == '__main__':
    main()
