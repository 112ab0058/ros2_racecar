#!/usr/bin/env python3
"""
Orin: camera tape check with the car still (READ ONLY: camera + lidar, never the motors).

    python3 ~/racecar_ws/tools/cam_tape_snap.py                 # Ray, in ssh (asks questions)
    python tools/real/cam_tape_snap.py --check DIR [CONFIG_DIR]  # recompute a recorded DIR

Why (LOGBOOK 2026-11-23): on the 11-20 bags the camera's wall bottoms read 3-16 cm shorter
than the lidar at 0.6-1.0 m, and walls the island corner showed at the image's left edge
landed on the next corridor's floor. Before the camera calibration is changed, a third
measurement decides who is right (the 11-19 lesson): a black board standing on a white
floor, taped.

Each station (STATIONS: straight ahead 0.6 / 0.8 / 1.0 m, 30 deg left / right at 0.6 /
0.9 m, 40 deg left / right at 0.6 m = the image's edge): Ray sets the board up facing the
lidar and presses Enter; the script reads the raw lidar for 3 s (lidar_snap.py, minus
range_offset 0.036 m like the driver) and saves 5 camera frames (MJPG 1280x720 like the
calibration). Then Ray tapes the board's middle to the near side of the lidar cylinder;
plus half the cylinder's diameter (asked once) = the true range from the lidar's centre.

Board: the nearest object the lidar sees as one piece within +-25 deg of the station's
direction; its middle direction and a fitted line. Camera: wall.py and the calibration of
wall_node (without max_range), the points within the board's directions. Printed and
saved (stations.csv, summary.txt), in cm from the lidar centre along the board's middle
direction: tape, lidar, camera; and the camera's offset over the whole board, each point
along its own direction, from the board line through the tape point (lidar's direction).

Nothing else may read the lidar or the camera meanwhile (orin_auto.sh, sensors.launch.py,
wall_node): checked at the start. Output: ~/r550_camtape/<time>/ (photos, raw lidar .npy,
stations.csv, summary.txt, copies of the calibration files used).
"""
import csv
import glob
import math
import os
import shutil
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
INSTALL = os.path.expanduser('~/racecar_ws/install/wro2026_camera_c70')
for _p in (os.path.join(HERE, '..', '..', 'src', 'wro2026_camera_c70'),
           os.path.join(INSTALL, 'lib', 'python3.12', 'site-packages')):
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
from wro2026_camera_c70.wall import FloorCamera, wall_bottom  # noqa: E402

CONFIG_DIRS = [os.path.join(INSTALL, 'share', 'wro2026_camera_c70', 'config'),
               os.path.join(HERE, '..', '..', 'src', 'wro2026_camera_c70', 'config')]
INTRINSICS, EXTRINSICS = 'c70_ost_1280x720.yaml', 'c70_extrinsics.yaml'
LASER_X = 0.13          # m, lidar in front of base_link (urdf)
RANGE_OFFSET = 0.036    # m, src/wro2026_lidar_n10p/config/n10p.yaml range_offset_m
SECTOR = 25.0           # deg either side of the station's direction
FRAMES = 5
BUSY = ['wro2026_lidar_n10p/scan_node', 'wro2026_camera_c70/wall_node', 'lidar_snap.py',
        'cam_snap.py', 'orin_auto.sh']

# label, direction from the lidar (deg, + = left), what, where the board's middle goes
# (lidar 0.11 m behind the front bumper: lidar range r at angle a = bumper + r cos a - 0.11)
STATIONS = [
    ('front_60', 0.0, '正前方、約 0.6 m', '車頭前 50 cm，在車子中線上'),
    ('front_80', 0.0, '正前方、約 0.8 m', '車頭前 70 cm，在車子中線上'),
    ('front_100', 0.0, '正前方、約 1.0 m', '車頭前 90 cm，在車子中線上'),
    ('left30_60', 30.0, '左前 30°、約 0.6 m', '車頭前 41 cm、從車子中線往左 30 cm'),
    ('left30_90', 30.0, '左前 30°、約 0.9 m', '車頭前 67 cm、從車子中線往左 45 cm'),
    ('right30_60', -30.0, '右前 30°、約 0.6 m', '車頭前 41 cm、從車子中線往右 30 cm'),
    ('right30_90', -30.0, '右前 30°、約 0.9 m', '車頭前 67 cm、從車子中線往右 45 cm'),
    ('left40_60', 40.0, '左邊緣 40°、約 0.6 m（畫面最左邊）', '車頭前 35 cm、從車子中線往左 39 cm'),
    ('right40_60', -40.0, '右邊緣 40°、約 0.6 m（畫面最右邊）', '車頭前 35 cm、從車子中線往右 39 cm'),
]
FIELDS = ['label', 'deg', 'tape_near_cm', 'lidar_diameter_cm', 'time']


def side(deg):
    """'左 29.6°' / '右 12.0°' / '正前 0.0°'."""
    return ('左' if deg > 0.05 else '右' if deg < -0.05 else '正前') + ' %.1f°' % abs(deg)


def lidar_points(rows):
    """lidar_snap.py rows (ROS deg, mm, intensity) -> angles (rad), ranges (m, offset off)."""
    a = np.asarray(rows, float).reshape(-1, 3)
    a = a[a[:, 1] > 0]
    return np.radians(a[:, 0]), a[:, 1] / 1000.0 - RANGE_OFFSET


def wrap(a):
    """Angle(s) to -pi..pi."""
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def find_board(th, r, deg):
    """
    Return the nearest one-piece object within deg +- SECTOR, or None.

    (points (n, 2) in the lidar frame, first and last direction (rad)). 1 deg bins; from the
    bin with the nearest median range, neighbours join while their medians differ < 4 cm.
    """
    d = np.degrees(wrap(th - math.radians(deg)))
    sel = (np.abs(d) <= SECTOR) & (r > 0.2) & (r < 1.6)
    if sel.sum() < 10:
        return None
    b = np.floor(d[sel]).astype(int)
    rr, tt = r[sel], th[sel]
    med = {int(k): float(np.median(rr[b == k])) for k in np.unique(b)}
    lo = hi = min(med, key=med.get)
    while lo - 1 in med and abs(med[lo - 1] - med[lo]) < 0.04:
        lo -= 1
    while hi + 1 in med and abs(med[hi + 1] - med[hi]) < 0.04:
        hi += 1
    if hi - lo < 2:
        return None
    near = np.abs(rr - np.array([med.get(int(k), np.inf) for k in b])) < 0.03
    keep = (b >= lo) & (b <= hi) & near
    pts = np.c_[rr[keep] * np.cos(tt[keep]), rr[keep] * np.sin(tt[keep])]
    base = math.radians(deg)
    return pts, base + math.radians(lo), base + math.radians(hi + 1)


def fit_line(p):
    """Line through points: (unit normal away from the lidar, its distance from the lidar)."""
    c = p.mean(axis=0)
    n = np.linalg.svd(p - c)[2][1]
    if n @ c < 0:
        n = -n
    return n, float(n @ c)


def camera_points(cam, images):
    """Wall bottoms of all images -> floor points (n, 2) in the lidar frame (no max range)."""
    out = [np.zeros((0, 2))]
    for img in images:
        p = cam.floor_points(wall_bottom(img))
        out.append(np.c_[p[:, 0] - LASER_X, p[:, 1]])
    return np.vstack(out)


def measure(th, r, cam_pts, deg, tape=None):
    """
    Compare tape, lidar and camera at one station; see the module docstring.

    tape: true range (m) from the lidar centre to the board's middle, or None.
    Returns a dict of numbers (m, deg) for printing and the summary.
    """
    res = {'deg': deg, 'tape': tape, 'lidar_n': 0, 'cam_n': len(cam_pts)}
    board = find_board(th, r, deg)
    cam_r = np.hypot(cam_pts[:, 0], cam_pts[:, 1])
    cam_a = np.arctan2(cam_pts[:, 1], cam_pts[:, 0])
    if board is None:
        mid, n = math.radians(deg), None
        res.update(board=False, mid=deg)
    else:
        pts, a0, a1 = board
        mid = 0.5 * (a0 + a1)
        n, dist = fit_line(pts)
        res.update(board=True, mid=math.degrees(mid), lo=math.degrees(a0), hi=math.degrees(a1),
                   lidar_n=len(pts))
        pa = np.arctan2(pts[:, 1], pts[:, 0])
        c = np.abs(wrap(pa - mid)) < math.radians(2.0)
        res['lidar'] = (float(np.median(np.hypot(pts[c, 0], pts[c, 1]))) if c.sum() >= 3
                        else dist / float(n @ [math.cos(mid), math.sin(mid)]))
    # camera points in the board's directions, not far off the board line
    if n is not None:
        ray = np.c_[np.cos(cam_a), np.sin(cam_a)]
        on = ((wrap(cam_a - (a0 + math.radians(1.0))) > 0)
              & (wrap(cam_a - (a1 - math.radians(1.0))) < 0) & (ray @ n > 0.2))
        line_r = np.where(on, dist / np.maximum(ray @ n, 0.2), np.inf)
        on &= np.abs(cam_r - line_r) < 0.25
    else:
        on = np.abs(wrap(cam_a - mid)) < math.radians(2.0)
    c = on & (np.abs(wrap(cam_a - mid)) < math.radians(2.0))
    if c.sum():
        res['cam_mid'] = float(np.median(cam_r[c]))
        res['cam_mid_n'] = int(c.sum())
    if n is not None and on.sum():
        res['cam_board'] = float(np.median(cam_r[on] - line_r[on]))       # vs the lidar line
        res['cam_board_n'] = int(on.sum())
        res['cam_lo'] = float(np.degrees(cam_a[on].min()))
        res['cam_hi'] = float(np.degrees(cam_a[on].max()))
        if tape is not None:
            u = np.array([math.cos(mid), math.sin(mid)])
            tape_r = tape * float(u @ n) / (ray[on] @ n)                   # line through tape
            e = cam_r[on] - tape_r
            res['cam_board_tape'] = float(np.median(e))
            # the 5 deg of the board farthest out in the image (the edge stations)
            if deg:
                a_deg = np.degrees(cam_a[on])
                outer = ((a_deg >= res['cam_hi'] - 5.0) if deg > 0
                         else (a_deg <= res['cam_lo'] + 5.0))
                res['cam_outer_tape'] = float(np.median(e[outer]))
    return res


def describe(res):
    """Chinese lines for one station's result."""
    out = []
    if res['board']:
        out.append('光達：板子在 %s（%s 到 %s），中間 %.1f cm（%d 點）' % (
            side(res['mid']), side(res['lo']), side(res['hi']), 100 * res['lidar'],
            res['lidar_n']))
        if abs(res['mid'] - res['deg']) > 12.0:
            out.append('注意：這一站應該在 %s，找到的最近物體偏了 %.0f°。板子旁邊有更近的東西嗎？'
                       '有的話拿開，輸入 r 重錄' % (side(res['deg']), abs(res['mid'] - res['deg'])))
    else:
        out.append('光達：在 %s ±%.0f° 內沒找到板子（太近、太遠或被擋住）' % (side(res['deg']), SECTOR))
    if 'cam_mid' in res:
        line = '相機：中間 %.1f cm（%d 點）' % (100 * res['cam_mid'], res['cam_mid_n'])
        if res['board']:
            line += '，比光達%s %.1f cm' % ('長' if res['cam_mid'] > res['lidar'] else '短',
                                          100 * abs(res['cam_mid'] - res['lidar']))
        out.append(line)
    elif res['cam_n'] == 0:
        out.append('相機：整張照片都沒找到牆腳（地板夠白嗎？燈夠亮嗎？）')
    else:
        out.append('相機：板子中間的方向沒有牆腳點（照片已存，之後再看）')
    if 'cam_board' in res:
        out.append('相機：整塊板子（%s 到 %s，%d 點）平均比光達%s %.1f cm' % (
            side(res['cam_lo']), side(res['cam_hi']), res['cam_board_n'],
            '長' if res['cam_board'] > 0 else '短', 100 * abs(res['cam_board'])))
    if res['tape'] is not None:
        parts = ['捲尺換算到光達中心 %.1f cm' % (100 * res['tape'])]
        if res['board']:
            parts.append('光達 %+.1f' % (100 * (res['lidar'] - res['tape'])))
        if 'cam_mid' in res:
            parts.append('相機中間 %+.1f' % (100 * (res['cam_mid'] - res['tape'])))
        if 'cam_board_tape' in res:
            parts.append('相機整塊 %+.1f' % (100 * res['cam_board_tape']))
        if 'cam_outer_tape' in res:
            parts.append('畫面外側 5° %+.1f' % (100 * res['cam_outer_tape']))
        out.append('→ ' + '，'.join(parts) + ' cm（＋ = 比捲尺長）')
    return out


def load_station(folder, label):
    """Saved raw lidar rows and photos of one station."""
    import cv2
    rows = np.load(os.path.join(folder, label + '_lidar.npy'))
    images = [cv2.imread(p) for p in sorted(glob.glob(os.path.join(folder, label + '_*.jpg')))]
    return rows, [im for im in images if im is not None]


def summarize(folder, config=None):
    """Recompute every station of folder from its files; print and write summary.txt."""
    config = config or folder
    cam = FloorCamera.from_files(os.path.join(config, INTRINSICS),
                                 os.path.join(config, EXTRINSICS))
    with open(os.path.join(folder, 'stations.csv'), encoding='utf-8') as f:
        stations = list(csv.DictReader(f))
    lines = ['相機捲尺檢查 %s（校正檔：%s）' % (folder, config),
             '距離都是從光達中心算（cm）；＋ = 比捲尺長', '',
             '%-11s %-9s %6s %8s %9s %9s %11s  %s' % (
                 '站', '板子方向', '捲尺', '光達差', '相機中間差', '相機整塊差', '畫面外側5°差',
                 '相機看到的範圍')]
    for s in stations:
        rows, images = load_station(folder, s['label'])
        th, r = lidar_points(rows)
        tape = (float(s['tape_near_cm']) + float(s['lidar_diameter_cm']) / 2.0) / 100.0
        res = measure(th, r, camera_points(cam, images), float(s['deg']), tape)

        def cm(key, ref=None):
            if key not in res:
                return '-'
            v = res[key] - (ref if ref is not None else 0.0)
            return '%+.1f' % (100 * v)
        lines.append('%-11s %-9s %6.1f %8s %9s %9s %11s  %s' % (
            s['label'], side(res['mid']), 100 * tape,
            cm('lidar', tape) if res['board'] else '-', cm('cam_mid', tape),
            cm('cam_board_tape'), cm('cam_outer_tape'),
            '%s 到 %s（%d 點）' % (side(res['cam_lo']), side(res['cam_hi']), res['cam_board_n'])
            if 'cam_lo' in res else '-'))
    text = '\n'.join(lines)
    print(text)
    with open(os.path.join(folder, 'summary.txt'), 'w', encoding='utf-8') as f:
        f.write(text + '\n')


def running(names):
    """
    Return which of names some other process runs (an argument ending in the name).

    Not pgrep -f: that also matches a shell whose command line merely mentions the name
    (an ssh 'bash -c' string), see docs/real pitfalls.
    """
    found = set()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open('/proc/%s/cmdline' % pid, 'rb') as f:
                args = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        found.update(n for n in names if any(a.endswith(n) for a in args))
    return sorted(found)


def ask_float(prompt):
    """Ask until a number comes back."""
    while True:
        a = input(prompt).strip()
        try:
            return float(a)
        except ValueError:
            print('  請輸入數字，例如 54.5')


def record():
    """Interactive recording on the Orin; see the module docstring."""
    import cv2
    busy = running(BUSY)
    if busy:
        sys.exit('有程式正在用光達或相機：%s\n先停掉（啟動頁按停止，或在它的視窗按 Ctrl+C）再跑。'
                 % ', '.join(busy))
    sys.path.insert(0, HERE)
    from lidar_snap import read_points
    config = next(d for d in CONFIG_DIRS if os.path.exists(os.path.join(d, EXTRINSICS)))
    cam = FloorCamera.from_files(os.path.join(config, INTRINSICS),
                                 os.path.join(config, EXTRINSICS))
    out = os.path.expanduser(time.strftime('~/r550_camtape/%Y%m%d_%H%M%S'))
    os.makedirs(out)
    for name in (INTRINSICS, EXTRINSICS):
        shutil.copy(os.path.join(config, name), out)
    w, h = cam.size
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if not cap.isOpened():
        sys.exit('相機打不開：/dev/video0')
    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if got != (w, h):
        sys.exit('相機給 %dx%d，校正檔是 %dx%d' % (got + (w, h)))
    for _ in range(15):                         # auto exposure settles
        cap.read()
    print('相機捲尺檢查（只讀：相機＋光達，馬達不會轉）。結果存在 %s' % out)
    print('共 %d 站，每站：放板子 → 按 Enter → 錄 5 秒 → 量捲尺。' % len(STATIONS))
    dia = ask_float('先用尺量一次光達圓筒的直徑（cm，例如 7.2）：')
    with open(os.path.join(out, 'stations.csv'), 'w', newline='', encoding='utf-8') as f:
        csv.DictWriter(f, FIELDS).writeheader()
    done = 0
    for i, (label, deg, what, where) in enumerate(STATIONS):
        while True:
            print('\n[%d/%d] %s' % (i + 1, len(STATIONS), what))
            print('  板子中間放在：%s。板子直立，正面對著光達。' % where)
            a = input('  放好、人站到車子後面，再按 Enter（s 跳過這站、q 結束）：').strip().lower()
            if a in ('s', 'q'):
                break
            print('  錄資料中（光達 3 秒、相機 %d 張），不要動……' % FRAMES)
            rows, bad = read_points(3.0)
            np.save(os.path.join(out, label + '_lidar.npy'), rows)
            for _ in range(10):                 # drop the frames buffered meanwhile
                cap.grab()
            images = []
            for k in range(FRAMES):
                ok, img = cap.read()
                if ok:
                    images.append(img)
                    cv2.imwrite(os.path.join(out, '%s_%d.jpg' % (label, k)), img,
                                [cv2.IMWRITE_JPEG_QUALITY, 95])
                time.sleep(0.2)
            th, r = lidar_points(rows)
            pts = camera_points(cam, images)
            print('  （光達 %d 點、壞封包 %d；相機 %d 張、平均亮度 %.0f）' % (
                len(rows), bad, len(images), np.mean([im.mean() for im in images])
                if images else 0.0))
            for line in describe(measure(th, r, pts, deg)):
                print('  ' + line)
            near = ask_float('  捲尺：板子中間到光達圓筒最近的一邊（cm）：')
            res = measure(th, r, pts, deg, (near + dia / 2.0) / 100.0)
            print('  ' + describe(res)[-1])
            if input('  Enter 下一站、r 重錄這一站：').strip().lower() != 'r':
                with open(os.path.join(out, 'stations.csv'), 'a', newline='',
                          encoding='utf-8') as f:
                    csv.DictWriter(f, FIELDS).writerow(
                        {'label': label, 'deg': deg, 'tape_near_cm': near,
                         'lidar_diameter_cm': dia, 'time': time.strftime('%H:%M:%S')})
                done += 1
                break
        if a == 'q':
            break
    cap.release()
    print('\n完成 %d 站。' % done)
    if done:
        summarize(out)
    print('RESULT_DIR=%s' % out)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--check':
        if len(sys.argv) < 3:
            sys.exit(__doc__)
        summarize(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None)
    else:
        record()


if __name__ == '__main__':
    main()
