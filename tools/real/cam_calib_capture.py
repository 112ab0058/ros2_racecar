#!/usr/bin/env python3
"""
Orin: take checkerboard photos for camera calibration (READ ONLY: camera only, no motors).

    python3 ~/racecar_ws/tools/cam_calib_capture.py intrinsic   # hold the board, ~2 min
    python3 ~/racecar_ws/tools/cam_calib_capture.py floor       # board flat on the floor

Board: docs/real/calib/checkerboard_9x6_25mm_A4.pdf (9 x 6 inner corners, 25 mm squares).
intrinsic: keeps a photo only when the whole board is found, still, AND it differs from every
           photo kept so far (position, size or tilt), until --count photos (25).
floor:     saves 3 photos, 1 s apart, without looking for the board (OpenCV 4.6 on the
           Orin cannot find a board lying flat); calibrate_camera.py floor on the PC.
Photos are saved unchanged (no drawing on them) to ~/r550_calib/cam_<mode>_<time>/,
then calibrate_camera.py computes the result. Progress lines go to stdout.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np


def board_pose(corners, w, h):
    """Rough description of where the board is: centre x, y, size, two tilts (all 0..1-ish)."""
    pts = corners.reshape(-1, 2)
    cx, cy = pts[:, 0].mean() / w, pts[:, 1].mean() / h
    hull = cv2.convexHull(pts.astype(np.float32))
    size = cv2.contourArea(hull) / (w * h)
    xs = np.linalg.norm(pts[8] - pts[0]), np.linalg.norm(pts[-1] - pts[-9])     # top, bottom rows
    ys = np.linalg.norm(pts[-9] - pts[0]), np.linalg.norm(pts[-1] - pts[8])     # left, right columns
    tilt_x = (xs[0] - xs[1]) / max(xs)
    tilt_y = (ys[0] - ys[1]) / max(ys)
    return np.array([cx, cy, size * 3.0, tilt_x * 2.0, tilt_y * 2.0])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('mode', choices=['intrinsic', 'floor'])
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--cols', type=int, default=9, help='inner corners per row')
    ap.add_argument('--rows', type=int, default=6, help='inner corners per column')
    ap.add_argument('--count', type=int, default=0, help='photos to keep (intrinsic 25, floor 3)')
    ap.add_argument('--seconds', type=float, default=240.0, help='give up after this long')
    ap.add_argument('--min-change', type=float, default=0.12, help='how different a new photo must be')
    a = ap.parse_args()
    count = a.count or (25 if a.mode == 'intrinsic' else 3)
    out = os.path.expanduser('~/r550_calib/cam_%s_%s' % (a.mode, time.strftime('%Y%m%d_%H%M%S')))
    os.makedirs(out, exist_ok=True)

    # OpenCV 4.6 on the Orin: opening by path gave no frames, by index works (2026-11-12);
    # and after re-plugging, only MJPG 1280x720 delivered frames.
    dev = a.device
    if dev.startswith('/dev/video') and dev[10:].isdigit():
        dev = int(dev[10:])
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    ok, frame = False, None
    for _ in range(30):
        ok, frame = cap.read()
        if ok:
            break
    if not ok:
        print('相機打不開：%s' % a.device, flush=True)
        return 1
    h, w = frame.shape[:2]
    if (w, h) != (a.width, a.height):
        print('相機給的是 %dx%d，不是 %dx%d（USB 速度不夠？看 lsusb -t）' % (w, h, a.width, a.height),
              flush=True)
        return 1
    print('RESULT_DIR=%s' % out, flush=True)
    print('開始：%s 模式，%dx%d，要收 %d 張，最多 %.0f 秒' % (a.mode, w, h, count, a.seconds), flush=True)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    kept, poses = 0, []
    start = last_save = last_note = time.monotonic()
    seen = 0
    previous = None                  # pose in the previous frame: the board must be still
    while kept < count and time.monotonic() - start < a.seconds:
        ok, frame = cap.read()
        if not ok:
            continue
        now = time.monotonic()
        if a.mode == 'floor':
            # The board lies flat and looks very thin; OpenCV 4.6 on the Orin does not
            # find it there (4.13 on the PC does), so save without looking. Calculate on
            # the PC: calibrate_camera.py floor.
            if now - last_save >= 1.0:
                kept += 1
                last_save = now
                cv2.imwrite(os.path.join(out, '%02d.jpg' % kept), frame,
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
                print('已收 %d/%d 張' % (kept, count), flush=True)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (a.cols, a.rows), flags)
        seen += found
        if not found:
            previous = None
        if found:
            pose = board_pose(corners, w, h)
            # moving boards gave blurred photos (2-4 px error, 2026-11-12): only keep a
            # photo when the board hardly moved since the previous frame
            still = previous is not None and np.abs(pose - previous).max() < 0.01
            previous = pose
            change = min((np.abs(pose - p).max() for p in poses), default=1.0)
            take = still and change >= a.min_change and now - last_save >= 0.6
            if take:
                kept += 1
                last_save = now
                poses.append(pose)
                cv2.imwrite(os.path.join(out, '%02d.jpg' % kept), frame,
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
                print('已收 %2d/%d 張（中心 %.2f,%.2f 大小 %.2f 傾斜 %+.2f,%+.2f）' % (
                    kept, count, *pose), flush=True)
        if now - last_note >= 10.0:
            print('…已過 %.0f 秒，這 10 秒找到棋盤格 %d 次' % (now - start, seen), flush=True)
            last_note, seen = now, 0
    cap.release()
    print('完成：收了 %d 張，存在 %s' % (kept, out), flush=True)
    return 0 if kept >= min(count, 10 if a.mode == 'intrinsic' else 1) else 2


if __name__ == '__main__':
    sys.exit(main())
