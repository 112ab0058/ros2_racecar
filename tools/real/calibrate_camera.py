#!/usr/bin/env python3
"""
Compute the camera calibration from cam_calib_capture.py photos (no ROS, no motors).

    python3 calibrate_camera.py intrinsic ~/r550_calib/cam_intrinsic_<time>
    python3 calibrate_camera.py floor ~/r550_calib/cam_floor_<time> \\
        --intrinsics ~/r550_calib/cam_intrinsic_<time>/ost.yaml --front-to-row 0.20

intrinsic  Zhang's method (cv2.calibrateCamera, the same as ROS camera_calibration):
           fx, fy, cx, cy and 5 distortion coefficients. Photos whose reprojection
           error is more than 3x the median are dropped and the rest solved again.
           Writes ost.yaml (ROS camera_info format, as the vendor tutorial produces)
           and undistort_check.jpg (left original, right straightened).
floor      the board lies flat on the floor in front of the car, rows parallel to
           the front bumper; --front-to-row = metres from the front bumper to the
           NEAREST row of inner corners, measured along the floor. solvePnP gives the
           camera height above the floor, its pitch (down = +), roll and yaw, and how
           far it sits behind the bumper. Writes camera_extrinsics.yaml.
Board: 9 x 6 inner corners, 25 mm squares (docs/real/calib/checkerboard_9x6_25mm_A4.pdf).
"""
import argparse
import glob
import math
import os
import sys

import cv2
import numpy as np

FRONT_FROM_BASE_LINK = 0.24     # m, rear axle (base_link) to the front bumper, R550_HARDWARE.md


def board_points(cols, rows, square):
    obj = np.zeros((cols * rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return obj


def find(path, cols, rows):
    img = cv2.imread(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(
        gray, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return img, None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    return img, cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)


def yaml_matrix(name, m):
    rows, cols = m.shape
    vals = ', '.join('%.8g' % v for v in m.ravel())
    return '%s:\n  rows: %d\n  cols: %d\n  data: [%s]\n' % (name, rows, cols, vals)


def load_intrinsics(path):
    fs = {}
    key = None
    for line in open(path):
        if not line.startswith(' ') and line.strip().endswith(':'):
            key = line.strip()[:-1]
        elif 'data:' in line and key:
            fs[key] = np.array([float(v) for v in line.split('[')[1].split(']')[0].split(',')])
    return fs['camera_matrix'].reshape(3, 3), fs['distortion_coefficients']


def intrinsic(a):
    files = sorted(glob.glob(os.path.join(a.dir, '*.jpg')))
    obj = board_points(a.cols, a.rows, a.square)
    objs, imgs, used, size = [], [], [], None
    for f in files:
        img, c = find(f, a.cols, a.rows)
        size = img.shape[1::-1]
        if c is not None:
            objs.append(obj)
            imgs.append(c)
            used.append(f)
    print('照片 %d 張，找到棋盤格 %d 張' % (len(files), len(used)))
    if len(used) < 10:
        print('少於 10 張，不夠算，請再拍一次')
        return 1
    for attempt in range(2):
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objs, imgs, size, None, None)
        errs = []
        for o, i, r, t in zip(objs, imgs, rvecs, tvecs):
            proj, _ = cv2.projectPoints(o, r, t, K, dist)
            errs.append(float(np.sqrt(np.mean(np.sum((proj - i) ** 2, axis=2)))))
        med = float(np.median(errs))
        bad = [k for k, e in enumerate(errs) if e > 3 * med]
        if attempt == 1 or not bad:
            break
        print('丟掉誤差特別大的 %d 張：%s' % (len(bad), ', '.join(os.path.basename(used[k])
                                                          for k in bad)))
        keep = [k for k in range(len(used)) if k not in bad]
        objs, imgs, used = [objs[k] for k in keep], [imgs[k] for k in keep], [used[k] for k in keep]
    w, h = size
    fov_x = 2 * math.degrees(math.atan(w / (2 * K[0, 0])))
    fov_y = 2 * math.degrees(math.atan(h / (2 * K[1, 1])))
    print('用了 %d 張；重投影誤差 RMS %.3f 像素（< 0.5 算好），最差一張 %.3f' % (
        len(used), rms, max(errs)))
    print('fx %.1f fy %.1f cx %.1f cy %.1f；視角 水平 %.1f° 垂直 %.1f°' % (
        K[0, 0], K[1, 1], K[0, 2], K[1, 2], fov_x, fov_y))
    print('畸變 k1 k2 p1 p2 k3 = %s' % ' '.join('%+.4f' % v for v in dist.ravel()))
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, size, 0)
    P = np.hstack([newK, np.zeros((3, 1))])
    with open(os.path.join(a.dir, 'ost.yaml'), 'w') as f:
        f.write('image_width: %d\nimage_height: %d\ncamera_name: c70\n' % (w, h))
        f.write(yaml_matrix('camera_matrix', K))
        f.write('distortion_model: plumb_bob\n')
        f.write(yaml_matrix('distortion_coefficients', dist.reshape(1, -1)))
        f.write(yaml_matrix('rectification_matrix', np.eye(3)))
        f.write(yaml_matrix('projection_matrix', P))
        f.write('# rms_reprojection_error_px: %.4f  photos: %d\n' % (rms, len(used)))
    img = cv2.imread(used[len(used) // 2])
    both = np.hstack([img, cv2.undistort(img, K, dist, None, newK)])
    cv2.imwrite(os.path.join(a.dir, 'undistort_check.jpg'), both)
    print('寫好 %s/ost.yaml 和 undistort_check.jpg' % a.dir)
    return 0


def floor(a):
    K, dist = load_intrinsics(a.intrinsics)
    obj = board_points(a.cols, a.rows, a.square)
    res = []
    for f in sorted(glob.glob(os.path.join(a.dir, '*.jpg'))):
        _, c = find(f, a.cols, a.rows)
        if c is None:
            continue
        ok, rvec, tvec = cv2.solvePnP(obj, c, K, dist)
        R, _ = cv2.Rodrigues(rvec)
        cam = (-R.T @ tvec).ravel()                    # camera position in the board frame
        n = np.array([0.0, 0.0, 1.0])                   # board (floor) normal
        if cam[2] < 0:
            n = -n
        height = abs(cam[2])
        axis = R.T @ np.array([0.0, 0.0, 1.0])          # optical axis in the board frame
        pitch = math.degrees(math.asin(-np.dot(axis, n)))          # + = looking down
        roll = math.degrees(math.asin(np.dot(R.T @ np.array([1.0, 0.0, 0.0]), n)))
        fwd = axis - np.dot(axis, n) * n
        fwd /= np.linalg.norm(fwd)
        foot = cam - np.dot(cam, n) * n
        ahead = (obj - foot) @ fwd                      # forward distance of every corner
        # rows of the board run across the car: the yaw is the angle between the
        # forward direction and the board edge that is closest to perpendicular to it
        yaw = min((math.degrees(math.atan2(np.cross(e, fwd) @ n, e @ fwd)) for e in
                   (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([-1.0, 0, 0]),
                    np.array([0, -1.0, 0]))), key=abs)
        res.append((height, pitch, roll, yaw, float(ahead.min())))
    if not res:
        print('沒有一張找得到棋盤格')
        return 1
    m = np.median(np.array(res), axis=0)
    behind = m[4] - a.front_to_row                      # camera is this far behind the bumper
    x_base = FRONT_FROM_BASE_LINK - behind
    print('用了 %d 張（取中位數）' % len(res))
    print('相機離地高度 %.3f m（請用尺量相機鏡頭中心對照）' % m[0])
    print('往下看 %.1f°，左右歪 %.1f°，偏左右 %.1f°' % (m[1], m[2], m[3]))
    print('相機在車頭後方 %.3f m，也就是後軸中心（base_link）前方 %.3f m' % (behind, x_base))
    with open(os.path.join(a.dir, 'camera_extrinsics.yaml'), 'w') as f:
        f.write('# from calibrate_camera.py floor; base_link = rear axle centre\n')
        f.write('height_m: %.4f\npitch_down_deg: %.2f\nroll_deg: %.2f\nyaw_deg: %.2f\n'
                'x_from_base_link_m: %.4f\nfront_to_row_m: %.4f\nphotos: %d\n'
                % (m[0], m[1], m[2], m[3], x_base, a.front_to_row, len(res)))
    print('寫好 %s/camera_extrinsics.yaml' % a.dir)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('mode', choices=['intrinsic', 'floor'])
    ap.add_argument('dir')
    ap.add_argument('--cols', type=int, default=9)
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--square', type=float, default=0.025, help='m; measure the printed board')
    ap.add_argument('--intrinsics', help='ost.yaml from the intrinsic step (floor mode)')
    ap.add_argument('--front-to-row', type=float, help='m, bumper to nearest corner row (floor)')
    a = ap.parse_args()
    a.dir = os.path.expanduser(a.dir)
    if a.mode == 'floor' and (not a.intrinsics or a.front_to_row is None):
        ap.error('floor needs --intrinsics and --front-to-row')
    if a.intrinsics:
        a.intrinsics = os.path.expanduser(a.intrinsics)
    return intrinsic(a) if a.mode == 'intrinsic' else floor(a)


if __name__ == '__main__':
    sys.exit(main())
