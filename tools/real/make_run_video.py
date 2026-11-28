#!/usr/bin/env python3
"""
Orin: a video of one recorded run: camera, top view of what the car knows, and its decisions.

    source /opt/ros/jazzy/setup.bash; source ~/racecar_ws/install/setup.bash
    python3 ~/racecar_ws/tools/make_run_video.py ~/r550_map/<run>_auto OUT.mp4 ["TITLE"]

READ ONLY (bag in, video out; no motors, nothing published). One frame per /scan_filled
(10 per second, so the video plays in real time), from 3 s before the car first moves to
4 s after it last moves. 1280 x 720, H.264.
  top        title, time since the start of the video, bc_node's state line (/bc/state)
  left       the camera frame (/camera/image/compressed, ~4 per second, held until the next)
             and below it bc_node's decision (/bc/decision), the speed (EKF) and how many
             points of this scan are measured / lidar memory / camera
  right      top view around the car (car frame, heading up, 10 cm grid): the points
             bc_node got (/scan_filled: intensity 0 = measured now, 1 = lidar memory,
             2 = camera), the car outline (base_link: rear axle; FRONT / REAR / HALF_WIDTH
             as turn_predict.py), the last 4 s of the path (/odometry/filtered), and the
             ring 0.2-0.47 m from the lidar where it sees the black walls only sometimes
For the professor (FIELD_REVIEW C5): shows that the walls up close come from memory and
the camera, not from a shortcut such as reflectors on the walls.
"""
import math
import sys

import cv2
import numpy as np
from nav_msgs.msg import Odometry
from PIL import Image, ImageDraw, ImageFont
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist

FRONT, REAR, HALF_WIDTH, LASER_X = 0.24, 0.031, 0.09, 0.13   # turn_predict.py
DECISIONS = {-1: '停', 0: '左', 1: '直', 2: '右', 3: '左滿舵', 4: '右滿舵'}
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
FONT_BOLD = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
W, H, TOP = 1280, 720, 70
CAM_W, CAM_H = 640, 360
VIEW_X0, VIEW_W, VIEW_H = 640, 640, H - TOP
PX_PER_M = 430.0
ORIGIN = (VIEW_X0 + VIEW_W / 2, TOP + 1.05 * PX_PER_M)    # base_link origin in the image
COLORS = {0: (31, 99, 214), 1: (224, 134, 0), 2: (42, 157, 75)}
NAMES = {0: '光達這一圈量到', 1: '光達記憶', 2: '相機'}
BG, INK, DIM, GRID = (250, 250, 247), (27, 28, 26), (104, 107, 100), (226, 227, 222)


def font(size, bold=False):
    """Noto Sans CJK, the Traditional Chinese face of the collection."""
    path = FONT_BOLD if bold else FONT
    for i in range(5):
        f = ImageFont.truetype(path, size, index=i)
        if 'TC' in f.getname()[0]:
            return f
    return ImageFont.truetype(path, size)


def to_px(x, y):
    """Car frame (m, x forward, y left) -> image pixel (heading up)."""
    return ORIGIN[0] - y * PX_PER_M, ORIGIN[1] - x * PX_PER_M


def read_bag(run):
    r = SequentialReader()
    r.open(StorageOptions(uri=run + '/bag', storage_id=''), ConverterOptions('', ''))
    r.set_filter(StorageFilter(topics=['/scan_filled', '/camera/image/compressed',
                                       '/odometry/filtered', '/bc/state', '/bc/decision',
                                       '/cmd_vel_out']))
    out = {k: [] for k in ('scan', 'cam', 'odom', 'state', 'dec', 'cmd')}
    while r.has_next():
        topic, raw, t = r.read_next()
        t *= 1e-9
        if topic == '/scan_filled':
            m = deserialize_message(raw, LaserScan)
            rr = np.asarray(m.ranges, float)
            kind = (np.asarray(m.intensities, float) if len(m.intensities) == len(rr)
                    else np.zeros(len(rr)))
            out['scan'].append((t, rr, kind, m.angle_min, m.angle_increment))
        elif topic == '/camera/image/compressed':
            out['cam'].append((t, bytes(deserialize_message(raw, CompressedImage).data)))
        elif topic == '/odometry/filtered':
            m = deserialize_message(raw, Odometry)
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            out['odom'].append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                                m.twist.twist.linear.x))
        elif topic == '/bc/state':
            out['state'].append((t, deserialize_message(raw, String).data))
        elif topic == '/bc/decision':
            out['dec'].append((t, deserialize_message(raw, Int8).data))
        else:
            out['cmd'].append((t, deserialize_message(raw, Twist).linear.x))
    return out


def latest(seq, t, k=0):
    """Index of the last item at or before t in a time-sorted list (or -1)."""
    lo, hi = 0, len(seq)
    while lo < hi:
        mid = (lo + hi) // 2
        if seq[mid][k] <= t:
            lo = mid + 1
        else:
            hi = mid
    return lo - 1


def static_view():
    """The parts of the top view that never change: grid, blind ring, legend."""
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img, 'RGBA')
    x_lo, x_hi = (TOP + VIEW_H - ORIGIN[1]) / -PX_PER_M, (ORIGIN[1] - TOP) / PX_PER_M
    y_half = VIEW_W / 2 / PX_PER_M
    for k in range(int(x_lo * 10) - 1, int(x_hi * 10) + 2):
        _, py = to_px(k / 10.0, 0)
        if TOP <= py <= H:
            d.line([(VIEW_X0, py), (W, py)], fill=GRID if k % 5 else (205, 207, 200), width=1)
    for k in range(-int(y_half * 10) - 1, int(y_half * 10) + 2):
        px, _ = to_px(0, k / 10.0)
        if VIEW_X0 <= px <= W:
            d.line([(px, TOP), (px, H)], fill=GRID if k % 5 else (205, 207, 200), width=1)
    # where the lidar sees the black walls only sometimes (FIELD_REVIEW section 5)
    cx, cy = to_px(LASER_X, 0.0)
    ring = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    rd = ImageDraw.Draw(ring)
    for rad, fill in ((0.47, (214, 40, 40, 38)), (0.20, (0, 0, 0, 0))):
        rp = rad * PX_PER_M
        rd.ellipse([cx - rp, cy - rp, cx + rp, cy + rp], fill=fill)
    img = Image.alpha_composite(img.convert('RGBA'), ring).convert('RGB')
    d = ImageDraw.Draw(img, 'RGBA')
    d.text((VIEW_X0 + 12, H - 34), '淡紅色：離光達 0.2–0.47 m，黑牆在這裡時有時無', font=font(17),
           fill=(170, 40, 40))
    d.text((VIEW_X0 + 12, TOP + 8), '俯視圖：車頭朝上，1 格 = 10 cm', font=font(17), fill=DIM)
    return img


def frame(base, cam, scan, poses, state, decision, speed, t_video, title):
    img = base.copy()
    d = ImageDraw.Draw(img, 'RGBA')
    # top bar
    d.rectangle([0, 0, W, TOP], fill=(255, 255, 255))
    d.line([(0, TOP), (W, TOP)], fill=GRID, width=1)
    d.text((16, 6), title, font=font(24, True), fill=INK)
    d.text((W - 150, 6), 't = %5.1f s' % t_video, font=font(24, True), fill=INK)
    s = state if len(state) <= 66 else state[:65] + '…'
    d.text((16, 40), s, font=font(18), fill=DIM)
    # camera
    if cam is not None:
        img.paste(cam, (0, TOP))
    else:
        d.rectangle([0, TOP, CAM_W, TOP + CAM_H], fill=(60, 60, 60))
        d.text((20, TOP + 20), '（相機還沒有畫面）', font=font(20), fill=(230, 230, 230))
    # top view: path, points, car
    if len(poses) > 1:
        x0, y0, yaw0 = poses[-1]
        c, s_ = math.cos(yaw0), math.sin(yaw0)
        pts = [to_px(c * (x - x0) + s_ * (y - y0), -s_ * (x - x0) + c * (y - y0))
               for x, y, _ in poses]
        d.line(pts, fill=(150, 150, 150), width=2)
    rr, kind, amin, inc = scan
    a = amin + inc * np.arange(len(rr))
    ok = np.isfinite(rr) & (rr > 0)
    counts = {}
    for k in (1, 2, 0):                       # measured points on top
        sel = ok & (kind == k)
        counts[k] = int(sel.sum())
        for x, y in zip(rr[sel] * np.cos(a[sel]) + LASER_X, rr[sel] * np.sin(a[sel])):
            px, py = to_px(x, y)
            if px >= VIEW_X0:
                d.ellipse([px - 2.5, py - 2.5, px + 2.5, py + 2.5], fill=COLORS[k])
    corners = [to_px(FRONT, HALF_WIDTH), to_px(FRONT, -HALF_WIDTH), to_px(-REAR, -HALF_WIDTH),
               to_px(-REAR, HALF_WIDTH)]
    d.polygon(corners, fill=(80, 80, 80, 200), outline=(30, 30, 30))
    lx, ly = to_px(LASER_X, 0.0)
    d.ellipse([lx - 5, ly - 5, lx + 5, ly + 5], fill=(255, 255, 255), outline=(0, 0, 0))
    # info under the camera
    y = TOP + CAM_H + 16
    d.text((20, y), '程式的決定', font=font(18), fill=DIM)
    d.text((20, y + 24), DECISIONS.get(decision, '-'), font=font(44, True),
           fill=(198, 40, 40) if decision == -1 else INK)
    d.text((260, y), '速度（EKF）', font=font(18), fill=DIM)
    d.text((260, y + 24), '%.2f m/s' % (0.0 if abs(speed) < 0.005 else speed), font=font(44, True),
           fill=INK)
    y += 100
    d.text((20, y), '這一圈畫在俯視圖上的點', font=font(18), fill=DIM)
    for i, k in enumerate((0, 1, 2)):
        yy = y + 30 + 34 * i
        d.ellipse([24, yy + 6, 40, yy + 22], fill=COLORS[k])
        d.text((52, yy), '%s　%d 點' % (NAMES[k], counts.get(k, 0)), font=font(22), fill=INK)
    d.text((340, y + 30), '灰色線：車子剛走過的路', font=font(18), fill=DIM)
    d.text((340, y + 64), '深灰框：車身（後輪軸在原點）', font=font(18), fill=DIM)
    d.text((340, y + 98), '白點：光達', font=font(18), fill=DIM)
    return img


def main():
    run, out_path = sys.argv[1].rstrip('/'), sys.argv[2]
    title = sys.argv[3] if len(sys.argv) > 3 else 'R550 自動駕駛：' + run.split('/')[-1]
    data = read_bag(run)
    moving = [t for t, vx in data['cmd'] if abs(vx) > 1e-3]
    if not moving or not data['scan']:
        sys.exit('no driving in this bag')
    t0, t1 = moving[0] - 3.0, moving[-1] + 4.0
    base = static_view()
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'avc1'), 10, (W, H))
    if not writer.isOpened():
        sys.exit('cannot open the video writer')
    cam_i, cam_img, n = -1, None, 0
    odom = data['odom']
    for t, rr, kind, amin, inc in data['scan']:
        if not t0 <= t <= t1:
            continue
        i = latest(data['cam'], t)
        if i >= 0 and i != cam_i:
            cam_i = i
            bgr = cv2.imdecode(np.frombuffer(data['cam'][i][1], np.uint8), cv2.IMREAD_COLOR)
            if bgr is not None:
                cam_img = Image.fromarray(cv2.cvtColor(cv2.resize(bgr, (CAM_W, CAM_H)),
                                                       cv2.COLOR_BGR2RGB))
        j = latest(odom, t)
        poses = [(x, y, yaw) for tt, x, y, yaw, _ in odom[max(0, j - 120):j + 1] if tt >= t - 4.0]
        speed = odom[j][4] if j >= 0 else 0.0
        k = latest(data['state'], t)
        state = data['state'][k][1] if k >= 0 else ''
        k = latest(data['dec'], t)
        dec = data['dec'][k][1] if k >= 0 else None
        img = frame(base, cam_img, (rr, kind, amin, inc), poses, state, dec, speed, t - t0,
                    title)
        writer.write(cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR))
        n += 1
    writer.release()
    print('%s: %d frames, %.1f s' % (out_path, n, n / 10.0))


if __name__ == '__main__':
    main()
