#!/usr/bin/env python3
"""
Orin: raw N10P points for a few seconds, straight from the serial port (READ ONLY, no ROS).

    python3 ~/racecar_ws/tools/lidar_snap.py LABEL [SECONDS]     # default 3 s = ~30 turns

Nothing else may read the lidar port at the same time (no sensors.launch / scan_node):
two readers steal bytes from each other. Nothing is sent to the lidar.
Saves ~/r550_lidartest/LABEL.npy, one row per point: (ROS deg: 0 front, +90 left,
-90 right; distance mm, 0 = no return; intensity), and prints per 10 deg bin the share
of points with a return, their median distance and median intensity. No filtering or
self-mask: this is what the lidar itself reports (protocol: wro2026_lidar_n10p/protocol.py).
Used on 2026-11-13 to measure how close the field's black walls stop returning (LOGBOOK).
"""
import os
import sys
import time

import numpy as np
import serial

PORT = '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0001-if00'


def read_points(secs):
    ser = serial.Serial(PORT, 460800, timeout=0.05)
    buf, rows, bad = bytearray(), [], 0
    t_end = time.time() + secs
    while time.time() < t_end:
        buf += ser.read(ser.in_waiting or 1)
        while True:
            i = buf.find(b'\xa5\x5a')
            if i < 0:
                del buf[:-1]
                break
            if len(buf) - i < 108:
                del buf[:i]
                break
            p = bytes(buf[i:i + 108])
            if (sum(p[:107]) & 0xFF) != p[107]:
                bad += 1
                del buf[:i + 2]
                continue
            del buf[:i + 108]
            start = int.from_bytes(p[5:7], 'big') / 100.0
            end = int.from_bytes(p[105:107], 'big') / 100.0
            span = end - start if end >= start else end + 360.0 - start
            if span > 30:
                continue
            for n in range(16):
                j = 7 + n * 6
                d = int.from_bytes(p[j:j + 2], 'big')
                ros = (360.0 - (start + span / 15 * n)) % 360.0     # lidar is clockwise
                rows.append((ros - 360.0 if ros >= 180 else ros, 0 if d == 0xFFFF else d, p[j + 2]))
    ser.close()
    return np.array(rows, float), bad


def main():
    label = sys.argv[1]
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    a, bad = read_points(secs)
    out = os.path.expanduser('~/r550_lidartest')
    os.makedirs(out, exist_ok=True)
    np.save(os.path.join(out, label + '.npy'), a)
    print('%s: %d points in %.0f s, bad packets %d  (ROS deg: 0 front, +90 left, -90 right)'
          % (label, len(a), secs, bad))
    print(' bin(deg)  return%  dist_cm  intensity')
    for lo in range(-180, 180, 10):
        sel = (a[:, 0] >= lo) & (a[:, 0] < lo + 10)
        v = sel & (a[:, 1] > 0)
        if sel.sum() == 0:
            continue
        print('%5d..%-4d %5.0f%%  %7s  %5s' % (
            lo, lo + 10, 100.0 * v.sum() / sel.sum(),
            '%.1f' % (np.median(a[v, 1]) / 10) if v.any() else '-',
            '%.0f' % np.median(a[v, 2]) if v.any() else '-'))


if __name__ == '__main__':
    main()
