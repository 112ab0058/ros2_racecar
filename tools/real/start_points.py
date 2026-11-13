#!/usr/bin/env python3
"""
Orin: wall points around the car at one moment of an orin_auto.sh bag (lidar frame).

    source /opt/ros/jazzy/setup.bash
    python3 ~/racecar_ws/tools/start_points.py BAG T_PRESS

Uses the first /scan_filled at or after T_PRESS (unix time, e.g. when 自動 was pressed).
For each side, 5 cm x-bins from -0.8 to +0.8 m with points 0.12-0.45 m to that side:
'm' = at least one measured point, 'M' = memory only (scan_memory intensity 1.0),
'.' = nothing. The two '|' mark CornerPolicy's fit_window (-0.25..+0.30 m). Then the
measured points themselves (x, y, range). READ ONLY. Used for the 2026-11-13 field report.
"""
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import LaserScan


def main():
    bag, t_press = sys.argv[1], float(sys.argv[2])
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag), ConverterOptions('', ''))
    msg = None
    while reader.has_next():
        topic, raw, t = reader.read_next()
        if topic == '/scan_filled' and t * 1e-9 >= t_press:
            msg = deserialize_message(raw, LaserScan)
            break
    if msg is None:
        sys.exit('no /scan_filled after T_PRESS')
    rr = np.asarray(msg.ranges, float)
    mem = np.asarray(msg.intensities, float) == 1.0
    a = msg.angle_min + msg.angle_increment * np.arange(len(rr))
    ok = np.isfinite(rr)
    x, y = np.where(ok, rr, 0) * np.cos(a), np.where(ok, rr, 0) * np.sin(a)
    edges = np.arange(-0.8, 0.8001, 0.05)
    print('x bins (5 cm):', ' '.join('|' if abs(e + 0.25) < 1e-6 or abs(e - 0.30) < 1e-6
                                     else ' ' for e in edges[:-1]))
    for name, s in (('left ', 1.0), ('right', -1.0)):
        row = ''
        for lo in edges[:-1]:
            sel = ok & (x >= lo) & (x < lo + 0.05) & (s * y > 0.12) & (s * y < 0.45)
            row += 'm' if (sel & ~mem).any() else ('M' if sel.any() else '.')
        print('%s %s' % (name, ' '.join(row)))
    for name, s in (('left ', 1.0), ('right', -1.0)):
        sel = ok & ~mem & (s * y > 0.12) & (s * y < 0.45) & (np.abs(x) < 0.8)
        print('%s measured: %s' % (name, ' '.join('(%.2f,%.2f r%.2f)' % (x[i], y[i], rr[i])
                                                 for i in np.nonzero(sel)[0][:14])))


if __name__ == '__main__':
    main()
