"""
Make training samples from orin_map.sh recordings. Reads files only, no motors.

    ros2 run wro2026_imitation bc_extract ~/r550_map/<time> [more run folders]

Writes <run folder>/dataset.npz: ranges (one row per scan), label, t, and the
speed and turn radius the driver used (bc_node drives with the same).
"""
import argparse
import collections
import os
import sys

import numpy as np

from .dataset import command_stats, label_scans, NO_LABEL
from .features import LABELS


def read_bag(uri):
    """Return (scan times, scan ranges, command times, vx, wz) from a rosbag folder."""
    from geometry_msgs.msg import Twist    # imported here so the tests need no ROS
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from sensor_msgs.msg import LaserScan

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan', '/cmd_vel_out']))
    scan_t, ranges, cmd = [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp / 1e9
        if topic == '/scan':
            scan_t.append(t)
            ranges.append(np.array(deserialize_message(data, LaserScan).ranges, np.float32))
        else:
            msg = deserialize_message(data, Twist)
            cmd.append((t, msg.linear.x, msg.angular.z))
    c = np.array(cmd) if cmd else np.zeros((0, 3))
    return np.array(scan_t), ranges, c[:, 0], c[:, 1], c[:, 2]


def extract(folder):
    """Write dataset.npz for one run folder and return the number of usable samples."""
    scan_t, ranges, cmd_t, vx, wz = read_bag(os.path.join(folder, 'bag'))
    if not ranges or len({len(r) for r in ranges}) != 1:
        print('%s: no scans, or scans of different sizes; skipped' % folder)
        return 0
    labels = label_scans(scan_t, cmd_t, vx, wz)
    keep = labels != NO_LABEL
    counts = collections.Counter(int(a) for a in labels[keep])
    print('%s: %d scans, %d usable: %s' % (folder, len(scan_t), keep.sum(), ', '.join(
        '%s %d' % (name, counts[i]) for i, name in enumerate(LABELS))))
    if not keep.any():
        return 0
    speed, radius = command_stats(vx, wz)
    print('%s: speed %.3f m/s, turn radius %.2f m' % (folder, speed, radius))
    np.savez(os.path.join(folder, 'dataset.npz'), ranges=np.stack(ranges)[keep],
             label=labels[keep], t=scan_t[keep], speed=speed, turn_radius=radius)
    return int(keep.sum())


def main(argv=None):
    """Entry point."""
    ap = argparse.ArgumentParser(description='Make training samples from recorded runs.')
    ap.add_argument('folders', nargs='+', help='run folders containing bag/')
    args = ap.parse_args(argv)
    total = sum(extract(f) for f in args.folders)
    print('total usable samples: %d' % total)
    return 0 if total else 1


if __name__ == '__main__':
    sys.exit(main())
