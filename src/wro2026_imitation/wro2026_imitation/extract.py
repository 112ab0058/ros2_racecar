"""
Make training samples from recorded runs. Reads files only, no motors.

    ros2 run wro2026_imitation bc_extract ~/r550_map/<time> [more run folders]

Writes <run folder>/dataset.npz: ranges (one row per scan), label, t, source (who drove:
0 Ray with the phone keys / joystick, 1 the corner rule while 自動 was held), and the
speed and turn radii the driver used (bc_node drives with the same).

Scans: /scan_filled when the bag has it (orin_auto.sh runs: the lidar with its black-wall
blind spot filled from memory and the camera, what bc_node drives from), else /scan.
Labels: the executed command (/cmd_vel_out, after base_node's limits), 5 kinds
(features.LABELS). Who drove: the phone page's log (web.log, 'phone: 自動' or a key);
without web.log every sample counts as Ray's. Samples in the last drop_before_stop
seconds before a too-close stop (/bc/state 鎖住) are dropped: that driving ended at a wall.
Samples where the lidar's view did not change for a second although the car was driving
are dropped too (dataset.scene_moved): the wheels were in the air (stand tests).
"""
import argparse
import collections
import os
import re
import sys

import numpy as np

from .dataset import before_events, command_stats, label_scans, NO_LABEL, scene_moved
from .features import LABELS, scan_features

HAND, RULE = 0, 1
SOURCE_NAMES = ('hand', 'rule')
DROP_BEFORE_STOP = 1.5      # s
# ROS log line of web_teleop_node: [INFO] [1795471730.123456789] [web_teleop_node]: phone: 自動 (...)
PHONE = re.compile(r'\[(\d+\.\d+)\] \[web_teleop_node\]: phone: (\S+)')


def read_bag(uri, scan_topic='auto'):
    """
    Return a dict from a rosbag folder.

    scan_topic, scan_t, ranges, cmd_t, vx, wz (the executed commands), stops (times
    the too-close latch began).
    """
    from geometry_msgs.msg import Twist    # imported here so the tests need no ROS
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    topics = {t.name for t in reader.get_all_topics_and_types()}
    if scan_topic == 'auto':
        scan_topic = '/scan_filled' if '/scan_filled' in topics else '/scan'
    reader.set_filter(rosbag2_py.StorageFilter(topics=[scan_topic, '/cmd_vel_out', '/bc/state']))
    scan_t, ranges, cmd, stops, latched = [], [], [], [], False
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp / 1e9
        if topic == scan_topic:
            scan_t.append(t)
            ranges.append(np.array(deserialize_message(data, LaserScan).ranges, np.float32))
        elif topic == '/cmd_vel_out':
            msg = deserialize_message(data, Twist)
            cmd.append((t, msg.linear.x, msg.angular.z))
        else:
            now = '鎖住' in deserialize_message(data, String).data
            if now and not latched:
                stops.append(t)
            latched = now
    c = np.array(cmd) if cmd else np.zeros((0, 3))
    return {'scan_topic': scan_topic, 'scan_t': np.array(scan_t), 'ranges': ranges,
            'cmd_t': c[:, 0], 'vx': c[:, 1], 'wz': c[:, 2], 'stops': stops}


def phone_actions(folder):
    """Return (times, action names) of the phone page's log in a run folder, or None."""
    path = os.path.join(folder, 'web.log')
    if not os.path.exists(path):
        return None
    times, names = [], []
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            m = PHONE.search(line)
            if m:
                times.append(float(m.group(1)))
                names.append(m.group(2))
    return np.array(times), names


def sources(scan_t, actions):
    """Return who drove at each scan time: RULE while 自動 was the phone's action, else HAND."""
    src = np.full(len(scan_t), HAND, dtype=np.int8)
    if actions is None or not len(actions[0]):
        return src
    times, names = actions
    last = np.searchsorted(times, scan_t, side='right') - 1
    for s, k in enumerate(last):
        if k >= 0 and names[k] == '自動':
            src[s] = RULE
    return src


def extract(folder, scan_topic='auto', drop_before_stop=DROP_BEFORE_STOP):
    """Write dataset.npz for one run folder and return the number of usable samples."""
    bag = read_bag(os.path.join(folder, 'bag'), scan_topic)
    scan_t, ranges = bag['scan_t'], bag['ranges']
    if not ranges or len({len(r) for r in ranges}) != 1:
        print('%s: no scans, or scans of different sizes; skipped' % folder)
        return 0
    labels = label_scans(scan_t, bag['cmd_t'], bag['vx'], bag['wz'])
    used = labels != NO_LABEL
    dropped = used & before_events(scan_t, bag['stops'], drop_before_stop)
    still = used & ~dropped & ~scene_moved(scan_t, scan_features(np.stack(ranges)))
    keep = used & ~dropped & ~still
    src = sources(scan_t, phone_actions(folder))
    counts = collections.Counter(int(a) for a in labels[keep])
    who = collections.Counter(int(a) for a in src[keep])
    kinds = ', '.join('%s %d' % (name, counts[i]) for i, name in enumerate(LABELS))
    drivers = ', '.join('%s %d' % (name, who[i]) for i, name in enumerate(SOURCE_NAMES))
    print('%s: %s, %d scans, %d usable (dropped: %d before %d too-close stops, %d with the '
          'view standing still): %s; %s'
          % (folder, bag['scan_topic'], len(scan_t), keep.sum(), dropped.sum(),
             len(bag['stops']), still.sum(), kinds, drivers))
    if still.sum() > 0.8 * max(used.sum(), 1):
        print('%s: the view stood still while driving: wheels in the air? not used' % folder)
    if not keep.any():
        return 0
    speed, radius, full = command_stats(bag['vx'], bag['wz'])
    print('%s: speed %.3f m/s, turn radius %.2f m, full lock %.2f m'
          % (folder, speed, radius, full))
    np.savez(os.path.join(folder, 'dataset.npz'), ranges=np.stack(ranges)[keep],
             label=labels[keep], t=scan_t[keep], source=src[keep], speed=speed,
             turn_radius=radius, full_turn_radius=full, scan_topic=bag['scan_topic'])
    return int(keep.sum())


def main(argv=None):
    """Entry point."""
    ap = argparse.ArgumentParser(description='Make training samples from recorded runs.')
    ap.add_argument('folders', nargs='+', help='run folders containing bag/')
    ap.add_argument('--scan-topic', default='auto',
                    help="'auto' = /scan_filled if recorded, else /scan; or a topic name")
    ap.add_argument('--drop-before-stop', type=float, default=DROP_BEFORE_STOP,
                    help='drop the samples this many seconds before a too-close stop (s)')
    args = ap.parse_args(argv)
    total = 0
    for f in args.folders:
        if not os.path.isdir(os.path.join(f, 'bag')):
            print('%s: no bag/ folder; skipped' % f)
            continue
        try:
            total += extract(f, args.scan_topic, args.drop_before_stop)
        except Exception as e:      # a bag cut short (power off) must not stop the others
            print('%s: could not read the bag (%s); skipped' % (f, e))
    print('total usable samples: %d' % total)
    return 0 if total else 1


if __name__ == '__main__':
    sys.exit(main())
