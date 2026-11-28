"""
Log a long still recording (soak test) for later analysis. Read only.

    ros2 run wro2026_base_r550 soak_log_node --ros-args -p dir:=/path/to/dir

Writes (t = message stamp, s):
  imu_raw.csv   t, gx, gy, gz        /imu/data_raw (gyro as the STM32 sends it)
  imu.csv       t, gz                /imu/data (after the ZUPT bias)
  odom.csv      t, x, y, yaw, vx, wz /odom
  ekf.csv       t, x, y, yaw         /odometry/filtered
  battery.csv   t, v                 /battery, once per second
  bias.csv      t, gyro_bias_z, zupt_learning   from /diagnostics
  scan_minute.csv  t, scans, valid_fraction       one row per minute
  scan_bins.csv    bin, angle, n, mean, std, valid_fraction   written at the end
                   (running mean / variance per beam: Welford, no big files)
Analysis: tools/real/analyze_soak.py. Publishes nothing.
"""
import math
import os
import sys

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, LaserScan


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


class Welford:
    """Running mean and variance of one beam (finite readings only)."""

    def __init__(self):
        self.n = self.total = 0
        self.mean = self.m2 = 0.0

    def add(self, x):
        self.total += 1
        if not math.isfinite(x):
            return
        self.n += 1
        d = x - self.mean
        self.mean += d / self.n
        self.m2 += d * (x - self.mean)

    def std(self):
        return math.sqrt(self.m2 / self.n) if self.n > 1 else float('nan')


class SoakLog(Node):
    def __init__(self):
        super().__init__('r550_soak_log')
        self.dir = self.declare_parameter('dir', '.').value
        os.makedirs(self.dir, exist_ok=True)
        self.files = {}
        heads = {'imu_raw': 't,gx,gy,gz', 'imu': 't,gz', 'odom': 't,x,y,yaw,vx,wz',
                 'ekf': 't,x,y,yaw', 'battery': 't,v', 'bias': 't,gyro_bias_z,zupt_learning',
                 'scan_minute': 't,scans,valid_fraction'}
        for name, head in heads.items():
            f = open(os.path.join(self.dir, name + '.csv'), 'w', buffering=1)
            f.write(head + '\n')
            self.files[name] = f
        self.bins = None
        self.angle_min = self.angle_inc = 0.0
        self.minute = None
        self.minute_scans = self.minute_valid = self.minute_total = 0
        self.last_battery = -1.0
        self.create_subscription(Imu, 'imu/data_raw', self._imu_raw, 50)
        self.create_subscription(Imu, 'imu/data', self._imu, 50)
        self.create_subscription(Odometry, 'odom', self._odom, 50)
        self.create_subscription(Odometry, 'odometry/filtered', self._ekf, 50)
        self.create_subscription(BatteryState, 'battery', self._battery, 10)
        self.create_subscription(DiagnosticArray, 'diagnostics', self._diag, 10)
        self.create_subscription(LaserScan, 'scan', self._scan, 10)
        self.get_logger().info('soak log -> %s' % self.dir)

    def _w(self, name, fmt, *values):
        self.files[name].write(fmt % values + '\n')

    def _imu_raw(self, m):
        g = m.angular_velocity
        self._w('imu_raw', '%.4f,%.6f,%.6f,%.6f', stamp(m), g.x, g.y, g.z)

    def _imu(self, m):
        self._w('imu', '%.4f,%.6f', stamp(m), m.angular_velocity.z)

    def _odom(self, m):
        p, v = m.pose.pose.position, m.twist.twist
        self._w('odom', '%.4f,%.5f,%.5f,%.6f,%.5f,%.5f', stamp(m), p.x, p.y,
                yaw_of(m.pose.pose.orientation), v.linear.x, v.angular.z)

    def _ekf(self, m):
        p = m.pose.pose.position
        self._w('ekf', '%.4f,%.6f,%.6f,%.7f', stamp(m), p.x, p.y,
                yaw_of(m.pose.pose.orientation))

    def _battery(self, m):
        t = stamp(m)
        if t - self.last_battery >= 1.0:
            self.last_battery = t
            self._w('battery', '%.2f,%.3f', t, m.voltage)

    def _diag(self, m):
        for s in m.status:
            values = {kv.key: kv.value for kv in s.values}
            if 'gyro_bias_z' in values:
                t = self.get_clock().now().nanoseconds * 1e-9
                self._w('bias', '%.2f,%s,%s', t, values['gyro_bias_z'],
                        values.get('zupt_learning', ''))

    def _scan(self, m):
        if self.bins is None or len(self.bins) != len(m.ranges):
            self.bins = [Welford() for _ in m.ranges]
            self.angle_min, self.angle_inc = m.angle_min, m.angle_increment
        valid = 0
        for w, r in zip(self.bins, m.ranges):
            w.add(r)
            valid += math.isfinite(r)
        t = stamp(m)
        minute = int(t // 60)
        if self.minute is not None and minute != self.minute and self.minute_total:
            self._w('scan_minute', '%d,%d,%.4f', self.minute * 60, self.minute_scans,
                    self.minute_valid / self.minute_total)
            self.minute_scans = self.minute_valid = self.minute_total = 0
        self.minute = minute
        self.minute_scans += 1
        self.minute_valid += valid
        self.minute_total += len(m.ranges)

    def close(self):
        if self.bins:
            with open(os.path.join(self.dir, 'scan_bins.csv'), 'w') as f:
                f.write('bin,angle,n,mean,std,valid_fraction\n')
                for i, w in enumerate(self.bins):
                    f.write('%d,%.4f,%d,%.4f,%.5f,%.4f\n' % (
                        i, self.angle_min + i * self.angle_inc, w.n, w.mean, w.std(),
                        w.n / w.total if w.total else 0.0))
        for f in self.files.values():
            f.close()
        self.get_logger().info('soak log closed')


def main(args=None):
    rclpy.init(args=args)
    node = SoakLog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
