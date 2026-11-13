"""
/scan + odometry -> /scan_filled: the lidar's near blind beams filled from memory.

READ ONLY: subscribes and publishes topics, never touches hardware. See scan_memory.py
for the rules. Filled beams have intensity 1.0, measured beams 0.0, so a viewer or a
policy can tell memory from measurement.

Odometry: /odometry/filtered (EKF, heading from the gyro) by default. On the 2026-11-11
field recordings the wheel-only /odom heading smeared the remembered walls in turns;
the EKF heading kept them straight.
"""
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .scan_memory import ScanMemory

PARAMETERS = [
    ('scan_in', 'scan', 'raw scan topic'),
    ('scan_out', 'scan_filled', 'output topic'),
    ('odom_topic', '/odometry/filtered', 'odometry (EKF) topic; /odom if no EKF runs'),
    ('laser_x', 0.13, 'm; lidar in front of base_link (urdf laser_link)'),
    ('remember_range', 1.2, 'm; returns closer than this are remembered'),
    ('fill_range', 0.6, 'm; only remembered points closer than this fill a beam'),
    ('keep_time', 30.0, 's; forget points older than this (walls do not move; 8 s lost them '
     'while the car waited)'),
    ('keep_dist', 1.0, 'm; forget points stored more than this many metres of driving ago'),
    ('clear_margin', 0.05, 'm; a return this far behind a remembered point clears it'),
]


class ScanMemoryNode(Node):
    """ROS wiring around ScanMemory."""

    def __init__(self):
        """Declare parameters, create the memory, subscribe and publish."""
        super().__init__('scan_memory')
        self.p = {n: self.declare_parameter(n, d).value for n, d, _ in PARAMETERS}
        keys = ('laser_x', 'remember_range', 'fill_range', 'keep_time', 'keep_dist',
                'clear_margin')
        self.mem = ScanMemory(**{k: float(self.p[k]) for k in keys})
        self.pub = self.create_publisher(LaserScan, self.p['scan_out'], 10)
        self.create_subscription(Odometry, self.p['odom_topic'], self._odom, 20)
        self.create_subscription(LaserScan, self.p['scan_in'], self._scan, 10)
        self.warned = False
        self.get_logger().info('%s + %s -> %s (READ ONLY)' % (
            self.p['scan_in'], self.p['odom_topic'], self.p['scan_out']))

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.mem.on_odom(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _scan(self, msg):
        if self.mem.pose is None and not self.warned:
            self.warned = True
            self.get_logger().warn('no odometry on %s yet: passing scans through unchanged'
                                   % self.p['odom_topic'])
        ranges, mask = self.mem.on_scan(time.monotonic(), msg.ranges, msg.angle_min,
                                        msg.angle_increment)
        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_max = msg.angle_min, msg.angle_max
        out.angle_increment, out.time_increment = msg.angle_increment, msg.time_increment
        out.scan_time, out.range_min, out.range_max = msg.scan_time, msg.range_min, msg.range_max
        out.ranges = [float(r) for r in ranges]
        out.intensities = [1.0 if m else 0.0 for m in mask]
        self.pub.publish(out)


def main():
    """Entry point."""
    rclpy.init()
    node = ScanMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
