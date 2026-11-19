"""
/scan + odometry -> /scan_filled: the lidar's near blind beams filled from memory.

READ ONLY: subscribes and publishes topics, never touches hardware. See scan_memory.py
for the rules. Measured beams have intensity 0.0, beams filled from lidar memory 1.0 and
from camera memory 2.0, so a viewer or a policy can tell memory from measurement.

Camera: camera/wall_scan (wro2026_camera_c70 wall_node) wall points go into the same
memory, placed with the odometry pose at the frame's time stamp. Without the camera
node the subscription just stays silent.

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

from .scan_memory import PoseHistory, ScanMemory

PARAMETERS = [
    ('scan_in', 'scan', 'raw scan topic'),
    ('scan_out', 'scan_filled', 'output topic'),
    ('odom_topic', '/odometry/filtered', 'odometry (EKF) topic; /odom if no EKF runs'),
    ('camera_topic', 'camera/wall_scan', "camera wall points (wro2026_camera_c70); '' = none"),
    ('camera_max_gap', 0.2, 's; drop a camera frame whose stamp is this far outside the '
     'odometry poses kept (2 s)'),
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
        self.history = PoseHistory()
        self.pub = self.create_publisher(LaserScan, self.p['scan_out'], 10)
        self.create_subscription(Odometry, self.p['odom_topic'], self._odom, 20)
        self.create_subscription(LaserScan, self.p['scan_in'], self._scan, 10)
        if self.p['camera_topic']:
            self.create_subscription(LaserScan, self.p['camera_topic'], self._camera, 10)
        self.warned = False
        self.camera_frames = self.camera_dropped = 0
        self.create_timer(10.0, self._report)
        self.get_logger().info('%s + %s + %s -> %s (READ ONLY)' % (
            self.p['scan_in'], self.p['camera_topic'] or '(no camera)', self.p['odom_topic'],
            self.p['scan_out']))

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.mem.on_odom(x, y, yaw)
        self.history.add(_seconds(msg.header.stamp), x, y, yaw)

    def _camera(self, msg):
        pose = self.history.at(_seconds(msg.header.stamp), self.p['camera_max_gap'])
        if pose is None:
            self.camera_dropped += 1
            return
        self.camera_frames += 1
        self.mem.add_camera(time.monotonic(), msg.ranges, msg.angle_min, msg.angle_increment,
                            pose)

    def _report(self):
        if self.p['camera_topic'] and (self.camera_frames or self.camera_dropped):
            self.get_logger().info('camera frames in the last 10 s: %d used, %d dropped (no '
                                   'odometry at their time)'
                                   % (self.camera_frames, self.camera_dropped))
        self.camera_frames = self.camera_dropped = 0

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
        cam = self.mem.filled_by_camera
        out.intensities = [(2.0 if c else 1.0) if m else 0.0 for m, c in zip(mask, cam)]
        self.pub.publish(out)


def _seconds(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


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
