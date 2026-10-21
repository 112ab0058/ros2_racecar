"""
Logs /odom and /imu/data into two csv files for calibration (read only).

    ros2 run wro2026_base_r550 sensor_log_node --ros-args -p dir:=/path/to/dir

  odom.csv  t, x, y, yaw, vx, wz        (t = message stamp, s; yaw in rad)
  imu.csv   t, gx, gy, gz, ax, ay, az   (rad/s, m/s^2, as published)

Analysis: tools/real/analyze_calib.py. Publishes nothing, does not touch the
serial port; the topics may start later than this node.
"""
import math
import os
import sys

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class SensorLog(Node):
    def __init__(self):
        super().__init__('r550_sensor_log')
        d = self.declare_parameter('dir', '.').value
        os.makedirs(d, exist_ok=True)
        self.odom = open(os.path.join(d, 'odom.csv'), 'w', buffering=1)
        self.imu = open(os.path.join(d, 'imu.csv'), 'w', buffering=1)
        self.odom.write('t,x,y,yaw,vx,wz\n')
        self.imu.write('t,gx,gy,gz,ax,ay,az\n')
        self.n = [0, 0]
        self.create_subscription(Odometry, 'odom', self._odom, 50)
        self.create_subscription(Imu, 'imu/data', self._imu, 50)
        self.get_logger().info('logging /odom and /imu/data -> %s' % d)

    def _odom(self, m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        yaw = 2.0 * math.atan2(q.z, q.w)
        self.odom.write('%.4f,%.5f,%.5f,%.5f,%.5f,%.5f\n'
                        % (stamp(m), p.x, p.y, yaw, v.linear.x, v.angular.z))
        self.n[0] += 1

    def _imu(self, m):
        g, a = m.angular_velocity, m.linear_acceleration
        self.imu.write('%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n'
                       % (stamp(m), g.x, g.y, g.z, a.x, a.y, a.z))
        self.n[1] += 1

    def close(self):
        self.odom.close()
        self.imu.close()
        self.get_logger().info('rows: odom %d, imu %d' % tuple(self.n))


def main(args=None):
    rclpy.init(args=args)
    node = SensorLog()
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
