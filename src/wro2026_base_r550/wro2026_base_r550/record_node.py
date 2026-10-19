"""
Records /cmd_vel, /cmd_vel_out (Twist) and /odom (Odometry) into ONE csv with
the time each message ARRIVED, so "key released -> wheels stopped" can be
measured (Twist has no time stamp; `ros2 topic echo --csv` prints none).

    ros2 run wro2026_base_r550 record_node --ros-args -p out:=/path/drive.csv

Columns: t (s, wall clock), topic, vx (m/s), wz (rad/s).
The topics may appear later than this node starts: the types are fixed here,
so the subscriptions simply wait (echo gave up on 2026-10-19 instead).
Read only: it publishes nothing.
"""
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Recorder(Node):
    def __init__(self):
        super().__init__('r550_record')
        out = self.declare_parameter('out', 'drive.csv').value
        self.f = open(out, 'w', buffering=1)          # line buffered
        self.f.write('t,topic,vx,wz\n')
        self.n = 0
        self.create_subscription(Twist, 'cmd_vel', lambda m: self._row('cmd_vel', m), 50)
        self.create_subscription(Twist, 'cmd_vel_out',
                                 lambda m: self._row('cmd_vel_out', m), 50)
        self.create_subscription(Odometry, 'odom', lambda m: self._row('odom', m.twist.twist),
                                 50)
        self.get_logger().info('recording cmd_vel, cmd_vel_out, odom -> %s' % out)

    def _row(self, topic, twist):
        self.f.write('%.3f,%s,%.4f,%.4f\n' % (time.time(), topic, twist.linear.x,
                                              twist.angular.z))
        self.n += 1

    def close(self):
        self.f.close()
        self.get_logger().info('%d rows written' % self.n)


def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
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
