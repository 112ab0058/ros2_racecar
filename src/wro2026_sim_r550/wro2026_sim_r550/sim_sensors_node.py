"""
Turn the Gazebo sensor topics into the REAL car's topics. SIMULATION ONLY.

  /sim/odom (wheel odometry of the Ackermann plugin)
      -> /odom: frame odom -> base_link, covariances from r550_feedback.yaml,
         and TF odom -> base_link when publish_odom_tf is true (false with the EKF)
  /sim/imu
      -> /imu/data_raw: frame imu_link, gyro as simulated (with its bias)
      -> /imu/data:     gyro minus the ZUPT bias (same zupt.GyroBias as r550_feedback)
So the EKF, slam_toolbox and Nav2 see exactly what they see on the car.
Parameters and defaults come from wro2026_base_r550.feedback_node.PARAMETERS.
Sim calibration (default 1.0 = unchanged), the same steps the real car needs (vx_scale,
gyro scale; ICM20948 sensitivity tolerance is about +-1.5 %). The 2026-10-27 campaign
measured, over 2 laps against the true pose:
  odom_vx_scale  x /odom twist vx: the wheel odometry counts 1.9 % too much distance
                 (contact sinking makes the rolling radius smaller), 0.981 corrects it
  gyro_scale     x gyro z of /imu/data(_raw): the gyro reads 0.5 % less turning
                 (EKF heading -3.6 deg after 2 laps), 1.005 corrects it
"""
import sys

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from wro2026_base_r550.feedback_node import PARAMETERS, UNUSED_COVARIANCE
from wro2026_base_r550.zupt import GyroBias

USED = ('odom_frame_id', 'base_frame_id', 'imu_frame_id', 'publish_odom_tf',
        'odom_pose_covariance_xy', 'odom_pose_covariance_yaw', 'odom_twist_covariance_vx',
        'odom_twist_covariance_wz', 'imu_gyro_covariance', 'imu_accel_covariance',
        'zupt_enable', 'zupt_still_time', 'zupt_time_constant', 'zupt_max_rate')


class SimSensors(Node):
    def __init__(self):
        super().__init__('r550_sim_sensors')
        self.p = {name: self.declare_parameter(name, default).value
                  for name, default, _ in PARAMETERS if name in USED}
        self.gyro_scale = float(self.declare_parameter('gyro_scale', 1.0).value)
        self.odom_vx_scale = float(self.declare_parameter('odom_vx_scale', 1.0).value)
        self.zupt = GyroBias(enabled=self.p['zupt_enable'],
                             still_time=self.p['zupt_still_time'],
                             time_constant=self.p['zupt_time_constant'],
                             max_rate=self.p['zupt_max_rate'])
        self.vx = self.wz = 0.0
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.raw_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.tf = TransformBroadcaster(self) if self.p['publish_odom_tf'] else None
        self.create_subscription(Odometry, 'sim/odom', self._odom, 10)
        self.create_subscription(Imu, 'sim/imu', self._imu, 10)
        self.get_logger().info('SIM sensors: /sim/odom -> /odom, /sim/imu -> /imu/data(_raw); '
                               'publish_odom_tf %s' % self.p['publish_odom_tf'])

    def _odom(self, m):
        self.vx, self.wz = m.twist.twist.linear.x, m.twist.twist.angular.z
        m.twist.twist.linear.x *= self.odom_vx_scale
        m.header.frame_id = self.p['odom_frame_id']
        m.child_frame_id = self.p['base_frame_id']
        pc = [0.0] * 36
        pc[0] = pc[7] = self.p['odom_pose_covariance_xy']
        pc[14] = pc[21] = pc[28] = UNUSED_COVARIANCE
        pc[35] = self.p['odom_pose_covariance_yaw']
        tc = [0.0] * 36
        tc[0] = tc[7] = self.p['odom_twist_covariance_vx']
        tc[14] = tc[21] = tc[28] = UNUSED_COVARIANCE
        tc[35] = self.p['odom_twist_covariance_wz']
        m.pose.covariance, m.twist.covariance = pc, tc
        self.odom_pub.publish(m)
        if self.tf is not None:
            t = TransformStamped()
            t.header = m.header
            t.child_frame_id = m.child_frame_id
            p, q = m.pose.pose.position, m.pose.pose.orientation
            t.transform.translation.x, t.transform.translation.y = p.x, p.y
            t.transform.rotation = q
            self.tf.sendTransform(t)

    def _imu(self, m):
        m.header.frame_id = self.p['imu_frame_id']
        m.orientation_covariance = [-1.0] + [0.0] * 8      # REP-145: no orientation
        g, a = self.p['imu_gyro_covariance'], self.p['imu_accel_covariance']
        m.angular_velocity_covariance = [g, 0.0, 0.0, 0.0, g, 0.0, 0.0, 0.0, g]
        m.linear_acceleration_covariance = [a, 0.0, 0.0, 0.0, a, 0.0, 0.0, 0.0, a]
        w = m.angular_velocity
        w.z *= self.gyro_scale
        self.raw_pub.publish(m)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        w.x, w.y, w.z = self.zupt.update(t, self.vx, self.wz, w.x, w.y, w.z)
        self.imu_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SimSensors()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
