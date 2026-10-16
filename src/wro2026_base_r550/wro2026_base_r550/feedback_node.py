"""
R550 feedback node: STM32 serial 3 -> ROS 2 topics. READ ONLY.

This node never writes to the serial port and does not subscribe to
/cmd_vel, so it cannot make the motors move. The command side (with the
/cmd_vel timeout, speed limit and low-battery checks) is a later step.

Publishes:
  /odom + TF odom->base_link  nav_msgs/Odometry  (STM32 vx, wz from encoders)
  /imu/data                   sensor_msgs/Imu    (raw gyro + accel; no orientation)
  /battery                    sensor_msgs/BatteryState
  /diagnostics                diagnostic_msgs/DiagnosticArray

Protocol: see protocol.py (official WHEELTEC document, 24-byte frames,
20 Hz by default). Signs and axes are as the document describes them and
are not yet checked on the car.
"""
import sys
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from tf2_ros import TransformBroadcaster

from .feedback import monotonic, OdomIntegrator, quaternion_from_yaw, RateMeter
from .protocol import BAUD_RATE, FeedbackStream

UNUSED_COVARIANCE = 1.0e6  # z / roll / pitch are not estimated (planar odometry)

# name, default, description. Defaults must match config/r550_feedback.yaml.
PARAMETERS = [
    ('port', '', 'serial device of C30D serial 3 (middle USB-C); required, see README'),
    ('baud', BAUD_RATE, 'serial baud rate (official: 115200)'),
    ('odom_frame_id', 'odom', 'odometry frame'),
    ('base_frame_id', 'base_link', 'robot base frame'),
    ('imu_frame_id', 'imu_link', 'frame of /imu/data'),
    ('battery_frame_id', 'base_link', 'frame of /battery'),
    ('publish_odom_tf', True, 'broadcast TF odom -> base_link (false when an EKF owns it)'),
    ('odom_max_gap', 0.5, 'do not integrate across a gap between frames longer than this (s)'),
    # 2026-10-16: STM32 reports 26.4 cm per wheel turn, tyre circumference 21.5 cm
    ('vx_scale', 0.814, 'multiply STM32 vx by this (wheels-in-the-air test, provisional)'),
    ('wz_scale', 0.814, 'multiply STM32 wz by this (assumed same as vx; NOT verified)'),
    ('feedback_timeout', 0.5, '/diagnostics ERROR when no valid frame for this long (s)'),
    ('expected_rate', 20.0, 'expected frame rate (Hz); WARN below half of it'),
    ('low_battery_voltage', 10.8, 'WARN below this voltage (vendor: charge below 10.8 V)'),
    ('diagnostics_period', 1.0, '/diagnostics publish period (s)'),
    ('odom_pose_covariance_xy', 0.01, 'x/y variance (m^2); placeholder until measured'),
    ('odom_pose_covariance_yaw', 0.05, 'yaw variance (rad^2); placeholder until measured'),
    ('odom_twist_covariance_vx', 0.001, 'vx variance ((m/s)^2); placeholder until measured'),
    ('odom_twist_covariance_wz', 0.01, 'wz variance ((rad/s)^2); placeholder until measured'),
    ('imu_gyro_covariance', 0.0001, 'gyro variance ((rad/s)^2); placeholder until measured'),
    ('imu_accel_covariance', 0.01, 'accel variance ((m/s^2)^2); placeholder until measured'),
]


class SerialReader:
    """Background thread that reads the port and hands frames to on_frame. Never writes."""

    def __init__(self, port, baud, on_frame, log):
        import serial  # imported here so the rest of the package needs no pyserial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.stream = FeedbackStream()
        self.on_frame = on_frame
        self.log = log
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        """Start reading."""
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:  # unplugged cable, permission lost
                self.log(f'serial read failed: {e}')
                self.running = False
                return
            if not chunk:
                continue
            now = monotonic()
            for fb in self.stream.feed(chunk):
                self.on_frame(now, fb)

    def close(self):
        """Stop the thread and close the port."""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.ser.close()


class R550Feedback(Node):
    """ROS wiring around SerialReader / OdomIntegrator."""

    def __init__(self):
        super().__init__('r550_feedback')
        self.p = {name: self.declare_parameter(name, default).value
                  for name, default, _ in PARAMETERS}
        if not self.p['port']:
            raise ValueError("parameter 'port' is empty: pass port:=/dev/... "
                             '(find it with: ls -l /dev/serial/by-id/)')
        self.lock = threading.Lock()
        self.odom = OdomIntegrator(max_gap=self.p['odom_max_gap'])
        self.rate = RateMeter(window=2.0)
        self.last_frame_time = None
        self.last_fb = None
        self.read_error = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.bat_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.tf_pub = TransformBroadcaster(self) if self.p['publish_odom_tf'] else None

        self.reader = SerialReader(self.p['port'], self.p['baud'], self._on_frame,
                                   self._on_read_error)
        self.get_logger().info(f"listening on {self.p['port']} @ {self.p['baud']} "
                               '(READ ONLY: nothing is sent to the STM32, no /cmd_vel)')
        self.create_timer(self.p['diagnostics_period'], self._publish_diagnostics)
        self.reader.start()

    # -------------------------------------------------------------- frames
    def _on_read_error(self, text):
        self.read_error = text
        try:
            self.get_logger().error(text)
        except Exception:
            print(text, file=sys.stderr, flush=True)

    def _on_frame(self, now, fb):
        with self.lock:
            vx = fb.vx * self.p['vx_scale']
            wz = fb.wz * self.p['wz_scale']
            self.odom.update(now, vx, wz)
            x, y, yaw = self.odom.x, self.odom.y, self.odom.yaw
            self.rate.tick(now)
            self.last_frame_time = now
            self.last_fb = fb
        stamp = self.get_clock().now().to_msg()
        try:
            self._publish_odom(stamp, x, y, yaw, vx, wz)
            self._publish_imu(stamp, fb)
            self._publish_battery(stamp, fb)
        except Exception:  # context shut down while the thread was publishing
            pass

    def _publish_odom(self, stamp, x, y, yaw, vx, wz):
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.p['odom_frame_id']
        o.child_frame_id = self.p['base_frame_id']
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = vx
        o.twist.twist.angular.z = wz
        pc = [0.0] * 36
        pc[0] = pc[7] = self.p['odom_pose_covariance_xy']
        pc[14] = pc[21] = pc[28] = UNUSED_COVARIANCE
        pc[35] = self.p['odom_pose_covariance_yaw']
        tc = [0.0] * 36
        tc[0] = tc[7] = self.p['odom_twist_covariance_vx']
        tc[14] = tc[21] = tc[28] = UNUSED_COVARIANCE
        tc[35] = self.p['odom_twist_covariance_wz']
        o.pose.covariance = pc
        o.twist.covariance = tc
        self.odom_pub.publish(o)
        if self.tf_pub is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.p['odom_frame_id']
            t.child_frame_id = self.p['base_frame_id']
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf_pub.sendTransform(t)

    def _publish_imu(self, stamp, fb):
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = self.p['imu_frame_id']
        m.orientation_covariance[0] = -1.0  # REP-145: no orientation estimate
        m.angular_velocity.x = fb.gyro_x
        m.angular_velocity.y = fb.gyro_y
        m.angular_velocity.z = fb.gyro_z
        m.linear_acceleration.x = fb.accel_x
        m.linear_acceleration.y = fb.accel_y
        m.linear_acceleration.z = fb.accel_z
        g, a = self.p['imu_gyro_covariance'], self.p['imu_accel_covariance']
        m.angular_velocity_covariance[0] = m.angular_velocity_covariance[4] = g
        m.angular_velocity_covariance[8] = g
        m.linear_acceleration_covariance[0] = m.linear_acceleration_covariance[4] = a
        m.linear_acceleration_covariance[8] = a
        self.imu_pub.publish(m)

    def _publish_battery(self, stamp, fb):
        b = BatteryState()
        b.header.stamp = stamp
        b.header.frame_id = self.p['battery_frame_id']
        b.voltage = float(fb.voltage)
        nan = float('nan')
        b.current = b.charge = b.capacity = b.design_capacity = b.percentage = nan
        b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        b.present = True
        self.bat_pub.publish(b)

    # --------------------------------------------------------- diagnostics
    def _publish_diagnostics(self):
        now = monotonic()
        with self.lock:
            last_t, fb = self.last_frame_time, self.last_fb
            rate = self.rate.rate(now)
            gaps = self.odom.gaps
        stream = self.reader.stream
        age = None if last_t is None else now - last_t
        level, msgs = DiagnosticStatus.OK, []
        if self.read_error:
            level, msgs = DiagnosticStatus.ERROR, [self.read_error]
        elif age is None or age > self.p['feedback_timeout']:
            level, msgs = DiagnosticStatus.ERROR, ['no valid feedback frame']
        else:
            if rate < self.p['expected_rate'] / 2.0:
                level = DiagnosticStatus.WARN
                msgs.append(f'low frame rate {rate:.1f} Hz')
            if fb.voltage < self.p['low_battery_voltage']:
                level = DiagnosticStatus.WARN
                msgs.append(f'battery low {fb.voltage:.2f} V')
        status = DiagnosticStatus()
        status.level = level
        status.name = 'r550_feedback: base'
        status.hardware_id = 'wheeltec_r550_c30d'
        status.message = '; '.join(msgs) or 'ok'
        values = {
            'mode': 'READ ONLY (no commands sent)',
            'port': self.p['port'],
            'frame_rate_hz': f'{rate:.1f}',
            'feedback_age_s': 'none' if age is None else f'{age:.2f}',
            'bad_frames': str(stream.bad_frames),
            'dropped_bytes': str(stream.dropped_bytes),
            'odom_gaps': str(gaps),
            'battery_v': 'unknown' if fb is None else f'{fb.voltage:.2f}',
            # not the Motor switch (reads 0 with the switch at OFF); meaning UNKNOWN
            'flag_stop': 'unknown' if fb is None else str(fb.stop_flag),
        }
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.diag_pub.publish(arr)

    def close(self):
        """Stop reading and close the port."""
        self.reader.close()


def main():
    """Entry point."""
    rclpy.init()
    try:
        node = R550Feedback()
    except Exception as e:  # bad parameter, port missing or busy: nothing was sent
        print(f'r550_feedback: {e}', file=sys.stderr, flush=True)
        rclpy.try_shutdown()
        return 1
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
