"""
WAVE ROVER ROS 2 driver node (wiring only; the logic is in core.py).

Subscribes /cmd_vel (geometry_msgs/Twist), converts it to left/right wheel
commands and sends Waveshare JSON {"T":1,"L":..,"R":..} to the ESP32.
L/R are NOT m/s: the WAVE ROVER firmware applies PWM = L * 512 (0.5 = full
power), so max_speed is a PWM fraction cap.

Publishes:
  /battery              sensor_msgs/BatteryState  (from T:130 -> T:1001 "v")
  /imu/data             sensor_msgs/Imu           (from T:126 -> T:1002)
  /odom + TF odom->base_link  nav_msgs/Odometry   (dead reckoning, LOW QUALITY)
  /diagnostics          diagnostic_msgs/DiagnosticArray
  /wave_rover/feedback  std_msgs/String           (every raw JSON reply/line)
  /wave_rover/sent_cmd  std_msgs/String           (every T:1 actually sent)

Units (Waveshare ugv_base_general firmware, QMI8658_UINT_MG_DPS):
  r/p/y degrees, ax/ay/az mg, gx/gy/gz deg/s, mx/my/mz raw AK09918 counts.

Transport:
  serial (default): /dev/ttyTHS1 @ 115200 (Orin 40-pin: pin 8 TX, pin 10 RX, pin 6 GND)
  http:             http://<host>/js?json=...

All ESP32 I/O runs in one worker thread so a slow HTTP request never blocks
the ROS executor. Wheel commands have priority over feedback polling.

Safety: stop when no /cmd_vel arrives within cmd_timeout seconds, when no
T:130 reply arrives within feedback_timeout seconds, and refuse non-zero
commands below critical_battery_voltage. The ESP32 also stops by itself
after ~3 s without commands. Stop is sent on shutdown.

Odometry: the WAVE ROVER has no wheel encoders. Linear speed = wheel
commands actually sent x pwm_to_mps (UNKNOWN until measured, default 0, so
/odom only rotates); yaw rate = IMU gz. Do not use it as the main odometry
source for Nav2.
"""
import sys
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .core import (BaseController, DriverLoop, encode_command, imu_values, LEVEL_ERROR,
                   LEVEL_OK, LEVEL_WARN, odom_twist, OdomIntegrator, quaternion_from_rpy)
from .process_guard import find_other_drivers
from .transport import make_transport

UNUSED_COVARIANCE = 1.0e6  # z / roll / pitch are not estimated (planar odometry)
DIAG_LEVELS = {LEVEL_OK: DiagnosticStatus.OK, LEVEL_WARN: DiagnosticStatus.WARN,
               LEVEL_ERROR: DiagnosticStatus.ERROR}

# name, default, description. Defaults must match config/wave_rover.yaml.
PARAMETERS = [
    ('transport', 'serial', "'serial' or 'http'"),
    ('port', '/dev/ttyTHS1', 'serial device'),
    ('baud', 115200, 'serial baud rate'),
    ('host', '192.168.4.1', 'ESP32 address for http transport'),
    ('track_width', 0.125, 'left/right wheel distance (m); firmware constant, NOT measured yet'),
    ('cmd_vel_units', 'pwm',
     "'pwm': /cmd_vel is PWM fractions; 'mps': m/s, rad/s (needs pwm_to_mps)"),
    ('max_speed', 0.05, 'wheel command cap, PWM fraction (0.5 = 100%)'),
    ('send_rate', 10.0, 'serial: T:1 resend rate (Hz)'),
    ('http_cmd_period', 0.5, 'http: resend unchanged T:1 every (s)'),
    ('http_timeout', 0.3, 'http request timeout (s)'),
    ('cmd_timeout', 0.3, 'send stop if no /cmd_vel for this long (s)'),
    ('feedback_period', 0.2, 'poll T:130 and T:126 every (s)'),
    ('poll_base', True, 'poll T:130 (battery, applied PWM)'),
    ('poll_imu', True, 'poll T:126 (IMU)'),
    ('serial_continuous_feedback', False, 'serial: send T:131 cmd 1 at start'),
    ('imu_frame_id', 'imu_link', 'frame of /imu/data'),
    ('battery_frame_id', 'base_link', 'frame of /battery'),
    ('low_battery_voltage', 11.0, 'WARN below this voltage'),
    ('critical_battery_voltage', 10.5, 'refuse non-zero commands below this voltage'),
    ('feedback_timeout', 2.0, 'stop + ERROR when no T:130 reply for this long (s); 0 disables'),
    ('diagnostics_period', 1.0, '/diagnostics publish period (s)'),
    ('publish_odom', True, 'publish /odom'),
    ('publish_odom_tf', True, 'broadcast TF odom -> base_link'),
    ('odom_frame_id', 'odom', 'odometry frame'),
    ('base_frame_id', 'base_link', 'robot base frame'),
    ('odom_rate', 20.0, 'odometry integration / publish rate (Hz)'),
    ('pwm_to_mps', 0.0, 'm/s per unit wheel command; UNKNOWN, 0 = odom never translates'),
    ('imu_timeout', 0.5, 'IMU sample older than this is ignored for odom yaw rate (s)'),
    ('imu_yaw_rate_sign', 1.0, 'multiply IMU gz by this for odom (axis direction UNKNOWN)'),
    ('odom_hold_when_stopped', True, 'zero wheel command -> odom does not move'),
    ('odom_pose_covariance_xy', 100.0, 'x/y variance (m^2), large: low-quality odom'),
    ('odom_pose_covariance_yaw', 10.0, 'yaw variance (rad^2)'),
    ('odom_twist_covariance_vx', 1.0, 'vx variance ((m/s)^2)'),
    ('odom_twist_covariance_wz', 0.1, 'wz variance ((rad/s)^2)'),
]


class WaveRoverDriver(Node):
    """ROS wiring around BaseController / DriverLoop."""

    def __init__(self):
        super().__init__('wave_rover_driver')
        self.p = {name: self.declare_parameter(name, default).value
                  for name, default, _ in PARAMETERS}
        p = self.p
        if p['transport'] not in ('serial', 'http'):
            raise ValueError("transport must be 'serial' or 'http'")

        # raises ValueError (before any I/O) if cmd_vel_units cannot be used
        self.controller = BaseController(
            p['track_width'], p['max_speed'], p['cmd_timeout'], p['feedback_timeout'],
            p['low_battery_voltage'], p['critical_battery_voltage'],
            cmd_vel_units=p['cmd_vel_units'], pwm_to_mps=p['pwm_to_mps'])
        self.transport = make_transport(p['transport'], host=p['host'],
                                        http_timeout=p['http_timeout'],
                                        port=p['port'], baud=p['baud'])
        cmd_period = 1.0 / p['send_rate'] if p['transport'] == 'serial' else p['http_cmd_period']
        self.loop = DriverLoop(self.controller, self.transport, cmd_period,
                               feedback_period=p['feedback_period'],
                               poll_base=p['poll_base'], poll_imu=p['poll_imu'],
                               on_sent=self._on_sent, on_line=self._on_line, log=self._log)
        self.running = True
        self.imu_lock = threading.Lock()
        self.last_gz = None
        self.last_imu_time = None
        self.odom = OdomIntegrator()
        self.last_odom_time = None

        self.fb_pub = self.create_publisher(String, 'wave_rover/feedback', 10)
        self.sent_pub = self.create_publisher(String, 'wave_rover/sent_cmd', 10)
        self.bat_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10) if p['publish_odom'] else None
        self.tf_pub = TransformBroadcaster(self) if p['publish_odom_tf'] else None

        if p['transport'] == 'serial':
            self.get_logger().info(f"Serial {p['port']} @ {p['baud']}")
        else:
            self.get_logger().info(f"HTTP http://{p['host']}/js")
        self.transport.start(self.loop.handle_line)
        if p['transport'] == 'serial' and p['serial_continuous_feedback']:
            self.transport.send({'T': 131, 'cmd': 1})

        self.get_logger().info(
            f"max_speed={self.controller.max_speed} cmd_timeout={p['cmd_timeout']}s "
            f"feedback_period={p['feedback_period']}s feedback_timeout={p['feedback_timeout']}s "
            f"track_width={p['track_width']} (firmware constant, not measured) "
            f"cmd_vel_units={p['cmd_vel_units']}")
        if p['feedback_timeout'] <= 0:
            self.get_logger().warn('feedback watchdog disabled (feedback_timeout <= 0)')
        elif not p['poll_base'] and not p['serial_continuous_feedback']:
            self.get_logger().warn('poll_base is false: no T:130 replies will arrive, so the '
                                   'feedback watchdog will refuse every non-zero command')
        if p['pwm_to_mps'] == 0.0:
            self.get_logger().warn(
                'odom: pwm_to_mps = 0 (UNKNOWN, not measured yet): /odom will only rotate '
                '(IMU gz) and never translate')
        self.get_logger().warn(
            'odom is LOW-QUALITY dead reckoning (no wheel encoders, commanded speed x '
            f"pwm_to_mps={p['pwm_to_mps']}, IMU yaw rate); do not use it as the main odometry "
            'source for Nav2')

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(p['diagnostics_period'], self._publish_diagnostics)
        if self.odom_pub is not None or self.tf_pub is not None:
            self.create_timer(1.0 / p['odom_rate'], self._update_odom)
        self.worker = threading.Thread(target=self.loop.run,
                                       args=(lambda: self.running and rclpy.ok(),),
                                       daemon=True)
        self.worker.start()

    # ------------------------------------------------------------- callbacks
    def _log(self, level, text):
        # errors / info come from state transitions (once each); warnings are
        # repeated send failures, so only those are throttled
        logger = self.get_logger()
        try:
            if level == 'error':
                logger.error(text)
            elif level == 'warn':
                logger.warn(text, throttle_duration_sec=2.0)
            else:
                logger.info(text)
        except Exception:
            print(f'[{level}] {text}', flush=True)

    def _on_cmd_vel(self, msg):
        if not self.controller.set_cmd_vel(msg.linear.x, msg.angular.z):
            self.get_logger().warn('/cmd_vel with NaN or inf: treated as stop',
                                   throttle_duration_sec=2.0)

    def _on_sent(self, rec):
        try:
            self.sent_pub.publish(String(data=encode_command(rec)))
        except Exception:
            pass

    def _on_line(self, line, msg):
        try:
            self.fb_pub.publish(String(data=line))
        except Exception:
            return
        if msg is None:
            return
        if msg.get('T') == 1001:
            self._publish_battery(msg)
        elif msg.get('T') == 1002:
            self._publish_imu(msg)

    def _publish_battery(self, msg):
        v = msg.get('v')
        if v is None:
            return
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.header.frame_id = self.p['battery_frame_id']
        b.voltage = float(v)
        nan = float('nan')
        b.current = b.charge = b.capacity = b.design_capacity = b.percentage = nan
        b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        b.present = True
        self.bat_pub.publish(b)
        if b.voltage < self.p['critical_battery_voltage']:
            self.get_logger().error(f'battery critical: {b.voltage:.2f} V, non-zero commands '
                                    'refused', throttle_duration_sec=10.0)
        elif b.voltage < self.p['low_battery_voltage']:
            self.get_logger().warn(f'battery low: {b.voltage:.2f} V', throttle_duration_sec=10.0)

    def _publish_imu(self, msg):
        vals = imu_values(msg)
        if vals is None:
            return
        with self.imu_lock:
            self.last_gz = vals['gz']
            self.last_imu_time = time.monotonic()
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.p['imu_frame_id']
        (m.orientation.x, m.orientation.y,
         m.orientation.z, m.orientation.w) = quaternion_from_rpy(vals['r'], vals['p'], vals['y'])
        m.angular_velocity.x = vals['gx']
        m.angular_velocity.y = vals['gy']
        m.angular_velocity.z = vals['gz']
        m.linear_acceleration.x = vals['ax']
        m.linear_acceleration.y = vals['ay']
        m.linear_acceleration.z = vals['az']
        # covariances left at 0 (= unknown) until the IMU is characterised
        self.imu_pub.publish(m)

    # ------------------------------------------------------------------ odom
    def _update_odom(self):
        now = time.monotonic()
        dt = 0.0 if self.last_odom_time is None else now - self.last_odom_time
        self.last_odom_time = now
        with self.loop.lock:
            left, right = self.loop.sent_ok_wheels
        with self.imu_lock:
            gz, t_imu = self.last_gz, self.last_imu_time
        fresh = t_imu is not None and now - t_imu <= self.p['imu_timeout']
        v, wz = odom_twist(left, right, gz, self.p['pwm_to_mps'], imu_fresh=fresh,
                           hold_when_stopped=self.p['odom_hold_when_stopped'],
                           yaw_rate_sign=self.p['imu_yaw_rate_sign'])
        self.odom.update(dt, v, wz)
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, self.odom.yaw)
        if self.odom_pub is not None:
            o = Odometry()
            o.header.stamp = stamp
            o.header.frame_id = self.p['odom_frame_id']
            o.child_frame_id = self.p['base_frame_id']
            o.pose.pose.position.x = self.odom.x
            o.pose.pose.position.y = self.odom.y
            o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
            o.twist.twist.linear.x = v
            o.twist.twist.angular.z = wz
            pc = [0.0] * 36
            pc[0] = pc[7] = self.p['odom_pose_covariance_xy']
            pc[14] = pc[21] = pc[28] = UNUSED_COVARIANCE
            pc[35] = self.p['odom_pose_covariance_yaw']
            tc = [0.0] * 36
            tc[0] = self.p['odom_twist_covariance_vx']
            tc[7] = tc[14] = tc[21] = tc[28] = UNUSED_COVARIANCE
            tc[35] = self.p['odom_twist_covariance_wz']
            o.pose.covariance = pc
            o.twist.covariance = tc
            self.odom_pub.publish(o)
        if self.tf_pub is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.p['odom_frame_id']
            t.child_frame_id = self.p['base_frame_id']
            t.transform.translation.x = self.odom.x
            t.transform.translation.y = self.odom.y
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf_pub.sendTransform(t)

    # ----------------------------------------------------------- diagnostics
    def _publish_diagnostics(self):
        now = time.monotonic()
        level, message = self.controller.status(now)
        age = self.controller.feedback_age(now)
        with self.imu_lock:
            t_imu = self.last_imu_time
        imu_age = None if t_imu is None else now - t_imu
        imu_stale = imu_age is None or imu_age > self.p['imu_timeout']
        if self.p['poll_imu'] and imu_stale:
            level = max(level, LEVEL_WARN)
            message = 'IMU (T:126) stale' if message == 'ok' else message + '; IMU stale'
        with self.loop.lock:
            left, right = self.loop.sent_ok_wheels
        status = DiagnosticStatus()
        status.level = DIAG_LEVELS[level]
        status.name = 'wave_rover_driver: base'
        status.hardware_id = 'waveshare_wave_rover'
        status.message = message
        values = {
            'transport': self.p['transport'],
            'feedback_state': self.controller.feedback_state(now),
            'feedback_age_s': 'none' if age is None else f'{age:.2f}',
            'battery_v': 'unknown' if self.controller.battery_v is None
            else f'{self.controller.battery_v:.2f}',
            'battery_state': self.controller.battery_state(),
            'imu_age_s': 'none' if imu_age is None else f'{imu_age:.2f}',
            'last_sent_L': str(left),
            'last_sent_R': str(right),
            'gate': self.loop.last_reason,
            'send_failures': str(self.loop.send_failures),
            'max_speed': str(self.controller.max_speed),
            'pwm_to_mps': f"{self.p['pwm_to_mps']} (UNKNOWN until measured)"
            if self.p['pwm_to_mps'] == 0.0 else str(self.p['pwm_to_mps']),
            'odom_quality': 'LOW (no encoders; not a Nav2 main source)',
        }
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.diag_pub.publish(arr)

    # -------------------------------------------------------------- shutdown
    def stop_and_close(self):
        """Stop the worker, send stop (up to 3 tries) and close the transport."""
        self.running = False
        if self.worker.is_alive():
            self.worker.join(timeout=1.5)
        ok = self.loop.send_stop(attempts=3, pause=0.1)
        text = f'shutdown stop sent: {"ok" if ok else "FAILED"}'
        try:
            self.get_logger().info(text)
        except Exception:
            print(text, flush=True)
        self.transport.close()


def main():
    """Entry point: refuse to run next to another driver, then spin."""
    others = find_other_drivers()
    if others:
        print('another WAVE ROVER driver is already running, refusing to start: '
              + '; '.join(f'pid {pid}: {cmd}' for pid, cmd in others), file=sys.stderr, flush=True)
        return 1
    rclpy.init()
    try:
        node = WaveRoverDriver()
    except ValueError as e:  # bad configuration: nothing has been sent yet
        print(f'wave_rover_driver: {e}', file=sys.stderr, flush=True)
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_and_close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
