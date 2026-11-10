"""
Phone teleop node: a web page with hold-to-drive keys / joystick / tilt -> /cmd_vel.

    ros2 run wro2026_base_r550 web_teleop_node        (normally started by orin_field.sh)

Open the URL from the log on the phone (same WiFi / hotspot as the Orin). See
web_teleop.py for the page (the vendor app's functions, through ROS) and the
rules: the car drives only while a key, the joystick or the tilt hold button
is pressed and the phone keeps sending; otherwise 0 within hold_timeout
(0.35 s). Reads /battery, /odom, /imu/data and /diagnostics for the gauges.

https:=true serves https only, with a certificate signed by the Orin's own CA
in cert_dir for the addresses it has at start (web_cert.py). The phone trusts
that CA once (the page's 憑證 link); https also enables the tilt mode.

Publishes /cmd_vel at a FIXED rate from a timer, 0 included (like teleop_node).
It does not touch the serial port: only base_node with enable_commands:=true
moves the motors, and its limits still apply. SIGINT and SIGTERM publish a few
zeros, then exit (own handlers: a program started in the background by a
script inherits "ignore SIGINT").
"""
import os
import secrets
import signal
import ssl
import sys
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState, Imu

from .web_cert import ensure_ca, local_addresses, make_server_cert
from .web_teleop import make_server, NAMES, WebTeleop

# name, default, description
PARAMETERS = [
    ('speed', 0.1, 'forward speed at start (m/s); keep 0.1 for training data'),
    ('max_speed', 0.0, 'the page may raise the speed up to this (m/s, <= 0.15); 0 = speed'),
    ('back_speed', 0.08, 'reverse speed (m/s), at most the forward speed'),
    ('turn_radius', 0.5, 'turn radius of left / right (m), >= 0.35'),
    ('hold_timeout', 0.35, 'stop when the phone has not sent for this long (s)'),
    ('rate', 20.0, '/cmd_vel messages per second'),
    ('port', 8080, 'web server port'),
    ('token', '', 'secret in the URL (?k=); empty = a new random one'),
    ('https', False, 'true = https only, certificate from the CA in cert_dir'),
    ('cert_dir', '~/.r550_web', 'CA and page certificate (CA key stays here, mode 600)'),
    ('dns_names', '112ab-jno.netbird.cloud', 'comma-separated names for the certificate'),
]


class WebTeleopNode(Node):
    """See the module docstring."""

    def __init__(self):
        super().__init__('r550_web_teleop')
        p = {name: self.declare_parameter(name, default).value
             for name, default, _ in PARAMETERS}
        self.teleop = WebTeleop(speed=p['speed'], turn_radius=p['turn_radius'],
                                hold_timeout=p['hold_timeout'], back_speed=p['back_speed'],
                                max_speed=p['max_speed'] or p['speed'])
        self.token = p['token'] or secrets.token_hex(3)
        self.state = {'battery': None, 'odom_vx': None, 'gyro_z': None, 'base': None}
        self.last_action = 'none'
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(Imu, 'imu/data', self._on_imu, 10)
        self.create_subscription(DiagnosticArray, 'diagnostics', self._on_diagnostics, 10)
        context, ca_cert, scheme = None, None, 'http'
        if p['https']:
            cert_dir = os.path.expanduser(p['cert_dir'])
            ca_cert = ensure_ca(cert_dir)
            names = [n.strip() for n in p['dns_names'].split(',') if n.strip()]
            crt, key = make_server_cert(cert_dir, local_addresses(), names)
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(crt, key)
            scheme = 'https'
        self.server = make_server(self.teleop, self.token, p['port'], time.monotonic,
                                  self._status, context, ca_cert)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.create_timer(1.0 / p['rate'], self._tick)
        self.get_logger().warn(
            'phone page: %s://<orin ip>:%d/?k=%s  speed %.2f (max %.2f) m/s, turn radius '
            '%.2f m, hold timeout %.2f s' % (scheme, p['port'], self.token, p['speed'],
                                             self.teleop.max_speed, p['turn_radius'],
                                             p['hold_timeout']))

    # Gauges for the page (read only; a dict update is atomic for the web thread).
    def _on_battery(self, msg):
        self.state['battery'] = round(float(msg.voltage), 2)

    def _on_odom(self, msg):
        self.state['odom_vx'] = round(float(msg.twist.twist.linear.x), 3)

    def _on_imu(self, msg):
        self.state['gyro_z'] = round(float(msg.angular_velocity.z), 3)

    def _on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == 'r550_base: command':
                self.state['base'] = status.message

    def _status(self):
        return dict(self.state)

    def _tick(self):
        now = time.monotonic()
        vx, wz = self.teleop.command(now)
        self.publish(vx, wz)
        action = self.teleop.current(now)
        if action != self.last_action:
            self.get_logger().info('phone: %s (vx %+.2f, wz %+.2f)' % (NAMES[action], vx, wz))
            self.last_action = action

    def publish(self, vx, wz):
        """Publish one /cmd_vel."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def finish(self):
        """Close the web server and publish a few zeros."""
        self.server.shutdown()
        self.server.server_close()
        if rclpy.ok():
            for _ in range(5):
                self.publish(0.0, 0.0)
                time.sleep(0.05)
        print('web_teleop_node: stopped after %d requests' % self.teleop.requests, flush=True)


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    """Entry point."""
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = WebTeleopNode()
    except Exception as e:  # bad parameter, port in use: nothing was published
        print('web_teleop_node: %s' % e, file=sys.stderr, flush=True)
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
