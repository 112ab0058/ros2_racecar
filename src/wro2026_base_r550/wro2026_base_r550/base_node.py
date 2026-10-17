"""
R550 base node: everything feedback_node does, plus /cmd_vel -> STM32.

THIS NODE CAN MAKE THE MOTORS TURN (when enable_commands is true and the
C30D Motor switch is ON). Do not run it next to feedback_node: both open
the same serial port.

With enable_commands = false (default) it behaves like feedback_node: no
subscription, nothing written to the port.

With enable_commands = true it sends a command frame every 1/command_rate s.
What it sends comes from command.CommandGate:
  - /cmd_vel older than cmd_timeout (0.3 s)   -> stop
  - no STM32 feedback for feedback_timeout    -> stop
  - battery below min_battery_voltage         -> stop
  - armed longer than max_run_time (if > 0)   -> stop until restart
  - speeds clamped to max_vx / max_reverse_vx / max_wz, vx ramped by max_accel
  - no turning on the spot (Ackermann)
On shutdown (Ctrl+C) it sends stop frames before closing the port.

Unit conversion: the STM32 thinks a wheel turn is 26.4 cm, the real tyre is
21.5 cm (vx_scale 0.814). /odom multiplies the STM32's numbers by vx_scale,
so commands are divided by it: /cmd_vel 0.1 m/s really moves 0.1 m/s.

Publishes in addition to feedback_node:
  cmd_vel_out  geometry_msgs/Twist  what was sent, in real units
  /diagnostics status 'r550_base: command' (reason of the last decision)
"""
import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException

from .command import CMD_TIMEOUT, CommandGate, DISABLED, NO_CMD, OK
from .feedback import monotonic
from .feedback_node import R550Feedback
from .protocol import encode_twist_command, stop_command

# name, default, description. Defaults must match config/r550_base.yaml.
COMMAND_PARAMETERS = [
    ('enable_commands', False, 'false = read only like feedback_node; true = send /cmd_vel'),
    ('cmd_vel_stamped', False, 'subscribe TwistStamped instead of Twist (Nav2 Jazzy default)'),
    ('command_rate', 20.0, 'command frames per second (zeros are sent too)'),
    ('cmd_timeout', 0.3, 'stop when the newest /cmd_vel is older than this (s)'),
    ('max_vx', 0.3, 'forward speed limit (m/s, real)'),
    ('max_reverse_vx', 0.15, 'reverse speed limit (m/s, real)'),
    ('max_wz', 1.0, 'yaw rate limit (rad/s)'),
    ('max_accel', 0.5, 'vx ramp (m/s^2); safety stops are immediate'),
    ('min_turn_speed', 0.02, 'below this |vx| (m/s) wz is forced to 0 (no turning on the spot)'),
    ('min_battery_voltage', 10.5, 'refuse to move below this filtered voltage (V)'),
    ('max_run_time', 0.0, 'stop for good after this many seconds armed (0 = no limit)'),
    ('stop_frames_on_exit', 5, 'stop frames sent when the node shuts down'),
]


class R550Base(R550Feedback):
    """Feedback node + command side."""

    mode_text = 'command side in r550_base (see enable_commands)'

    def __init__(self):
        super().__init__('r550_base')
        self.c = {name: self.declare_parameter(name, default).value
                  for name, default, _ in COMMAND_PARAMETERS}
        self.gate = CommandGate(
            enabled=bool(self.c['enable_commands']),
            cmd_timeout=self.c['cmd_timeout'],
            max_vx=self.c['max_vx'],
            max_reverse_vx=self.c['max_reverse_vx'],
            max_wz=self.c['max_wz'],
            max_accel=self.c['max_accel'],
            min_turn_speed=self.c['min_turn_speed'],
            feedback_timeout=self.p['feedback_timeout'],
            min_battery_voltage=self.c['min_battery_voltage'],
            max_run_time=self.c['max_run_time'],
        )
        self.scale = self.p['vx_scale']
        if not self.scale > 0:
            raise ValueError('vx_scale must be > 0')
        self.frames_sent = 0
        self.write_error = None
        self.out_pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.create_timer(self.p['diagnostics_period'], self._publish_command_diagnostics)
        if not self.gate.enabled:
            self.get_logger().info('enable_commands is false: READ ONLY, nothing is sent')
            return
        if self.c['cmd_vel_stamped']:
            self.create_subscription(TwistStamped, 'cmd_vel',
                                     lambda m: self._on_cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_timer(1.0 / self.c['command_rate'], self._send_tick)
        self.get_logger().warn(
            f"COMMANDS ENABLED: max_vx {self.c['max_vx']} m/s, reverse "
            f"{self.c['max_reverse_vx']} m/s, max_wz {self.c['max_wz']} rad/s, "
            f"cmd_timeout {self.c['cmd_timeout']} s, max_run_time {self.c['max_run_time']} s. "
            'Hardware stop: Motor switch OFF.')

    # ------------------------------------------------------------ commands
    def _on_cmd(self, msg):
        if not self.gate.set_cmd(monotonic(), msg.linear.x, msg.angular.z):
            self.get_logger().warn('ignored /cmd_vel with NaN or Inf')

    def _send_tick(self):
        now = monotonic()
        with self.lock:
            last_t, fb = self.last_frame_time, self.last_fb
        if fb is not None and last_t is not None:
            self.gate.set_voltage(last_t, fb.voltage)
        vx, wz, reason = self.gate.update(now, last_t)
        if self.write_error:
            vx, wz = 0.0, 0.0
        frame = encode_twist_command(vx / self.scale, wz / self.scale)
        self._write(frame)
        out = Twist()
        out.linear.x, out.angular.z = vx, wz
        self.out_pub.publish(out)
        if reason != getattr(self, '_last_reason', None):
            self.get_logger().info(f'command state: {reason}')
            self._last_reason = reason

    def _write(self, frame):
        try:
            self.reader.ser.write(frame)
            self.frames_sent += 1
        except Exception as e:  # cable unplugged
            if not self.write_error:
                self.write_error = f'serial write failed: {e}'
                self.get_logger().error(self.write_error)

    def _publish_command_diagnostics(self):
        g = self.gate
        status = DiagnosticStatus()
        status.name = 'r550_base: command'
        status.hardware_id = 'wheeltec_r550_c30d'
        if self.write_error:
            status.level, status.message = DiagnosticStatus.ERROR, self.write_error
        elif g.reason in (OK, DISABLED, NO_CMD, CMD_TIMEOUT):
            status.level, status.message = DiagnosticStatus.OK, g.reason
        else:
            status.level, status.message = DiagnosticStatus.WARN, g.reason
        values = {
            'enable_commands': str(g.enabled).lower(),
            'reason': g.reason,
            'sent_vx': f'{g.out_vx:+.3f}',
            'sent_wz': f'{g.out_wz:+.3f}',
            'frames_sent': str(self.frames_sent),
            'filtered_battery_v': 'unknown' if g.voltage is None else f'{g.voltage:.2f}',
            'max_vx': str(g.max_vx),
            'max_run_time': str(g.max_run_time),
            'rejected_cmds': str(g.rejected_cmds),
        }
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.diag_pub.publish(arr)

    def close(self):
        """Send stop frames (if commands were enabled), then close the port."""
        if self.gate.enabled and not self.write_error:
            for _ in range(int(self.c['stop_frames_on_exit'])):
                try:
                    self.reader.ser.write(stop_command())
                    self.reader.ser.flush()
                except Exception:
                    break
        super().close()


def main():
    """Entry point."""
    rclpy.init()
    try:
        node = R550Base()
    except Exception as e:  # bad parameter, port missing or busy: nothing was sent
        print(f'r550_base: {e}', file=sys.stderr, flush=True)
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
