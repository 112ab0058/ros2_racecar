"""
Simulated base: /cmd_vel -> the SAME CommandGate as the real base_node -> /sim/cmd_vel.

SIMULATION ONLY. Everything that drives the simulated car goes through the
real safety rules (0.3 s timeout, speed limits, ramp, no turning on the spot,
max_run_time), with the same default parameters as config/r550_base.yaml, so
teleop / Nav2 behave in Gazebo like on the car. Differences to the real car:
  - "feedback" = the simulated wheel odometry (/sim/odom) arriving
  - battery: a fixed simulated voltage (sim_battery_voltage, 12.0 V)
  - times are simulation time (use_sim_time true in sim.launch.py)
  - no vx_scale: Gazebo wheels have the real radius already
Publishes /cmd_vel_out (what was sent) like base_node.
"""
import sys

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from wro2026_base_r550.base_node import COMMAND_PARAMETERS
from wro2026_base_r550.command import CommandGate


class SimBase(Node):
    def __init__(self):
        super().__init__('r550_sim_base')
        c = {name: self.declare_parameter(name, default).value
             for name, default, _ in COMMAND_PARAMETERS
             if name not in ('enable_commands', 'stop_frames_on_exit')}
        self.voltage = self.declare_parameter('sim_battery_voltage', 12.0).value
        feedback_timeout = self.declare_parameter('feedback_timeout', 0.5).value
        self.gate = CommandGate(
            enabled=True, cmd_timeout=c['cmd_timeout'], max_vx=c['max_vx'],
            max_reverse_vx=c['max_reverse_vx'], max_wz=c['max_wz'], max_accel=c['max_accel'],
            min_turn_speed=c['min_turn_speed'], feedback_timeout=feedback_timeout,
            min_battery_voltage=c['min_battery_voltage'], max_run_time=c['max_run_time'])
        self.last_feedback = None
        self.reason = None
        self.out = self.create_publisher(Twist, 'sim/cmd_vel', 10)
        self.out_echo = self.create_publisher(Twist, 'cmd_vel_out', 10)
        if c['cmd_vel_stamped']:
            self.create_subscription(TwistStamped, 'cmd_vel', lambda m: self._cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self._cmd, 10)
        self.create_subscription(Odometry, 'sim/odom', self._feedback, 10)
        self.create_timer(1.0 / c['command_rate'], self._tick)
        self.get_logger().info('SIM base: /cmd_vel -> CommandGate -> /sim/cmd_vel (max_vx %.2f, '
                               'cmd_timeout %.2f s)' % (c['max_vx'], c['cmd_timeout']))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd(self, m):
        self.gate.set_cmd(self._now(), m.linear.x, m.angular.z)

    def _feedback(self, _m):
        self.last_feedback = self._now()

    def _tick(self):
        now = self._now()
        if now <= 0.0:          # no /clock yet
            return
        self.gate.set_voltage(now, self.voltage)
        vx, wz, reason = self.gate.update(now, self.last_feedback)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(vx), float(wz)
        self.out.publish(msg)
        self.out_echo.publish(msg)
        if reason != self.reason:
            self.get_logger().info('command state: %s' % reason)
            self.reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = SimBase()
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
