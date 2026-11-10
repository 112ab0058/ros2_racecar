"""
Drive from /scan with the imitation network, or with the keep-centred baseline.

    ros2 run wro2026_imitation bc_node --ros-args -p model:=$HOME/r550_map/model.npz
    ros2 run wro2026_imitation bc_node --ros-args -p policy:=center

By default (publish_cmd false) it only shows what it WOULD do on bc/decision and
never publishes /cmd_vel: safe next to sensors.launch.py, the motors cannot turn.

With publish_cmd true it also publishes /cmd_vel at a fixed rate from a timer
(like teleop_node). It never touches the serial port: only base_node with
enable_commands:=true moves the car, and base_node's limits and 0.3 s timeout
still apply. policy.Driver's rules come first (too close -> latched stop, no or
bad scan -> stop). Ctrl+C, kill -INT and kill -TERM publish a few zeros, then exit.

Speed and turn radius default to the ones of the recordings (stored in the model
by bc_train), so left / right drive the same curve the network learned from.

Publishes:
  bc/decision  std_msgs/Int8               -1 stop, 0 left, 1 straight, 2 right
  bc/probs     std_msgs/Float32MultiArray  network output for each scan (policy model)
  cmd_vel      geometry_msgs/Twist         only with publish_cmd true
"""
import collections
import signal
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int8

from .features import LABELS
from .model import MLP
from .policy import center_policy, check_model, Driver, model_policy, OK, STOP

# name, default, description
PARAMETERS = [
    ('policy', 'model', "'model' = network from bc_train, 'center' = keep-centred baseline"),
    ('model', '', 'model file from bc_train (policy model)'),
    ('publish_cmd', False, 'false = only bc/decision, no /cmd_vel; true = drive'),
    ('speed', 0.0, 'm/s; 0 = speed of the recordings (model) or 0.1 (center)'),
    ('turn_radius', 0.0, 'm; 0 = radius of the recordings (model) or 0.5 (center)'),
    ('max_speed', 0.15, 'refuse to start above this speed (m/s)'),
    ('min_turn_radius', 0.35, 'refuse to start below this commanded radius (m), as orin_map.sh; '
                              '0.35 = full steering lock, 0.40 m driven (2026-11-10)'),
    ('stop_margin', 0.04, 'stop for good when a lidar point is this close to the car (m)'),
    ('scan_timeout', 0.3, 'stop when the newest scan is older than this (s)'),
    ('rate', 20.0, 'bc/decision and cmd_vel messages per second'),
    ('front_dist', 0.5, 'center: turn to the open side when the front is closer (m)'),
    ('side_margin', 0.05, 'center: steer when one side is this much closer (m)'),
]


class BcNode(Node):
    """See the module docstring."""

    def __init__(self):
        super().__init__('bc_node')
        p = {name: self.declare_parameter(name, default).value
             for name, default, _ in PARAMETERS}
        speed, radius = p['speed'], p['turn_radius']
        if p['policy'] == 'model':
            if not p['model']:
                raise ValueError('policy model needs model:=<file from bc_train>')
            model, meta = MLP.load(p['model'])
            check_model(model, meta)
            decide = model_policy(model)
            speed = speed or meta.get('speed') or 0.0
            radius = radius or meta.get('turn_radius') or 0.0
        elif p['policy'] == 'center':
            decide = center_policy(p['front_dist'], p['side_margin'])
            speed, radius = speed or 0.1, radius or 0.5
        else:
            raise ValueError("policy must be 'model' or 'center', not %r" % p['policy'])
        if not 0.0 < speed <= p['max_speed']:
            raise ValueError('speed %.3f m/s is not in (0, max_speed %.2f]; set speed:='
                             % (speed, p['max_speed']))
        if radius < p['min_turn_radius']:
            raise ValueError('turn radius %.2f m is below min_turn_radius %.2f; set turn_radius:='
                             % (radius, p['min_turn_radius']))
        self.driver = Driver(decide, speed, radius, p['stop_margin'], p['scan_timeout'])
        self.publish_cmd = bool(p['publish_cmd'])
        self.counts = collections.Counter()
        self.last_reason = None

        self.decision_pub = self.create_publisher(Int8, 'bc/decision', 10)
        self.probs_pub = self.create_publisher(Float32MultiArray, 'bc/probs', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10) if self.publish_cmd else None
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / p['rate'], self._tick)
        mode = ('PUBLISHING /cmd_vel (base_node with enable_commands moves the car)'
                if self.publish_cmd else 'DRY RUN: bc/decision only, no /cmd_vel')
        self.get_logger().warn('%s; policy %s, speed %.3f m/s, turn radius %.2f m'
                               % (mode, p['policy'], speed, radius))

    def _on_scan(self, msg):
        self.driver.on_scan(time.monotonic(), msg.ranges, msg.angle_min, msg.angle_increment)
        if self.driver.probs is not None:
            self.probs_pub.publish(Float32MultiArray(data=[float(v) for v in self.driver.probs]))

    def _tick(self):
        vx, wz, decision, reason = self.driver.command(time.monotonic())
        self.decision_pub.publish(Int8(data=decision))
        if self.publish_cmd:
            self.publish(vx, wz)
        self.counts[decision] += 1
        if reason != self.last_reason:     # rclpy: one severity per logging call site
            if reason == OK:
                self.get_logger().info('state: ok')
            else:
                self.get_logger().warn('state: %s' % reason)
            self.last_reason = reason

    def publish(self, vx, wz):
        """Publish one /cmd_vel."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def finish(self):
        """Publish a few zeros (if driving) and print how often each decision was made."""
        if self.publish_cmd and rclpy.ok():
            for _ in range(5):
                self.publish(0.0, 0.0)
                time.sleep(0.05)
        total = max(sum(self.counts.values()), 1)
        names = {**dict(enumerate(LABELS)), STOP: 'stop'}
        print('bc_node decisions: ' + ', '.join(
            '%s %.0f %%' % (names[k], 100.0 * self.counts[k] / total)
            for k in (0, 1, 2, STOP)) + ' (%d ticks)' % total, flush=True)


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    """Entry point."""
    # Our own SIGINT / SIGTERM handling, so the final zeros can still be published.
    # Installed explicitly: a program started in the background by a script inherits
    # "ignore SIGINT", and then Ctrl+C / kill -INT did nothing (WSL, 2026-11-10).
    # The 20 Hz timer wakes spin(), so the handler runs within 0.05 s.
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = BcNode()
    except Exception as e:  # missing or wrong model, bad parameter: nothing was published
        print('bc_node: %s' % e, file=sys.stderr, flush=True)
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
