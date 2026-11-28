"""
Keyboard teleop node: publishes /cmd_vel (Twist) at a FIXED rate from a timer.

    ros2 run wro2026_base_r550 teleop_node        (needs a real terminal, e.g. ssh)

Hold a key to drive, let go and it publishes 0 (see teleop.py for the keys
and how "let go" is detected). It never stops publishing while running:
0 is published too, so base_node sees a steady stream. Ctrl+C publishes
a few zeros and restores the terminal.

Why not `ros2 topic pub`: on 2026-10-19 it paused for > 0.3 s twice (with
several `ros2 topic echo` running), so base_node's timeout stopped the car.
A timer in our own node was steady (max gap 0.051 s at 20 Hz).

This node does not touch the serial port. Only base_node with
enable_commands:=true makes the motors turn, and its limits still apply.
"""
import os
import select
import sys
import termios
import threading
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .teleop import Teleop

HELP = """
  q w e    forward-left / forward / forward-right   (hold the key)
  s or space = STOP        z x c = back-left / back / back-right
  + / -  speed             Ctrl+C = quit (publishes 0)
  a / d do nothing: Ackermann cannot turn on the spot.
"""


class TeleopNode(Node):
    def __init__(self):
        super().__init__('r550_teleop')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('speed', 0.1), ('max_speed', 0.15), ('turn_radius', 0.5),
            ('rate', 20.0), ('first_hold', 0.6), ('release', 0.25))}
        self.teleop = Teleop(speed=p['speed'], max_speed=p['max_speed'],
                             turn_radius=p['turn_radius'], first_hold=p['first_hold'],
                             release=p['release'])
        self.lock = threading.Lock()
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.last = None
        self.timer = self.create_timer(1.0 / p['rate'], self._tick)
        self.get_logger().info(
            'publishing /cmd_vel at %.0f Hz; speed %.2f (max %.2f) m/s, turn radius %.2f m'
            % (p['rate'], p['speed'], p['max_speed'], p['turn_radius']))

    def on_key(self, ch):
        with self.lock:
            text = self.teleop.key(time.monotonic(), ch)
        if text:
            self._show(text)

    def _tick(self):
        with self.lock:
            vx, wz = self.teleop.command(time.monotonic())
        self.publish(vx, wz)
        if (vx, wz) != self.last:
            self._show('vx %+.2f m/s  wz %+.2f rad/s' % (vx, wz))
            self.last = (vx, wz)

    def publish(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    @staticmethod
    def _show(text):
        print('  ' + text, flush=True)


def read_keys(node, fd, stop):
    """Thread: read single key presses (terminal in cbreak mode)."""
    while not stop.is_set():
        ready, _, _ = select.select([fd], [], [], 0.05)
        if not ready:
            continue
        data = os.read(fd, 32).decode(errors='ignore')
        for ch in data:
            node.on_key(ch)


def main(args=None):
    if not sys.stdin.isatty():
        print('teleop_node needs a terminal (run it in ssh, not from a launch file).')
        return 1
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    # Our own Ctrl+C handling: rclpy's handler would shut the context down
    # before we could publish the final zeros.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopNode()
    stop = threading.Event()
    print(HELP)
    try:
        tty.setcbreak(fd)   # keys arrive one by one, not echoed; Ctrl+C still raises
        threading.Thread(target=read_keys, args=(node, fd, stop), daemon=True).start()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stop.set()
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        if rclpy.ok():
            for _ in range(5):
                node.publish(0.0, 0.0)
                time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
