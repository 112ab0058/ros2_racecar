"""
N10P scan node: serial -> sensor_msgs/LaserScan on /scan. READ ONLY.

Never writes to the serial port (the N10P streams by itself). One message
per revolution, fixed number of beams (parameter `bins`). Protocol and
caveats: see protocol.py.
"""
import math
import sys
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .protocol import BAUD_RATE, PacketStream, ScanAssembler

# name, default, description. Defaults must match config/n10p.yaml.
PARAMETERS = [
    ('port', '', 'serial device of the N10P adapter; required'),
    ('baud', BAUD_RATE, 'serial baud rate (official driver: 460800 for N10_P)'),
    ('frame_id', 'laser_link', 'frame of /scan'),
    ('bins', 720, 'beams per scan (720 = 0.5 deg); must stay constant for slam_toolbox'),
    ('range_min', 0.1, 'm; closer returns are dropped (N10P minimum range ~0.1 m)'),
    ('range_max', 25.0, 'm; farther returns are dropped'),
    ('angle_offset_deg', 0.0, 'added to every angle (lidar zero vs laser_link x axis)'),
    ('scan_timeout', 1.0, '/diagnostics ERROR when no scan for this long (s)'),
]


class N10PScan(Node):
    """ROS wiring around PacketStream / ScanAssembler."""

    def __init__(self):
        super().__init__('n10p_scan')
        self.p = {n: self.declare_parameter(n, d).value for n, d, _ in PARAMETERS}
        if not self.p['port']:
            raise ValueError("parameter 'port' is empty: pass port:=/dev/... "
                             '(find it with: ls -l /dev/serial/by-id/)')
        import serial
        self.ser = serial.Serial(self.p['port'], self.p['baud'], timeout=0.05)
        self.stream = PacketStream()
        self.scans = ScanAssembler(self.p['bins'], self.p['range_min'], self.p['range_max'],
                                   self.p['angle_offset_deg'])
        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.lock = threading.Lock()
        self.last_scan_time = None
        self.scan_period = None
        self.read_error = None
        self.running = True
        self.create_timer(1.0, self._publish_diagnostics)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.get_logger().info(f"listening on {self.p['port']} @ {self.p['baud']} (READ ONLY)")
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:
                self.read_error = f'serial read failed: {e}'
                self.running = False
                return
            for pkt in self.stream.feed(chunk):
                ranges = self.scans.add(pkt)
                if ranges is not None:
                    self._publish_scan(ranges)

    def _publish_scan(self, ranges):
        now = time.monotonic()
        with self.lock:
            if self.last_scan_time is not None:
                self.scan_period = now - self.last_scan_time
            self.last_scan_time = now
            period = self.scan_period or 0.0
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['frame_id']
        msg.angle_min = 0.0
        msg.angle_increment = 2.0 * math.pi / self.p['bins']
        msg.angle_max = msg.angle_increment * (self.p['bins'] - 1)
        msg.scan_time = period
        msg.time_increment = period / self.p['bins']
        msg.range_min = float(self.p['range_min'])
        msg.range_max = float(self.p['range_max'])
        msg.ranges = [float(r) for r in ranges]
        try:
            self.pub.publish(msg)
        except Exception:
            pass

    def _publish_diagnostics(self):
        now = time.monotonic()
        with self.lock:
            last, period = self.last_scan_time, self.scan_period
        age = None if last is None else now - last
        status = DiagnosticStatus()
        status.name = 'n10p_scan: lidar'
        status.hardware_id = 'lslidar_n10p'
        if self.read_error:
            status.level, status.message = DiagnosticStatus.ERROR, self.read_error
        elif age is None or age > self.p['scan_timeout']:
            status.level, status.message = DiagnosticStatus.ERROR, 'no scan'
        else:
            status.level, status.message = DiagnosticStatus.OK, 'ok'
        values = {
            'mode': 'READ ONLY',
            'scan_rate_hz': 'unknown' if not period else f'{1.0 / period:.1f}',
            'scan_age_s': 'none' if age is None else f'{age:.2f}',
            'bad_packets': str(self.stream.bad_packets),
            'dropped_bytes': str(self.stream.dropped_bytes),
        }
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.diag_pub.publish(arr)

    def close(self):
        """Stop reading and close the port."""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.ser.close()


def main():
    """Entry point."""
    rclpy.init()
    try:
        node = N10PScan()
    except Exception as e:
        print(f'n10p_scan: {e}', file=sys.stderr, flush=True)
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
