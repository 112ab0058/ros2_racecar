"""WAVE ROVER ROS2 driver.

Subscribes /cmd_vel (geometry_msgs/Twist), converts it to left/right wheel
commands and sends Waveshare JSON {"T":1,"L":..,"R":..} to the ESP32.
L/R are NOT m/s: the WAVE ROVER firmware applies PWM = L * 512 (0.5 = full
power), so max_speed is a PWM fraction cap.

It also polls the ESP32 for feedback and publishes:
  /battery              sensor_msgs/BatteryState  (from T:130 -> T:1001 "v")
  /imu/data             sensor_msgs/Imu           (from T:126 -> T:1002)
  /wave_rover/feedback  std_msgs/String           (every raw JSON reply/line)
  /wave_rover/sent_cmd  std_msgs/String           (every T:1 actually sent)

Units (Waveshare ugv_base_general firmware, QMI8658_UINT_MG_DPS):
  r/p/y degrees, ax/ay/az mg, gx/gy/gz deg/s, mx/my/mz raw AK09918 counts.

Transport:
  serial (default): /dev/ttyTHS1 @ 115200 (Orin 40-pin: pin 8 TX, pin 10 RX, pin 6 GND)
  http:             http://<host>/js?json=...

All I/O runs in one worker thread so a slow HTTP request never blocks the
ROS executor. Wheel commands have priority over feedback polling.

Safety: if no /cmd_vel arrives within cmd_timeout seconds, it sends stop.
The ESP32 also stops by itself after ~3 s without commands.
"""
import json
import math
import threading
import time
import urllib.parse
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String

MAX_WHEEL = 0.5  # ESP32 accepts -0.5 .. 0.5 (= 100% PWM)
G = 9.80665


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class WaveRoverDriver(Node):
    def __init__(self):
        super().__init__('wave_rover_driver')
        p = self.declare_parameter
        self.transport = p('transport', 'serial').value
        self.port = p('port', '/dev/ttyTHS1').value
        self.baud = p('baud', 115200).value
        self.host = p('host', '192.168.4.1').value
        # distance between left and right wheels (m) - measure your car
        self.track = p('track_width', 0.17).value
        self.max_speed = min(abs(p('max_speed', 0.05).value), MAX_WHEEL)
        # serial: resend T:1 at send_rate; http: resend unchanged T:1 every
        # http_cmd_period (changed commands are sent immediately)
        self.rate = p('send_rate', 10.0).value
        self.http_cmd_period = p('http_cmd_period', 0.5).value
        self.http_timeout = p('http_timeout', 0.3).value
        self.cmd_timeout = p('cmd_timeout', 0.3).value
        self.feedback_period = p('feedback_period', 0.2).value
        self.poll_base = p('poll_base', True).value
        self.poll_imu = p('poll_imu', True).value
        self.serial_continuous_feedback = p('serial_continuous_feedback', False).value
        self.imu_frame = p('imu_frame_id', 'imu_link').value
        self.battery_frame = p('battery_frame_id', 'base_link').value
        self.low_battery = p('low_battery_voltage', 11.0).value

        if self.transport not in ('serial', 'http'):
            raise ValueError("transport must be 'serial' or 'http'")

        self.lock = threading.Lock()
        self.left = 0.0
        self.right = 0.0
        self.last_cmd = None  # time.monotonic() of last /cmd_vel
        self.running = True
        self.ser = None

        self.fb_pub = self.create_publisher(String, 'wave_rover/feedback', 10)
        self.sent_pub = self.create_publisher(String, 'wave_rover/sent_cmd', 10)
        self.bat_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        if self.transport == 'serial':
            import serial  # python3-serial
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f'Serial {self.port} @ {self.baud}')
            threading.Thread(target=self._read_loop, daemon=True).start()
            if self.serial_continuous_feedback:
                self._write_serial({'T': 131, 'cmd': 1})
        else:
            self.get_logger().info(f'HTTP http://{self.host}/js')

        self.get_logger().info(
            f'max_speed={self.max_speed} cmd_timeout={self.cmd_timeout}s '
            f'feedback_period={self.feedback_period}s track_width={self.track}')

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

    # ---------------------------------------------------------------- commands
    def _clamp(self, v):
        return max(-self.max_speed, min(self.max_speed, v))

    def _on_cmd_vel(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        with self.lock:
            self.left = self._clamp(v - w * self.track / 2.0)
            self.right = self._clamp(v + w * self.track / 2.0)
            self.last_cmd = time.monotonic()

    def _desired(self):
        with self.lock:
            if self.last_cmd is None or time.monotonic() - self.last_cmd > self.cmd_timeout:
                return 0.0, 0.0
            return round(self.left, 3), round(self.right, 3)

    # ------------------------------------------------------------------ worker
    def _worker_loop(self):
        cmd_period = 1.0 / self.rate if self.transport == 'serial' else self.http_cmd_period
        polls = []
        if self.poll_base:
            polls.append({'T': 130})
        if self.poll_imu:
            polls.append({'T': 126})
        next_poll = {json.dumps(c): 0.0 for c in polls}
        last_sent = None
        last_sent_time = 0.0

        while self.running and rclpy.ok():
            now = time.monotonic()
            desired = self._desired()
            if desired != last_sent or now - last_sent_time >= cmd_period:
                self._send_wheels(*desired)
                last_sent, last_sent_time = desired, now
                continue
            due = [k for k, t in next_poll.items() if t <= now]
            if due:
                key = min(due, key=lambda k: next_poll[k])
                next_poll[key] = now + self.feedback_period
                self._poll(json.loads(key))
                continue
            time.sleep(0.005)

    def _send_wheels(self, left, right):
        cmd = {'T': 1, 'L': left, 'R': right}
        t0 = time.time()
        ok = self._send(cmd) is not False
        rec = {'t': t0, 'dt': round(time.time() - t0, 4), 'ok': ok, 'cmd': cmd}
        try:
            self.sent_pub.publish(String(data=json.dumps(rec, separators=(',', ':'))))
        except Exception:
            pass
        return ok

    def _poll(self, cmd):
        reply = self._send(cmd)
        if isinstance(reply, str) and reply:
            self._handle_line(reply)

    def _send(self, cmd: dict):
        """Returns reply text (http), None (serial written) or False (failed)."""
        data = json.dumps(cmd, separators=(',', ':'))
        try:
            if self.ser:
                self._write_serial(cmd)
                return None
            url = f'http://{self.host}/js?json=' + urllib.parse.quote(data)
            return urllib.request.urlopen(url, timeout=self.http_timeout).read().decode(errors='ignore')
        except Exception as e:  # keep running, just report
            self.get_logger().warn(f'send {data} failed: {e}', throttle_duration_sec=2.0)
            return False

    def _write_serial(self, cmd: dict):
        data = json.dumps(cmd, separators=(',', ':'))
        self.ser.write((data + '\n').encode())

    def _read_loop(self):
        while self.running and rclpy.ok():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
            except Exception:
                continue
            if line:
                self._handle_line(line)

    # ---------------------------------------------------------------- feedback
    def _handle_line(self, line):
        line = line.strip()
        try:
            self.fb_pub.publish(String(data=line))
        except Exception:
            return
        try:
            msg = json.loads(line)
        except ValueError:
            return
        if not isinstance(msg, dict):
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
        b.header.frame_id = self.battery_frame
        b.voltage = float(v)
        nan = float('nan')
        b.current = b.charge = b.capacity = b.design_capacity = b.percentage = nan
        b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        b.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        b.present = True
        self.bat_pub.publish(b)
        if b.voltage < self.low_battery:
            self.get_logger().warn(f'battery low: {b.voltage:.2f} V', throttle_duration_sec=10.0)

    def _publish_imu(self, msg):
        try:
            r, p, y = (math.radians(float(msg[k])) for k in ('r', 'p', 'y'))
            gx, gy, gz = (math.radians(float(msg[k])) for k in ('gx', 'gy', 'gz'))
            ax, ay, az = (float(msg[k]) * G / 1000.0 for k in ('ax', 'ay', 'az'))
        except (KeyError, TypeError, ValueError):
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.imu_frame
        (m.orientation.x, m.orientation.y,
         m.orientation.z, m.orientation.w) = quaternion_from_rpy(r, p, y)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gx, gy, gz
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = ax, ay, az
        # covariances left at 0 (= unknown) until the IMU is characterised
        self.imu_pub.publish(m)

    # ---------------------------------------------------------------- shutdown
    def stop_and_close(self):
        self.running = False
        if self.worker.is_alive():
            self.worker.join(timeout=1.5)
        ok = False
        for _ in range(3):
            if self._send({'T': 1, 'L': 0, 'R': 0}) is not False:
                ok = True
                break
            time.sleep(0.1)
        text = f'shutdown stop sent: {"ok" if ok else "FAILED"}'
        try:
            self.get_logger().info(text)
        except Exception:
            print(text, flush=True)
        if self.ser:
            self.ser.close()


def main():
    rclpy.init()
    node = WaveRoverDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_and_close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
