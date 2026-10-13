#!/usr/bin/env python3
"""ROS 2 wrapper for the pose-free race controller.

Subscribes:  /scan (LaserScan), /odom (Odometry), /line_detection
             (Float32MultiArray from line_detector.py), optional /imu (Imu)
Publishes:   /cmd_vel (Twist), /race_status (String, JSON)

The node only converts messages; every decision is in ``core.py``.

Odometry use: only yaw and the travelled path length are taken from /odom
(or yaw from /imu when ``use_imu_yaw`` is true).  The absolute x / y position
is never used for decisions, so the same node runs on Gazebo odom, wheel
odom or IMU dead reckoning.

Safety: the node publishes zero velocity when the scan or odometry is older
than ``sensor_timeout``, on a ROS time jump backwards (simulation reset, the
controller is also reset), and on shutdown.  It never moves before
``start`` is true (parameter, can be set at runtime with ``ros2 param set``).
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray, String

from wro2026_control.core import Params, RaceController, Scan, State

LD_ORANGE_TRIG = 3
LD_BLUE_TRIG = 7


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RaceNode(Node):
    def __init__(self):
        super().__init__("race_controller")
        defaults = Params()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter("start", True)
        self.declare_parameter("use_imu_yaw", False)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("sensor_timeout", 0.5)
        self.declare_parameter("line_timeout", 0.3)
        self.declare_parameter("max_linear", 0.20)    # final clamp on /cmd_vel
        self.declare_parameter("max_angular_out", 1.0)

        self.ctl = RaceController(self._params())
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/race_status", 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(Imu, "/imu", self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "/line_detection", self._line_cb, 10)

        self._scan: Scan | None = None
        self._scan_t = None
        self._odom_t = None
        self._odom_yaw = None
        self._imu_yaw = None
        self._imu_t = None
        self._last_xy = None
        self._s = 0.0
        self._orange = self._blue = False
        self._line_t = None
        self._last_now = None
        self._last_status_t = None

        period = 1.0 / float(self.get_parameter("control_rate").value)
        self.create_timer(period, self._tick)
        self.get_logger().info("race_controller ready (pose-free controller)")

    # ----------------------------------------------------------- parameters
    def _params(self) -> Params:
        p = Params()
        for name in vars(p):
            setattr(p, name, type(getattr(p, name))(self.get_parameter(name).value))
        return p

    def _on_params(self, params):
        for prm in params:
            if prm.name == "start" and prm.value:
                self.get_logger().info("start requested")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ callbacks
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan_cb(self, msg: LaserScan):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self._now()
        if stamp <= 0.0 or abs(now - stamp) > 1.0:
            stamp = now                       # missing or foreign clock: use arrival time
        self._scan = Scan(angle_min=msg.angle_min, angle_increment=msg.angle_increment,
                          ranges=list(msg.ranges), range_min=msg.range_min,
                          range_max=msg.range_max, stamp=stamp)
        self._scan_t = now

    def _odom_cb(self, msg: Odometry):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self._last_xy is not None:
            dx, dy = x - self._last_xy[0], y - self._last_xy[1]
            step = math.hypot(dx, dy)
            if step < 0.2:                      # ignore odometry resets / jumps
                forward = dx * math.cos(yaw) + dy * math.sin(yaw)
                self._s += step if forward >= 0.0 else -step
        self._last_xy = (x, y)
        self._odom_yaw = yaw
        self._odom_t = self._now()

    def _imu_cb(self, msg: Imu):
        self._imu_yaw = yaw_from_quaternion(msg.orientation)
        self._imu_t = self._now()

    def _line_cb(self, msg: Float32MultiArray):
        if len(msg.data) <= LD_BLUE_TRIG:
            return
        self._orange = msg.data[LD_ORANGE_TRIG] > 0.5
        self._blue = msg.data[LD_BLUE_TRIG] > 0.5
        self._line_t = self._now()

    # ----------------------------------------------------------------- tick
    def _publish(self, linear: float, angular: float):
        lim_v = float(self.get_parameter("max_linear").value)
        lim_w = float(self.get_parameter("max_angular_out").value)
        msg = Twist()
        msg.linear.x = max(-lim_v, min(lim_v, linear))
        msg.angular.z = max(-lim_w, min(lim_w, angular))
        self.cmd_pub.publish(msg)

    def _tick(self):
        now = self._now()
        if self._last_now is not None and now < self._last_now - 0.5:
            self.get_logger().warn("ROS time jumped backwards: resetting controller")
            self.ctl.reset()
            self._s, self._last_xy = 0.0, None
            self._scan = self._scan_t = self._odom_t = None
        self._last_now = now

        timeout = float(self.get_parameter("sensor_timeout").value)
        use_imu = bool(self.get_parameter("use_imu_yaw").value)
        yaw, yaw_t = ((self._imu_yaw, self._imu_t) if use_imu
                      else (self._odom_yaw, self._odom_t))
        fresh = (self._scan_t is not None and now - self._scan_t < timeout
                 and yaw_t is not None and now - yaw_t < timeout
                 and self._odom_t is not None and now - self._odom_t < timeout)
        if not fresh:
            self._publish(0.0, 0.0)
            return
        if bool(self.get_parameter("start").value):
            self.ctl.start()
        line_ok = self._line_t is not None and now - self._line_t < float(
            self.get_parameter("line_timeout").value)
        cmd = self.ctl.step(now, yaw, self._s, self._scan,
                            self._orange and line_ok, self._blue and line_ok)
        self._publish(cmd.linear, cmd.angular)

        if self._last_status_t is None or now - self._last_status_t > 0.5:
            self._last_status_t = now
            st = self.ctl.status().as_dict()
            self.status_pub.publish(String(data=json.dumps(st)))
            if self.ctl.state is State.DONE:
                self.get_logger().info(f"DONE: {st['reason']}", throttle_duration_sec=5.0)

    def stop(self):
        self._publish(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = RaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
