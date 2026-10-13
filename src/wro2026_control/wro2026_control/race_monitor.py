#!/usr/bin/env python3
"""Simulation run monitor: prints one RESULT line and exits.

Watches /race_status (controller), /odom (in simulation: ground truth) and
/scan.  Reports laps, time, minimum body-to-obstacle clearance seen in the
scans, and the distance between the stop pose and the start pose.  Exits
when the controller reports DONE or the time limit passes.  Simulation
only: it does not command the robot.
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# TurtleBot3 burger footprint in base_footprint (matches core.Params defaults)
LIDAR_X = -0.032
X0, X1, HW = LIDAR_X - 0.07, LIDAR_X + 0.07, 0.09


class RaceMonitor(Node):
    def __init__(self):
        super().__init__("race_monitor")
        self.declare_parameter("time_limit", 300.0)
        self.declare_parameter("start_x", -0.05)
        self.declare_parameter("start_y", -0.50)
        self.declare_parameter("status_topic", "/race_status")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.create_subscription(String, str(self.get_parameter("status_topic").value),
                                 self._status_cb, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value),
                                 self._odom_cb, 20)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value),
                                 self._scan_cb, qos_profile_sensor_data)
        self.t0 = None
        self.status = {}
        self.pose = None
        self.min_clear = math.inf
        self.finished = False
        self.create_timer(0.5, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _status_cb(self, msg: String):
        self.status = json.loads(msg.data)
        if self.t0 is None and self.status.get("state") not in (None, "WAIT"):
            self.t0 = self._now()

    def _odom_cb(self, msg: Odometry):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _scan_cb(self, msg: LaserScan):
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + msg.angle_increment * np.arange(r.size)
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        x = LIDAR_X + r[ok] * np.cos(a[ok])
        y = r[ok] * np.sin(a[ok])
        dx = np.maximum(np.maximum(X0 - x, x - X1), 0.0)
        dy = np.maximum(np.abs(y) - HW, 0.0)
        if x.size:
            self.min_clear = min(self.min_clear, float(np.hypot(dx, dy).min()))

    def _tick(self):
        if self.finished or self.t0 is None:
            return
        elapsed = self._now() - self.t0
        done = self.status.get("state") == "DONE"
        limit = float(self.get_parameter("time_limit").value)
        if not done and elapsed < limit:
            return
        sx = float(self.get_parameter("start_x").value)
        sy = float(self.get_parameter("start_y").value)
        offset = None if self.pose is None else math.hypot(self.pose[0] - sx, self.pose[1] - sy)
        corners = int(self.status.get("corners", 0))
        laps = corners // 4
        reason = self.status.get("reason", "")
        ok = done and laps >= 3 and reason == "laps complete" and self.min_clear > 0.0
        result = {
            "result": "PASS" if ok else "FAIL",
            "reason": reason if done else "time limit",
            "laps": laps, "corners": corners,
            "time_s": round(elapsed, 1),
            "min_clearance_mm": round(self.min_clear * 1000, 1),
            "stop_offset_cm": None if offset is None else round(offset * 100, 1),
            "backoffs": self.status.get("backoffs"),
            "direction": self.status.get("direction"),
        }
        self.get_logger().info("RESULT " + json.dumps(result))
        print("RESULT " + json.dumps(result), flush=True)
        self.finished = True
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = RaceMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
