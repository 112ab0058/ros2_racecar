#!/usr/bin/env python3

import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


EXPECTED_CONTROLLER_SHA = (
    "de79e41c18748b2a20b897ec320c13848533a25a3adf6be5d690822609408cc2"
)
OUT = Path("/tmp/phase7d_work")
STATUS_PATH = OUT / "motion_status.json"
TIMELINE_PATH = OUT / "motion_timeline.csv"
ROUTE_TIMEOUT_SIM_SECONDS = 180.0
STOP_SETTLE_SIM_SECONDS = 2.0


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def controller_class():
    controller_path = (
        Path(get_package_prefix("wro2026_sim"))
        / "lib"
        / "wro2026_sim"
        / "line_follower.py"
    )
    actual_sha = hashlib.sha256(controller_path.read_bytes()).hexdigest()
    if actual_sha != EXPECTED_CONTROLLER_SHA:
        raise RuntimeError(
            f"controller identity mismatch: {actual_sha} != "
            f"{EXPECTED_CONTROLLER_SHA}"
        )
    spec = importlib.util.spec_from_file_location(
        "phase7d_final_accepted_controller", controller_path
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.SquareCourseFollower, str(controller_path), actual_sha


class RouteMonitor(Node):
    def __init__(self, controller_path, controller_sha):
        super().__init__("phase7d_mapping_route_monitor")
        self.controller_path = controller_path
        self.controller_sha = controller_sha
        self.clock_seconds = None
        self.route_start_sim = None
        self.route_complete_sim = None
        self.stop_sim = None
        self.stop_settled = False
        self.stop_requested = False
        self.route_result = "PENDING"
        self.stage = 0
        self.stage_names = [
            "WAIT_ROUTE_START",
            "EAST_REACHED",
            "NORTH_EAST_REACHED",
            "NORTH_WEST_REACHED",
            "SOUTH_WEST_REACHED",
            "FULL_LAP_COMPLETE",
        ]
        self.stage_history = []
        self.last_xy = None
        self.xy = None
        self.path_m = 0.0
        self.odom_count = 0
        self.cmd_count = 0
        self.nonzero_cmd_count = 0
        self.zero_cmd_count = 0
        self.last_command = [0.0, 0.0]
        self.maximum_publishers_seen = 0
        self.failure_reason = None
        self.wall_start = time.time()
        self.create_subscription(
            Clock, "/clock", self.clock_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_timer(0.10, self.tick)
        self.write_status()

    def add_stage(self, name):
        self.stage_history.append(
            {
                "stage": name,
                "clock_seconds": self.clock_seconds,
                "x": None if self.xy is None else self.xy[0],
                "y": None if self.xy is None else self.xy[1],
                "path_m": self.path_m,
            }
        )

    def clock_callback(self, message):
        self.clock_seconds = stamp_seconds(message.clock)

    def odom_callback(self, message):
        current = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        if self.last_xy is not None:
            delta = math.hypot(
                current[0] - self.last_xy[0], current[1] - self.last_xy[1]
            )
            if delta <= 0.20:
                self.path_m += delta
        self.last_xy = current
        self.xy = current
        self.odom_count += 1

        if self.route_start_sim is None:
            return
        x, y = current
        if self.stage == 0 and x > 0.35:
            self.stage = 1
            self.add_stage("EAST_REACHED")
        elif self.stage == 1 and x > 0.20 and y > 0.45:
            self.stage = 2
            self.add_stage("NORTH_EAST_REACHED")
        elif self.stage == 2 and x < -0.25 and y > 0.45:
            self.stage = 3
            self.add_stage("NORTH_WEST_REACHED")
        elif self.stage == 3 and x < -0.25 and y < 0.15:
            self.stage = 4
            self.add_stage("SOUTH_WEST_REACHED")
        elif (
            self.stage == 4
            and x > -0.05
            and y < 0.15
            and self.path_m >= 3.0
        ):
            self.stage = 5
            self.route_result = "FULL_NO_OBSTACLE_ROUTE_COMPLETE"
            self.route_complete_sim = self.clock_seconds
            self.stop_requested = True
            self.add_stage("FULL_LAP_COMPLETE")

    def cmd_callback(self, message):
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        self.cmd_count += 1
        self.last_command = [linear, angular]
        if abs(linear) > 1e-12 or abs(angular) > 1e-12:
            self.nonzero_cmd_count += 1
            if self.route_start_sim is None and self.clock_seconds is not None:
                self.route_start_sim = self.clock_seconds
                self.add_stage("ROUTE_START")
        else:
            self.zero_cmd_count += 1

    def mark_stopped(self):
        self.stop_sim = self.clock_seconds
        if self.route_result == "PENDING":
            self.route_result = "STOPPED_WITH_MOTION_GAP"
        self.write_status()

    def tick(self):
        if (
            self.route_start_sim is not None
            and not self.stop_requested
            and self.clock_seconds is not None
            and self.clock_seconds - self.route_start_sim
            >= ROUTE_TIMEOUT_SIM_SECONDS
        ):
            self.failure_reason = "route_timeout"
            self.route_result = "ROUTE_TIMEOUT_MOTION_GAP"
            self.stop_requested = True
        if (
            self.stop_sim is not None
            and self.clock_seconds is not None
            and self.clock_seconds - self.stop_sim >= STOP_SETTLE_SIM_SECONDS
        ):
            self.stop_settled = True
        self.write_status()

    def status(self):
        return {
            "controller_path": self.controller_path,
            "controller_sha": self.controller_sha,
            "route_result": self.route_result,
            "failure_reason": self.failure_reason,
            "route_started": self.route_start_sim is not None,
            "route_start_sim": self.route_start_sim,
            "route_complete_sim": self.route_complete_sim,
            "stop_sim": self.stop_sim,
            "stop_settled": self.stop_settled,
            "stage": self.stage,
            "stage_name": self.stage_names[self.stage],
            "stage_history": self.stage_history,
            "path_m": self.path_m,
            "xy": self.xy,
            "clock_seconds": self.clock_seconds,
            "odom_count": self.odom_count,
            "cmd_count": self.cmd_count,
            "nonzero_cmd_count": self.nonzero_cmd_count,
            "zero_cmd_count": self.zero_cmd_count,
            "last_command": self.last_command,
            "wall_elapsed_seconds": time.time() - self.wall_start,
        }

    def write_status(self):
        STATUS_PATH.write_text(
            json.dumps(self.status(), indent=2, sort_keys=True) + "\n"
        )

    def write_timeline(self):
        with TIMELINE_PATH.open("w", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=["stage", "clock_seconds", "x", "y", "path_m"],
            )
            writer.writeheader()
            writer.writerows(self.stage_history)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    follower_class, controller_path, controller_sha = controller_class()
    rclpy.init()
    controller = follower_class()
    monitor = RouteMonitor(controller_path, controller_sha)
    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(monitor)
    controller_in_executor = True
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            if monitor.stop_requested and controller_in_executor:
                controller._stop()
                executor.remove_node(controller)
                controller_in_executor = False
                monitor.mark_stopped()
            if monitor.stop_settled:
                monitor.write_status()
    except KeyboardInterrupt:
        pass
    finally:
        if controller_in_executor:
            controller._stop()
            executor.remove_node(controller)
        monitor.write_status()
        monitor.write_timeline()
        executor.remove_node(monitor)
        controller.destroy_node()
        monitor.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
