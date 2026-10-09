#!/usr/bin/env python3
"""Phase 7 simulation-only differential-drive mapping traversal.

This node is solely for SLAM map acquisition with the current simulated
TurtleBot3. It is not a V550 Ackermann controller and must never be used as the
Final Accepted competition route controller.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time
import traceback

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import yaml


STATUS_PATH = Path("/tmp/phase7dr_work/traversal_status.json")
MOTION_STATUS_PATH = Path("/tmp/phase7dr_work/motion_status.json")
TIMELINE_PATH = Path("/tmp/phase7dr_work/traversal_timeline.csv")
WAYPOINT_RESULTS_PATH = Path(
    "/tmp/phase7dr_work/waypoint_runtime_results.csv"
)


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def distance_to_rectangle(x, y, rectangle):
    dx = max(rectangle["x_min"] - x, 0.0, x - rectangle["x_max"])
    dy = max(rectangle["y_min"] - y, 0.0, y - rectangle["y_max"])
    if dx == 0.0 and dy == 0.0:
        return -min(
            x - rectangle["x_min"],
            rectangle["x_max"] - x,
            y - rectangle["y_min"],
            rectangle["y_max"] - y,
        )
    return math.hypot(dx, dy)


def static_footprint_clearance(world_x, world_y, safety):
    radius = float(safety["footprint_radius_m"])
    outer = float(safety["outer_wall_inner_bound_m"])
    outer_clearance = min(
        outer - abs(world_x), outer - abs(world_y)
    ) - radius
    island_clearance = (
        distance_to_rectangle(
            world_x,
            world_y,
            safety["central_island_envelope"],
        )
        - radius
    )
    if outer_clearance <= island_clearance:
        return outer_clearance, "outer_wall_inner_face"
    return island_clearance, "central_island_envelope"


def zero_command():
    return 0.0, 0.0


class TraversalCore:
    """Single deterministic waypoint state machine with no avoidance/recovery."""

    TERMINAL_STATES = {"COMPLETE", "FAILED", "SHUTDOWN"}

    def __init__(self, config):
        self.config = config
        self.waypoints = list(config["route"]["waypoints"])
        self.initial = config["initial_pose"]
        self.limits = config["limits"]
        self.tolerances = config["tolerances"]
        self.timeouts = config["timeouts"]
        self.safety = config["safety"]
        self.state = "WAIT_READY"
        self.waypoint_index = 0
        self.route_started = False
        self.route_start_sim = None
        self.route_complete_sim = None
        self.stop_sim = None
        self.stop_settled = False
        self.failure_reason = None
        self.route_result = "PENDING"
        self.last_command = [0.0, 0.0]
        self.path_m = 0.0
        self.last_observed_xy = None
        self.segment_start_sim = None
        self.segment_start_path_m = 0.0
        self.segment_start_xy = None
        self.state_history = []
        self.waypoint_results = []
        self.min_static_clearance_m = None
        self.min_static_clearance_pair = None
        self.min_static_clearance_pose = None
        self.final_pose = None
        self.final_position_error_m = None
        self.final_yaw_error_rad = None
        self.shutdown_requested = False

    def transition(self, new_state, now_sim, detail=""):
        if self.state == new_state:
            return
        self.state = new_state
        self.state_history.append(
            {
                "state": new_state,
                "clock_seconds": now_sim,
                "waypoint_index": self.waypoint_index,
                "detail": detail,
            }
        )

    def observe_pose(self, x, y):
        current = (x, y)
        if self.route_started and self.last_observed_xy is not None:
            delta = math.hypot(
                current[0] - self.last_observed_xy[0],
                current[1] - self.last_observed_xy[1],
            )
            if delta <= 0.20:
                self.path_m += delta
        self.last_observed_xy = current

        world_x = x + float(self.initial["world_x"])
        world_y = y + float(self.initial["world_y"])
        clearance, pair = static_footprint_clearance(
            world_x, world_y, self.safety
        )
        if (
            self.min_static_clearance_m is None
            or clearance < self.min_static_clearance_m
        ):
            self.min_static_clearance_m = clearance
            self.min_static_clearance_pair = pair
            self.min_static_clearance_pose = {
                "odom_x": x,
                "odom_y": y,
                "world_x": world_x,
                "world_y": world_y,
            }
        return clearance

    def fail(self, reason, now_sim):
        if self.state in self.TERMINAL_STATES:
            return zero_command()
        self.failure_reason = reason
        self.route_result = "FAILED"
        self.stop_sim = now_sim
        self.transition("FAILED", now_sim, reason)
        self.last_command = [0.0, 0.0]
        return zero_command()

    def request_shutdown(self, now_sim):
        self.shutdown_requested = True
        if self.state not in {"COMPLETE", "FAILED"}:
            self.route_result = "SHUTDOWN"
            self.transition("SHUTDOWN", now_sim, "shutdown_requested")
        self.last_command = [0.0, 0.0]
        return zero_command()

    def start_route(self, now_sim, x, y, yaw):
        start_position_error = math.hypot(
            x - float(self.initial["x"]),
            y - float(self.initial["y"]),
        )
        start_yaw_error = abs(
            wrap_angle(yaw - float(self.initial["yaw"]))
        )
        if start_position_error > float(
            self.tolerances["initial_position_m"]
        ):
            return self.fail("initial_position_out_of_tolerance", now_sim)
        if start_yaw_error > float(self.tolerances["initial_yaw_rad"]):
            return self.fail("initial_yaw_out_of_tolerance", now_sim)
        self.route_started = True
        self.route_start_sim = now_sim
        self.segment_start_sim = now_sim
        self.segment_start_path_m = self.path_m
        self.segment_start_xy = [x, y]
        self.last_observed_xy = (x, y)
        self.transition(
            "ALIGN_PATH",
            now_sim,
            f"target={self.waypoints[0]['id']}",
        )
        return zero_command()

    def segment_timeout_seconds(self):
        expected = float(
            self.waypoints[self.waypoint_index][
                "expected_segment_length_m"
            ]
        )
        return float(self.timeouts["segment_base_sim_s"]) + expected * float(
            self.timeouts["segment_per_meter_sim_s"]
        )

    def finish_waypoint(self, now_sim, x, y, yaw):
        waypoint = self.waypoints[self.waypoint_index]
        position_error = math.hypot(
            float(waypoint["x"]) - x,
            float(waypoint["y"]) - y,
        )
        yaw_error = abs(
            wrap_angle(float(waypoint["target_yaw"]) - yaw)
        )
        actual_segment_path = self.path_m - self.segment_start_path_m
        self.waypoint_results.append(
            {
                "index": self.waypoint_index,
                "id": waypoint["id"],
                "side": waypoint["side"],
                "target_x": float(waypoint["x"]),
                "target_y": float(waypoint["y"]),
                "target_yaw": float(waypoint["target_yaw"]),
                "actual_x": x,
                "actual_y": y,
                "actual_yaw": yaw,
                "position_error_m": position_error,
                "yaw_error_rad": yaw_error,
                "expected_segment_length_m": float(
                    waypoint["expected_segment_length_m"]
                ),
                "actual_segment_path_m": actual_segment_path,
                "segment_elapsed_sim_s": now_sim
                - float(self.segment_start_sim),
                "result": "PASS",
            }
        )
        if self.waypoint_index + 1 >= len(self.waypoints):
            self.route_complete_sim = now_sim
            self.stop_sim = now_sim
            self.final_pose = [x, y, yaw]
            self.final_position_error_m = math.hypot(
                x - float(self.initial["x"]),
                y - float(self.initial["y"]),
            )
            self.final_yaw_error_rad = abs(
                wrap_angle(yaw - float(self.initial["yaw"]))
            )
            self.route_result = "COMPLETE_LOOP"
            self.transition(
                "STOP_SETTLE",
                now_sim,
                "all_waypoints_complete",
            )
            return
        self.waypoint_index += 1
        self.segment_start_sim = now_sim
        self.segment_start_path_m = self.path_m
        self.segment_start_xy = [x, y]
        self.transition(
            "ALIGN_PATH",
            now_sim,
            f"target={self.waypoints[self.waypoint_index]['id']}",
        )

    def step(
        self,
        now_sim,
        x,
        y,
        yaw,
        *,
        odom_fresh,
        tf_ready,
    ):
        clearance = self.observe_pose(x, y)
        if clearance < float(self.safety["runtime_stop_clearance_m"]):
            return self.fail("runtime_static_clearance_below_stop", now_sim)

        if self.state == "WAIT_READY":
            if odom_fresh and tf_ready:
                return self.start_route(now_sim, x, y, yaw)
            self.last_command = [0.0, 0.0]
            return zero_command()

        if self.state in self.TERMINAL_STATES:
            self.last_command = [0.0, 0.0]
            return zero_command()

        if self.state == "STOP_SETTLE":
            if now_sim - float(self.stop_sim) >= float(
                self.timeouts["stop_settle_sim_s"]
            ):
                self.stop_settled = True
                self.transition(
                    "COMPLETE",
                    now_sim,
                    "stop_settle_complete",
                )
            self.last_command = [0.0, 0.0]
            return zero_command()

        if not odom_fresh:
            return self.fail("stale_odom", now_sim)
        if not tf_ready:
            return self.fail("required_tf_unavailable", now_sim)
        if now_sim - float(self.route_start_sim) > float(
            self.timeouts["overall_sim_s"]
        ):
            return self.fail("overall_timeout", now_sim)
        if now_sim - float(self.segment_start_sim) > (
            self.segment_timeout_seconds()
        ):
            return self.fail("segment_timeout", now_sim)

        waypoint = self.waypoints[self.waypoint_index]
        dx = float(waypoint["x"]) - x
        dy = float(waypoint["y"]) - y
        distance = math.hypot(dx, dy)

        if self.state == "ALIGN_PATH":
            path_yaw = math.atan2(dy, dx)
            error = wrap_angle(path_yaw - yaw)
            if abs(error) <= float(self.tolerances["yaw_rad"]):
                self.transition(
                    "DRIVE_SEGMENT",
                    now_sim,
                    f"target={waypoint['id']}",
                )
                self.last_command = [0.0, 0.0]
                return zero_command()
            angular = clamp(
                float(self.limits["rotate_gain"]) * error,
                -float(self.limits["angular_speed_radps"]),
                float(self.limits["angular_speed_radps"]),
            )
            self.last_command = [0.0, angular]
            return 0.0, angular

        if self.state == "DRIVE_SEGMENT":
            if distance <= float(self.tolerances["position_m"]):
                self.transition(
                    "ALIGN_TARGET_YAW",
                    now_sim,
                    f"target={waypoint['id']}",
                )
                self.last_command = [0.0, 0.0]
                return zero_command()
            path_yaw = math.atan2(dy, dx)
            heading_error = wrap_angle(path_yaw - yaw)
            angular = clamp(
                float(self.limits["drive_heading_gain"]) * heading_error,
                -float(
                    self.limits["drive_heading_angular_limit_radps"]
                ),
                float(
                    self.limits["drive_heading_angular_limit_radps"]
                ),
            )
            if abs(heading_error) > float(
                self.limits["heading_stop_threshold_rad"]
            ):
                linear = 0.0
            else:
                linear = min(
                    float(self.limits["linear_speed_mps"]),
                    max(
                        float(self.limits["minimum_drive_speed_mps"]),
                        0.8 * distance,
                    ),
                )
            self.last_command = [linear, angular]
            return linear, angular

        if self.state == "ALIGN_TARGET_YAW":
            yaw_error = wrap_angle(
                float(waypoint["target_yaw"]) - yaw
            )
            if abs(yaw_error) <= float(self.tolerances["yaw_rad"]):
                self.finish_waypoint(now_sim, x, y, yaw)
                self.last_command = [0.0, 0.0]
                return zero_command()
            angular = clamp(
                float(self.limits["rotate_gain"]) * yaw_error,
                -float(self.limits["angular_speed_radps"]),
                float(self.limits["angular_speed_radps"]),
            )
            self.last_command = [0.0, angular]
            return 0.0, angular

        return self.fail("unknown_state", now_sim)

    def status(self, now_sim, x, y, yaw, odom_fresh, tf_ready):
        return {
            "scope": self.config["scope"],
            "state": self.state,
            "route_started": self.route_started,
            "route_start_sim": self.route_start_sim,
            "route_complete_sim": self.route_complete_sim,
            "route_result": self.route_result,
            "failure_reason": self.failure_reason,
            "stop_sim": self.stop_sim,
            "stop_settled": self.stop_settled,
            "waypoint_index": self.waypoint_index,
            "waypoint_count": len(self.waypoints),
            "waypoint_results": self.waypoint_results,
            "state_history": self.state_history,
            "path_m": self.path_m,
            "expected_polyline_length_m": float(
                self.config["route"]["expected_polyline_length_m"]
            ),
            "pose": [x, y, yaw],
            "final_pose": self.final_pose,
            "final_position_error_m": self.final_position_error_m,
            "final_yaw_error_rad": self.final_yaw_error_rad,
            "clock_seconds": now_sim,
            "odom_fresh": odom_fresh,
            "tf_ready": tf_ready,
            "last_command": self.last_command,
            "min_static_clearance_m": self.min_static_clearance_m,
            "min_static_clearance_pair": self.min_static_clearance_pair,
            "min_static_clearance_pose": self.min_static_clearance_pose,
            "shutdown_requested": self.shutdown_requested,
        }


class MappingTraversalNode(Node):
    def __init__(self, config_path):
        super().__init__("phase7_mapping_traversal")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        document = yaml.safe_load(Path(config_path).read_text())
        self.config = document["mapping_traversal"]
        self.core = TraversalCore(self.config)
        self.pose = None
        self.last_odom_wall = None
        self.wall_start = time.monotonic()
        self.last_history_count = 0
        self.nonzero_command_count = 0
        self.first_nonzero_sim = None
        self.publisher = self.create_publisher(
            Twist, self.config["topics"]["cmd_vel"], 10
        )
        self.create_subscription(
            Odometry,
            self.config["topics"]["odom"],
            self.odom_callback,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().warning(
            "PHASE7_MAPPING_ONLY: simulation TurtleBot3 differential-drive "
            "SLAM traversal; not V550 Ackermann and not Final Accepted route"
        )

    def odom_callback(self, message):
        position = message.pose.pose.position
        self.pose = (
            float(position.x),
            float(position.y),
            quaternion_yaw(message.pose.pose.orientation),
        )
        self.last_odom_wall = time.monotonic()

    def tf_ready(self):
        try:
            return self.tf_buffer.can_transform(
                self.config["frames"]["odom"],
                self.config["frames"]["base"],
                Time(),
            ) and self.tf_buffer.can_transform(
                self.config["frames"]["base"],
                self.config["frames"]["scan"],
                Time(),
            )
        except Exception:
            return False

    def publish(self, linear, angular):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.publisher.publish(message)

    def write_outputs(
        self,
        now_sim,
        x,
        y,
        yaw,
        odom_fresh,
        tf_ready,
    ):
        status = self.core.status(
            now_sim, x, y, yaw, odom_fresh, tf_ready
        )
        status["nonzero_command_count"] = self.nonzero_command_count
        status["first_nonzero_sim"] = self.first_nonzero_sim
        payload = json.dumps(status, indent=2, sort_keys=True) + "\n"
        for path in (STATUS_PATH, MOTION_STATUS_PATH):
            temporary = path.with_suffix(path.suffix + ".tmp")
            temporary.write_text(payload)
            temporary.replace(path)
        with TIMELINE_PATH.open("w", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "state",
                    "clock_seconds",
                    "waypoint_index",
                    "detail",
                ],
            )
            writer.writeheader()
            writer.writerows(self.core.state_history)
        with WAYPOINT_RESULTS_PATH.open("w", newline="") as stream:
            fieldnames = [
                "index",
                "id",
                "side",
                "target_x",
                "target_y",
                "target_yaw",
                "actual_x",
                "actual_y",
                "actual_yaw",
                "position_error_m",
                "yaw_error_rad",
                "expected_segment_length_m",
                "actual_segment_path_m",
                "segment_elapsed_sim_s",
                "result",
            ]
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.core.waypoint_results)

    def timer_callback(self):
        now_sim = self.get_clock().now().nanoseconds / 1e9
        if self.pose is None:
            if (
                time.monotonic() - self.wall_start
                > float(self.config["timeouts"]["readiness_wall_s"])
            ):
                self.core.fail("odom_readiness_timeout", now_sim)
            self.publish(*zero_command())
            self.write_outputs(
                now_sim, 0.0, 0.0, 0.0, False, False
            )
            return
        x, y, yaw = self.pose
        odom_fresh = (
            self.last_odom_wall is not None
            and time.monotonic() - self.last_odom_wall
            <= float(
                self.config["timeouts"]["odom_freshness_wall_s"]
            )
        )
        tf_ready = self.tf_ready()
        try:
            command = self.core.step(
                now_sim,
                x,
                y,
                yaw,
                odom_fresh=odom_fresh,
                tf_ready=tf_ready,
            )
        except Exception as exception:
            self.get_logger().error(
                f"mapping traversal exception: {exception}\n"
                + traceback.format_exc()
            )
            command = self.core.fail("exception", now_sim)
        if abs(command[0]) > 1e-12 or abs(command[1]) > 1e-12:
            self.nonzero_command_count += 1
            if self.first_nonzero_sim is None:
                self.first_nonzero_sim = now_sim
        self.publish(*command)
        self.write_outputs(
            now_sim, x, y, yaw, odom_fresh, tf_ready
        )

    def safe_shutdown(self):
        now_sim = self.get_clock().now().nanoseconds / 1e9
        self.core.request_shutdown(now_sim)
        try:
            self.publish(*zero_command())
        except Exception:
            pass


def load_core(config_path):
    document = yaml.safe_load(Path(config_path).read_text())
    return TraversalCore(document["mapping_traversal"])


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    arguments, ros_arguments = parser.parse_known_args(argv)
    rclpy.init(args=ros_arguments)
    node = MappingTraversalNode(arguments.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
