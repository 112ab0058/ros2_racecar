#!/usr/bin/env python3
"""
Step 6: LiDAR wall-centering + odom-yaw corner turn controller.

States: STRAIGHT -> TURN -> POST_TURN -> STRAIGHT
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

# Tunable parameters
Kp = 0.35
Ki = 0.0
Kd = 0.04

LINEAR_SPEED = 0.14
MAX_ANGULAR = 0.7
INTEGRAL_CLAMP = 3.0

LEFT_FRONT_ANGLE_DEG = 45.0
RIGHT_FRONT_ANGLE_DEG = 315.0
FRONT_ANGLE_DEG = 0.0
WINDOW_DEG = 8.0

# Fallback turn trigger from front LiDAR distance.
FRONT_TURN_DIST = 0.50

# Turn control. TURN completion is based on /odom yaw, not front distance.
TURN_LINEAR_SPEED = 0.06
TURN_ANGULAR_SPEED = 0.65
TURN_ANGLE_RAD = math.pi / 2.0
TURN_YAW_TOLERANCE_RAD = math.radians(8.0)
TURN_SLOWDOWN_REMAINING_RAD = math.radians(25.0)
TURN_TIMEOUT_SEC = 4.0

POST_TURN_SEC = 0.6

DEFAULT_TURN_DIRECTION = -1.0  # +1=left, -1=right

DEAD_ZONE_M = 0.03
MIN_VALID_RANGE = 0.05
MAX_VALID_RANGE = 10.0

# /line_detection indices from line_detector.py
LD_ORANGE_TRIG = 3
LD_BLUE_TRIG = 7
LD_DIRECTION = 8  # 1=CW, -1=CCW, 0=unknown


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q) -> float:
    """Extract yaw from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(target: float, current: float) -> float:
    """Shortest signed angular distance from current to target."""
    return normalize_angle(target - current)


class LidarPIDFollower(Node):
    def __init__(self):
        super().__init__("lidar_pid_follower")

        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.scan_cb, 10
        )
        self.sub_line = self.create_subscription(
            Float32MultiArray, "/line_detection", self.line_cb, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()

        self._state = "STRAIGHT"
        self._state_start_time = time.time()

        self._turn_direction = DEFAULT_TURN_DIRECTION
        self._turn_start_yaw = None
        self._turn_target_yaw = None
        self._line_dir = 0.0
        self._current_yaw = None

        self.get_logger().info("LiDAR PID Follower started (odom yaw turns)")

    # /odom callback
    def odom_cb(self, msg: Odometry):
        self._current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    # /line_detection callback
    def line_cb(self, msg: Float32MultiArray):
        if self._state != "STRAIGHT":
            return

        d = msg.data
        if len(d) < 9:
            return

        self._line_dir = d[LD_DIRECTION]
        orange_trig = d[LD_ORANGE_TRIG] > 0.5
        blue_trig = d[LD_BLUE_TRIG] > 0.5

        if self._line_dir == 1.0 and orange_trig:
            self._turn_direction = -1.0
            self._start_turn("line orange trigger, CW -> RIGHT")
            return

        if self._line_dir == -1.0 and blue_trig:
            self._turn_direction = 1.0
            self._start_turn("line blue trigger, CCW -> LEFT")
            return

        if (orange_trig or blue_trig) and self._line_dir == 0.0:
            self._turn_direction = DEFAULT_TURN_DIRECTION
            self._start_turn("line trigger with unknown direction -> DEFAULT")
            return

    # Helpers
    def _angle_to_index(self, msg: LaserScan, angle_deg: float) -> int | None:
        if msg.angle_increment <= 0.0 or len(msg.ranges) == 0:
            return None

        angle = math.radians(angle_deg)
        while angle < msg.angle_min:
            angle += 2.0 * math.pi
        while angle > msg.angle_max:
            angle -= 2.0 * math.pi

        if angle < msg.angle_min or angle > msg.angle_max:
            return None

        idx = int(round((angle - msg.angle_min) / msg.angle_increment))
        if idx < 0 or idx >= len(msg.ranges):
            return None
        return idx

    def _get_range(self, msg: LaserScan, angle_deg: float) -> float | None:
        center_idx = self._angle_to_index(msg, angle_deg)
        if center_idx is None:
            return None

        window_idx = max(1, int(round(math.radians(WINDOW_DEG) / msg.angle_increment)))
        n = len(msg.ranges)
        samples = []
        for i in range(center_idx - window_idx, center_idx + window_idx + 1):
            if 0 <= i < n:
                r = msg.ranges[i]
                if math.isfinite(r) and MIN_VALID_RANGE < r < MAX_VALID_RANGE:
                    samples.append(r)

        if not samples:
            return None

        samples.sort()
        return samples[len(samples) // 2]

    def _pid(self, error: float) -> float:
        now = time.time()
        dt = max(now - self._last_time, 1e-6)
        self._last_time = now

        self._integral += error * dt
        self._integral = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, self._integral))

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = Kp * error + Ki * self._integral + Kd * derivative
        return max(-MAX_ANGULAR, min(MAX_ANGULAR, output))

    def _reset_pid(self):
        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()

    def _set_state(self, state: str):
        self._state = state
        self._state_start_time = time.time()
        self.get_logger().info(f"-> {state}")

    def _publish(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def _turn_cmd_angular(self) -> float:
        speed = TURN_ANGULAR_SPEED
        if self._current_yaw is not None and self._turn_target_yaw is not None:
            remaining = abs(angle_diff(self._turn_target_yaw, self._current_yaw))
            if remaining < TURN_SLOWDOWN_REMAINING_RAD:
                speed *= 0.6
        return self._turn_direction * speed

    def _start_turn(self, reason: str):
        if self._state != "STRAIGHT":
            return

        if self._turn_direction == 0.0:
            self._turn_direction = DEFAULT_TURN_DIRECTION

        self._turn_start_yaw = self._current_yaw
        if self._turn_start_yaw is None:
            self._turn_target_yaw = None
            self.get_logger().warn(
                f"TURN start without /odom yaw; direction={self._turn_direction:+.0f}, "
                f"reason={reason}"
            )
        else:
            self._turn_target_yaw = normalize_angle(
                self._turn_start_yaw + self._turn_direction * TURN_ANGLE_RAD
            )
            self.get_logger().info(
                "TURN start "
                f"direction={self._turn_direction:+.0f} "
                f"start_yaw={self._turn_start_yaw:+.3f} "
                f"target_yaw={self._turn_target_yaw:+.3f} "
                f"reason={reason}"
            )

        self._reset_pid()
        self._set_state("TURN")
        self._publish(TURN_LINEAR_SPEED, self._turn_cmd_angular())

    def _ensure_turn_target(self):
        if (
            self._state == "TURN"
            and self._turn_start_yaw is None
            and self._current_yaw is not None
        ):
            self._turn_start_yaw = self._current_yaw
            self._turn_target_yaw = normalize_angle(
                self._turn_start_yaw + self._turn_direction * TURN_ANGLE_RAD
            )
            self.get_logger().warn(
                "TURN target initialized late from first /odom yaw: "
                f"start_yaw={self._turn_start_yaw:+.3f} "
                f"target_yaw={self._turn_target_yaw:+.3f}"
            )

    # Main scan callback
    def scan_cb(self, msg: LaserScan):
        elapsed = time.time() - self._state_start_time
        front_dist = self._get_range(msg, FRONT_ANGLE_DEG)
        left_front = self._get_range(msg, LEFT_FRONT_ANGLE_DEG)
        right_front = self._get_range(msg, RIGHT_FRONT_ANGLE_DEG)

        # TURN: complete by odom yaw target, not by front_dist.
        if self._state == "TURN":
            self._ensure_turn_target()

            yaw_error = None
            if self._current_yaw is not None and self._turn_target_yaw is not None:
                yaw_error = angle_diff(self._turn_target_yaw, self._current_yaw)
                if abs(yaw_error) <= TURN_YAW_TOLERANCE_RAD:
                    self.get_logger().info(
                        "TURN done by yaw "
                        f"current={self._current_yaw:+.3f} "
                        f"target={self._turn_target_yaw:+.3f} "
                        f"error={yaw_error:+.3f} "
                        f"front={front_dist}"
                    )
                    self._reset_pid()
                    self._set_state("POST_TURN")
                    return

            if elapsed >= TURN_TIMEOUT_SEC:
                self.get_logger().warn(
                    "TURN timeout -> POST_TURN "
                    f"elapsed={elapsed:.2f}s "
                    f"current_yaw={self._current_yaw} "
                    f"target_yaw={self._turn_target_yaw} "
                    f"yaw_error={yaw_error} "
                    f"front={front_dist}"
                )
                self._reset_pid()
                self._set_state("POST_TURN")
                return

            angular = self._turn_cmd_angular()
            self._publish(TURN_LINEAR_SPEED, angular)
            self.get_logger().info(
                "TURN "
                f"{elapsed:.2f}/{TURN_TIMEOUT_SEC:.2f}s "
                f"yaw={self._current_yaw} "
                f"target={self._turn_target_yaw} "
                f"error={yaw_error} "
                f"front={front_dist} "
                f"dir={self._turn_direction:+.0f} "
                f"ang={angular:+.3f}",
                throttle_duration_sec=0.3,
            )
            return

        # POST_TURN: drive straight briefly and ignore wall/line triggers.
        if self._state == "POST_TURN":
            if elapsed < POST_TURN_SEC:
                self._publish(LINEAR_SPEED, 0.0)
                self.get_logger().info(
                    f"POST_TURN {elapsed:.2f}/{POST_TURN_SEC:.2f}s",
                    throttle_duration_sec=0.3,
                )
                return

            self._reset_pid()
            self._set_state("STRAIGHT")
            return

        # STRAIGHT fallback turn trigger from front LiDAR.
        if front_dist is not None and front_dist < FRONT_TURN_DIST:
            self._turn_direction = (
                self._turn_direction if self._turn_direction != 0.0
                else DEFAULT_TURN_DIRECTION
            )
            self._start_turn(f"front wall {front_dist:.3f}m")
            return

        # STRAIGHT wall-centering PID.
        if left_front is None or right_front is None:
            self.get_logger().warn(
                f"invalid scan LF={left_front} RF={right_front}",
                throttle_duration_sec=1.0,
            )
            self._publish(LINEAR_SPEED * 0.5, 0.0)
            return

        error = left_front - right_front
        if abs(error) < DEAD_ZONE_M:
            error = 0.0

        angular = self._pid(error)
        self._publish(LINEAR_SPEED, angular)

        self.get_logger().info(
            f"STRAIGHT front={front_dist} "
            f"LF={left_front:.3f} RF={right_front:.3f} "
            f"err={error:+.3f} ang={angular:+.3f}",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarPIDFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
