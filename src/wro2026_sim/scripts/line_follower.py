#!/usr/bin/env python3
"""
Step 6: robust LiDAR wall-centering + odom-yaw corner controller.
Changes from previous version:
  - TURN_FRONT_DIST 0.52 -> 0.62 (trigger turn earlier)
  - LINE_TURN_FRONT_MAX 0.70 -> 0.80
  - WIDE_SECTOR_WIDTH_DEG 14 -> 8 (avoid corner opening false reads)
  - TURN angular speed soft cap after 60deg progress
  - Step 6 square course uses a fixed right-turn direction; LiDAR/camera
    triggers the turn but no longer chooses left/right from corner openings.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

# Motion tuning
STRAIGHT_SPEED       = 0.115
APPROACH_SPEED       = 0.060
ALIGN_SPEED          = 0.065
POST_TURN_SPEED      = 0.090
TURN_LINEAR_SPEED    = 0.0

MAX_STRAIGHT_ANGULAR = 0.55
MAX_TURN_ANGULAR     = 0.78
MIN_TURN_ANGULAR     = 0.30
TURN_KP              = 1.35

# LiDAR geometry
FRONT_DEG            = 0.0
LEFT_FRONT_DEG       = 45.0
RIGHT_FRONT_DEG      = 315.0
LEFT_SIDE_DEG        = 90.0
RIGHT_SIDE_DEG       = 270.0

SECTOR_WIDTH_DEG     = 8.0
WIDE_SECTOR_WIDTH_DEG = 8.0   # was 14.0 — narrowed to avoid corner opening reads
MIN_VALID_RANGE      = 0.05
MAX_VALID_RANGE      = 10.0
MAX_SIDE_FOR_CENTERING = 1.20
MAX_DIAG_FOR_CENTERING = 1.20

# Corner detection
APPROACH_FRONT_DIST      = 0.70
TURN_FRONT_DIST          = 0.62   # was 0.52
EMERGENCY_FRONT_DIST     = 0.30
LINE_APPROACH_FRONT_MAX  = 0.95
LINE_TURN_FRONT_MAX      = 0.80   # was 0.70
LINE_TRIGGER_LATCH_SEC   = 1.0
TURN_OPEN_MARGIN_M       = 0.18
TURN_OPEN_RATIO          = 1.20

# Turn completion
TURN_ANGLE_RAD           = math.pi / 2.0
TURN_YAW_TOLERANCE_RAD   = math.radians(5.0)
TURN_MIN_PROGRESS_RAD    = math.radians(76.0)
TURN_TIMEOUT_SEC         = 10.0
TURN_SOFT_CAP_DEG        = 60.0   # after this progress, cap angular speed
TURN_SOFT_CAP_ANGULAR    = 0.55   # soft cap value
TURN_DONE_MIN_FRONT_DIST = 0.38
TURN_SEARCH_LIMIT_RAD    = math.radians(112.0)

# Align / post-turn
ALIGN_MIN_SEC            = 0.45
ALIGN_MAX_SEC            = 2.2
ALIGN_EXIT_FRONT_DIST    = 0.48
POST_TURN_MIN_SEC        = 0.75
POST_TURN_MIN_DIST       = 0.22
POST_TURN_MAX_SEC        = 5.0
POST_TURN_CLEAR_FRONT_DIST = 0.45
POST_TURN_RECOVER_FRONT_DIST = 0.32

# Recovery
RECOVER_STOP_SEC         = 0.25
RECOVER_BACKUP_SEC       = 1.0
RECOVER_TIMEOUT_SEC      = 5.0
RECOVER_BACKUP_SPEED     = -0.050
RECOVER_FORWARD_SPEED    = 0.045

# Wall-centering gains
SIDE_KP                  = 0.62
DIAG_KP                  = 0.22
WALL_KD                  = 0.10
DEAD_ZONE_M              = 0.025
CLOSE_SIDE_DIST          = 0.18
STRAIGHT_HEADING_KP      = 1.20
STRAIGHT_CENTER_MAX_ANGULAR = 0.20
APPROACH_CENTER_MAX_ANGULAR = 0.16

HEADING_HOLD_KP               = 1.15
ALIGN_HEADING_MAX_ANGULAR     = 0.28
POST_TURN_HEADING_MAX_ANGULAR = 0.20

DEFAULT_TURN_DIRECTION   = -1.0   # +1=left, -1=right
USE_LIDAR_TURN_DIRECTION = False  # Step 6 square course: fixed right turns.

# /line_detection indices
LD_ORANGE_TRIG  = 3
LD_BLUE_TRIG    = 7
LD_DIRECTION    = 8


@dataclass
class ScanSummary:
    front:       float | None
    front_min:   float | None
    left_front:  float | None
    right_front: float | None
    left_side:   float | None
    right_side:  float | None


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_diff(target: float, current: float) -> float:
    return normalize_angle(target - current)


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def nearest_cardinal_yaw(yaw: float) -> float:
    return normalize_angle(round(yaw / (math.pi / 2.0)) * (math.pi / 2.0))


class SquareCourseFollower(Node):
    def __init__(self):
        super().__init__("lidar_pid_follower")

        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.sub_line = self.create_subscription(
            Float32MultiArray, "/line_detection", self.line_cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._yaw: float | None = None
        self._odom_t: float | None = None
        self._xy: tuple[float, float] | None = None

        self._state = "STRAIGHT"
        self._state_start_t = self._now()
        self._state_start_xy: tuple[float, float] | None = None

        self._turn_direction = DEFAULT_TURN_DIRECTION
        self._turn_start_yaw: float | None = None
        self._turn_target_yaw: float | None = None
        self._straight_target_yaw: float | None = None

        self._line_latched_until = -1.0
        self._line_turn_direction = DEFAULT_TURN_DIRECTION
        self._course_turn_direction = DEFAULT_TURN_DIRECTION

        self._prev_wall_error = 0.0
        self._prev_wall_t = self._now()

        self.get_logger().info("LiDAR PID Follower started (square course FSM)")

    def _now(self) -> float:
        return self._odom_t if self._odom_t is not None else time.monotonic()

    def _elapsed(self) -> float:
        return max(0.0, self._now() - self._state_start_t)

    def _distance_from_state_start(self) -> float:
        if self._xy is None or self._state_start_xy is None:
            return 0.0
        dx = self._xy[0] - self._state_start_xy[0]
        dy = self._xy[1] - self._state_start_xy[1]
        return math.hypot(dx, dy)

    def _pose_text(self) -> str:
        if self._xy is None or self._yaw is None:
            return "x=None y=None yaw=None"
        return f"x={self._xy[0]:+.3f} y={self._xy[1]:+.3f} yaw={self._yaw:+.3f}"

    def _nearest_cardinal_yaw(self, yaw: float) -> float:
        return nearest_cardinal_yaw(yaw)

    def odom_cb(self, msg: Odometry):
        stamp = msg.header.stamp
        self._odom_t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self._yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self._xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self._straight_target_yaw is None:
            self._straight_target_yaw = self._nearest_cardinal_yaw(self._yaw)

    def line_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 9:
            return
        direction    = msg.data[LD_DIRECTION]
        orange_trig  = msg.data[LD_ORANGE_TRIG] > 0.5
        blue_trig    = msg.data[LD_BLUE_TRIG]   > 0.5
        if not orange_trig and not blue_trig:
            return
        if direction == 1.0 and orange_trig:
            self._line_turn_direction = -1.0
        elif direction == -1.0 and blue_trig:
            self._line_turn_direction = 1.0
        else:
            self._line_turn_direction = DEFAULT_TURN_DIRECTION
        self._line_latched_until = self._now() + LINE_TRIGGER_LATCH_SEC

    def _angle_to_index(self, msg: LaserScan, angle_deg: float) -> int | None:
        if msg.angle_increment <= 0.0 or not msg.ranges:
            return None
        angle = normalize_angle(math.radians(angle_deg))
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        while angle < angle_min:
            angle += 2.0 * math.pi
        while angle > angle_max:
            angle -= 2.0 * math.pi
        if angle < angle_min or angle > angle_max:
            return None
        idx = int(round((angle - angle_min) / msg.angle_increment))
        if idx < 0 or idx >= len(msg.ranges):
            return None
        return idx

    def _sector_values(self, msg: LaserScan, center_deg: float,
                       width_deg: float) -> list[float]:
        samples: list[float] = []
        step_deg = max(1.0, math.degrees(msg.angle_increment))
        count = int(round((width_deg * 2.0) / step_deg)) + 1
        start = center_deg - width_deg
        for i in range(count):
            angle = start + i * step_deg
            idx = self._angle_to_index(msg, angle)
            if idx is None:
                continue
            r = msg.ranges[idx]
            if math.isfinite(r) and MIN_VALID_RANGE < r < MAX_VALID_RANGE:
                samples.append(float(r))
        return samples

    def _sector_median(self, msg: LaserScan, center_deg: float,
                       width_deg: float = SECTOR_WIDTH_DEG) -> float | None:
        values = self._sector_values(msg, center_deg, width_deg)
        if not values:
            return None
        values.sort()
        return values[len(values) // 2]

    def _sector_min(self, msg: LaserScan, center_deg: float,
                    width_deg: float = SECTOR_WIDTH_DEG) -> float | None:
        values = self._sector_values(msg, center_deg, width_deg)
        return min(values) if values else None

    def _summarize_scan(self, msg: LaserScan) -> ScanSummary:
        return ScanSummary(
            front       = self._sector_median(msg, FRONT_DEG,       WIDE_SECTOR_WIDTH_DEG),
            front_min   = self._sector_min   (msg, FRONT_DEG,       WIDE_SECTOR_WIDTH_DEG),
            left_front  = self._sector_median(msg, LEFT_FRONT_DEG,  SECTOR_WIDTH_DEG),
            right_front = self._sector_median(msg, RIGHT_FRONT_DEG, SECTOR_WIDTH_DEG),
            left_side   = self._sector_median(msg, LEFT_SIDE_DEG,   WIDE_SECTOR_WIDTH_DEG),
            right_side  = self._sector_median(msg, RIGHT_SIDE_DEG,  WIDE_SECTOR_WIDTH_DEG),
        )

    def _set_state(self, state: str):
        self._state = state
        self._state_start_t = self._now()
        self._state_start_xy = self._xy
        self._prev_wall_error = 0.0
        self._prev_wall_t = self._now()
        self.get_logger().info(f"-> {state} {self._pose_text()}")

    def _publish(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def _stop(self):
        self._publish(0.0, 0.0)

    def _front_is_clear(self, front: float | None,
                        threshold: float = POST_TURN_CLEAR_FRONT_DIST) -> bool:
        return front is None or front > threshold

    def _line_latched(self) -> bool:
        return self._now() <= self._line_latched_until

    def _clear_line_latch(self):
        self._line_latched_until = -1.0

    def _wall_angular(self, s: ScanSummary,
                      max_angular: float) -> tuple[float, float]:
        left_side = (
            s.left_side
            if s.left_side is not None and s.left_side < MAX_SIDE_FOR_CENTERING
            else None
        )
        right_side = (
            s.right_side
            if s.right_side is not None and s.right_side < MAX_SIDE_FOR_CENTERING
            else None
        )
        left_front = (
            s.left_front
            if s.left_front is not None and s.left_front < MAX_DIAG_FOR_CENTERING
            else None
        )
        right_front = (
            s.right_front
            if s.right_front is not None and s.right_front < MAX_DIAG_FOR_CENTERING
            else None
        )

        side_error = None
        diag_error = None
        if left_side is not None and right_side is not None:
            side_error = left_side - right_side
        if left_front is not None and right_front is not None:
            diag_error = left_front - right_front
        if side_error is None and diag_error is None:
            return 0.0, 0.0
        if side_error is None:
            error = diag_error if diag_error is not None else 0.0
        elif diag_error is None:
            error = side_error
        else:
            error = SIDE_KP * side_error + DIAG_KP * diag_error
        if left_side is not None and left_side < CLOSE_SIDE_DIST:
            error -= (CLOSE_SIDE_DIST - left_side) * 1.4
        if right_side is not None and right_side < CLOSE_SIDE_DIST:
            error += (CLOSE_SIDE_DIST - right_side) * 1.4
        if abs(error) < DEAD_ZONE_M:
            error = 0.0
        now = self._now()
        dt = max(now - self._prev_wall_t, 1e-3)
        derivative = (error - self._prev_wall_error) / dt
        self._prev_wall_error = error
        self._prev_wall_t = now
        angular = error + WALL_KD * derivative
        return clamp(angular, -max_angular, max_angular), error

    def _drive_centered(self, s: ScanSummary, speed: float,
                        max_angular: float) -> tuple[float, float]:
        center_limit = (APPROACH_CENTER_MAX_ANGULAR
                        if self._state == "APPROACH"
                        else STRAIGHT_CENTER_MAX_ANGULAR)
        center_angular, error = self._wall_angular(s, center_limit)
        heading_angular = 0.0
        if self._straight_target_yaw is not None and self._yaw is not None:
            heading_error = angle_diff(self._straight_target_yaw, self._yaw)
            heading_angular = clamp(
                STRAIGHT_HEADING_KP * heading_error,
                -max_angular,
                max_angular,
            )
        angular = clamp(heading_angular + center_angular,
                        -max_angular, max_angular)
        if abs(angular) > max_angular * 0.75:
            speed *= 0.75
        self._publish(speed, angular)
        return angular, error

    def _heading_hold_angular(self, max_angular: float) -> tuple[float, float | None]:
        if self._turn_target_yaw is None or self._yaw is None:
            return 0.0, None
        yaw_error = angle_diff(self._turn_target_yaw, self._yaw)
        angular = clamp(HEADING_HOLD_KP * yaw_error, -max_angular, max_angular)
        return angular, yaw_error

    def _front_distance(self, s: ScanSummary) -> float | None:
        if s.front_min is not None:
            return s.front_min
        return s.front

    def _open_score(self, *values: float | None) -> float:
        valid = [v for v in values if v is not None]
        return max(valid) if valid else 0.0

    def _choose_turn_direction(self, s: ScanSummary | None,
                               requested: float | None) -> tuple[float, str, float, float]:
        requested_dir = None
        if requested is not None and requested != 0.0:
            requested_dir = 1.0 if requested > 0.0 else -1.0
        if s is None:
            chosen = requested_dir if requested_dir is not None else DEFAULT_TURN_DIRECTION
            return chosen, "requested/default", 0.0, 0.0
        left_open  = self._open_score(s.left_front,  s.left_side)
        right_open = self._open_score(s.right_front, s.right_side)
        if not USE_LIDAR_TURN_DIRECTION:
            source = "fixed-course-right"
            if requested_dir is not None and requested_dir != DEFAULT_TURN_DIRECTION:
                source = "fixed-course-right-line-ignored"
            return DEFAULT_TURN_DIRECTION, source, left_open, right_open
        right_is_open = (right_open >= left_open + TURN_OPEN_MARGIN_M
                         or right_open >= left_open * TURN_OPEN_RATIO)
        left_is_open  = (left_open >= right_open + TURN_OPEN_MARGIN_M
                         or left_open >= right_open * TURN_OPEN_RATIO)
        if right_is_open and not left_is_open:
            return -1.0, "lidar-right-open", left_open, right_open
        if left_is_open and not right_is_open:
            return  1.0, "lidar-left-open",  left_open, right_open
        return DEFAULT_TURN_DIRECTION, "ambiguous-default", left_open, right_open

    def _start_approach(self, reason: str):
        if self._state != "STRAIGHT":
            return
        self._set_state("APPROACH")
        self.get_logger().info(f"APPROACH start reason={reason}")

    def _start_turn(self, reason: str, direction: float | None = None,
                    scan: ScanSummary | None = None):
        chosen, source, left_open, right_open = self._choose_turn_direction(scan, direction)
        self._turn_direction = chosen
        self._turn_start_yaw = self._yaw
        if self._turn_start_yaw is None:
            self._turn_target_yaw = None
            self.get_logger().warn(
                f"TURN start without odom yaw direction={self._turn_direction:+.0f} "
                f"source={source} {self._pose_text()} reason={reason}")
        else:
            base_yaw = (self._straight_target_yaw
                        if self._straight_target_yaw is not None
                        else self._turn_start_yaw)
            self._turn_target_yaw = normalize_angle(
                base_yaw + self._turn_direction * TURN_ANGLE_RAD)
            self.get_logger().info(
                f"TURN start direction={self._turn_direction:+.0f} "
                f"start_yaw={self._turn_start_yaw:+.3f} "
                f"target_yaw={self._turn_target_yaw:+.3f} "
                f"straight_target={self._straight_target_yaw} "
                f"source={source} left={left_open:.3f} right={right_open:.3f} "
                f"{self._pose_text()} reason={reason}")
        self._clear_line_latch()
        self._set_state("TURN")
        self._publish(TURN_LINEAR_SPEED, self._turn_angular())

    def _start_recover(self, reason: str):
        if self._state == "RECOVER_FAILED":
            return
        self._clear_line_latch()
        self._stop()
        self._set_state("RECOVER")
        self.get_logger().warn(f"RECOVER start {self._pose_text()} reason={reason}")

    def _ensure_turn_target(self):
        if self._state != "TURN" or self._turn_target_yaw is not None:
            return
        if self._yaw is None:
            return
        self._turn_start_yaw  = self._yaw
        base_yaw = (self._straight_target_yaw
                    if self._straight_target_yaw is not None
                    else self._turn_start_yaw)
        self._turn_target_yaw = normalize_angle(
            base_yaw + self._turn_direction * TURN_ANGLE_RAD)
        self.get_logger().warn(
            f"TURN target initialized late start={self._turn_start_yaw:+.3f} "
            f"target={self._turn_target_yaw:+.3f}")

    def _turn_progress(self) -> float:
        if self._turn_start_yaw is None or self._yaw is None:
            return 0.0
        return abs(angle_diff(self._yaw, self._turn_start_yaw))

    def _turn_yaw_error(self) -> float | None:
        if self._turn_target_yaw is None or self._yaw is None:
            return None
        return angle_diff(self._turn_target_yaw, self._yaw)

    def _turn_angular(self) -> float:
        yaw_error = self._turn_yaw_error()
        progress  = self._turn_progress()

        if yaw_error is None:
            return self._turn_direction * 0.45

        angular = clamp(TURN_KP * yaw_error, -MAX_TURN_ANGULAR, MAX_TURN_ANGULAR)

        # Soft speed cap after 60 deg to reduce overshoot
        if progress > math.radians(TURN_SOFT_CAP_DEG):
            angular = clamp(angular, -TURN_SOFT_CAP_ANGULAR, TURN_SOFT_CAP_ANGULAR)

        if abs(yaw_error) > TURN_YAW_TOLERANCE_RAD:
            min_speed = MIN_TURN_ANGULAR if abs(yaw_error) < math.radians(25.0) else 0.55
            if abs(angular) < min_speed:
                angular = math.copysign(min_speed, yaw_error)

        return angular

    def scan_cb(self, msg: LaserScan):
        s     = self._summarize_scan(msg)
        front = self._front_distance(s)

        if self._state == "STRAIGHT":
            if front is not None and front < EMERGENCY_FRONT_DIST:
                self._start_turn(f"emergency front={front:.3f}m",
                                 self._course_turn_direction, s)
                return
            if front is not None and front < APPROACH_FRONT_DIST:
                self._start_approach(f"front={front:.3f}m")
                return
            if self._line_latched() and front is not None and front < LINE_APPROACH_FRONT_MAX:
                self._start_approach(f"line latch + front={front:.3f}m")
                return
            angular, error = self._drive_centered(s, STRAIGHT_SPEED, MAX_STRAIGHT_ANGULAR)
            self.get_logger().info(
                f"STRAIGHT front={front} LS={s.left_side} RS={s.right_side} "
                f"LF={s.left_front} RF={s.right_front} "
                f"err={error:+.3f} ang={angular:+.3f} {self._pose_text()}",
                throttle_duration_sec=0.6)
            return

        if self._state == "APPROACH":
            if front is not None and front < TURN_FRONT_DIST:
                self._start_turn(f"front={front:.3f}m",
                                 self._course_turn_direction, s)
                return
            if self._line_latched() and front is not None and front < LINE_TURN_FRONT_MAX:
                self._start_turn(f"line latch + front={front:.3f}m",
                                 self._line_turn_direction, s)
                return
            angular, error = self._drive_centered(s, APPROACH_SPEED, 0.42)
            self.get_logger().info(
                f"APPROACH {self._elapsed():.2f}s front={front} "
                f"err={error:+.3f} ang={angular:+.3f}",
                throttle_duration_sec=0.4)
            return

        if self._state == "TURN":
            self._ensure_turn_target()
            yaw_error = self._turn_yaw_error()
            progress  = self._turn_progress()
            if (yaw_error is not None
                    and abs(yaw_error) <= TURN_YAW_TOLERANCE_RAD
                    and progress >= TURN_MIN_PROGRESS_RAD):
                if front is not None and front < TURN_DONE_MIN_FRONT_DIST:
                    if progress < TURN_SEARCH_LIMIT_RAD:
                        angular = self._turn_direction * MIN_TURN_ANGULAR
                        self._publish(TURN_LINEAR_SPEED, angular)
                        self.get_logger().warn(
                            f"TURN yaw reached but front too close {front:.3f}m; "
                            f"continuing turn progress={math.degrees(progress):.1f}deg "
                            f"{self._pose_text()}",
                            throttle_duration_sec=0.3)
                        return
                    self._start_recover(
                        f"turn front still close after search front={front:.3f}m "
                        f"progress={math.degrees(progress):.1f}deg")
                    return
                self.get_logger().info(
                    f"TURN done by yaw current={self._yaw:+.3f} "
                    f"target={self._turn_target_yaw:+.3f} "
                    f"error={yaw_error:+.3f} progress={math.degrees(progress):.1f}deg "
                    f"front={front} {self._pose_text()}")
                self._straight_target_yaw = self._turn_target_yaw
                self._stop()
                self._set_state("ALIGN")
                return
            if self._elapsed() > TURN_TIMEOUT_SEC:
                self.get_logger().warn(
                    f"TURN timeout elapsed={self._elapsed():.2f}s "
                    f"progress={math.degrees(progress):.1f}deg")
                if self._turn_target_yaw is not None:
                    self._straight_target_yaw = self._turn_target_yaw
                self._stop()
                self._set_state("ALIGN")
                return
            angular = self._turn_angular()
            self._publish(TURN_LINEAR_SPEED, angular)
            self.get_logger().info(
                f"TURN {self._elapsed():.2f}/{TURN_TIMEOUT_SEC:.2f}s "
                f"yaw={self._yaw:+.3f} target={self._turn_target_yaw:+.3f} "
                f"error={yaw_error:+.3f} progress={math.degrees(progress):.1f}deg "
                f"front={front} ang={angular:+.3f}",
                throttle_duration_sec=0.25)
            return

        if self._state == "ALIGN":
            elapsed = self._elapsed()
            if front is not None and front < EMERGENCY_FRONT_DIST:
                self._publish(0.0, self._turn_direction * MIN_TURN_ANGULAR)
                self.get_logger().warn(
                    f"ALIGN front too close {front:.3f}m continuing turn",
                    throttle_duration_sec=0.3)
                return
            angular, heading_error = self._heading_hold_angular(ALIGN_HEADING_MAX_ANGULAR)
            if heading_error is None:
                angular, heading_error = self._wall_angular(s, ALIGN_HEADING_MAX_ANGULAR)
            self._publish(ALIGN_SPEED, angular)
            can_exit = (elapsed >= ALIGN_MIN_SEC
                        and (front is None or front > ALIGN_EXIT_FRONT_DIST))
            if can_exit:
                self._clear_line_latch()
                self._set_state("POST_TURN")
                return
            if elapsed >= ALIGN_MAX_SEC:
                if front is not None and front < ALIGN_EXIT_FRONT_DIST:
                    self._start_recover(
                        f"align timeout front still close {front:.3f}m")
                    return
                self._clear_line_latch()
                self._set_state("POST_TURN")
                return
            self.get_logger().info(
                f"ALIGN {elapsed:.2f}s front={front} "
                f"heading_err={heading_error:+.3f} ang={angular:+.3f} "
                f"{self._pose_text()}",
                throttle_duration_sec=0.4)
            return

        if self._state == "POST_TURN":
            elapsed  = self._elapsed()
            distance = self._distance_from_state_start()
            if front is not None and front < POST_TURN_RECOVER_FRONT_DIST:
                self._start_recover(
                    f"post-turn front too close {front:.3f}m dist={distance:.3f}m")
                return
            angular, heading_error = self._heading_hold_angular(
                POST_TURN_HEADING_MAX_ANGULAR)
            if heading_error is None:
                angular, heading_error = self._wall_angular(
                    s, POST_TURN_HEADING_MAX_ANGULAR)
            self._publish(POST_TURN_SPEED, angular)
            done = (elapsed >= POST_TURN_MIN_SEC
                    and distance >= POST_TURN_MIN_DIST
                    and self._front_is_clear(front, POST_TURN_CLEAR_FRONT_DIST))
            if done:
                self.get_logger().info(
                    f"POST_TURN done dist={distance:.3f} front={front} "
                    f"{self._pose_text()}")
                self._clear_line_latch()
                self._set_state("STRAIGHT")
                return
            if elapsed >= POST_TURN_MAX_SEC:
                if (distance >= POST_TURN_MIN_DIST
                        and self._front_is_clear(front, POST_TURN_CLEAR_FRONT_DIST)):
                    self.get_logger().warn(
                        f"POST_TURN timeout but corridor is clear; returning STRAIGHT "
                        f"dist={distance:.3f} front={front} {self._pose_text()}")
                    self._clear_line_latch()
                    self._set_state("STRAIGHT")
                    return
                self._start_recover(
                    f"post-turn timeout without clear exit dist={distance:.3f}m "
                    f"front={front}")
                return
            self.get_logger().info(
                f"POST_TURN {elapsed:.2f}s dist={distance:.3f} front={front} "
                f"heading_err={heading_error:+.3f} ang={angular:+.3f} "
                f"{self._pose_text()}",
                throttle_duration_sec=0.4)
            return

        if self._state == "RECOVER":
            elapsed = self._elapsed()
            angular, heading_error = self._heading_hold_angular(
                POST_TURN_HEADING_MAX_ANGULAR)
            if elapsed < RECOVER_STOP_SEC:
                self._stop()
            elif elapsed < RECOVER_STOP_SEC + RECOVER_BACKUP_SEC:
                self._publish(RECOVER_BACKUP_SPEED, angular)
            else:
                self._publish(RECOVER_FORWARD_SPEED, angular)
            if self._front_is_clear(front, POST_TURN_CLEAR_FRONT_DIST):
                self.get_logger().warn(
                    f"RECOVER clear front={front} -> POST_TURN {self._pose_text()}")
                self._set_state("POST_TURN")
                return
            if elapsed >= RECOVER_TIMEOUT_SEC:
                self._stop()
                self._set_state("RECOVER_FAILED")
                self.get_logger().error(
                    f"RECOVER_FAILED front={front} heading_err={heading_error} "
                    f"elapsed={elapsed:.2f}s {self._pose_text()}")
                return
            self.get_logger().warn(
                f"RECOVER {elapsed:.2f}/{RECOVER_TIMEOUT_SEC:.2f}s front={front} "
                f"heading_err={heading_error} ang={angular:+.3f} {self._pose_text()}",
                throttle_duration_sec=0.4)
            return

        if self._state == "RECOVER_FAILED":
            self._stop()
            self.get_logger().error(
                f"RECOVER_FAILED stopped front={front} {self._pose_text()}",
                throttle_duration_sec=1.0)
            return

        self.get_logger().error(f"Unknown state {self._state} stopping")
        self._stop()
        self._set_state("STRAIGHT")


def main(args=None):
    rclpy.init(args=args)
    node = SquareCourseFollower()
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
