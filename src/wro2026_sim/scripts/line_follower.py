#!/usr/bin/env python3
"""
160 cm field controller: line-triggered turns, speed zones, and LiDAR gap
avoidance for the upper continuous-obstacle section.

Normal cornering is triggered by /line_detection:
  - orange trigger: right turn
  - blue trigger: left turn

The controller keeps the earlier line/odom turn state machine, then adds a
bounded obstacle-avoidance layer for the 1600 x 1600 mm project field.
"""

from __future__ import annotations

import math
import json
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray

# Motion tuning
STRAIGHT_SPEED       = 0.125
FAST_STRAIGHT_SPEED  = 0.180
STARTUP_SPEED        = 0.110
OBSTACLE_ZONE_SPEED  = 0.090
APPROACH_SPEED       = 0.080
ALIGN_SPEED          = 0.070
POST_TURN_SPEED      = 0.095
TURN_LINEAR_SPEED    = 0.035

MAX_STRAIGHT_ANGULAR = 0.55
MAX_TURN_ANGULAR     = 0.58
MIN_TURN_ANGULAR     = 0.20
TURN_KP              = 0.95

# LiDAR geometry
FRONT_DEG            = 0.0
LEFT_FRONT_DEG       = 45.0
RIGHT_FRONT_DEG      = 315.0
LEFT_SIDE_DEG        = 90.0
RIGHT_SIDE_DEG       = 270.0

SECTOR_WIDTH_DEG     = 8.0
WIDE_SECTOR_WIDTH_DEG = 8.0   # was 14.0, narrowed to avoid corner opening reads
MIN_VALID_RANGE      = 0.05
MAX_VALID_RANGE      = 10.0
MAX_SIDE_FOR_CENTERING = 1.20
MAX_DIAG_FOR_CENTERING = 1.20

# Line-triggered corner detection
EMERGENCY_FRONT_DIST     = 0.20
STARTUP_IGNORE_LINES_SEC = 2.0
STARTUP_IGNORE_LINES_DIST = 0.25
LINE_TRIGGER_LATCH_SEC   = 1.2
LINE_TRIGGER_COOLDOWN_SEC = 2.0
LINE_TRIGGER_LOCKOUT_DIST = 0.60
LINE_APPROACH_MIN_SEC    = 0.35
LINE_APPROACH_DIST       = 0.18
LINE_APPROACH_MAX_SEC    = 2.0

# Turn completion
TURN_ANGLE_RAD           = math.pi / 2.0
TURN_YAW_TOLERANCE_RAD   = math.radians(5.0)
TURN_MIN_PROGRESS_RAD    = math.radians(76.0)
TURN_TIMEOUT_SEC         = 10.0
TURN_SOFT_CAP_DEG        = 55.0   # after this progress, cap angular speed
TURN_SOFT_CAP_ANGULAR    = 0.40   # soft cap value

# Align / post-turn
ALIGN_MIN_SEC            = 0.45
ALIGN_MAX_SEC            = 2.2
ALIGN_EXIT_FRONT_DIST    = 0.48
POST_TURN_MIN_SEC        = 0.75
POST_TURN_MIN_DIST       = 0.18
POST_TURN_MAX_SEC        = 5.0
POST_TURN_CLEAR_FRONT_DIST = 0.45
POST_TURN_RECOVER_FRONT_DIST = 0.24

# 160 cm field zones
FAST_ZONE_Y_MAX       = -0.32
OBSTACLE_ZONE_Y_MIN   = 0.28
OBSTACLE_ZONE_X_LIMIT = 0.58
CORNER_ZONE_ABS_X     = 0.46
CORNER_ZONE_ABS_Y     = 0.46
CORNER_SPEED          = 0.090

# Continuous obstacle avoidance
AVOID_TRIGGER_DIST    = 0.55
AVOID_DIAG_TRIGGER_DIST = 0.50
AVOID_SIDE_CLEAR_DIST = 0.38
AVOID_ASYMMETRY_MIN = 0.08
AVOID_TRIGGER_DEBUG_DIST = 0.62
AVOID_CLUSTER_ROI_DEG = 60.0
AVOID_CLUSTER_RANGE_THRESHOLD = 0.50
AVOID_CLUSTER_OBSTACLE_MAX_RANGE = 0.215
AVOID_CLUSTER_MIN_WIDTH_DEG = 2.0
AVOID_CLUSTER_MAX_WIDTH_DEG = 42.0
AVOID_CLUSTER_MIN_POINTS = 2
AVOID_CLUSTER_MAX_POINTS = 42
AVOID_LANE_SHIFT_MIN_SEC = 0.90
AVOID_LANE_SHIFT_DIST = 0.10
AVOID_LANE_SHIFT_MAX_SEC = 3.00
AVOID_LANE_SHIFT_SPEED = -0.055
AVOID_LANE_SHIFT_ANGULAR = 0.45
AVOID_LANE_SHIFT_YAW_PROGRESS = 0.75
AVOID_SHIFT_ANGLE_RAD = math.radians(45.0)
AVOID_LANE_SHIFT_GUARD_MIN_SEC = 0.90
AVOID_LANE_SHIFT_GUARD_MIN_DIST = 0.08
AVOID_LANE_SHIFT_FRONT_GUARD_DIST = 0.25
AVOID_LANE_SHIFT_FRONT_CLOSE_DIST = 0.20
AVOID_LANE_SHIFT_FRONT_PASS_MIN_DIST = 0.13
AVOID_LANE_SHIFT_EMERGENCY_FRONT_DIST = 0.13
AVOID_PASS_MIN_SEC    = 1.00
AVOID_PASS_MIN_DIST   = 0.12
AVOID_PASS_MAX_SEC    = 3.0
AVOID_PASS_CLEAR_DIST = 0.55
AVOID_PASS_SPEED      = 0.105
AVOID_PASS_MAX_ANGULAR = 0.18
AVOID_PASS_ELAPSED_CAP_SEC = 0.500
AVOID_RETURN_HEADING_SPEED = 0.080
AVOID_RETURN_HEADING_MAX_ANGULAR = 0.25
AVOID_RETURN_HEADING_TOLERANCE_RAD = 0.20
AVOID_RETURN_HEADING_TIMEOUT_TOLERANCE_RAD = 0.35
AVOID_RETURN_HEADING_MAX_SEC = 3.0
AVOID_RETURN_HEADING_HARD_MAX_SEC = 5.0
AVOID_RETURN_CLEAR_DIST = 0.50
AVOID_RETRIGGER_LOCKOUT_SEC = 3.0
AVOID_RETRIGGER_LOCKOUT_DIST = 0.24
POST_AVOID_PUSH_SEC   = 3.2
POST_AVOID_PUSH_SPEED = 0.105
POST_AVOID_PUSH_MAX_ANGULAR = 0.16
POST_AVOID_TOP_CORNER_GUARD_SEC = 6.0
POST_AVOID_TOP_CORNER_Y_MIN = 0.32
POST_AVOID_TOP_CORNER_APPROACH_X_MAX = -0.35
POST_AVOID_TOP_CORNER_TURN_X_MAX = -0.50
POST_AVOID_TOP_CORNER_APPROACH_DIST = 0.07
POST_AVOID_TOP_CORNER_TURN_FRONT_SLOW_DIST = 0.28
POST_AVOID_TOP_CORNER_TURN_FRONT_MIN_DIST = 0.14
POST_AVOID_TOP_CORNER_TURN_LINEAR_SPEED = 0.0
RETURN_TO_LINE_SEC    = 0.90
RETURN_TO_LINE_DIST   = 0.11
RETURN_TO_LINE_MAX_SEC = 2.5
RETURN_TO_LINE_HARD_TIMEOUT_SEC = 6.0
RETURN_TO_LINE_CLEAR_DIST = 0.50
RETURN_TO_LINE_HEADING_TOLERANCE_RAD = 0.35
RETURN_TO_LINE_SPEED  = 0.085
RETURN_TO_LINE_MAX_ANGULAR = 0.34
RETURN_TO_LINE_TOP_Y = 0.42
RETURN_TO_LINE_TOP_ESCAPE_Y = 0.44
RETURN_TO_LINE_Y_WORSEN_DIST = 0.03
RETURN_TO_LINE_TOP_HEADING_ERR = 0.80
RETURN_TO_LINE_TOP_TARGET_YAW = -math.pi / 2.0
RETURN_TO_LINE_TOP_SPEED = 0.055
RETURN_TO_LINE_TOP_TURN_ONLY_ERR = 0.60
RETURN_TO_LINE_TOP_MAX_ANGULAR = 0.42
POST_AVOID_LEAVE_TOP_Y_MIN = 0.32
POST_AVOID_LEAVE_TOP_EXIT_Y = 0.20
POST_AVOID_LEAVE_TOP_SAFE_X = -0.32
POST_AVOID_LEAVE_TOP_SOFT_SAFE_X = -0.20
POST_AVOID_LEAVE_TOP_SHIFT_TARGET_YAW = math.pi
POST_AVOID_LEAVE_TOP_SHIFT_SPEED = 0.075
POST_AVOID_LEAVE_TOP_SHIFT_STRONG_SPEED = 0.090
POST_AVOID_LEAVE_TOP_SHIFT_MAX_ANGULAR = 0.45
POST_AVOID_LEAVE_TOP_SHIFT_STRONG_MAX_ANGULAR = 0.50
POST_AVOID_LEAVE_TOP_SHIFT_MOVE_HEADING_ERR = 0.25
POST_AVOID_LEAVE_TOP_TARGET_YAW = -math.pi / 2.0
POST_AVOID_LEAVE_TOP_SPEED = 0.075
POST_AVOID_LEAVE_TOP_PROGRESS_SPEED = 0.085
POST_AVOID_LEAVE_TOP_MAX_ANGULAR = 0.42
POST_AVOID_LEAVE_TOP_PROGRESS_MAX_ANGULAR = 0.45
POST_AVOID_LEAVE_TOP_MOVE_HEADING_ERR = 0.45
POST_AVOID_LEAVE_TOP_GOAL_X = -0.08
POST_AVOID_LEAVE_TOP_GOAL_Y = 0.10
POST_AVOID_LEAVE_TOP_WAYPOINT_SPEED = 0.090
POST_AVOID_LEAVE_TOP_WAYPOINT_SLOW_SPEED = 0.025
POST_AVOID_LEAVE_TOP_WAYPOINT_FALLBACK_SPEED = 0.085
POST_AVOID_LEAVE_TOP_WAYPOINT_ALIGN_ERR = 0.30
POST_AVOID_LEAVE_TOP_WAYPOINT_SLOW_ERR = 0.18
POST_AVOID_LEAVE_TOP_WAYPOINT_ALIGN_MAX_ANGULAR = 0.55
POST_AVOID_LEAVE_TOP_WAYPOINT_SLOW_MAX_ANGULAR = 0.45
POST_AVOID_LEAVE_TOP_WAYPOINT_DRIVE_MAX_ANGULAR = 0.35
POST_AVOID_LEAVE_TOP_WAYPOINT_FALLBACK_MAX_ANGULAR = 0.55
POST_AVOID_LEAVE_TOP_FALLBACK_YAW = -math.pi / 2.0
POST_AVOID_LEAVE_TOP_FINAL_DOWN_Y = 0.35
POST_AVOID_LEAVE_TOP_FINAL_DOWN_PROGRESS = 0.07
POST_AVOID_LEAVE_TOP_FINAL_DOWN_FORCE_SEC = 6.0
POST_AVOID_LEAVE_TOP_FINAL_DOWN_EARLY_SEC = 2.0
POST_AVOID_LEAVE_TOP_FINAL_DOWN_EARLY_PROGRESS = 0.03
POST_AVOID_LEAVE_TOP_FINAL_DOWN_MIN_SEC = 6.0
POST_AVOID_LEAVE_TOP_FINAL_DOWN_ALIGN_ERR = 0.25
POST_AVOID_LEAVE_TOP_FINAL_DOWN_SLOW_ERR = 0.16
POST_AVOID_LEAVE_TOP_FINAL_DOWN_CREEP_SPEED = 0.035
POST_AVOID_LEAVE_TOP_FINAL_DOWN_SPEED = 0.085
POST_AVOID_LEAVE_TOP_FINAL_DOWN_ALIGN_MAX_ANGULAR = 0.60
POST_AVOID_LEAVE_TOP_FINAL_DOWN_CREEP_MAX_ANGULAR = 0.45
POST_AVOID_LEAVE_TOP_FINAL_DOWN_DRIVE_MAX_ANGULAR = 0.30
POST_AVOID_LEAVE_TOP_FINAL_DOWN_BLOCKED_SEC = 1.5
POST_AVOID_LEAVE_TOP_FINAL_DOWN_BLOCKED_PROGRESS = 0.03
POST_AVOID_LEAVE_TOP_FINAL_DOWN_DANGER_DIST = 0.11
POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_DIST = 0.20
POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_SAFE_X = -0.05
POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_YAW = -2.20
POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_SPEED = 0.055
POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_WAIT_SPEED = 0.025
POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_MAX_ANGULAR = 0.45
POST_AVOID_LEAVE_TOP_GOAL_PROGRESS_SEC = 2.0
POST_AVOID_LEAVE_TOP_GOAL_PROGRESS_MIN = 0.03
POST_AVOID_LEAVE_TOP_Y_PROGRESS_SEC = 2.0
POST_AVOID_LEAVE_TOP_Y_PROGRESS_MIN = 0.04
POST_AVOID_LEAVE_TOP_FRONT_STOP_DIST = 0.18
POST_AVOID_LEAVE_TOP_FRONT_DANGER_DIST = 0.11
POST_AVOID_LEAVE_TOP_NO_PROGRESS_SEC = 1.5
POST_AVOID_LEAVE_TOP_NO_PROGRESS_DIST = 0.03
POST_AVOID_LEAVE_TOP_LEFT_FAILED_SEC = 3.0
POST_AVOID_LEAVE_TOP_LEFT_FAILED_DIST = 0.04
POST_AVOID_LEAVE_TOP_STUCK_SEC = 4.0
POST_AVOID_LEAVE_TOP_STUCK_PROGRESS_DIST = 0.02
POST_AVOID_LEAVE_TOP_MAX_SEC = 12.0
POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_SEC = 2.0
POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_MAX_COUNT = 2
POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_Y_MAX = 0.34
POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_MIN_PROGRESS = 0.05
POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_FRONT_MIN = 0.12
POST_AVOID_LEAVE_TOP_RECENT_DY_WINDOW_SEC = 2.0
POST_AVOID_LEAVE_TOP_RECENT_DY_NEGATIVE_MIN = -0.003
POST_AVOID_LEAVE_TOP_FINISH_DOWN_Y = 0.30
POST_AVOID_LEAVE_TOP_FINISH_DOWN_FRONT_MIN = 0.14
POST_AVOID_LEAVE_TOP_FINISH_DOWN_SPEED = 0.065
POST_AVOID_DOWN_PATH_MARGIN = 0.0075
POST_AVOID_ISLAND_NORTH_WALL_MIN_X = -0.150
POST_AVOID_ISLAND_NORTH_WALL_MAX_X = 0.150
POST_AVOID_BASE_COLLISION_SIZE_X = 0.140
POST_AVOID_BASE_COLLISION_SIZE_Y = 0.140
POST_AVOID_BASE_COLLISION_OFFSET_X = -0.032
POST_AVOID_BASE_COLLISION_OFFSET_Y = 0.0
POST_AVOID_CLEARANCE_PEEL_TARGET_YAW = math.pi
POST_AVOID_CLEARANCE_ALIGN_TARGET_YAW = -math.pi / 2.0
POST_AVOID_CLEARANCE_YAW_TOLERANCE = 0.08
POST_AVOID_RECENT_SEC = 60.0
POST_AVOID_TOP_EXIT_RETRY_MAX = 2
POST_AVOID_TOP_EXIT_RETRY_Y_MAX = 0.50
POST_AVOID_TOP_EXIT_RETRY_SPEED = 0.075
GAP_LEFT_DEG          = 35.0
GAP_RIGHT_DEG         = 325.0
GAP_WIDTH_DEG         = 18.0

# Camera yellow obstacle gate
YELLOW_HSV_LOWER      = (18, 60, 60)
YELLOW_HSV_UPPER      = (40, 255, 255)
YELLOW_ROI_X1_RATIO   = 0.25
YELLOW_ROI_X2_RATIO   = 0.75
YELLOW_ROI_Y1_RATIO   = 0.12
YELLOW_ROI_Y2_RATIO   = 0.68
YELLOW_GATE_BASE_WIDTH  = 320.0
YELLOW_GATE_BASE_HEIGHT = 240.0
YELLOW_BLOB_AREA_MIN  = 2000.0
YELLOW_BBOX_W_MIN     = 30.0
YELLOW_BBOX_W_MAX     = 110.0
YELLOW_BBOX_H_MIN     = 60.0
YELLOW_BBOX_H_MAX     = 145.0
YELLOW_CENTER_X_MIN   = 140.0
YELLOW_CENTER_X_MAX   = 250.0
YELLOW_CENTER_Y_MIN   = 25.0
YELLOW_CENTER_Y_MAX   = 140.0
YELLOW_HIGH_CONF_AREA_MIN = 5000.0
YELLOW_HIGH_CONF_EARLY_AREA_MIN = 4800.0
YELLOW_HIGH_CONF_BBOX_W_MIN = 25.0
YELLOW_HIGH_CONF_BBOX_W_MAX = 125.0
YELLOW_HIGH_CONF_BBOX_H_MAX = 155.0
YELLOW_HIGH_CONF_CENTER_X_MIN = 100.0
YELLOW_HIGH_CONF_CENTER_X_MAX = 255.0
YELLOW_HIGH_CONF_CENTER_Y_MIN = 25.0
YELLOW_HIGH_CONF_CENTER_Y_MAX = 150.0
YELLOW_HIGH_CONF_GATE_Y_MAX = 0.49
YELLOW_GATE_MIN_CONSECUTIVE = 2
YELLOW_GATE_STALE_SEC = 0.6
YELLOW_GATE_X_MIN     = 0.05
YELLOW_GATE_X_MAX     = 0.25
YELLOW_GATE_Y_MIN     = 0.34
YELLOW_GATE_Y_MAX     = 0.45
YELLOW_GATE_YAW_TOLERANCE_RAD = 0.45

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

RECOVER_EGRESS_STATE = "RECOVER_EGRESS_EXTENSION"
RECOVER_EGRESS_EXTENSION_PATH_M = 0.107710342260001
RECOVER_EGRESS_EXTENSION_TIMEOUT_SEC = 2.730
RECOVER_EGRESS_ALIGN_DOWN_TIMEOUT_SEC = 3.651
RECOVER_EGRESS_ROUTE_HARD_TIMEOUT_SEC = 5.896

# Phase 5Q temporary screening candidate.  This entire route and its
# diagnostics are removed at the end of the Level 1 phase.
BOTTOM_FALLBACK_STATE = "BOTTOM_DIRECT_REORIENT_ODOM_HANDOFF"
BOTTOM_FALLBACK_DIRECT_TARGET_YAW = math.pi / 4.0
BOTTOM_FALLBACK_REVERSE_PATH_M = 0.027969393301043588
BOTTOM_FALLBACK_DIRECT_TIMEOUT_SEC = 3.490
BOTTOM_FALLBACK_REVERSE_TIMEOUT_SEC = 2.350
BOTTOM_FALLBACK_RESTORE_TIMEOUT_SEC = 3.505
BOTTOM_FALLBACK_ROUTE_HARD_TIMEOUT_SEC = 8.380
BOTTOM_FALLBACK_TURN_ANGULAR = 0.600

DEFAULT_TURN_DIRECTION   = -1.0   # +1=left, -1=right
RIGHT_TURN_DIRECTION     = -1.0
LEFT_TURN_DIRECTION      = 1.0

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


@dataclass
class ScanClusterSummary:
    cluster_count: int
    min_range: float | None
    angle_center_deg: float | None
    width_deg: float | None
    point_count: int


@dataclass
class ObstacleTriggerDecision:
    trigger: bool
    trigger_dist: float | None
    near_front: bool
    diag_near: bool
    cluster: ScanClusterSummary
    cluster_obstacle: bool
    side_clear: bool
    obstacle_asymmetry: bool
    narrow_geometry: bool
    in_zone: bool
    lockout: bool


@dataclass
class YellowObstacleSummary:
    detected: bool
    gate: bool
    gate_reason: str
    false_reason: str
    high_conf_early_ok: bool
    pose_in_top_band: bool
    area: float
    bbox_x: int
    bbox_y: int
    bbox_w: int
    bbox_h: int
    center_x: float | None
    center_y: float | None
    aspect_ratio: float | None
    blob_count: int
    consecutive: int
    image_width: int
    image_height: int
    roi_x1: int
    roi_y1: int
    roi_x2: int
    roi_y2: int


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_diff(target: float, current: float) -> float:
    return normalize_angle(target - current)


def _base_collision_world_x_bounds(x: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    half_x = POST_AVOID_BASE_COLLISION_SIZE_X * 0.5
    half_y = POST_AVOID_BASE_COLLISION_SIZE_Y * 0.5
    min_x = math.inf
    max_x = -math.inf
    for corner_x, corner_y in (
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y)):
        local_x = POST_AVOID_BASE_COLLISION_OFFSET_X + corner_x
        local_y = POST_AVOID_BASE_COLLISION_OFFSET_Y + corner_y
        world_x = x + cos_yaw * local_x - sin_yaw * local_y
        min_x = min(min_x, world_x)
        max_x = max(max_x, world_x)
    return min_x, max_x


def _is_down_path_geometrically_clear(x: float, y: float,
                                      yaw: float) -> bool:
    del y
    footprint_min_x, footprint_max_x = _base_collision_world_x_bounds(x, yaw)
    clear_right = (
        footprint_min_x
        > POST_AVOID_ISLAND_NORTH_WALL_MAX_X + POST_AVOID_DOWN_PATH_MARGIN
    )
    clear_left = (
        footprint_max_x
        < POST_AVOID_ISLAND_NORTH_WALL_MIN_X - POST_AVOID_DOWN_PATH_MARGIN
    )
    return clear_left or clear_right


def _select_down_path_clearance_side(x: float, yaw: float) -> tuple[str, float, float]:
    footprint_min_x, footprint_max_x = _base_collision_world_x_bounds(x, yaw)
    right_required = (
        POST_AVOID_ISLAND_NORTH_WALL_MAX_X + POST_AVOID_DOWN_PATH_MARGIN
        - footprint_min_x
    )
    left_required = (
        footprint_max_x
        - (POST_AVOID_ISLAND_NORTH_WALL_MIN_X - POST_AVOID_DOWN_PATH_MARGIN)
    )
    selected_side = "right" if right_required <= left_required else "left"
    return selected_side, right_required, left_required


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def nearest_cardinal_yaw(yaw: float) -> float:
    return normalize_angle(round(yaw / (math.pi / 2.0)) * (math.pi / 2.0))


class SquareCourseFollower(Node):
    _PHASE5X_RUNTIME_FIELDS = (
        "_yaw", "_odom_t", "_xy", "_startup_t", "_startup_xy",
        "_startup_line_lockout_done", "_state", "_state_start_t",
        "_state_start_xy", "_state_start_yaw", "_turn_direction",
        "_turn_start_yaw", "_turn_target_yaw", "_straight_target_yaw",
        "_line_latched_until", "_line_turn_direction", "_line_reason",
        "_last_line_trigger_t", "_last_line_trigger_xy", "_avoid_direction",
        "_avoid_base_yaw", "_avoid_shift_target_yaw",
        "_avoid_pass_target_yaw", "_lane_shift_previous_yaw",
        "_lane_shift_yaw_progress", "_lane_shift_positive_yaw_sum",
        "_lane_shift_negative_yaw_sum", "_avoid_lockout_until",
        "_post_avoid_push_until", "_post_avoid_top_corner_guard_until",
        "_recent_obstacle_avoidance_until", "_post_avoid_leave_top_last_x",
        "_post_avoid_leave_top_last_y", "_post_avoid_final_down_entry_reason",
        "_post_avoid_final_down_start_t", "_post_avoid_final_down_start_xy",
        "_post_avoid_final_down_drive_start_t",
        "_post_avoid_final_down_clearance_start_t",
        "_post_avoid_final_down_motion_blocked_logged",
        "_post_avoid_clearance_subphase", "_post_avoid_clearance_side",
        "_post_avoid_clearance_start_t", "_post_avoid_clearance_start_x",
        "_post_avoid_clearance_failed", "_post_avoid_final_down_extension_count",
        "_post_avoid_final_down_extension_until", "_post_avoid_recent_dy",
        "_post_avoid_top_exit_retry_count", "_post_avoid_top_exit_retry_reason",
        "_recover_count", "_recover_failed_count", "_recover_source_state",
        "_bottom_recover_entry_yaw", "_bottom_recover_previous_target_yaw",
        "_last_avoid_exit_xy", "_upper_pass_recover_egress_seen",
        "_recover_egress_phase", "_recover_egress_route_start_t",
        "_recover_egress_segment_start_t", "_recover_egress_extension_path",
        "_recover_egress_extension_last_xy", "_bottom_fallback_phase",
        "_bottom_fallback_route_start_t", "_bottom_fallback_segment_start_t",
        "_bottom_fallback_saved_target_yaw", "_bottom_fallback_reverse_path",
        "_bottom_fallback_reverse_last_xy", "_phase5q_last_published_command",
        "_yellow_obstacle_consecutive", "_yellow_obstacle_detected",
        "_yellow_obstacle_last_seen_time", "_yellow_obstacle_summary",
        "_pending_yellow_handoff", "_pending_yellow_stamp",
        "_pending_yellow_generation", "_prev_wall_error", "_prev_wall_t",
    )

    def __init__(self):
        super().__init__("lidar_pid_follower")

        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.sub_image = self.create_subscription(
            Image, "/camera/image_raw", self.image_cb, 10)
        self.sub_line = self.create_subscription(
            Float32MultiArray, "/line_detection", self.line_cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._phase6g_awaiting_fresh_odom = False
        self._last_observed_ros_time_ns: int | None = None
        self._phase5x_rewind_reset_count = 0
        self._initialize_phase5x_run_diagnostics()
        self._initialize_runtime_defaults()

        self._phase5x_log_event(
            "constructor_snapshot",
            callback="constructor",
            now_ns=int(self.get_clock().now().nanoseconds),
            reset_count=0,
            snapshot=self._runtime_snapshot())

        self.get_logger().info(
            "160 cm line-triggered follower with camera yellow obstacle gate started")

    def _initialize_runtime_defaults(self):
        self._yaw: float | None = None
        self._odom_t: float | None = None
        self._xy: tuple[float, float] | None = None
        self._startup_t: float | None = None
        self._startup_xy: tuple[float, float] | None = None
        self._startup_line_lockout_done = False

        self._state = "STRAIGHT"
        self._state_start_t = self._now()
        self._state_start_xy: tuple[float, float] | None = None
        self._state_start_yaw: float | None = None

        self._turn_direction = DEFAULT_TURN_DIRECTION
        self._turn_start_yaw: float | None = None
        self._turn_target_yaw: float | None = None
        self._straight_target_yaw: float | None = None

        self._line_latched_until = -1.0
        self._line_turn_direction = DEFAULT_TURN_DIRECTION
        self._line_reason = "none"
        self._last_line_trigger_t = -1.0
        self._last_line_trigger_xy: tuple[float, float] | None = None
        self._avoid_direction = 0.0
        self._avoid_base_yaw: float | None = None
        self._avoid_shift_target_yaw: float | None = None
        self._avoid_pass_target_yaw: float | None = None
        self._lane_shift_previous_yaw: float | None = None
        self._lane_shift_yaw_progress = 0.0
        self._lane_shift_positive_yaw_sum = 0.0
        self._lane_shift_negative_yaw_sum = 0.0
        self._avoid_lockout_until = -1.0
        self._post_avoid_push_until = -1.0
        self._post_avoid_top_corner_guard_until = -1.0
        self._recent_obstacle_avoidance_until = -1.0
        self._post_avoid_leave_top_last_x: float | None = None
        self._post_avoid_leave_top_last_y: float | None = None
        self._post_avoid_final_down_entry_reason: str | None = None
        self._post_avoid_final_down_start_t: float | None = None
        self._post_avoid_final_down_start_xy: tuple[float, float] | None = None
        self._post_avoid_final_down_drive_start_t: float | None = None
        self._post_avoid_final_down_clearance_start_t: float | None = None
        self._post_avoid_final_down_motion_blocked_logged = False
        self._post_avoid_clearance_subphase: str | None = None
        self._post_avoid_clearance_side: str | None = None
        self._post_avoid_clearance_start_t: float | None = None
        self._post_avoid_clearance_start_x: float | None = None
        self._post_avoid_clearance_failed = False
        self._post_avoid_final_down_extension_count = 0
        self._post_avoid_final_down_extension_until: float | None = None
        self._post_avoid_recent_dy: list[tuple[float, float]] = []
        self._post_avoid_top_exit_retry_count = 0
        self._post_avoid_top_exit_retry_reason: str | None = None
        self._recover_count = 0
        self._recover_failed_count = 0
        self._recover_source_state: str | None = None
        self._bottom_recover_entry_yaw: float | None = None
        self._bottom_recover_previous_target_yaw: float | None = None
        self._last_avoid_exit_xy: tuple[float, float] | None = None
        self._upper_pass_recover_egress_seen = False
        self._recover_egress_phase: str | None = None
        self._recover_egress_route_start_t: float | None = None
        self._recover_egress_segment_start_t: float | None = None
        self._recover_egress_extension_path = 0.0
        self._recover_egress_extension_last_xy: tuple[float, float] | None = None
        self._bottom_fallback_phase: str | None = None
        self._bottom_fallback_route_start_t: float | None = None
        self._bottom_fallback_segment_start_t: float | None = None
        self._bottom_fallback_saved_target_yaw: float | None = None
        self._bottom_fallback_reverse_path = 0.0
        self._bottom_fallback_reverse_last_xy: tuple[float, float] | None = None
        self._phase5q_last_published_command = (0.0, 0.0)
        self._yellow_obstacle_consecutive = 0
        self._yellow_obstacle_detected = False
        self._yellow_obstacle_last_seen_time = -1.0
        self._yellow_obstacle_summary = YellowObstacleSummary(
            detected=False,
            gate=False,
            gate_reason="none",
            false_reason="not_initialized",
            high_conf_early_ok=False,
            pose_in_top_band=False,
            area=0.0,
            bbox_x=0,
            bbox_y=0,
            bbox_w=0,
            bbox_h=0,
            center_x=None,
            center_y=None,
            aspect_ratio=None,
            blob_count=0,
            consecutive=0,
            image_width=0,
            image_height=0,
            roi_x1=0,
            roi_y1=0,
            roi_x2=0,
            roi_y2=0,
        )
        self._pending_yellow_handoff = False
        self._pending_yellow_stamp = -1.0
        self._pending_yellow_generation: int | None = None

        self._prev_wall_error = 0.0
        self._prev_wall_t = self._now()

    def _initialize_phase5x_run_diagnostics(self):
        self._phase5x_first_line_logged = False
        self._phase5x_first_nonzero_logged = False

    def _phase5x_jsonable(self, value):
        if value is None or isinstance(value, (bool, int, float, str)):
            return value
        if isinstance(value, (tuple, list)):
            return [self._phase5x_jsonable(item) for item in value]
        dataclass_fields = getattr(value, "__dataclass_fields__", None)
        if dataclass_fields is not None:
            return {
                name: self._phase5x_jsonable(getattr(value, name))
                for name in dataclass_fields
            }
        return repr(value)

    def _runtime_snapshot(self) -> dict:
        return {
            name: self._phase5x_jsonable(getattr(self, name))
            for name in self._PHASE5X_RUNTIME_FIELDS
        }

    def _phase5x_log_event(self, event: str, **fields):
        record = {"event": event, **fields}
        self.get_logger().warn(
            "PHASE5X_REWIND_V2 "
            + json.dumps(record, sort_keys=True, separators=(",", ":")))

    def _check_and_reset_on_time_rewind(
            self, now_ns: int, callback: str) -> bool:
        previous_high_water_ns = self._last_observed_ros_time_ns
        if previous_high_water_ns is None:
            self._last_observed_ros_time_ns = now_ns
            return False
        if now_ns < previous_high_water_ns:
            before_snapshot = self._runtime_snapshot()
            self._stop()
            self._initialize_runtime_defaults()
            self._initialize_phase5x_run_diagnostics()
            self._phase5x_rewind_reset_count += 1
            self._last_observed_ros_time_ns = now_ns
            self._phase5x_log_event(
                "time_rewind_reset",
                callback=callback,
                previous_high_water_ns=previous_high_water_ns,
                now_ns=now_ns,
                reset_count=self._phase5x_rewind_reset_count,
                stop_publish_count=1,
                current_callback_continues=True,
                before_snapshot=before_snapshot,
                after_snapshot=self._runtime_snapshot())
            return True
        self._last_observed_ros_time_ns = max(previous_high_water_ns, now_ns)
        return False

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

    def _distance_from_xy(self, xy: tuple[float, float] | None) -> float | None:
        if self._xy is None or xy is None:
            return None
        dx = self._xy[0] - xy[0]
        dy = self._xy[1] - xy[1]
        return math.hypot(dx, dy)

    def _startup_elapsed(self) -> float:
        if self._startup_t is None:
            return 0.0
        return max(0.0, self._now() - self._startup_t)

    def _startup_distance(self) -> float:
        return self._distance_from_xy(self._startup_xy) or 0.0

    def _startup_line_lockout_active(self) -> bool:
        if self._startup_line_lockout_done:
            return False
        if self._startup_t is None or self._startup_xy is None:
            return True
        active = (self._startup_elapsed() < STARTUP_IGNORE_LINES_SEC
                  or self._startup_distance() < STARTUP_IGNORE_LINES_DIST)
        if not active:
            self._startup_line_lockout_done = True
        return active

    def _fmt_range(self, value: float | None) -> str:
        return "None" if value is None else f"{value:.3f}"

    def _scan_text(self, s: ScanSummary | None) -> str:
        if s is None:
            return "scan=None"
        return (
            f"front={self._fmt_range(s.front)} "
            f"front_min={self._fmt_range(s.front_min)} "
            f"LF={self._fmt_range(s.left_front)} "
            f"RF={self._fmt_range(s.right_front)} "
            f"LS={self._fmt_range(s.left_side)} "
            f"RS={self._fmt_range(s.right_side)}"
        )

    def _pose_text(self) -> str:
        if self._xy is None or self._yaw is None:
            return "x=None y=None yaw=None"
        return f"x={self._xy[0]:+.3f} y={self._xy[1]:+.3f} yaw={self._yaw:+.3f}"

    def _nearest_cardinal_yaw(self, yaw: float) -> float:
        return nearest_cardinal_yaw(yaw)

    def _phase6g_odom_header_ns(self, msg: Odometry) -> int | None:
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return None
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _phase6g_odom_cache(self) -> dict:
        return {
            "odom_t": self._odom_t,
            "xy": self._xy,
            "yaw": self._yaw,
            "startup_t": self._startup_t,
            "startup_xy": self._startup_xy,
            "straight_target_yaw": self._straight_target_yaw,
            "last_command": self._phase5q_last_published_command,
        }

    def _phase6g_preprocess_odom(
            self, msg: Odometry, node_now_ns: int) -> str:
        header_ns = self._phase6g_odom_header_ns(msg)
        if self._phase6g_awaiting_fresh_odom:
            rejection_reason = None
            if header_ns is None:
                rejection_reason = "missing_header"
            elif header_ns > node_now_ns:
                rejection_reason = "header_after_node_now"
            if rejection_reason is not None:
                cache_before = self._phase6g_odom_cache()
                self._stop()
                self._phase5x_log_event(
                    "phase6g_odom_rejected",
                    callback="odom_cb",
                    wall_monotonic_ns=time.monotonic_ns(),
                    node_now_ns=node_now_ns,
                    message_header_ns=header_ns,
                    reset_count=self._phase5x_rewind_reset_count,
                    rejection_reason=rejection_reason,
                    awaiting_fresh_odom=True,
                    cache_before=cache_before,
                    cache_after=self._phase6g_odom_cache(),
                    callback_action="stop_then_return")
                return "rejected"
            self._phase6g_awaiting_fresh_odom = False
            self._phase5x_log_event(
                "phase6g_fresh_odom_accepted",
                callback="odom_cb",
                wall_monotonic_ns=time.monotonic_ns(),
                node_now_ns=node_now_ns,
                message_header_ns=header_ns,
                reset_count=self._phase5x_rewind_reset_count,
                acceptance_predicate="header_exists_and_header_ns_le_node_now_ns",
                awaiting_fresh_odom=False,
                cache_before=self._phase6g_odom_cache())
            decision = "fresh"
        else:
            decision = "normal"

        if self._check_and_reset_on_time_rewind(node_now_ns, "odom_cb"):
            self._phase6g_awaiting_fresh_odom = True
            self._phase5x_log_event(
                "phase6g_reset_callback_return",
                callback="odom_cb",
                wall_monotonic_ns=time.monotonic_ns(),
                node_now_ns=node_now_ns,
                message_header_ns=header_ns,
                reset_count=self._phase5x_rewind_reset_count,
                awaiting_fresh_odom=True,
                cache_after_reset=self._phase6g_odom_cache(),
                callback_action="immediate_return_without_odom_processing")
            return "reset_return"
        return decision

    def odom_cb(self, msg: Odometry):
        node_now_ns = int(self.get_clock().now().nanoseconds)
        phase6g_decision = self._phase6g_preprocess_odom(msg, node_now_ns)
        if phase6g_decision in ("rejected", "reset_return"):
            return
        stamp = msg.header.stamp
        self._odom_t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self._yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self._xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if (self._state == RECOVER_EGRESS_STATE
                and self._recover_egress_phase == "RECOVER_EGRESS_EXTENSION"):
            if self._recover_egress_extension_last_xy is not None:
                self._recover_egress_extension_path += math.hypot(
                    self._xy[0] - self._recover_egress_extension_last_xy[0],
                    self._xy[1] - self._recover_egress_extension_last_xy[1])
            self._recover_egress_extension_last_xy = self._xy
        if (self._state == BOTTOM_FALLBACK_STATE
                and self._bottom_fallback_phase == "DIAGONAL_REVERSE"):
            if self._bottom_fallback_reverse_last_xy is not None:
                self._bottom_fallback_reverse_path += math.hypot(
                    self._xy[0] - self._bottom_fallback_reverse_last_xy[0],
                    self._xy[1] - self._bottom_fallback_reverse_last_xy[1])
            self._bottom_fallback_reverse_last_xy = self._xy
        if self._startup_t is None:
            self._startup_t = self._odom_t
            self._startup_xy = self._xy
        if self._straight_target_yaw is None:
            self._straight_target_yaw = self._nearest_cardinal_yaw(self._yaw)
        if phase6g_decision == "fresh":
            self._phase5x_log_event(
                "phase6g_fresh_odom_repopulated",
                callback="odom_cb",
                wall_monotonic_ns=time.monotonic_ns(),
                node_now_ns=node_now_ns,
                message_header_ns=self._phase6g_odom_header_ns(msg),
                reset_count=self._phase5x_rewind_reset_count,
                awaiting_fresh_odom=False,
                cache_after=self._phase6g_odom_cache())

    def _image_array(self, msg: Image, channels: int) -> np.ndarray | None:
        if msg.height <= 0 or msg.width <= 0 or msg.step <= 0:
            return None
        data = np.frombuffer(msg.data, dtype=np.uint8)
        row_bytes = int(msg.step)
        width_bytes = int(msg.width) * channels
        expected = int(msg.height) * row_bytes
        if data.size < expected or row_bytes < width_bytes:
            return None
        rows = data[:expected].reshape((int(msg.height), row_bytes))
        return rows[:, :width_bytes].reshape((int(msg.height), int(msg.width), channels))

    def _image_to_hsv(self, msg: Image) -> np.ndarray | None:
        encoding = (msg.encoding or "").lower()
        if encoding == "rgb8":
            image = self._image_array(msg, 3)
            return None if image is None else cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        if encoding == "bgr8":
            image = self._image_array(msg, 3)
            return None if image is None else cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if encoding == "rgba8":
            image = self._image_array(msg, 4)
            if image is None:
                return None
            rgb = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        if encoding == "bgra8":
            image = self._image_array(msg, 4)
            if image is None:
                return None
            bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        return None

    def _yellow_pose_in_top_band(self, y_max: float = YELLOW_GATE_Y_MAX) -> bool:
        if self._xy is None or self._yaw is None:
            return False
        x, y = self._xy
        yaw_ok = abs(abs(self._yaw) - math.pi) < YELLOW_GATE_YAW_TOLERANCE_RAD
        return (
            YELLOW_GATE_X_MIN <= x <= YELLOW_GATE_X_MAX
            and YELLOW_GATE_Y_MIN <= y <= y_max
            and yaw_ok
        )

    def _scaled_x(self, value: float, width: int) -> float:
        return value * float(width) / YELLOW_GATE_BASE_WIDTH

    def _scaled_y(self, value: float, height: int) -> float:
        return value * float(height) / YELLOW_GATE_BASE_HEIGHT

    def _scaled_area(self, value: float, width: int, height: int) -> float:
        return value * float(width * height) / (
            YELLOW_GATE_BASE_WIDTH * YELLOW_GATE_BASE_HEIGHT)

    def _yellow_obstacle_text(self) -> str:
        y = self._yellow_obstacle_summary
        center = (
            "None"
            if y.center_x is None or y.center_y is None
            else f"({y.center_x:.1f},{y.center_y:.1f})"
        )
        aspect = "None" if y.aspect_ratio is None else f"{y.aspect_ratio:.3f}"
        return (
            f"yellow_area={y.area:.1f} "
            f"bbox=({y.bbox_x},{y.bbox_y},{y.bbox_w},{y.bbox_h}) "
            f"center={center} aspect={aspect} blobs={y.blob_count} "
            f"consecutive={y.consecutive} gate={y.gate} "
            f"gate_reason={y.gate_reason}"
        )

    def _camera_yellow_gate_active(self) -> bool:
        if not self._yellow_obstacle_summary.gate:
            return False
        if self._now() - self._yellow_obstacle_last_seen_time > YELLOW_GATE_STALE_SEC:
            return False
        return self._yellow_obstacle_summary.pose_in_top_band

    def _pending_yellow_age(self) -> float | None:
        if not self._pending_yellow_handoff or self._pending_yellow_stamp < 0.0:
            return None
        return self._now() - self._pending_yellow_stamp

    def _clear_pending_yellow_handoff(self, reason: str):
        if not self._pending_yellow_handoff:
            return
        age = self._pending_yellow_age()
        age_text = "None" if age is None else f"{age:.3f}"
        self.get_logger().info(
            f"YELLOW handoff clear raw={self._yellow_obstacle_summary.gate} "
            f"raw_generation=unavailable raw_age=unavailable "
            f"pending=True pending_generation={self._pending_yellow_generation} "
            f"pending_age={age_text} clear_reason={reason} "
            f"state_before={self._state} state_after={self._state} "
            f"cmd=none transition_reason=none")
        self._pending_yellow_handoff = False
        self._pending_yellow_stamp = -1.0
        self._pending_yellow_generation = None

    def _post_turn_yellow_handoff_eligible(self) -> bool:
        return (
            self._state == "POST_TURN"
            and self._elapsed() >= POST_TURN_MIN_SEC
        )

    def _pending_yellow_pose_eligible(self) -> bool:
        return self._yellow_pose_in_top_band(YELLOW_HIGH_CONF_GATE_Y_MAX)

    def _pending_yellow_control_active(self) -> bool:
        if not self._pending_yellow_handoff:
            return False
        age = self._pending_yellow_age()
        if age is None or age < 0.0 or age > YELLOW_GATE_STALE_SEC:
            self._clear_pending_yellow_handoff("expired")
            return False
        if self._state != "STRAIGHT":
            self._clear_pending_yellow_handoff("control_state_not_straight")
            return False
        if not self._in_obstacle_zone():
            self._clear_pending_yellow_handoff("obstacle_zone_exit")
            return False
        if self._avoid_retrigger_lockout_active():
            self._clear_pending_yellow_handoff("retrigger_lockout")
            return False
        if not self._pending_yellow_pose_eligible():
            self._clear_pending_yellow_handoff("pose_exit")
            return False
        return True

    def image_cb(self, msg: Image):
        self._check_and_reset_on_time_rewind(
            int(self.get_clock().now().nanoseconds), "image_cb")
        hsv = self._image_to_hsv(msg)
        if hsv is None:
            self.get_logger().warn(
                f"CAM yellow_check unsupported encoding={msg.encoding}",
                throttle_duration_sec=2.0)
            self._yellow_obstacle_consecutive = 0
            self._yellow_obstacle_detected = False
            return

        height, width = hsv.shape[:2]
        x1 = int(YELLOW_ROI_X1_RATIO * width)
        x2 = int(YELLOW_ROI_X2_RATIO * width)
        y1 = int(YELLOW_ROI_Y1_RATIO * height)
        y2 = int(YELLOW_ROI_Y2_RATIO * height)
        roi = hsv[y1:y2, x1:x2]

        lower = np.array(YELLOW_HSV_LOWER, dtype=np.uint8)
        upper = np.array(YELLOW_HSV_UPPER, dtype=np.uint8)
        mask = cv2.inRange(roi, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blobs: list[tuple[float, int, int, int, int]] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 4.0:
                continue
            rx, ry, rw, rh = cv2.boundingRect(contour)
            blobs.append((area, x1 + rx, y1 + ry, rw, rh))
        blobs.sort(key=lambda item: item[0], reverse=True)

        area = 0.0
        bbox_x = bbox_y = bbox_w = bbox_h = 0
        center_x = center_y = aspect_ratio = None
        if blobs:
            area, bbox_x, bbox_y, bbox_w, bbox_h = blobs[0]
            center_x = bbox_x + bbox_w / 2.0
            center_y = bbox_y + bbox_h / 2.0
            aspect_ratio = bbox_h / float(bbox_w) if bbox_w > 0 else None

        area_min = self._scaled_area(YELLOW_BLOB_AREA_MIN, width, height)
        bbox_w_min = self._scaled_x(YELLOW_BBOX_W_MIN, width)
        bbox_w_max = self._scaled_x(YELLOW_BBOX_W_MAX, width)
        bbox_h_min = self._scaled_y(YELLOW_BBOX_H_MIN, height)
        bbox_h_max = self._scaled_y(YELLOW_BBOX_H_MAX, height)
        center_x_min = self._scaled_x(YELLOW_CENTER_X_MIN, width)
        center_x_max = self._scaled_x(YELLOW_CENTER_X_MAX, width)
        center_y_min = self._scaled_y(YELLOW_CENTER_Y_MIN, height)
        center_y_max = self._scaled_y(YELLOW_CENTER_Y_MAX, height)
        early_area_min = self._scaled_area(
            YELLOW_HIGH_CONF_EARLY_AREA_MIN, width, height)
        early_bbox_w_min = self._scaled_x(YELLOW_HIGH_CONF_BBOX_W_MIN, width)
        early_bbox_w_max = self._scaled_x(YELLOW_HIGH_CONF_BBOX_W_MAX, width)
        early_bbox_h_max = self._scaled_y(YELLOW_HIGH_CONF_BBOX_H_MAX, height)
        early_center_x_min = self._scaled_x(YELLOW_HIGH_CONF_CENTER_X_MIN, width)
        early_center_x_max = self._scaled_x(YELLOW_HIGH_CONF_CENTER_X_MAX, width)
        early_center_y_min = self._scaled_y(YELLOW_HIGH_CONF_CENTER_Y_MIN, height)
        early_center_y_max = self._scaled_y(YELLOW_HIGH_CONF_CENTER_Y_MAX, height)
        area_ok = area > area_min
        bbox_w_ok = bbox_w_min <= bbox_w <= bbox_w_max
        bbox_h_ok = bbox_h_min <= bbox_h <= bbox_h_max
        center_x_ok = (
            center_x is not None
            and center_x_min <= center_x <= center_x_max
        )
        center_y_ok = (
            center_y is not None
            and center_y_min <= center_y <= center_y_max
        )
        high_conf_area_min = self._scaled_area(
            YELLOW_HIGH_CONF_AREA_MIN, width, height)
        high_confidence_ok = (
            area > high_conf_area_min
            and bbox_w_ok
            and bbox_h_ok
            and center_x_ok
            and center_y_ok
        )
        early_bbox_w_ok = early_bbox_w_min <= bbox_w <= early_bbox_w_max
        early_bbox_h_ok = bbox_h_min <= bbox_h <= early_bbox_h_max
        early_center_x_ok = (
            center_x is not None
            and early_center_x_min <= center_x <= early_center_x_max
        )
        early_center_y_ok = (
            center_y is not None
            and early_center_y_min <= center_y <= early_center_y_max
        )
        high_conf_early_ok = (
            area >= early_area_min
            and early_bbox_w_ok
            and early_bbox_h_ok
            and early_center_x_ok
            and early_center_y_ok
        )
        normal_detected = (
            area_ok
            and bbox_w_ok
            and bbox_h_ok
            and center_x_ok
            and center_y_ok
        )
        normal_pose_in_top_band = self._yellow_pose_in_top_band()
        early_pose_in_top_band = self._yellow_pose_in_top_band(
            YELLOW_HIGH_CONF_GATE_Y_MAX)
        pose_in_top_band = normal_pose_in_top_band or (
            high_conf_early_ok and early_pose_in_top_band)
        detected = normal_detected or high_conf_early_ok
        if detected and pose_in_top_band:
            self._yellow_obstacle_consecutive += 1
            self._yellow_obstacle_last_seen_time = self._now()
        else:
            self._yellow_obstacle_consecutive = 0
        consecutive_ok = (
            self._yellow_obstacle_consecutive >= YELLOW_GATE_MIN_CONSECUTIVE
        )
        gate_reason = "none"
        if normal_detected and normal_pose_in_top_band and consecutive_ok:
            gate = True
            gate_reason = "consecutive_yellow_gate"
        elif normal_detected and normal_pose_in_top_band and high_confidence_ok:
            gate = True
            gate_reason = "high_conf_yellow_gate"
        elif high_conf_early_ok and early_pose_in_top_band:
            gate = True
            gate_reason = "high_conf_yellow_early_gate"
        else:
            gate = False
        false_reasons = []
        if area <= 0.0:
            false_reasons.append("no_blob")
        if not area_ok:
            false_reasons.append("area")
        if not bbox_w_ok:
            false_reasons.append("bbox_w")
        if not bbox_h_ok:
            false_reasons.append("bbox_h")
        if not center_x_ok:
            false_reasons.append("center_x")
        if not center_y_ok:
            false_reasons.append("center_y")
        if not normal_pose_in_top_band:
            false_reasons.append("pose_top_band")
        if not consecutive_ok and not high_confidence_ok and not high_conf_early_ok:
            false_reasons.append("consecutive_or_high_conf")
        false_reason = "none" if gate else ",".join(false_reasons) or "unknown"
        self._yellow_obstacle_detected = detected
        self._yellow_obstacle_summary = YellowObstacleSummary(
            detected=detected,
            gate=gate,
            gate_reason=gate_reason,
            false_reason=false_reason,
            high_conf_early_ok=high_conf_early_ok,
            pose_in_top_band=pose_in_top_band,
            area=area,
            bbox_x=bbox_x,
            bbox_y=bbox_y,
            bbox_w=bbox_w,
            bbox_h=bbox_h,
            center_x=center_x,
            center_y=center_y,
            aspect_ratio=aspect_ratio,
            blob_count=len(blobs),
            consecutive=self._yellow_obstacle_consecutive,
            image_width=width,
            image_height=height,
            roi_x1=x1,
            roi_y1=y1,
            roi_x2=x2,
            roi_y2=y2,
        )
        raw_generation = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        pending_state_eligible = (
            self._state == "STRAIGHT"
            or self._post_turn_yellow_handoff_eligible()
        )
        pending_set_eligible = (
            gate
            and pending_state_eligible
            and self._in_obstacle_zone()
            and not self._avoid_retrigger_lockout_active()
            and self._pending_yellow_pose_eligible()
            and self._yellow_obstacle_last_seen_time >= 0.0
            and 0.0 <= self._now() - self._yellow_obstacle_last_seen_time
            <= YELLOW_GATE_STALE_SEC
        )
        if pending_set_eligible:
            pending_reason = (
                "post_turn_ready_valid_true"
                if self._state == "POST_TURN"
                else "straight_valid_true"
            )
            self._pending_yellow_handoff = True
            self._pending_yellow_stamp = self._yellow_obstacle_last_seen_time
            self._pending_yellow_generation = raw_generation
            self.get_logger().info(
                f"YELLOW handoff set raw=True raw_generation={raw_generation} "
                f"raw_age={self._now() - self._yellow_obstacle_last_seen_time:.3f} "
                f"pending=True pending_generation={self._pending_yellow_generation} "
                f"pending_age={self._pending_yellow_age():.3f} "
                f"pending_set_reason={pending_reason} "
                f"state_before={self._state} state_after={self._state} "
                f"cmd=none transition_reason=none")
        elif self._pending_yellow_handoff:
            pending_age = self._pending_yellow_age()
            if pending_age is None or pending_age < 0.0 or pending_age > YELLOW_GATE_STALE_SEC:
                self._clear_pending_yellow_handoff("expired_on_camera")
            elif self._state not in {"POST_TURN", "STRAIGHT"}:
                self._clear_pending_yellow_handoff("camera_state_ineligible")
            elif not self._in_obstacle_zone():
                self._clear_pending_yellow_handoff("obstacle_zone_exit_on_camera")
            elif self._avoid_retrigger_lockout_active():
                self._clear_pending_yellow_handoff("retrigger_lockout_on_camera")
            elif not self._pending_yellow_pose_eligible():
                self._clear_pending_yellow_handoff("pose_exit_on_camera")
            elif not gate:
                self.get_logger().info(
                    f"YELLOW handoff preserve raw=False raw_generation={raw_generation} "
                    f"raw_age={self._now() - self._yellow_obstacle_last_seen_time:.3f} "
                    f"pending=True pending_generation={self._pending_yellow_generation} "
                    f"pending_age={pending_age:.3f} "
                    f"false_overwrite_prevented=1 false_reason={false_reason} "
                    f"state_before={self._state} state_after={self._state} "
                    f"cmd=none transition_reason=none")
        center_text = (
            "None" if center_x is None or center_y is None
            else f"({center_x:.1f},{center_y:.1f})"
        )
        stale_age = (
            None
            if self._yellow_obstacle_last_seen_time < 0.0
            else self._now() - self._yellow_obstacle_last_seen_time
        )
        stale_ok = (
            stale_age is not None
            and stale_age <= YELLOW_GATE_STALE_SEC
        )
        stale_text = "None" if stale_age is None else f"{stale_age:.2f}"
        camera_gate_active = self._camera_yellow_gate_active()
        state_is_straight = self._state == "STRAIGHT"
        obstacle_zone = self._in_obstacle_zone()
        retrigger_locked = self._avoid_retrigger_lockout_active()
        trigger_ready = (
            state_is_straight
            and obstacle_zone
            and not retrigger_locked
            and camera_gate_active
        )
        self.get_logger().info(
            f"CAM yellow_check area={area:.1f} "
            f"area_min={area_min:.1f} area_ok={area_ok} "
            f"bbox=({bbox_x},{bbox_y},{bbox_w},{bbox_h}) "
            f"bbox_w_min={bbox_w_min:.1f} bbox_w_max={bbox_w_max:.1f} "
            f"bbox_w_ok={bbox_w_ok} "
            f"bbox_h_min={bbox_h_min:.1f} bbox_h_max={bbox_h_max:.1f} "
            f"bbox_h_ok={bbox_h_ok} "
            f"center={center_text} "
            f"center_x_min={center_x_min:.1f} center_x_max={center_x_max:.1f} "
            f"center_x_ok={center_x_ok} "
            f"center_y_min={center_y_min:.1f} center_y_max={center_y_max:.1f} "
            f"center_y_ok={center_y_ok} "
            f"detected={detected} "
            f"pose_in_top_band={pose_in_top_band} "
            f"normal_pose_in_top_band={normal_pose_in_top_band} "
            f"early_pose_in_top_band={early_pose_in_top_band} "
            f"consecutive={self._yellow_obstacle_consecutive} "
            f"consecutive_min={YELLOW_GATE_MIN_CONSECUTIVE} "
            f"consecutive_ok={consecutive_ok} "
            f"high_conf_area_min={high_conf_area_min:.1f} "
            f"high_confidence_ok={high_confidence_ok} "
            f"early_area_min={early_area_min:.1f} "
            f"early_bbox_w_min={early_bbox_w_min:.1f} "
            f"early_bbox_w_max={early_bbox_w_max:.1f} "
            f"early_bbox_h_max={early_bbox_h_max:.1f} "
            f"early_center_x_min={early_center_x_min:.1f} "
            f"early_center_x_max={early_center_x_max:.1f} "
            f"early_center_y_min={early_center_y_min:.1f} "
            f"early_center_y_max={early_center_y_max:.1f} "
            f"high_conf_early_ok={high_conf_early_ok} "
            f"stale_age={stale_text} stale_max={YELLOW_GATE_STALE_SEC:.2f} "
            f"stale_ok={stale_ok} "
            f"camera_yellow_gate={camera_gate_active} "
            f"gate_reason={gate_reason} "
            f"false_reason={false_reason} "
            f"state={self._state} state_is_straight={state_is_straight} "
            f"obstacle_zone={obstacle_zone} retrigger_locked={retrigger_locked} "
            f"trigger={trigger_ready} "
            f"roi=x{x1}:{x2},y{y1}:{y2} image={width}x{height} "
            f"{self._pose_text()}",
            throttle_duration_sec=0.5)

    def line_cb(self, msg: Float32MultiArray):
        self._check_and_reset_on_time_rewind(
            int(self.get_clock().now().nanoseconds), "line_cb")
        if len(msg.data) < 9:
            return
        if self._state != "STRAIGHT":
            return
        now = self._now()
        orange_trig  = msg.data[LD_ORANGE_TRIG] > 0.5
        blue_trig    = msg.data[LD_BLUE_TRIG]   > 0.5
        if not orange_trig and not blue_trig:
            return
        if self._startup_line_lockout_active():
            self.get_logger().info(
                f"Line trigger ignored during startup "
                f"elapsed={self._startup_elapsed():.2f}s "
                f"dist={self._startup_distance():.3f}m "
                f"orange={float(orange_trig):.0f} blue={float(blue_trig):.0f} "
                f"{self._pose_text()}",
                throttle_duration_sec=0.5)
            return
        if now < self._last_line_trigger_t + LINE_TRIGGER_COOLDOWN_SEC:
            return
        trigger_dist = self._distance_from_xy(self._last_line_trigger_xy)
        if trigger_dist is not None and trigger_dist < LINE_TRIGGER_LOCKOUT_DIST:
            return

        if orange_trig:
            turn_direction = RIGHT_TURN_DIRECTION
            reason = "orange line trigger -> right"
        elif blue_trig:
            turn_direction = LEFT_TURN_DIRECTION
            reason = "blue line trigger -> left"
        else:
            return

        previous_last_line_trigger_t = self._last_line_trigger_t
        self._line_turn_direction = turn_direction
        self._line_reason = reason
        self._line_latched_until = self._now() + LINE_TRIGGER_LATCH_SEC
        self._last_line_trigger_t = now
        self._last_line_trigger_xy = self._xy
        self._start_approach(reason, turn_direction, trigger_dist, orange_trig, blue_trig)
        if (self._phase5x_rewind_reset_count > 0
                and not self._phase5x_first_line_logged):
            self._phase5x_first_line_logged = True
            self._phase5x_log_event(
                "first_post_rewind_line_accepted",
                callback="line_cb",
                now_ns=int(self.get_clock().now().nanoseconds),
                reset_count=self._phase5x_rewind_reset_count,
                previous_last_line_trigger_t=previous_last_line_trigger_t,
                accepted_last_line_trigger_t=self._last_line_trigger_t,
                orange_trigger=orange_trig,
                blue_trigger=blue_trig,
                resulting_state=self._state)

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

    def _avoidance_cluster_summary(self, msg: LaserScan) -> ScanClusterSummary:
        step_deg = max(1.0, math.degrees(msg.angle_increment))
        samples: list[tuple[float, float, bool]] = []
        angle = -AVOID_CLUSTER_ROI_DEG
        while angle <= AVOID_CLUSTER_ROI_DEG + 1e-6:
            idx = self._angle_to_index(msg, angle)
            near = False
            value = math.inf
            if idx is not None:
                r = msg.ranges[idx]
                if math.isfinite(r) and MIN_VALID_RANGE < r < MAX_VALID_RANGE:
                    value = float(r)
                    near = value < AVOID_CLUSTER_RANGE_THRESHOLD
            samples.append((angle, value, near))
            angle += step_deg

        clusters: list[list[tuple[float, float, bool]]] = []
        current: list[tuple[float, float, bool]] = []
        for sample in samples:
            if sample[2]:
                current.append(sample)
            elif current:
                clusters.append(current)
                current = []
        if current:
            clusters.append(current)

        nearest: ScanClusterSummary | None = None
        for cluster in clusters:
            ranges = [sample[1] for sample in cluster if math.isfinite(sample[1])]
            if not ranges:
                continue
            min_index = min(range(len(cluster)), key=lambda i: cluster[i][1])
            min_range = cluster[min_index][1]
            width_deg = cluster[-1][0] - cluster[0][0] + step_deg
            summary = ScanClusterSummary(
                cluster_count=len(clusters),
                min_range=min_range,
                angle_center_deg=cluster[min_index][0],
                width_deg=width_deg,
                point_count=len(cluster),
            )
            if nearest is None or min_range < (nearest.min_range or math.inf):
                nearest = summary

        if nearest is None:
            return ScanClusterSummary(
                cluster_count=0,
                min_range=None,
                angle_center_deg=None,
                width_deg=None,
                point_count=0,
            )
        return nearest

    def _recent_obstacle_avoidance_active(self) -> bool:
        if self._state in {
            "AVOID_LANE_SHIFT",
            "AVOID_PASS_OBSTACLE",
            "AVOID_RETURN_HEADING",
            "RETURN_TO_LINE",
            "POST_AVOID_LEAVE_TOP",
            RECOVER_EGRESS_STATE,
        }:
            return True
        return self._now() <= self._recent_obstacle_avoidance_until

    def _set_state(self, state: str, reason: str = "state_change",
                   front: float | None = None,
                   y_progress: float | None = None,
                   final_down_y_progress: float | None = None):
        previous_state = self._state
        previous_elapsed = self._now() - self._state_start_t
        if previous_state != state:
            preserve_yellow_handoff = (
                previous_state == "POST_TURN" and state == "STRAIGHT"
            )
            if state == "POST_TURN":
                self._clear_pending_yellow_handoff("new_post_turn_attempt")
            elif not preserve_yellow_handoff:
                self._clear_pending_yellow_handoff(
                    f"state_transition_{previous_state}_to_{state}")
            if state == "RECOVER_FAILED":
                self._recover_failed_count += 1
            self.get_logger().info(
                f"STATE transition from_state={previous_state} "
                f"to_state={state} reason={reason} "
                f"front={front} "
                f"post_avoid_active={self._recent_obstacle_avoidance_active()} "
                f"recent_avoid={self._recent_obstacle_avoidance_active()} "
                f"y_progress={y_progress} "
                f"final_down_y_progress={final_down_y_progress} "
                f"elapsed={previous_elapsed:.2f}s "
                f"recover_count={self._recover_count} "
                f"recover_failed_count={self._recover_failed_count} "
                f"{self._pose_text()}",
                throttle_duration_sec=1.0)
        self._state = state
        self._state_start_t = self._now()
        self._state_start_xy = self._xy
        self._state_start_yaw = self._yaw
        self._prev_wall_error = 0.0
        self._prev_wall_t = self._now()
        self.get_logger().info(f"-> {state} {self._pose_text()}")

    def _publish(self, linear: float, angular: float):
        if (self._phase6g_awaiting_fresh_odom
                and (abs(linear) > 1.0e-12 or abs(angular) > 1.0e-12)):
            self._phase5x_log_event(
                "phase6g_nonzero_command_suppressed",
                callback="publish",
                wall_monotonic_ns=time.monotonic_ns(),
                now_ns=int(self.get_clock().now().nanoseconds),
                reset_count=self._phase5x_rewind_reset_count,
                requested_linear=linear,
                requested_angular=angular,
                awaiting_fresh_odom=True,
                published_linear=0.0,
                published_angular=0.0)
            linear = 0.0
            angular = 0.0
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)
        self._phase5q_last_published_command = (linear, angular)
        if abs(linear) > 1.0e-12 or abs(angular) > 1.0e-12:
            if (self._phase5x_rewind_reset_count > 0
                    and not self._phase5x_first_nonzero_logged):
                self._phase5x_first_nonzero_logged = True
                self._phase5x_log_event(
                    "first_post_rewind_nonzero_command",
                    now_ns=int(self.get_clock().now().nanoseconds),
                    reset_count=self._phase5x_rewind_reset_count,
                    state=self._state,
                    command_linear=linear,
                    command_angular=angular)

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

    def _heading_hold_to_yaw(self, target_yaw: float | None,
                             max_angular: float) -> tuple[float, float | None]:
        if target_yaw is None or self._yaw is None:
            return 0.0, None
        yaw_error = angle_diff(target_yaw, self._yaw)
        angular = clamp(HEADING_HOLD_KP * yaw_error, -max_angular, max_angular)
        return angular, yaw_error

    def _start_recover_egress_segment(self, phase: str):
        self._recover_egress_phase = phase
        self._recover_egress_segment_start_t = self._now()
        if phase == "RECOVER_EGRESS_EXTENSION":
            self._recover_egress_extension_path = 0.0
            self._recover_egress_extension_last_xy = self._xy
            self.get_logger().info(
                "recover_egress_segment_start "
                "phase=RECOVER_EGRESS_EXTENSION "
                f"path_target={RECOVER_EGRESS_EXTENSION_PATH_M:.15f} "
                f"timeout={RECOVER_EGRESS_EXTENSION_TIMEOUT_SEC:.3f} "
                f"target_yaw={self._turn_target_yaw} "
                f"cmd_lin={RECOVER_BACKUP_SPEED:.3f} "
                f"angular_cap={POST_TURN_HEADING_MAX_ANGULAR:.3f} "
                f"{self._pose_text()}")
            return
        self.get_logger().info(
            "recover_egress_segment_start phase=ALIGN_DOWN "
            f"target_yaw={-math.pi / 2.0:+.15f} "
            "cmd_lin=0.000 cmd_ang=+0.600 "
            f"timeout={RECOVER_EGRESS_ALIGN_DOWN_TIMEOUT_SEC:.3f} "
            f"{self._pose_text()}")

    def _start_recover_egress_route(self, front: float | None):
        self._recover_egress_route_start_t = self._now()
        self._recover_egress_extension_path = 0.0
        self._recover_egress_extension_last_xy = self._xy
        self._set_state(
            RECOVER_EGRESS_STATE,
            "first_upper_pass_recover_clear", front)
        self.get_logger().info(
            "recover_egress_route_start "
            f"hard_timeout={RECOVER_EGRESS_ROUTE_HARD_TIMEOUT_SEC:.3f} "
            f"front={front} {self._pose_text()}")
        self._start_recover_egress_segment("RECOVER_EGRESS_EXTENSION")
        self._recover_egress_route_tick(front)

    def _fail_recover_egress_route(
            self, reason: str, segment_elapsed: float,
            route_elapsed: float, heading_error: float | None):
        self._stop()
        self.get_logger().error(
            "recover_egress_failure "
            f"reason={reason} phase={self._recover_egress_phase} "
            f"segment_elapsed={segment_elapsed:.6f} "
            f"route_elapsed={route_elapsed:.6f} "
            f"path_progress={self._recover_egress_extension_path:.15f} "
            f"heading_err={heading_error} {self._pose_text()}")
        self._recover_egress_phase = "FAILED"

    def _recover_egress_route_tick(self, front: float | None):
        if (self._recover_egress_route_start_t is None
                or self._recover_egress_segment_start_t is None):
            self._fail_recover_egress_route(
                "missing_route_local_time", 0.0, 0.0, None)
            return
        route_elapsed = self._now() - self._recover_egress_route_start_t
        segment_elapsed = self._now() - self._recover_egress_segment_start_t
        phase = self._recover_egress_phase
        if phase == "RECOVER_EGRESS_EXTENSION":
            command_angular, heading_error = self._heading_hold_angular(
                POST_TURN_HEADING_MAX_ANGULAR)
            complete = (
                self._recover_egress_extension_path
                >= RECOVER_EGRESS_EXTENSION_PATH_M)
            timeout = RECOVER_EGRESS_EXTENSION_TIMEOUT_SEC
            command_linear = RECOVER_BACKUP_SPEED
        elif phase == "ALIGN_DOWN":
            target_yaw = -math.pi / 2.0
            heading_error = (
                None if self._yaw is None
                else angle_diff(target_yaw, self._yaw))
            complete = (
                heading_error is not None
                and abs(heading_error) <= TURN_YAW_TOLERANCE_RAD)
            timeout = RECOVER_EGRESS_ALIGN_DOWN_TIMEOUT_SEC
            command_linear, command_angular = 0.0, 0.600
        elif phase == "FAILED":
            self._stop()
            return
        else:
            self._fail_recover_egress_route(
                f"unexpected_phase_{phase}", segment_elapsed,
                route_elapsed, None)
            return

        # A callback at or beyond either fail-safe is a failure even when the
        # formal completion predicate is also true.
        if segment_elapsed >= timeout:
            self._fail_recover_egress_route(
                "segment_timeout", segment_elapsed,
                route_elapsed, heading_error)
            return
        if route_elapsed >= RECOVER_EGRESS_ROUTE_HARD_TIMEOUT_SEC:
            self._fail_recover_egress_route(
                "route_hard_fail_safe", segment_elapsed,
                route_elapsed, heading_error)
            return
        if complete:
            completion_source = (
                "odom_path_progress"
                if phase == "RECOVER_EGRESS_EXTENSION"
                else "yaw_tolerance")
            self.get_logger().info(
                "recover_egress_segment_complete "
                f"phase={phase} completion_source={completion_source} "
                f"segment_elapsed={segment_elapsed:.6f} "
                f"route_elapsed={route_elapsed:.6f} "
                f"path_progress={self._recover_egress_extension_path:.15f} "
                f"heading_err={heading_error} "
                f"cmd_lin={command_linear:.3f} "
                f"cmd_ang={command_angular:+.3f} {self._pose_text()}")
            if phase == "RECOVER_EGRESS_EXTENSION":
                self._start_recover_egress_segment("ALIGN_DOWN")
                self._recover_egress_route_tick(front)
                return
            self.get_logger().info(
                "recover_egress_downward_handoff "
                f"route_duration={route_elapsed:.6f} "
                f"path_progress={self._recover_egress_extension_path:.15f} "
                f"{self._pose_text()}")
            self._recover_egress_phase = None
            self._recover_egress_route_start_t = None
            self._recover_egress_segment_start_t = None
            self._recover_egress_extension_last_xy = None
            self._start_post_avoid_leave_top(
                "recover_egress_downward_handoff", front, retry=True)
            return
        self._publish(command_linear, command_angular)

    def _phase5q_event(self, event: str, front: float | None,
                       scan_stamp_ns: int | None, **fields):
        route_elapsed = (
            None if self._bottom_fallback_route_start_t is None
            else self._now() - self._bottom_fallback_route_start_t)
        segment_elapsed = (
            None if self._bottom_fallback_segment_start_t is None
            else self._now() - self._bottom_fallback_segment_start_t)
        data = {
            "event": event,
            "sim_t": self._now(),
            "odom_t": self._odom_t,
            "scan_stamp_ns": scan_stamp_ns,
            "state": self._state,
            "fallback_stage": self._bottom_fallback_phase,
            "stage_elapsed": segment_elapsed,
            "route_elapsed": route_elapsed,
            "x": None if self._xy is None else self._xy[0],
            "y": None if self._xy is None else self._xy[1],
            "yaw": self._yaw,
            "current_target_yaw": self._turn_target_yaw,
            "saved_target_yaw": self._bottom_fallback_saved_target_yaw,
            "path_progress": self._bottom_fallback_reverse_path,
            "front": front,
            "front_clear": self._front_is_clear(
                front, POST_TURN_CLEAR_FRONT_DIST),
        }
        data.update(fields)
        self.get_logger().info(
            "PHASE5Q_EVENT "
            + json.dumps(data, sort_keys=True, separators=(",", ":")))

    def _reset_bottom_fallback_route(self):
        self._bottom_fallback_phase = None
        self._bottom_fallback_route_start_t = None
        self._bottom_fallback_segment_start_t = None
        self._bottom_fallback_saved_target_yaw = None
        self._bottom_fallback_reverse_path = 0.0
        self._bottom_fallback_reverse_last_xy = None

    def _start_bottom_fallback_segment(self, phase: str,
                                       front: float | None,
                                       scan_stamp_ns: int | None):
        self._bottom_fallback_phase = phase
        self._bottom_fallback_segment_start_t = self._now()
        if phase == "DIRECT_REORIENT":
            self._turn_target_yaw = BOTTOM_FALLBACK_DIRECT_TARGET_YAW
        elif phase == "DIAGONAL_REVERSE":
            self._turn_target_yaw = BOTTOM_FALLBACK_DIRECT_TARGET_YAW
            self._bottom_fallback_reverse_path = 0.0
            self._bottom_fallback_reverse_last_xy = self._xy
        elif phase == "RESTORE":
            self._turn_target_yaw = self._bottom_fallback_saved_target_yaw
        self._phase5q_event(
            "segment_start", front, scan_stamp_ns,
            transition_reason=f"enter_{phase.lower()}")

    def _fail_bottom_fallback_route(self, reason: str,
                                    front: float | None,
                                    scan_stamp_ns: int | None,
                                    heading_error: float | None):
        self._stop()
        self._phase5q_event(
            "route_failure", front, scan_stamp_ns,
            transition_reason=reason,
            heading_error=heading_error,
            command_linear=0.0,
            command_angular=0.0)
        self._set_state("RECOVER_FAILED", reason, front)
        self._reset_bottom_fallback_route()

    def _start_bottom_fallback_route(self, front: float | None,
                                     scan_stamp_ns: int | None,
                                     recover_elapsed: float,
                                     previous_command_linear: float,
                                     previous_command_angular: float):
        saved_target = self._turn_target_yaw
        if (saved_target is None or self._xy is None or self._yaw is None
                or self._odom_t is None):
            self._bottom_fallback_saved_target_yaw = saved_target
            self._bottom_fallback_route_start_t = self._now()
            self._bottom_fallback_segment_start_t = self._now()
            self._bottom_fallback_phase = "ENTRY"
            self._fail_bottom_fallback_route(
                "missing_entry_state", front, scan_stamp_ns, None)
            return
        self._bottom_fallback_saved_target_yaw = saved_target
        self._bottom_fallback_route_start_t = self._now()
        self._bottom_fallback_segment_start_t = self._now()
        self._bottom_fallback_reverse_path = 0.0
        self._bottom_fallback_reverse_last_xy = None
        self._set_state(
            BOTTOM_FALLBACK_STATE, "bottom_nonclear_at_5s", front)
        self._start_bottom_fallback_segment(
            "DIRECT_REORIENT", front, scan_stamp_ns)
        self._publish(0.0, BOTTOM_FALLBACK_TURN_ANGULAR)
        self._phase5q_event(
            "fallback_entry", front, scan_stamp_ns,
            transition_reason="bottom_nonclear_at_5s",
            recover_elapsed=recover_elapsed,
            previous_command_linear=previous_command_linear,
            previous_command_angular=previous_command_angular,
            command_linear=0.0,
            command_angular=BOTTOM_FALLBACK_TURN_ANGULAR,
            heading_error=angle_diff(
                BOTTOM_FALLBACK_DIRECT_TARGET_YAW, self._yaw),
            same_callback_first_command=True,
            stop_dwell_inserted=False)

    def _bottom_fallback_route_tick(self, front: float | None,
                                    scan_stamp_ns: int | None):
        if (self._bottom_fallback_route_start_t is None
                or self._bottom_fallback_segment_start_t is None
                or self._xy is None or self._yaw is None
                or self._odom_t is None):
            self._fail_bottom_fallback_route(
                "missing_route_state", front, scan_stamp_ns, None)
            return
        route_elapsed = self._now() - self._bottom_fallback_route_start_t
        segment_elapsed = self._now() - self._bottom_fallback_segment_start_t
        phase = self._bottom_fallback_phase
        heading_error: float | None
        if phase == "DIRECT_REORIENT":
            heading_error = angle_diff(
                BOTTOM_FALLBACK_DIRECT_TARGET_YAW, self._yaw)
            complete = abs(heading_error) <= TURN_YAW_TOLERANCE_RAD
            timeout = BOTTOM_FALLBACK_DIRECT_TIMEOUT_SEC
            command_linear = 0.0
            command_angular = BOTTOM_FALLBACK_TURN_ANGULAR
            completion_source = "yaw_tolerance"
        elif phase == "DIAGONAL_REVERSE":
            command_angular, heading_error = self._heading_hold_to_yaw(
                BOTTOM_FALLBACK_DIRECT_TARGET_YAW,
                POST_TURN_HEADING_MAX_ANGULAR)
            complete = (
                self._bottom_fallback_reverse_path
                >= BOTTOM_FALLBACK_REVERSE_PATH_M)
            timeout = BOTTOM_FALLBACK_REVERSE_TIMEOUT_SEC
            command_linear = RECOVER_BACKUP_SPEED
            completion_source = "odom_cumulative_path"
        elif phase == "RESTORE":
            heading_error = angle_diff(
                self._bottom_fallback_saved_target_yaw, self._yaw)
            complete = abs(heading_error) <= TURN_YAW_TOLERANCE_RAD
            timeout = BOTTOM_FALLBACK_RESTORE_TIMEOUT_SEC
            command_linear = 0.0
            command_angular = math.copysign(
                BOTTOM_FALLBACK_TURN_ANGULAR, heading_error)
            completion_source = "yaw_tolerance"
        else:
            self._fail_bottom_fallback_route(
                f"unexpected_phase_{phase}", front, scan_stamp_ns, None)
            return

        # Timeout-first and hard-first relative to formal completion.
        if segment_elapsed >= timeout:
            self._fail_bottom_fallback_route(
                "segment_timeout", front, scan_stamp_ns, heading_error)
            return
        if route_elapsed >= BOTTOM_FALLBACK_ROUTE_HARD_TIMEOUT_SEC:
            self._fail_bottom_fallback_route(
                "route_hard_fail_safe", front, scan_stamp_ns, heading_error)
            return
        if complete:
            overshoot = 0.0
            if phase == "DIRECT_REORIENT":
                overshoot = max(0.0, -heading_error)
            elif phase == "RESTORE":
                overshoot = max(0.0, heading_error)
            self._phase5q_event(
                "segment_complete", front, scan_stamp_ns,
                transition_reason=f"{phase.lower()}_complete",
                completion_source=completion_source,
                heading_error=heading_error,
                yaw_overshoot=overshoot,
                command_linear=command_linear,
                command_angular=command_angular)
            if phase == "DIRECT_REORIENT":
                self._start_bottom_fallback_segment(
                    "DIAGONAL_REVERSE", front, scan_stamp_ns)
                self._bottom_fallback_route_tick(front, scan_stamp_ns)
                return
            if phase == "DIAGONAL_REVERSE":
                self._start_bottom_fallback_segment(
                    "RESTORE", front, scan_stamp_ns)
                self._bottom_fallback_route_tick(front, scan_stamp_ns)
                return
            self._stop()
            self._phase5q_event(
                "deterministic_handoff", front, scan_stamp_ns,
                transition_reason="restore_complete_to_post_turn",
                completion_source="direct_post_turn",
                heading_error=heading_error,
                command_linear=0.0,
                command_angular=0.0)
            self._set_state(
                "POST_TURN", "bottom_fallback_restore_complete", front)
            self._reset_bottom_fallback_route()
            return

        self._publish(command_linear, command_angular)
        self._phase5q_event(
            "route_callback", front, scan_stamp_ns,
            transition_reason="stage_active",
            heading_error=heading_error,
            completion_source=None,
            command_linear=command_linear,
            command_angular=command_angular)

    def _current_straight_target_yaw(self) -> float | None:
        if self._straight_target_yaw is not None:
            return self._straight_target_yaw
        if self._yaw is not None:
            return self._nearest_cardinal_yaw(self._yaw)
        return None

    def _front_distance(self, s: ScanSummary) -> float | None:
        if s.front_min is not None:
            return s.front_min
        return s.front

    def _in_fast_zone(self) -> bool:
        if self._xy is None:
            return False
        return self._xy[1] < FAST_ZONE_Y_MAX and abs(self._xy[0]) < OBSTACLE_ZONE_X_LIMIT

    def _in_obstacle_zone(self) -> bool:
        if self._xy is None:
            return False
        return self._xy[1] > OBSTACLE_ZONE_Y_MIN and abs(self._xy[0]) < OBSTACLE_ZONE_X_LIMIT

    def _in_corner_zone(self) -> bool:
        if self._xy is None:
            return False
        return abs(self._xy[0]) > CORNER_ZONE_ABS_X and abs(self._xy[1]) > CORNER_ZONE_ABS_Y

    def _straight_speed(self) -> float:
        if self._startup_line_lockout_active():
            return STARTUP_SPEED
        if self._in_corner_zone():
            return CORNER_SPEED
        if self._in_obstacle_zone():
            return OBSTACLE_ZONE_SPEED
        if self._in_fast_zone():
            return FAST_STRAIGHT_SPEED
        return STRAIGHT_SPEED

    def _gap_score(self, msg: LaserScan, center_deg: float) -> float:
        values = self._sector_values(msg, center_deg, GAP_WIDTH_DEG)
        if not values:
            return 0.0
        values.sort()
        return values[len(values) // 2]

    def _choose_gap_direction(self, msg: LaserScan) -> float:
        left_score = self._gap_score(msg, GAP_LEFT_DEG)
        right_score = self._gap_score(msg, GAP_RIGHT_DEG)
        if left_score == 0.0 and right_score == 0.0:
            return 1.0
        # Gazebo's scan frame is mirrored relative to the controller's turn sign:
        # steer away from the larger measured gap sign so the robot moves into it.
        return -1.0 if left_score >= right_score else 1.0

    def _range_diff(self, a: float | None, b: float | None) -> float | None:
        if a is None or b is None:
            return None
        return abs(a - b)

    def _min_range(self, *values: float | None) -> float | None:
        valid = [value for value in values if value is not None]
        return min(valid) if valid else None

    def _max_range(self, *values: float | None) -> float | None:
        valid = [value for value in values if value is not None]
        return max(valid) if valid else None

    def _avoidance_trigger_decision(
        self,
        msg: LaserScan,
        s: ScanSummary,
    ) -> ObstacleTriggerDecision:
        front_dist = self._front_distance(s)
        diag_dist = self._min_range(s.left_front, s.right_front)
        cluster = self._avoidance_cluster_summary(msg)
        trigger_dist = self._min_range(front_dist, diag_dist, cluster.min_range)

        near_front = (
            front_dist is not None
            and EMERGENCY_FRONT_DIST < front_dist < AVOID_TRIGGER_DIST
        )
        diag_near = (
            diag_dist is not None
            and EMERGENCY_FRONT_DIST < diag_dist < AVOID_DIAG_TRIGGER_DIST
        )

        side_max = self._max_range(s.left_side, s.right_side)
        side_diff = self._range_diff(s.left_side, s.right_side)
        diag_diff = self._range_diff(s.left_front, s.right_front)
        side_clear = (
            side_max is None
            or side_max > AVOID_SIDE_CLEAR_DIST
            or (side_diff is not None and side_diff > AVOID_ASYMMETRY_MIN)
        )
        obstacle_asymmetry = (
            (diag_diff is not None and diag_diff > AVOID_ASYMMETRY_MIN)
            or (side_diff is not None and side_diff > AVOID_ASYMMETRY_MIN)
        )

        both_sides_close = (
            s.left_side is not None
            and s.right_side is not None
            and s.left_side < AVOID_SIDE_CLEAR_DIST
            and s.right_side < AVOID_SIDE_CLEAR_DIST
        )
        both_diags_close = (
            s.left_front is not None
            and s.right_front is not None
            and s.left_front < AVOID_DIAG_TRIGGER_DIST
            and s.right_front < AVOID_DIAG_TRIGGER_DIST
        )
        narrow_geometry = (
            (both_sides_close or both_diags_close)
            and not obstacle_asymmetry
        )
        cluster_obstacle = (
            cluster.min_range is not None
            and cluster.angle_center_deg is not None
            and cluster.width_deg is not None
            and cluster.min_range < AVOID_CLUSTER_OBSTACLE_MAX_RANGE
            and AVOID_CLUSTER_MIN_WIDTH_DEG <= cluster.width_deg <= AVOID_CLUSTER_MAX_WIDTH_DEG
            and AVOID_CLUSTER_MIN_POINTS <= cluster.point_count <= AVOID_CLUSTER_MAX_POINTS
            and abs(cluster.angle_center_deg) <= AVOID_CLUSTER_ROI_DEG
        )
        in_zone = self._in_obstacle_zone()
        lockout = self._avoid_retrigger_lockout_active()
        trigger = (
            in_zone
            and not lockout
            and cluster_obstacle
            and (near_front or diag_near)
            and side_clear
            and not narrow_geometry
            and trigger_dist is not None
        )
        return ObstacleTriggerDecision(
            trigger=trigger,
            trigger_dist=trigger_dist,
            near_front=near_front,
            diag_near=diag_near,
            cluster=cluster,
            cluster_obstacle=cluster_obstacle,
            side_clear=side_clear,
            obstacle_asymmetry=obstacle_asymmetry,
            narrow_geometry=narrow_geometry,
            in_zone=in_zone,
            lockout=lockout,
        )

    def _log_obstacle_trigger_debug(
        self,
        s: ScanSummary,
        decision: ObstacleTriggerDecision,
    ):
        should_log = (
            decision.in_zone
            and decision.trigger_dist is not None
            and decision.trigger_dist < AVOID_TRIGGER_DEBUG_DIST
        )
        if not should_log:
            return
        self.get_logger().info(
            f"AVOID cluster_check front={self._fmt_range(s.front)} "
            f"front_min={self._fmt_range(s.front_min)} "
            f"LF={self._fmt_range(s.left_front)} RF={self._fmt_range(s.right_front)} "
            f"LS={self._fmt_range(s.left_side)} RS={self._fmt_range(s.right_side)} "
            f"near_front={decision.near_front} diag_near={decision.diag_near} "
            f"cluster_count={decision.cluster.cluster_count} "
            f"cluster_min={self._fmt_range(decision.cluster.min_range)} "
            f"cluster_center={self._fmt_range(decision.cluster.angle_center_deg)} "
            f"cluster_width={self._fmt_range(decision.cluster.width_deg)} "
            f"cluster_count_points={decision.cluster.point_count} "
            f"cluster_obstacle={decision.cluster_obstacle} "
            f"side_clear={decision.side_clear} "
            f"obstacle_asymmetry={decision.obstacle_asymmetry} "
            f"narrow_geometry={decision.narrow_geometry} "
            f"lockout={decision.lockout} lidar_candidate={decision.trigger} "
            f"trigger_dist={self._fmt_range(decision.trigger_dist)} "
            f"{self._pose_text()}",
            throttle_duration_sec=0.5,
        )

    def _avoid_retrigger_lockout_active(self) -> bool:
        time_active = self._now() < self._avoid_lockout_until
        dist = self._distance_from_xy(self._last_avoid_exit_xy)
        dist_active = (
            dist is not None
            and dist < AVOID_RETRIGGER_LOCKOUT_DIST
        )
        return time_active or dist_active

    def _post_avoid_push_active(self) -> bool:
        return self._now() < self._post_avoid_push_until

    def _post_avoid_top_corner_guard_active(self) -> bool:
        return self._now() < self._post_avoid_top_corner_guard_until

    def _post_avoid_top_corner_pose(self,
                                    x_limit: float = POST_AVOID_TOP_CORNER_APPROACH_X_MAX
                                    ) -> bool:
        if self._xy is None:
            return False
        x, y = self._xy
        return y > POST_AVOID_TOP_CORNER_Y_MIN and x < x_limit

    def _post_avoid_top_corner_approach_active(self) -> bool:
        return (
            self._post_avoid_top_corner_guard_active()
            and self._post_avoid_top_corner_pose()
        )

    def _post_avoid_top_corner_turn_active(self) -> bool:
        return (
            self._post_avoid_top_corner_guard_active()
            and self._post_avoid_top_corner_pose(
                POST_AVOID_TOP_CORNER_TURN_X_MAX)
        )

    def _line_approach_dist(self) -> float:
        if self._post_avoid_top_corner_approach_active():
            return POST_AVOID_TOP_CORNER_APPROACH_DIST
        return LINE_APPROACH_DIST

    def _start_avoid_retrigger_lockout(self, reason: str):
        self._avoid_lockout_until = self._now() + AVOID_RETRIGGER_LOCKOUT_SEC
        self._post_avoid_push_until = self._now() + POST_AVOID_PUSH_SEC
        self._recent_obstacle_avoidance_until = self._now() + POST_AVOID_RECENT_SEC
        if self._xy is not None and self._xy[1] > POST_AVOID_TOP_CORNER_Y_MIN:
            self._post_avoid_top_corner_guard_until = (
                self._now() + POST_AVOID_TOP_CORNER_GUARD_SEC)
        self._last_avoid_exit_xy = self._xy
        self.get_logger().info(
            f"AVOID retrigger lockout reason={reason} "
            f"sec={AVOID_RETRIGGER_LOCKOUT_SEC:.1f} "
            f"dist={AVOID_RETRIGGER_LOCKOUT_DIST:.3f} "
            f"push_sec={POST_AVOID_PUSH_SEC:.1f} "
            f"top_corner_guard_active={self._post_avoid_top_corner_guard_active()} "
            f"{self._pose_text()}")

    def _start_avoid_phase(self, state: str, reason: str,
                           front: float | None):
        if state in ("AVOID_RETURN_HEADING", "RETURN_TO_LINE"):
            self._stop()
        self._set_state(state)
        if state == "AVOID_LANE_SHIFT":
            self._lane_shift_previous_yaw = self._yaw
            self._lane_shift_yaw_progress = 0.0
            self._lane_shift_positive_yaw_sum = 0.0
            self._lane_shift_negative_yaw_sum = 0.0
            self.get_logger().warn(
                f"lane_shift_enter elapsed=0.00 "
                f"cumulative_progress={self._lane_shift_yaw_progress:.3f} "
                f"positive_delta_sum={self._lane_shift_positive_yaw_sum:.3f} "
                f"negative_delta_sum={self._lane_shift_negative_yaw_sum:.3f} "
                f"net_yaw_progress=0.000 current_yaw={self._yaw} "
                f"previous_yaw={self._lane_shift_previous_yaw} "
                f"entry_yaw={self._state_start_yaw} "
                f"cmd_lin={AVOID_LANE_SHIFT_SPEED:.3f} "
                f"cmd_ang={AVOID_LANE_SHIFT_ANGULAR:+.3f}")
        self.get_logger().warn(
            f"{state} start reason={reason} elapsed=0.00s dist=0.000 "
            f"front={front} avoid_direction={self._avoid_direction:+.0f} "
            f"{self._pose_text()}")

    def _needs_post_avoid_leave_top(self) -> bool:
        return (
            self._xy is not None
            and self._xy[1] > POST_AVOID_LEAVE_TOP_Y_MIN
        )

    def _post_clearance_geometry(self) -> tuple[
            float | None, float | None, bool | None,
            str | None, float | None, float | None]:
        if self._xy is None or self._yaw is None:
            return None, None, None, None, None, None
        x, y = self._xy
        footprint_min_x, footprint_max_x = _base_collision_world_x_bounds(
            x, self._yaw)
        down_path_clear = _is_down_path_geometrically_clear(x, y, self._yaw)
        selected_side, right_required, left_required = (
            _select_down_path_clearance_side(x, self._yaw)
        )
        return (
            footprint_min_x, footprint_max_x, down_path_clear,
            selected_side, right_required, left_required,
        )

    def _log_post_clearance_transition(
            self, old_subphase: str, new_subphase: str,
            transition_reason: str, cmd_source: str,
            target_yaw: float | None, planned_linear: float,
            planned_angular: float):
        (footprint_min_x, footprint_max_x, down_path_clear,
         selected_side, right_required, left_required) = (
            self._post_clearance_geometry()
        )
        expected_world_vx = (
            None if self._yaw is None
            else planned_linear * math.cos(self._yaw)
        )
        expected_world_vy = (
            None if self._yaw is None
            else planned_linear * math.sin(self._yaw)
        )
        self.get_logger().warn(
            f"POST clearance transition state={self._state} "
            f"old_subphase={old_subphase} new_subphase={new_subphase} "
            f"transition_reason={transition_reason} cmd_source={cmd_source} "
            f"x={self._xy[0] if self._xy else None} "
            f"y={self._xy[1] if self._xy else None} yaw={self._yaw} "
            f"footprint_min_x={footprint_min_x} "
            f"footprint_max_x={footprint_max_x} "
            f"wall_min_x={POST_AVOID_ISLAND_NORTH_WALL_MIN_X} "
            f"wall_max_x={POST_AVOID_ISLAND_NORTH_WALL_MAX_X} "
            f"margin={POST_AVOID_DOWN_PATH_MARGIN} "
            f"down_path_clear={down_path_clear} "
            f"selected_side={selected_side} "
            f"right_required={right_required} left_required={left_required} "
            f"target_yaw={target_yaw} planned_linear={planned_linear} "
            f"planned_angular={planned_angular} "
            f"expected_world_vx={expected_world_vx} "
            f"expected_world_vy={expected_world_vy} "
            f"published_linear=0.0 published_angular=0.0")

    def _start_post_clearance_escape(self, transition_reason: str,
                                     old_subphase: str):
        selected_side = self._post_clearance_geometry()[3]
        self._post_avoid_clearance_side = selected_side
        self._post_avoid_clearance_start_t = self._now()
        self._post_avoid_clearance_start_x = (
            None if self._xy is None else self._xy[0])
        self._post_avoid_final_down_drive_start_t = None
        self._post_avoid_final_down_motion_blocked_logged = False
        self._post_avoid_final_down_clearance_start_t = self._now()
        self._post_avoid_clearance_failed = selected_side != "right"
        self._post_avoid_clearance_subphase = (
            "peel_off" if selected_side == "right" else "blocked")
        planned_linear = (
            -POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_SPEED
            if selected_side == "right" else 0.0)
        planned_angular = (
            -POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_MAX_ANGULAR
            if selected_side == "right" else 0.0)
        self._stop()
        self._log_post_clearance_transition(
            old_subphase, self._post_avoid_clearance_subphase,
            transition_reason,
            "clearance_peel_off_right" if selected_side == "right"
            else "clearance_left_path_not_validated_stop",
            POST_AVOID_CLEARANCE_PEEL_TARGET_YAW
            if selected_side == "right" else None,
            planned_linear, planned_angular)

    def _transition_post_clearance_subphase(
            self, new_subphase: str | None, transition_reason: str,
            target_yaw: float | None, planned_linear: float,
            planned_angular: float, cmd_source: str):
        old_subphase = self._post_avoid_clearance_subphase
        self._post_avoid_clearance_subphase = new_subphase
        self._post_avoid_clearance_start_t = (
            None if new_subphase is None else self._now()
        )
        self._post_avoid_clearance_start_x = (
            None if new_subphase is None or self._xy is None else self._xy[0]
        )
        if new_subphase is None:
            self._post_avoid_final_down_clearance_start_t = None
        self._stop()
        self._log_post_clearance_transition(
            old_subphase, new_subphase or "normal_down", transition_reason,
            cmd_source, target_yaw, planned_linear, planned_angular)
        if new_subphase is None:
            self._post_avoid_clearance_side = None

    def _control_post_clearance_escape(self) -> bool:
        subphase = self._post_avoid_clearance_subphase
        if subphase is None and not self._post_avoid_clearance_failed:
            return False
        if self._xy is None or self._yaw is None:
            self._stop()
            return True
        current_x, _ = self._xy
        (_, _, down_path_clear, selected_side,
         _, _) = self._post_clearance_geometry()
        if self._post_avoid_clearance_failed or selected_side != "right":
            self._post_avoid_clearance_failed = True
            self._stop()
            return True
        clearance_elapsed = (
            None if self._post_avoid_clearance_start_t is None
            else self._now() - self._post_avoid_clearance_start_t
        )
        clearance_x_progress = (
            None if self._post_avoid_clearance_start_x is None
            else current_x - self._post_avoid_clearance_start_x
        )
        if (clearance_elapsed is not None
                and clearance_elapsed >= POST_AVOID_LEAVE_TOP_GOAL_PROGRESS_SEC
                and clearance_x_progress is not None
                and clearance_x_progress < POST_AVOID_LEAVE_TOP_GOAL_PROGRESS_MIN):
            self._post_avoid_clearance_failed = True
            self._transition_post_clearance_subphase(
                "blocked", "clearance_no_positive_x_progress", None,
                0.0, 0.0, "clearance_no_progress_stop")
            return True
        if subphase == "peel_off":
            peel_heading_error = angle_diff(
                POST_AVOID_CLEARANCE_PEEL_TARGET_YAW, self._yaw)
            if (down_path_clear
                    and abs(peel_heading_error) <= POST_AVOID_CLEARANCE_YAW_TOLERANCE):
                self._transition_post_clearance_subphase(
                    "align_after_clearance", "right_footprint_clear_begin_align",
                    POST_AVOID_CLEARANCE_ALIGN_TARGET_YAW,
                    -POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_SPEED,
                    POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_MAX_ANGULAR,
                    "clearance_align_right")
                return True
            self._publish(
                -POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_SPEED,
                math.copysign(
                    POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_MAX_ANGULAR,
                    peel_heading_error))
            return True
        if subphase == "align_after_clearance":
            if not down_path_clear:
                self._post_avoid_clearance_failed = True
                self._transition_post_clearance_subphase(
                    "blocked", "clearance_lost_during_align", None,
                    0.0, 0.0, "clearance_lost_stop")
                return True
            align_error = angle_diff(
                POST_AVOID_CLEARANCE_ALIGN_TARGET_YAW, self._yaw)
            if abs(align_error) <= POST_AVOID_CLEARANCE_YAW_TOLERANCE:
                self._transition_post_clearance_subphase(
                    None, "right_clearance_aligned_complete",
                    POST_AVOID_CLEARANCE_ALIGN_TARGET_YAW,
                    0.0, 0.0, "clearance_complete_stop")
                return True
            align_angular = (
                POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_MAX_ANGULAR
                if align_error > 0.0
                else -POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_MAX_ANGULAR
            )
            self._publish(
                -POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_SPEED,
                align_angular)
            return True
        self._stop()
        return True

    def _start_post_avoid_leave_top(self, reason: str, front: float | None,
                                    retry: bool = False):
        self._stop()
        phase = "waypoint_escape"
        if self._xy is not None:
            dx_to_goal = POST_AVOID_LEAVE_TOP_GOAL_X - self._xy[0]
            dy_to_goal = POST_AVOID_LEAVE_TOP_GOAL_Y - self._xy[1]
            target_yaw = math.atan2(dy_to_goal, dx_to_goal)
            dist_to_goal = math.hypot(dx_to_goal, dy_to_goal)
        else:
            dx_to_goal = None
            dy_to_goal = None
            dist_to_goal = None
            target_yaw = POST_AVOID_LEAVE_TOP_TARGET_YAW
        self._straight_target_yaw = target_yaw
        self._post_avoid_leave_top_last_x = (
            None if self._xy is None else self._xy[0]
        )
        self._post_avoid_leave_top_last_y = (
            None if self._xy is None else self._xy[1]
        )
        if retry:
            phase = "top_exit_retry"
            target_yaw = POST_AVOID_LEAVE_TOP_TARGET_YAW
            self._straight_target_yaw = target_yaw
            self._post_avoid_final_down_entry_reason = reason
            self._post_avoid_final_down_start_t = self._now()
            self._post_avoid_final_down_start_xy = self._xy
            self._post_avoid_top_exit_retry_reason = reason
        else:
            self._post_avoid_final_down_entry_reason = None
            self._post_avoid_final_down_start_t = None
            self._post_avoid_final_down_start_xy = None
            self._post_avoid_top_exit_retry_count = 0
            self._post_avoid_top_exit_retry_reason = None
        self._post_avoid_final_down_drive_start_t = None
        self._post_avoid_final_down_clearance_start_t = None
        self._post_avoid_final_down_motion_blocked_logged = False
        self._post_avoid_clearance_subphase = None
        self._post_avoid_clearance_side = None
        self._post_avoid_clearance_start_t = None
        self._post_avoid_clearance_start_x = None
        self._post_avoid_clearance_failed = False
        self._post_avoid_final_down_extension_count = 0
        self._post_avoid_final_down_extension_until = None
        self._post_avoid_recent_dy = []
        self._set_state("POST_AVOID_LEAVE_TOP", reason, front)
        heading_error = (
            None
            if self._yaw is None
            else abs(angle_diff(target_yaw, self._yaw))
        )
        self.get_logger().warn(
            f"POST_AVOID_LEAVE_TOP start reason={reason} "
            f"phase={phase} retry={retry} "
            f"retry_count={self._post_avoid_top_exit_retry_count} "
            f"goal_x={POST_AVOID_LEAVE_TOP_GOAL_X:+.3f} "
            f"goal_y={POST_AVOID_LEAVE_TOP_GOAL_Y:+.3f} "
            f"dx_to_goal={dx_to_goal} dy_to_goal={dy_to_goal} "
            f"dist_to_goal={dist_to_goal} "
            f"target_yaw={target_yaw:+.3f} "
            f"heading_err={heading_error} front={front} "
            f"start_x={self._state_start_xy[0] if self._state_start_xy else None} "
            f"start_y={self._state_start_xy[1] if self._state_start_xy else None} "
            f"cmd_lin=0.000 cmd_ang=0.000 {self._pose_text()}")

    def _try_reenter_top_exit(self, reason: str, front: float | None,
                              y_progress: float | None = None,
                              final_down_y_progress: float | None = None) -> bool:
        current_y = None if self._xy is None else self._xy[1]
        if (
                current_y is None
                or current_y <= POST_AVOID_LEAVE_TOP_EXIT_Y
                or current_y > POST_AVOID_TOP_EXIT_RETRY_Y_MAX
                or not self._recent_obstacle_avoidance_active()
                or self._recover_failed_count > 0):
            return False
        if front is not None and front < POST_AVOID_LEAVE_TOP_FRONT_DANGER_DIST:
            self.get_logger().error(
                f"POST_AVOID_LEAVE_TOP retry blocked reason=front_danger "
                f"from_reason={reason} front={front} "
                f"retry_count={self._post_avoid_top_exit_retry_count} "
                f"recover_count={self._recover_count} "
                f"recover_failed_count={self._recover_failed_count} "
                f"{self._pose_text()}")
            return False
        if self._post_avoid_top_exit_retry_count >= POST_AVOID_TOP_EXIT_RETRY_MAX:
            self.get_logger().error(
                f"POST_AVOID_LEAVE_TOP retry exhausted "
                f"reason=top_exit_retry_exhausted from_reason={reason} "
                f"front={front} y={current_y:+.3f} "
                f"retry_count={self._post_avoid_top_exit_retry_count} "
                f"max_retry={POST_AVOID_TOP_EXIT_RETRY_MAX} "
                f"y_progress={y_progress} "
                f"final_down_y_progress={final_down_y_progress} "
                f"recover_count={self._recover_count} "
                f"recover_failed_count={self._recover_failed_count} "
                f"{self._pose_text()}")
            self._stop()
            return True
        self._post_avoid_top_exit_retry_count += 1
        self._recent_obstacle_avoidance_until = self._now() + POST_AVOID_RECENT_SEC
        self.get_logger().warn(
            f"POST_AVOID_LEAVE_TOP reenter reason={reason} "
            f"retry_count={self._post_avoid_top_exit_retry_count} "
            f"front={front} y={current_y:+.3f} "
            f"y_progress={y_progress} "
            f"final_down_y_progress={final_down_y_progress} "
            f"recover_count={self._recover_count} "
            f"recover_failed_count={self._recover_failed_count} "
            f"{self._pose_text()}")
        self._start_post_avoid_leave_top(
            f"reenter_top_exit_after_{reason}", front, retry=True)
        return True

    def _transition_avoid_phase(self, next_state: str, reason: str,
                                elapsed: float, distance: float,
                                front: float | None):
        current_state = self._state
        heading_error = None
        target_yaw = self._avoid_base_yaw
        if current_state == "AVOID_LANE_SHIFT":
            target_yaw = self._avoid_shift_target_yaw
        elif current_state == "AVOID_PASS_OBSTACLE":
            target_yaw = self._avoid_pass_target_yaw
        if self._yaw is not None and target_yaw is not None:
            heading_error = abs(angle_diff(target_yaw, self._yaw))
        self.get_logger().warn(
            f"{current_state} exit reason={reason} elapsed={elapsed:.2f}s "
            f"dist={distance:.3f} front={front} "
            f"heading_err={heading_error} "
            f"avoid_direction={self._avoid_direction:+.0f} {self._pose_text()}")
        self._start_avoid_phase(next_state, f"{current_state}:{reason}", front)

    def _start_avoidance(self, msg: LaserScan, front: float | None,
                         reason: str = "obstacle_trigger"):
        self._recent_obstacle_avoidance_until = self._now() + POST_AVOID_RECENT_SEC
        self._post_avoid_top_exit_retry_count = 0
        self._post_avoid_top_exit_retry_reason = None
        self._avoid_direction = self._choose_gap_direction(msg)
        base_yaw = self._current_straight_target_yaw()
        if base_yaw is None and self._yaw is not None:
            base_yaw = self._nearest_cardinal_yaw(self._yaw)
        self._avoid_base_yaw = base_yaw
        self._avoid_shift_target_yaw = (
            None if base_yaw is None
            else normalize_angle(
                base_yaw + self._avoid_direction * AVOID_SHIFT_ANGLE_RAD)
        )
        self._avoid_pass_target_yaw = base_yaw
        self._clear_line_latch()
        self._start_avoid_phase("AVOID_LANE_SHIFT", reason, front)
        if reason == "camera_yellow_gate":
            self.get_logger().warn(
                f"AVOID start reason=camera_yellow_gate "
                f"{self._yellow_obstacle_text()} {self._pose_text()}")

    def _start_approach(self, reason: str, direction: float,
                        trigger_dist: float | None = None,
                        orange_trig: bool = False,
                        blue_trig: bool = False):
        if self._state != "STRAIGHT":
            return
        self._turn_direction = LEFT_TURN_DIRECTION if direction > 0.0 else RIGHT_TURN_DIRECTION
        self._line_turn_direction = self._turn_direction
        self._set_state("APPROACH")
        trigger_dist_text = (
            "None" if trigger_dist is None else f"{trigger_dist:.3f}m"
        )
        self.get_logger().info(
            f"APPROACH start direction={self._turn_direction:+.0f} "
            f"reason={reason} dist_from_last_trigger={trigger_dist_text} "
            f"orange={float(orange_trig):.0f} blue={float(blue_trig):.0f} "
            f"{self._pose_text()}")

    def _start_turn(self, reason: str, direction: float | None = None,
                    scan: ScanSummary | None = None):
        if direction is not None and direction != 0.0:
            self._turn_direction = LEFT_TURN_DIRECTION if direction > 0.0 else RIGHT_TURN_DIRECTION
        self._turn_start_yaw = self._yaw
        if self._turn_start_yaw is None:
            self._turn_target_yaw = None
            self.get_logger().warn(
                f"TURN start without odom yaw direction={self._turn_direction:+.0f} "
                f"linear={TURN_LINEAR_SPEED:.3f} "
                f"{self._pose_text()} reason={reason}")
        else:
            base_yaw = (self._straight_target_yaw
                        if self._straight_target_yaw is not None
                        else self._turn_start_yaw)
            self._turn_target_yaw = normalize_angle(
                base_yaw + self._turn_direction * TURN_ANGLE_RAD)
            self.get_logger().info(
                f"TURN start direction={self._turn_direction:+.0f} "
                f"linear={TURN_LINEAR_SPEED:.3f} "
                f"start_yaw={self._turn_start_yaw:+.3f} "
                f"target_yaw={self._turn_target_yaw:+.3f} "
                f"straight_target={self._straight_target_yaw} "
                f"{self._pose_text()} reason={reason}")
        self._clear_line_latch()
        self._set_state("TURN")
        self._publish(TURN_LINEAR_SPEED, self._turn_angular())

    def _start_recover(self, reason: str, scan: ScanSummary | None = None,
                       front: float | None = None):
        if self._state == "RECOVER_FAILED":
            return
        previous_state = self._state
        self._recover_source_state = previous_state
        self._clear_line_latch()
        self._stop()
        self._recover_count += 1
        self._set_state("RECOVER", reason, front)
        bottom_context = self._bottom_blue_line_recover_context()
        if bottom_context and self._yaw is not None:
            previous_target_yaw = self._turn_target_yaw
            self._bottom_recover_entry_yaw = self._yaw
            self._bottom_recover_previous_target_yaw = previous_target_yaw
            self._turn_start_yaw = self._yaw
            bottom_turn_direction = self._line_turn_direction
            self._turn_direction = bottom_turn_direction
            base_yaw = (self._straight_target_yaw
                        if self._straight_target_yaw is not None
                        else self._nearest_cardinal_yaw(self._yaw))
            self._turn_target_yaw = normalize_angle(
                base_yaw + bottom_turn_direction * TURN_ANGLE_RAD)
            initialized_heading_error = angle_diff(
                self._turn_target_yaw, self._yaw)
            selected_angular_sign = (
                1 if initialized_heading_error > 0.0
                else -1 if initialized_heading_error < 0.0
                else 0)
            self.get_logger().warn(
                f"bottom_recover_target_init "
                f"recover_source_state={previous_state} "
                f"bottom_context=True "
                f"entry_yaw={self._bottom_recover_entry_yaw:+.6f} "
                f"previous_target_yaw={previous_target_yaw} "
                f"base_yaw={base_yaw:+.6f} "
                f"turn_direction={bottom_turn_direction:+.0f} "
                f"initialized_target_yaw={self._turn_target_yaw:+.6f} "
                f"heading_err={initialized_heading_error:+.6f} "
                f"selected_angular_sign={selected_angular_sign:+d} "
                f"cmd_lin=0.000 cmd_ang=+0.000 "
                f"transition_reason=bottom_blue_line_recover_entry "
                f"{self._pose_text()}")
        self.get_logger().warn(
            f"recover_enter recover_source_state={previous_state} "
            f"bottom_context={bottom_context} "
            f"RECOVER start from={previous_state} "
            f"front={self._fmt_range(front)} {self._pose_text()} "
            f"{self._scan_text(scan)} reason={reason} "
            f"recent_avoid={self._recent_obstacle_avoidance_active()} "
            f"recover_count={self._recover_count} "
            f"recover_failed_count={self._recover_failed_count}")

    def _bottom_blue_line_recover_context(self) -> bool:
        return (
            self._recover_source_state in {"APPROACH", "TURN"}
            and self._line_reason == "blue line trigger -> left"
            and self._xy is not None
            and self._xy[1] < FAST_ZONE_Y_MAX
        )

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
            return self._turn_direction * MIN_TURN_ANGULAR

        angular = clamp(TURN_KP * yaw_error, -MAX_TURN_ANGULAR, MAX_TURN_ANGULAR)

        # Soft speed cap after 60 deg to reduce overshoot
        if progress > math.radians(TURN_SOFT_CAP_DEG):
            angular = clamp(angular, -TURN_SOFT_CAP_ANGULAR, TURN_SOFT_CAP_ANGULAR)

        if abs(yaw_error) > TURN_YAW_TOLERANCE_RAD:
            min_speed = MIN_TURN_ANGULAR if abs(yaw_error) < math.radians(25.0) else 0.36
            if abs(angular) < min_speed:
                angular = math.copysign(min_speed, yaw_error)

        return angular

    def scan_cb(self, msg: LaserScan):
        self._check_and_reset_on_time_rewind(
            int(self.get_clock().now().nanoseconds), "scan_cb")
        s     = self._summarize_scan(msg)
        front = self._front_distance(s)
        avoid_decision = self._avoidance_trigger_decision(msg, s)
        scan_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec))

        if self._state == RECOVER_EGRESS_STATE:
            self._recover_egress_route_tick(front)
            return

        if self._state == BOTTOM_FALLBACK_STATE:
            self._bottom_fallback_route_tick(front, scan_stamp_ns)
            return

        if self._state == "STRAIGHT":
            if front is not None and front < EMERGENCY_FRONT_DIST:
                self._start_recover(
                    f"emergency front={front:.3f}m in STRAIGHT", s, front)
                return
            self._log_obstacle_trigger_debug(s, avoid_decision)
            camera_trigger = (
                self._in_obstacle_zone()
                and not self._avoid_retrigger_lockout_active()
                and self._camera_yellow_gate_active()
            )
            raw_camera_trigger = camera_trigger
            pending_camera_trigger = self._pending_yellow_control_active()
            camera_trigger = raw_camera_trigger or pending_camera_trigger
            if camera_trigger:
                consume_source = "raw" if raw_camera_trigger else "pending"
                pending_age = self._pending_yellow_age()
                pending_age_text = (
                    "None" if pending_age is None else f"{pending_age:.3f}"
                )
                raw_age = (
                    None
                    if self._yellow_obstacle_last_seen_time < 0.0
                    else self._now() - self._yellow_obstacle_last_seen_time
                )
                raw_age_text = "None" if raw_age is None else f"{raw_age:.3f}"
                self.get_logger().warn(
                    f"YELLOW handoff consume consume_source={consume_source} "
                    f"raw={raw_camera_trigger} raw_generation=latest_camera_summary "
                    f"raw_age={raw_age_text} pending={pending_camera_trigger} "
                    f"pending_generation={self._pending_yellow_generation} "
                    f"pending_age={pending_age_text} "
                    f"state_before=STRAIGHT state_after=AVOID_LANE_SHIFT "
                    f"cmd=none transition_reason=camera_yellow_gate")
                self.get_logger().warn(
                    f"AVOID trigger source=camera_yellow_gate "
                    f"{self._yellow_obstacle_text()} front={front} "
                    f"{self._pose_text()}")
                self._clear_pending_yellow_handoff(
                    f"consumed_source_{consume_source}")
                self._start_avoidance(msg, front, "camera_yellow_gate")
                return
            speed = self._straight_speed()
            if self._post_avoid_push_active():
                angular, heading_error = self._heading_hold_to_yaw(
                    self._current_straight_target_yaw(),
                    POST_AVOID_PUSH_MAX_ANGULAR)
                if heading_error is None:
                    angular = 0.0
                self._publish(POST_AVOID_PUSH_SPEED, angular)
                exit_dist = self._distance_from_xy(self._last_avoid_exit_xy)
                exit_dist_text = (
                    "None" if exit_dist is None else f"{exit_dist:.3f}"
                )
                self.get_logger().info(
                    f"STRAIGHT post_avoid_push front={front} "
                    f"exit_dist={exit_dist_text} speed={POST_AVOID_PUSH_SPEED:.3f} "
                    f"heading_err={heading_error} ang={angular:+.3f} "
                    f"zones obs={self._in_obstacle_zone()} "
                    f"{self._pose_text()}",
                    throttle_duration_sec=0.4)
                return
            angular, error = self._drive_centered(s, speed, MAX_STRAIGHT_ANGULAR)
            self.get_logger().info(
                f"STRAIGHT front={front} LS={s.left_side} RS={s.right_side} "
                f"LF={s.left_front} RF={s.right_front} "
                f"speed={speed:.3f} err={error:+.3f} ang={angular:+.3f} "
                f"zones fast={self._in_fast_zone()} obs={self._in_obstacle_zone()} "
                f"corner={self._in_corner_zone()} {self._pose_text()}",
                throttle_duration_sec=0.6)
            return

        if self._state == "APPROACH":
            if front is not None and front < EMERGENCY_FRONT_DIST:
                self._start_recover(
                    f"emergency front={front:.3f}m in APPROACH", s, front)
                return
            elapsed = self._elapsed()
            distance = self._distance_from_state_start()
            target_dist = self._line_approach_dist()
            ready_by_distance = distance >= target_dist
            ready_by_time = elapsed >= LINE_APPROACH_MAX_SEC
            if elapsed >= LINE_APPROACH_MIN_SEC and (ready_by_distance or ready_by_time):
                self._start_turn(
                    f"{self._line_reason} approach_elapsed={elapsed:.2f}s "
                    f"approach_dist={distance:.3f}m "
                    f"target_dist={target_dist:.3f}m "
                    f"post_avoid_top_corner={self._post_avoid_top_corner_approach_active()}",
                    self._line_turn_direction,
                    s)
                return
            angular, error = self._drive_centered(s, APPROACH_SPEED, 0.42)
            self.get_logger().info(
                f"APPROACH {elapsed:.2f}s dist={distance:.3f} "
                f"target_dist={target_dist:.3f} "
                f"direction={self._line_turn_direction:+.0f} front={front} "
                f"err={error:+.3f} ang={angular:+.3f} {self._pose_text()}",
                throttle_duration_sec=0.4)
            return

        if self._state == "TURN":
            self._ensure_turn_target()
            yaw_error = self._turn_yaw_error()
            progress  = self._turn_progress()
            angular = self._turn_angular()
            post_avoid_corner_turn = (
                self._post_avoid_top_corner_turn_active()
                and front is not None
                and front < POST_AVOID_TOP_CORNER_TURN_FRONT_SLOW_DIST
            )
            if post_avoid_corner_turn:
                if front < POST_AVOID_TOP_CORNER_TURN_FRONT_MIN_DIST:
                    self._start_recover(
                        f"emergency front={front:.3f}m in post-avoid top TURN "
                        f"progress={math.degrees(progress):.1f}deg",
                        s, front)
                    return
                self._publish(POST_AVOID_TOP_CORNER_TURN_LINEAR_SPEED, angular)
                yaw_text = f"{self._yaw:+.3f}" if self._yaw is not None else "None"
                target_text = (
                    f"{self._turn_target_yaw:+.3f}"
                    if self._turn_target_yaw is not None
                    else "None"
                )
                error_text = f"{yaw_error:+.3f}" if yaw_error is not None else "None"
                self.get_logger().warn(
                    f"TURN safety reason=post_avoid_top_corner "
                    f"front={front:.3f} lin={POST_AVOID_TOP_CORNER_TURN_LINEAR_SPEED:.3f} "
                    f"ang={angular:+.3f} yaw={yaw_text} target={target_text} "
                    f"heading_err={error_text} progress={math.degrees(progress):.1f}deg "
                    f"{self._pose_text()}",
                    throttle_duration_sec=0.25)
                return
            if front is not None and front < EMERGENCY_FRONT_DIST:
                self._start_recover(
                    f"emergency front={front:.3f}m in TURN "
                    f"progress={math.degrees(progress):.1f}deg",
                    s, front)
                return
            if (yaw_error is not None
                    and abs(yaw_error) <= TURN_YAW_TOLERANCE_RAD
                    and progress >= TURN_MIN_PROGRESS_RAD):
                self.get_logger().info(
                    f"TURN done by yaw current={self._yaw:+.3f} "
                    f"target={self._turn_target_yaw:+.3f} "
                    f"error={yaw_error:+.3f} progress={math.degrees(progress):.1f}deg "
                    f"front={front} {self._pose_text()}")
                self._straight_target_yaw = self._turn_target_yaw
                self._set_state("ALIGN")
                return
            if self._elapsed() > TURN_TIMEOUT_SEC:
                self.get_logger().warn(
                    f"TURN timeout elapsed={self._elapsed():.2f}s "
                    f"progress={math.degrees(progress):.1f}deg")
                if self._turn_target_yaw is not None:
                    self._straight_target_yaw = self._turn_target_yaw
                self._set_state("ALIGN")
                return
            self._publish(TURN_LINEAR_SPEED, angular)
            yaw_text = f"{self._yaw:+.3f}" if self._yaw is not None else "None"
            target_text = (
                f"{self._turn_target_yaw:+.3f}"
                if self._turn_target_yaw is not None
                else "None"
            )
            error_text = f"{yaw_error:+.3f}" if yaw_error is not None else "None"
            self.get_logger().info(
                f"TURN {self._elapsed():.2f}/{TURN_TIMEOUT_SEC:.2f}s "
                f"yaw={yaw_text} target={target_text} "
                f"error={error_text} progress={math.degrees(progress):.1f}deg "
                f"front={front} lin={TURN_LINEAR_SPEED:.3f} ang={angular:+.3f}",
                throttle_duration_sec=0.25)
            return

        if self._state == "ALIGN":
            elapsed = self._elapsed()
            if front is not None and front < EMERGENCY_FRONT_DIST:
                self._start_recover(
                    f"emergency front={front:.3f}m in ALIGN", s, front)
                return
            angular, heading_error = self._heading_hold_angular(ALIGN_HEADING_MAX_ANGULAR)
            if heading_error is None:
                angular, heading_error = self._wall_angular(s, ALIGN_HEADING_MAX_ANGULAR)
            self._publish(ALIGN_SPEED, angular)
            can_exit = elapsed >= ALIGN_MIN_SEC
            if can_exit:
                self._clear_line_latch()
                self._set_state("POST_TURN")
                return
            if elapsed >= ALIGN_MAX_SEC:
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
            if (self._xy is not None
                    and self._xy[1] > POST_AVOID_LEAVE_TOP_EXIT_Y
                    and self._recent_obstacle_avoidance_active()):
                self.get_logger().warn(
                    f"POST_TURN guard reason=post_turn_upper_zone_reenter_top_exit "
                    f"elapsed={elapsed:.2f}s dist={distance:.3f} "
                    f"front={front} recent_avoid=True "
                    f"retry_count={self._post_avoid_top_exit_retry_count} "
                    f"recover_count={self._recover_count} "
                    f"recover_failed_count={self._recover_failed_count} "
                    f"{self._pose_text()}",
                    throttle_duration_sec=0.5)
                if self._try_reenter_top_exit(
                        "post_turn_upper_zone_reenter_top_exit",
                        front):
                    return
            if front is not None and front < POST_TURN_RECOVER_FRONT_DIST:
                self._start_recover(
                    f"post-turn front too close {front:.3f}m dist={distance:.3f}m",
                    s, front)
                return
            angular, heading_error = self._heading_hold_angular(
                POST_TURN_HEADING_MAX_ANGULAR)
            if heading_error is None:
                angular, heading_error = self._wall_angular(
                    s, POST_TURN_HEADING_MAX_ANGULAR)
            self._publish(POST_TURN_SPEED, angular)
            done = (elapsed >= POST_TURN_MIN_SEC
                    and distance >= POST_TURN_MIN_DIST)
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
                    f"front={front}",
                    s, front)
                return
            self.get_logger().info(
                f"POST_TURN {elapsed:.2f}s dist={distance:.3f} front={front} "
                f"heading_err={heading_error:+.3f} ang={angular:+.3f} "
                f"{self._pose_text()}",
                throttle_duration_sec=0.4)
            return

        if self._state == "AVOID_LANE_SHIFT":
            elapsed = self._elapsed()
            distance = self._distance_from_state_start()
            if (front is not None
                    and front < AVOID_LANE_SHIFT_EMERGENCY_FRONT_DIST):
                self._start_recover(
                    f"emergency front={front:.3f}m in AVOID_LANE_SHIFT", s, front)
                return
            previous_yaw = self._lane_shift_previous_yaw
            wrapped_delta = 0.0
            if self._yaw is not None:
                if self._lane_shift_previous_yaw is None:
                    self._lane_shift_previous_yaw = self._yaw
                    previous_yaw = self._yaw
                else:
                    wrapped_delta = math.atan2(
                        math.sin(self._yaw - self._lane_shift_previous_yaw),
                        math.cos(self._yaw - self._lane_shift_previous_yaw))
                    self._lane_shift_previous_yaw = self._yaw
                    self._lane_shift_positive_yaw_sum += max(0.0, wrapped_delta)
                    self._lane_shift_negative_yaw_sum += min(0.0, wrapped_delta)
                    self._lane_shift_yaw_progress += wrapped_delta
            net_yaw_progress = (
                None
                if self._yaw is None or self._state_start_yaw is None
                else normalize_angle(self._yaw - self._state_start_yaw)
            )
            signed_minus_net = (
                None
                if net_yaw_progress is None
                else self._lane_shift_yaw_progress - net_yaw_progress
            )
            if wrapped_delta < -1.0e-6:
                self.get_logger().warn(
                    f"lane_shift_negative_delta elapsed={elapsed:.3f} "
                    f"entry_yaw={self._state_start_yaw} "
                    f"previous_yaw={previous_yaw} current_yaw={self._yaw} "
                    f"wrapped_delta={wrapped_delta:+.6f} "
                    f"positive_delta_sum={self._lane_shift_positive_yaw_sum:.6f} "
                    f"negative_delta_sum={self._lane_shift_negative_yaw_sum:.6f} "
                    f"signed_cumulative={self._lane_shift_yaw_progress:.6f} "
                    f"direct_net={net_yaw_progress} "
                    f"signed_minus_net={signed_minus_net} "
                    f"cmd_lin={AVOID_LANE_SHIFT_SPEED:.3f} "
                    f"cmd_ang={AVOID_LANE_SHIFT_ANGULAR:+.3f} "
                    f"transition_reason=none",
                    throttle_duration_sec=0.3)
            shift_heading_error = (
                None
                if self._yaw is None or self._avoid_shift_target_yaw is None
                else angle_diff(self._avoid_shift_target_yaw, self._yaw)
            )
            angular = AVOID_LANE_SHIFT_ANGULAR
            ready_by_time = elapsed >= AVOID_LANE_SHIFT_MIN_SEC
            ready_by_dist = distance >= AVOID_LANE_SHIFT_DIST
            ready_by_yaw = (
                self._lane_shift_yaw_progress
                >= AVOID_LANE_SHIFT_YAW_PROGRESS
            )
            guard_time_ready = elapsed >= AVOID_LANE_SHIFT_GUARD_MIN_SEC
            guard_dist_ready = distance >= AVOID_LANE_SHIFT_GUARD_MIN_DIST
            front_guard_pass_ready = (
                front is not None
                and guard_time_ready
                and guard_dist_ready
                and AVOID_LANE_SHIFT_FRONT_PASS_MIN_DIST < front < AVOID_LANE_SHIFT_FRONT_GUARD_DIST
            )
            front_close_pass_ready = (
                front is not None
                and guard_dist_ready
                and AVOID_LANE_SHIFT_FRONT_PASS_MIN_DIST <= front < AVOID_LANE_SHIFT_FRONT_CLOSE_DIST
            )
            reason = None
            if front_close_pass_ready:
                reason = "front_close_pass"
            elif front_guard_pass_ready:
                reason = "front_guard_pass"
            if reason is not None:
                self.get_logger().warn(
                    f"AVOID_LANE_SHIFT exit reason={reason} "
                    f"elapsed={elapsed:.2f}s dist={distance:.3f} "
                    f"front={front} front_min={s.front_min} "
                    f"avoid_direction={self._avoid_direction:+.0f} "
                    f"shift_target_yaw={self._avoid_shift_target_yaw} "
                    f"heading_err={shift_heading_error} "
                    f"cmd_lin=0.000 cmd_ang=0.000 "
                    f"distance_ready={ready_by_dist} "
                    f"time_ready={ready_by_time} "
                    f"yaw_ready={ready_by_yaw} "
                    f"wrapped_delta={wrapped_delta:+.4f} "
                    f"cumulative_progress={self._lane_shift_yaw_progress:.4f} "
                    f"positive_delta_sum={self._lane_shift_positive_yaw_sum:.4f} "
                    f"negative_delta_sum={self._lane_shift_negative_yaw_sum:.4f} "
                    f"net_yaw_progress={net_yaw_progress} "
                    f"signed_minus_net={signed_minus_net} "
                    f"front_guard_pass_ready={front_guard_pass_ready} "
                    f"{self._pose_text()}")
                self._transition_avoid_phase(
                    "AVOID_PASS_OBSTACLE", reason, elapsed, distance, front)
                return
            if ready_by_time and ready_by_yaw:
                self._stop()
                self.get_logger().warn(
                    f"lane_shift_complete elapsed={elapsed:.3f} "
                    f"cumulative_progress={self._lane_shift_yaw_progress:.4f} "
                    f"positive_delta_sum={self._lane_shift_positive_yaw_sum:.4f} "
                    f"negative_delta_sum={self._lane_shift_negative_yaw_sum:.4f} "
                    f"net_yaw_progress={net_yaw_progress} "
                    f"signed_minus_net={signed_minus_net} "
                    f"completion_margin="
                    f"{self._lane_shift_yaw_progress - AVOID_LANE_SHIFT_YAW_PROGRESS:.4f} "
                    f"wrapped_delta={wrapped_delta:+.4f} "
                    f"current_yaw={self._yaw} previous_yaw={previous_yaw} "
                    f"entry_yaw={self._state_start_yaw} "
                    f"cmd_lin=0.000 cmd_ang=+0.000")
                self._transition_avoid_phase(
                    "AVOID_PASS_OBSTACLE", "signed_yaw_complete",
                    elapsed, distance, front)
                return
            if elapsed >= AVOID_LANE_SHIFT_MAX_SEC:
                self.get_logger().warn(
                    f"lane_shift_timeout reason=timeout "
                    f"elapsed={elapsed:.2f}s dist={distance:.3f} "
                    f"front={front} front_min={s.front_min} "
                    f"avoid_direction={self._avoid_direction:+.0f} "
                    f"shift_target_yaw={self._avoid_shift_target_yaw} "
                    f"heading_err={shift_heading_error} "
                    f"cmd_lin=0.000 cmd_ang=0.000 "
                    f"distance_ready={ready_by_dist} "
                    f"time_ready={ready_by_time} "
                    f"yaw_ready={ready_by_yaw} "
                    f"wrapped_delta={wrapped_delta:+.4f} "
                    f"cumulative_progress={self._lane_shift_yaw_progress:.4f} "
                    f"positive_delta_sum={self._lane_shift_positive_yaw_sum:.4f} "
                    f"negative_delta_sum={self._lane_shift_negative_yaw_sum:.4f} "
                    f"net_yaw_progress={net_yaw_progress} "
                    f"signed_minus_net={signed_minus_net} "
                    f"front_guard_pass_ready={front_guard_pass_ready} "
                    f"{self._pose_text()}")
                self._transition_avoid_phase(
                    "AVOID_PASS_OBSTACLE", "timeout", elapsed, distance, front)
                return
            self._publish(AVOID_LANE_SHIFT_SPEED, angular)
            self.get_logger().warn(
                f"AVOID_LANE_SHIFT tick reason=l5r059 "
                f"elapsed={elapsed:.2f}s dist={distance:.3f} "
                f"front={front} front_min={s.front_min} "
                f"avoid_direction={self._avoid_direction:+.0f} "
                f"shift_target_yaw={self._avoid_shift_target_yaw} "
                f"heading_err={shift_heading_error} "
                f"cmd_lin={AVOID_LANE_SHIFT_SPEED:.3f} cmd_ang={angular:+.3f} "
                f"distance_ready={ready_by_dist} "
                f"time_ready={ready_by_time} "
                f"yaw_ready={ready_by_yaw} "
                f"wrapped_delta={wrapped_delta:+.4f} "
                f"cumulative_progress={self._lane_shift_yaw_progress:.4f} "
                f"positive_delta_sum={self._lane_shift_positive_yaw_sum:.4f} "
                f"negative_delta_sum={self._lane_shift_negative_yaw_sum:.4f} "
                f"net_yaw_progress={net_yaw_progress} "
                f"signed_minus_net={signed_minus_net} "
                f"front_guard_pass_ready={front_guard_pass_ready} "
                f"{self._pose_text()}",
                throttle_duration_sec=0.3)
            return

        if self._state == "AVOID_PASS_OBSTACLE":
            elapsed = self._elapsed()
            distance = self._distance_from_state_start()
            front_emergency = (
                front is not None and front < EMERGENCY_FRONT_DIST)
            front_clear = self._front_is_clear(front, AVOID_PASS_CLEAR_DIST)
            existing_completion_ready = (
                elapsed >= AVOID_PASS_MIN_SEC
                and distance >= AVOID_PASS_MIN_DIST
                and front_clear
            )
            timeout_ready = elapsed >= AVOID_PASS_MAX_SEC
            if front_emergency:
                self._start_recover(
                    f"emergency front={front:.3f}m in AVOID_PASS_OBSTACLE", s, front)
                return
            if elapsed >= AVOID_PASS_ELAPSED_CAP_SEC:
                self._start_recover(
                    "elapsed_cap_500 in AVOID_PASS_OBSTACLE", s, front)
                return
            if self._avoid_pass_target_yaw is None:
                self._avoid_pass_target_yaw = self._current_straight_target_yaw()
            angular, heading_error = self._heading_hold_to_yaw(
                self._avoid_pass_target_yaw, AVOID_PASS_MAX_ANGULAR)
            self._publish(AVOID_PASS_SPEED, angular)
            if existing_completion_ready:
                self._transition_avoid_phase(
                    "AVOID_RETURN_HEADING",
                    "clear_distance",
                    elapsed,
                    distance,
                    front)
                return
            if timeout_ready:
                reason = "timeout_clear" if front_clear else "timeout"
                self._transition_avoid_phase(
                    "AVOID_RETURN_HEADING", reason, elapsed, distance, front)
                return
            self.get_logger().warn(
                f"AVOID_PASS_OBSTACLE {elapsed:.2f}s dist={distance:.3f} "
                f"front={front} clear={front_clear} "
                f"avoid_direction={self._avoid_direction:+.0f} "
                f"heading_err={heading_error} ang={angular:+.3f} "
                f"{self._pose_text()}",
                throttle_duration_sec=0.3)
            return

        if self._state == "AVOID_RETURN_HEADING":
            elapsed = self._elapsed()
            distance = self._distance_from_state_start()
            if front is not None and front < EMERGENCY_FRONT_DIST:
                self._start_recover(
                    f"emergency front={front:.3f}m in AVOID_RETURN_HEADING", s, front)
                return
            if self._avoid_base_yaw is None:
                self._avoid_base_yaw = self._current_straight_target_yaw()
            angular, heading_signed_error = self._heading_hold_to_yaw(
                self._avoid_base_yaw, AVOID_RETURN_HEADING_MAX_ANGULAR)
            heading_error = (
                None
                if heading_signed_error is None
                else abs(heading_signed_error)
            )
            heading_ready = (
                heading_error is not None
                and heading_error <= AVOID_RETURN_HEADING_TOLERANCE_RAD
            )
            front_clear = self._front_is_clear(front, AVOID_RETURN_CLEAR_DIST)
            if heading_ready:
                self._transition_avoid_phase(
                    "RETURN_TO_LINE", "heading_aligned", elapsed, distance, front)
                return
            timeout_ready = (
                heading_error is not None
                and heading_error <= AVOID_RETURN_HEADING_TIMEOUT_TOLERANCE_RAD
            )
            if elapsed >= AVOID_RETURN_HEADING_MAX_SEC and front_clear and timeout_ready:
                self._transition_avoid_phase(
                    "RETURN_TO_LINE", "timeout_clear", elapsed, distance, front)
                return
            if elapsed >= AVOID_RETURN_HEADING_HARD_MAX_SEC and front_clear:
                self._transition_avoid_phase(
                    "RETURN_TO_LINE",
                    "hard_timeout_clear",
                    elapsed,
                    distance,
                    front)
                return
            return_speed = (
                0.0
                if heading_error is not None
                and heading_error > AVOID_RETURN_HEADING_TIMEOUT_TOLERANCE_RAD
                else AVOID_RETURN_HEADING_SPEED
            )
            self._publish(return_speed, angular)
            self.get_logger().warn(
                f"AVOID_RETURN_HEADING {elapsed:.2f}s dist={distance:.3f} "
                f"front={front} clear={front_clear} "
                f"avoid_direction={self._avoid_direction:+.0f} "
                f"heading_err={heading_error} "
                f"lin={return_speed:.3f} ang={angular:+.3f} "
                f"{self._pose_text()}",
                throttle_duration_sec=0.3)
            return

        if self._state == "RETURN_TO_LINE":
            elapsed = self._elapsed()
            distance = self._distance_from_state_start()
            if front is not None and front < EMERGENCY_FRONT_DIST:
                self._start_recover(
                    f"emergency front={front:.3f}m in RETURN_TO_LINE", s, front)
                return

            return_start_x = None
            return_start_y = None
            return_start_yaw = self._state_start_yaw
            y_delta = None
            current_y = None
            if self._state_start_xy is not None:
                return_start_x, return_start_y = self._state_start_xy
            if self._xy is not None:
                current_y = self._xy[1]
                if return_start_y is not None:
                    y_delta = current_y - return_start_y

            top_return = (
                current_y is not None
                and current_y > RETURN_TO_LINE_TOP_Y
            )
            if top_return:
                return_target_yaw = RETURN_TO_LINE_TOP_TARGET_YAW
                angular_cap = RETURN_TO_LINE_TOP_MAX_ANGULAR
            else:
                return_target_yaw = (
                    self._avoid_base_yaw
                    if self._avoid_base_yaw is not None
                    else self._current_straight_target_yaw()
                )
                angular_cap = RETURN_TO_LINE_MAX_ANGULAR
            angular, heading_signed_error = self._heading_hold_to_yaw(
                return_target_yaw, angular_cap)
            error = heading_signed_error
            if heading_signed_error is None:
                angular, error = self._wall_angular(s, angular_cap)
            heading_error = (
                None
                if heading_signed_error is None
                else abs(heading_signed_error)
            )
            y_worsening = (
                (current_y is not None
                 and current_y > RETURN_TO_LINE_TOP_ESCAPE_Y)
                or (y_delta is not None
                    and y_delta > RETURN_TO_LINE_Y_WORSEN_DIST)
                or (current_y is not None
                    and current_y > RETURN_TO_LINE_TOP_Y
                    and heading_error is not None
                    and heading_error > RETURN_TO_LINE_TOP_HEADING_ERR)
            )
            if y_worsening:
                self._stop()
                self.get_logger().warn(
                    f"RETURN_TO_LINE exit reason=y_worsening "
                    f"elapsed={elapsed:.2f}s dist={distance:.3f} front={front} "
                    f"return_start_x={return_start_x} "
                    f"return_start_y={return_start_y} "
                    f"return_start_yaw={return_start_yaw} "
                    f"y_delta={y_delta} target_yaw={return_target_yaw:+.3f} "
                    f"heading_err={heading_error} "
                    f"cmd_lin=0.000 cmd_ang=0.000 {self._pose_text()}")
                self._start_post_avoid_leave_top(
                    "RETURN_TO_LINE:y_worsening", front)
                return

            control_reason = "normal_return"
            linear = RETURN_TO_LINE_SPEED
            if top_return:
                if (heading_error is not None
                        and heading_error > RETURN_TO_LINE_TOP_TURN_ONLY_ERR):
                    linear = 0.0
                    control_reason = "top_align_only"
                else:
                    linear = RETURN_TO_LINE_TOP_SPEED
                    control_reason = "top_drive_down"
            self._publish(linear, angular)
            heading_ready = (
                heading_error is not None
                and heading_error <= RETURN_TO_LINE_HEADING_TOLERANCE_RAD
            )
            if (elapsed >= RETURN_TO_LINE_SEC
                    and distance >= RETURN_TO_LINE_DIST
                    and heading_ready):
                self.get_logger().info(
                    f"RETURN_TO_LINE exit reason=distance "
                    f"elapsed={elapsed:.2f}s dist={distance:.3f} front={front} "
                    f"return_start_y={return_start_y} y_delta={y_delta} "
                    f"heading_err={heading_error} "
                    f"cmd_lin={linear:.3f} cmd_ang={angular:+.3f} "
                    f"{self._pose_text()}")
                if self._needs_post_avoid_leave_top():
                    self._start_post_avoid_leave_top(
                        "RETURN_TO_LINE:distance", front)
                    return
                self._start_avoid_retrigger_lockout("distance")
                self._set_state("STRAIGHT")
                self.get_logger().info(
                    f"STRAIGHT resumed reason=return_to_line_distance "
                    f"front={front} {self._pose_text()}")
                return
            timeout_clear = (
                elapsed >= RETURN_TO_LINE_MAX_SEC
                and self._front_is_clear(front, RETURN_TO_LINE_CLEAR_DIST)
                and heading_ready
            )
            hard_timeout = (
                elapsed >= RETURN_TO_LINE_HARD_TIMEOUT_SEC
                and self._front_is_clear(front, RETURN_TO_LINE_CLEAR_DIST)
            )
            if timeout_clear:
                self.get_logger().warn(
                    f"RETURN_TO_LINE exit reason=timeout_clear "
                    f"elapsed={elapsed:.2f}s dist={distance:.3f} front={front} "
                    f"return_start_y={return_start_y} y_delta={y_delta} "
                    f"heading_err={heading_error} "
                    f"cmd_lin={linear:.3f} cmd_ang={angular:+.3f} "
                    f"{self._pose_text()}")
                if self._needs_post_avoid_leave_top():
                    self._start_post_avoid_leave_top(
                        "RETURN_TO_LINE:timeout_clear", front)
                    return
                self._start_avoid_retrigger_lockout("timeout_clear")
                self._set_state("STRAIGHT")
                self.get_logger().warn(
                    f"STRAIGHT resumed reason=return_to_line_timeout_clear "
                    f"front={front} {self._pose_text()}")
                return
            if hard_timeout:
                self.get_logger().error(
                    f"RETURN_TO_LINE exit reason=hard_timeout "
                    f"elapsed={elapsed:.2f}s dist={distance:.3f} front={front} "
                    f"return_start_y={return_start_y} y_delta={y_delta} "
                    f"heading_err={heading_error} "
                    f"cmd_lin={linear:.3f} cmd_ang={angular:+.3f} "
                    f"{self._pose_text()}")
                if self._needs_post_avoid_leave_top():
                    self._start_post_avoid_leave_top(
                        "RETURN_TO_LINE:hard_timeout", front)
                    return
                self._start_avoid_retrigger_lockout("hard_timeout")
                self._set_state("STRAIGHT")
                self.get_logger().error(
                    f"STRAIGHT resumed reason=return_to_line_hard_timeout "
                    f"front={front} {self._pose_text()}")
                return
            if (elapsed >= RETURN_TO_LINE_MAX_SEC
                    and self._front_is_clear(front, RETURN_TO_LINE_CLEAR_DIST)
                    and not heading_ready):
                self.get_logger().warn(
                    f"RETURN_TO_LINE hold reason=heading_not_ready "
                    f"elapsed={elapsed:.2f}s dist={distance:.3f} front={front} "
                    f"return_start_y={return_start_y} y_delta={y_delta} "
                    f"heading_err={heading_error} "
                    f"cmd_lin={linear:.3f} cmd_ang={angular:+.3f} "
                    f"{self._pose_text()}",
                    throttle_duration_sec=0.5)
            self.get_logger().info(
                f"RETURN_TO_LINE tick reason={control_reason} "
                f"elapsed={elapsed:.2f}s dist={distance:.3f} front={front} "
                f"return_start_x={return_start_x} "
                f"return_start_y={return_start_y} "
                f"return_start_yaw={return_start_yaw} "
                f"y_delta={y_delta} target_yaw={return_target_yaw:+.3f} "
                f"err={error:+.3f} cmd_lin={linear:.3f} cmd_ang={angular:+.3f} "
                f"heading_err={heading_error} {self._pose_text()}",
                throttle_duration_sec=0.4)
            return

        if self._state == "POST_AVOID_LEAVE_TOP":
            now = self._now()
            elapsed = self._elapsed()
            phase = "waypoint_escape"
            start_x = None
            start_y = None
            x_progress = None
            y_progress = None
            start_dist_to_goal = None
            dist_progress = None
            current_x = None
            current_y = None
            previous_x = self._post_avoid_leave_top_last_x
            previous_y = self._post_avoid_leave_top_last_y
            actual_dx = None
            actual_dy = None
            actual_motion_yaw = None
            footprint_min_x = None
            footprint_max_x = None
            down_path_clear = None
            selected_clearance_side = None
            right_required = None
            left_required = None
            if self._state_start_xy is not None:
                start_x, start_y = self._state_start_xy
                start_dist_to_goal = math.hypot(
                    POST_AVOID_LEAVE_TOP_GOAL_X - start_x,
                    POST_AVOID_LEAVE_TOP_GOAL_Y - start_y)
                if self._xy is not None:
                    current_x, current_y = self._xy
                    x_progress = start_x - current_x
                    y_progress = start_y - current_y
                    if previous_x is not None and previous_y is not None:
                        actual_dx = current_x - previous_x
                        actual_dy = current_y - previous_y
                        if math.hypot(actual_dx, actual_dy) > 1.0e-4:
                            actual_motion_yaw = math.atan2(actual_dy, actual_dx)
                        self._post_avoid_recent_dy.append((now, actual_dy))
            if current_x is not None and self._yaw is not None:
                (footprint_min_x, footprint_max_x, down_path_clear,
                 selected_clearance_side, right_required, left_required) = (
                    self._post_clearance_geometry()
                )
            if self._control_post_clearance_escape():
                if current_x is not None:
                    self._post_avoid_leave_top_last_x = current_x
                if current_y is not None:
                    self._post_avoid_leave_top_last_y = current_y
                return
            if (self._post_avoid_final_down_entry_reason is not None
                    and down_path_clear is False):
                self._start_post_clearance_escape(
                    "active_final_down_without_geometric_clearance",
                    "normal_down")
                if current_x is not None:
                    self._post_avoid_leave_top_last_x = current_x
                if current_y is not None:
                    self._post_avoid_leave_top_last_y = current_y
                return
            recent_cutoff = now - POST_AVOID_LEAVE_TOP_RECENT_DY_WINDOW_SEC
            self._post_avoid_recent_dy = [
                (sample_t, sample_dy)
                for sample_t, sample_dy in self._post_avoid_recent_dy
                if sample_t >= recent_cutoff
            ]
            recent_actual_dy_values = [
                sample_dy for _, sample_dy in self._post_avoid_recent_dy
            ]
            recent_actual_dy_min = (
                min(recent_actual_dy_values)
                if recent_actual_dy_values
                else None
            )
            recent_actual_dy_negative = (
                recent_actual_dy_min is not None
                and recent_actual_dy_min
                <= POST_AVOID_LEAVE_TOP_RECENT_DY_NEGATIVE_MIN
            )

            if current_x is not None and current_y is not None:
                dx_to_goal = POST_AVOID_LEAVE_TOP_GOAL_X - current_x
                dy_to_goal = POST_AVOID_LEAVE_TOP_GOAL_Y - current_y
                dist_to_goal = math.hypot(dx_to_goal, dy_to_goal)
                target_yaw = math.atan2(dy_to_goal, dx_to_goal)
                if start_dist_to_goal is not None:
                    dist_progress = start_dist_to_goal - dist_to_goal
            else:
                dx_to_goal = None
                dy_to_goal = None
                dist_to_goal = None
                target_yaw = POST_AVOID_LEAVE_TOP_TARGET_YAW
            final_down_entry_candidate = None
            if (current_y is not None
                    and current_y <= POST_AVOID_LEAVE_TOP_FINAL_DOWN_Y):
                final_down_entry_candidate = "y_threshold"
            elif (y_progress is not None
                    and y_progress >= POST_AVOID_LEAVE_TOP_FINAL_DOWN_PROGRESS):
                final_down_entry_candidate = "y_progress_threshold"
            elif (elapsed >= POST_AVOID_LEAVE_TOP_FINAL_DOWN_EARLY_SEC
                    and current_y is not None
                    and current_y > 0.25
                    and y_progress is not None
                    and y_progress < POST_AVOID_LEAVE_TOP_FINAL_DOWN_EARLY_PROGRESS):
                final_down_entry_candidate = "no_y_progress_early_final_down"
            elif (elapsed > POST_AVOID_LEAVE_TOP_FINAL_DOWN_FORCE_SEC
                    and current_y is not None
                    and current_y > 0.25):
                final_down_entry_candidate = "elapsed_force"
            if (self._post_avoid_final_down_entry_reason is None
                    and final_down_entry_candidate is not None):
                if down_path_clear is False:
                    self._start_post_clearance_escape(
                        f"final_down_entry_blocked:{final_down_entry_candidate}",
                        "waypoint_escape")
                    if current_x is not None:
                        self._post_avoid_leave_top_last_x = current_x
                    if current_y is not None:
                        self._post_avoid_leave_top_last_y = current_y
                    return
                self._post_avoid_final_down_entry_reason = final_down_entry_candidate
                self._post_avoid_final_down_start_t = self._now()
                self._post_avoid_final_down_start_xy = self._xy
                self._post_avoid_final_down_drive_start_t = None
                self._post_avoid_final_down_motion_blocked_logged = False
            final_down_active = (
                self._post_avoid_final_down_entry_reason is not None
            )
            top_exit_retry_active = (
                final_down_active
                and self._post_avoid_top_exit_retry_reason is not None
            )
            final_down_entry_reason = self._post_avoid_final_down_entry_reason
            final_down_elapsed = (
                None
                if self._post_avoid_final_down_start_t is None
                else self._now() - self._post_avoid_final_down_start_t
            )
            final_down_start_x = None
            final_down_start_y = None
            final_down_y_progress = None
            if self._post_avoid_final_down_start_xy is not None:
                final_down_start_x, final_down_start_y = (
                    self._post_avoid_final_down_start_xy
                )
                if current_y is not None:
                    final_down_y_progress = final_down_start_y - current_y
            final_down_subphase = "none"
            clearance_reason = "none"
            clearance_elapsed = None
            final_down_clearance_active = False
            finish_down_after_clearance_active = False
            if final_down_active:
                phase = "top_exit_retry" if top_exit_retry_active else "final_down_exit"
                final_down_subphase = "normal_down"
                target_yaw = POST_AVOID_LEAVE_TOP_TARGET_YAW
                finish_down_after_clearance_active = (
                    front is not None
                    and current_y is not None
                    and current_y <= POST_AVOID_LEAVE_TOP_FINISH_DOWN_Y
                    and current_y > POST_AVOID_LEAVE_TOP_EXIT_Y
                    and front >= POST_AVOID_LEAVE_TOP_FINISH_DOWN_FRONT_MIN
                )
                if finish_down_after_clearance_active:
                    final_down_subphase = "finish_down_after_clearance"
                    clearance_reason = "finish_down_after_clearance"
                    target_yaw = POST_AVOID_LEAVE_TOP_TARGET_YAW
                    self._post_avoid_final_down_clearance_start_t = None
                elif (front is not None
                        and current_y is not None
                        and current_y > POST_AVOID_LEAVE_TOP_EXIT_Y
                        and front >= POST_AVOID_LEAVE_TOP_FINAL_DOWN_DANGER_DIST
                        and front < POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_DIST):
                    final_down_clearance_active = True
                    final_down_subphase = "clearance_escape"
                    if (current_x is None
                            or current_x > POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_SAFE_X):
                        target_yaw = POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_YAW
                        clearance_reason = "front_hold_clearance_downleft"
                    else:
                        target_yaw = POST_AVOID_LEAVE_TOP_TARGET_YAW
                        clearance_reason = "front_hold_wait_down"
                    if self._post_avoid_final_down_clearance_start_t is None:
                        self._post_avoid_final_down_clearance_start_t = self._now()
                else:
                    self._post_avoid_final_down_clearance_start_t = None
            else:
                self._post_avoid_final_down_clearance_start_t = None
            if self._post_avoid_final_down_clearance_start_t is not None:
                clearance_elapsed = (
                    self._now() - self._post_avoid_final_down_clearance_start_t
                )
            fallback_reason = None
            if (
                    not final_down_active
                    and
                    elapsed > POST_AVOID_LEAVE_TOP_Y_PROGRESS_SEC
                    and y_progress is not None
                    and y_progress < POST_AVOID_LEAVE_TOP_Y_PROGRESS_MIN):
                fallback_reason = "no_y_progress_fallback_downleft"
                target_yaw = POST_AVOID_LEAVE_TOP_FALLBACK_YAW
            elif (
                    not final_down_active
                    and
                    elapsed > POST_AVOID_LEAVE_TOP_GOAL_PROGRESS_SEC
                    and dist_progress is not None
                    and dist_progress < POST_AVOID_LEAVE_TOP_GOAL_PROGRESS_MIN):
                fallback_reason = "no_goal_progress"
                target_yaw = POST_AVOID_LEAVE_TOP_FALLBACK_YAW
            self._straight_target_yaw = target_yaw
            heading_signed_error = (
                None if self._yaw is None else angle_diff(target_yaw, self._yaw)
            )
            heading_error = (
                None
                if heading_signed_error is None
                else abs(heading_signed_error)
            )
            if heading_error is None:
                max_angular = 0.0
            elif finish_down_after_clearance_active:
                max_angular = POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_MAX_ANGULAR
            elif final_down_clearance_active:
                max_angular = POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_MAX_ANGULAR
            elif final_down_active and heading_error > POST_AVOID_LEAVE_TOP_FINAL_DOWN_ALIGN_ERR:
                max_angular = POST_AVOID_LEAVE_TOP_FINAL_DOWN_ALIGN_MAX_ANGULAR
            elif final_down_active and heading_error > POST_AVOID_LEAVE_TOP_FINAL_DOWN_SLOW_ERR:
                max_angular = POST_AVOID_LEAVE_TOP_FINAL_DOWN_CREEP_MAX_ANGULAR
            elif final_down_active:
                max_angular = POST_AVOID_LEAVE_TOP_FINAL_DOWN_DRIVE_MAX_ANGULAR
            elif heading_error > POST_AVOID_LEAVE_TOP_WAYPOINT_ALIGN_ERR:
                max_angular = POST_AVOID_LEAVE_TOP_WAYPOINT_ALIGN_MAX_ANGULAR
            elif heading_error > POST_AVOID_LEAVE_TOP_WAYPOINT_SLOW_ERR:
                max_angular = POST_AVOID_LEAVE_TOP_WAYPOINT_SLOW_MAX_ANGULAR
            else:
                max_angular = POST_AVOID_LEAVE_TOP_WAYPOINT_DRIVE_MAX_ANGULAR
            angular, heading_signed_error = self._heading_hold_to_yaw(
                target_yaw, max_angular)
            heading_error = (
                None
                if heading_signed_error is None
                else abs(heading_signed_error)
            )

            if current_y is not None and current_y < POST_AVOID_LEAVE_TOP_EXIT_Y:
                self.get_logger().warn(
                    f"POST_AVOID_LEAVE_TOP exit reason=left_top "
                    f"phase={phase} final_down_subphase={final_down_subphase} "
                    f"clearance_reason={clearance_reason} "
                    f"clearance_elapsed={clearance_elapsed} "
                    f"elapsed={elapsed:.2f}s front={front} "
                    f"start_x={start_x} start_y={start_y} "
                    f"previous_x={previous_x} previous_y={previous_y} "
                    f"actual_dx={actual_dx} actual_dy={actual_dy} "
                    f"actual_motion_yaw={actual_motion_yaw} "
                    f"x_progress={x_progress} y_progress={y_progress} "
                    f"final_down_entry_reason={final_down_entry_reason} "
                    f"final_down_elapsed={final_down_elapsed} "
                    f"final_down_start_x={final_down_start_x} "
                    f"final_down_start_y={final_down_start_y} "
                    f"final_down_y_progress={final_down_y_progress} "
                    f"start_dist_to_goal={start_dist_to_goal} "
                    f"dist_progress={dist_progress} "
                    f"goal_x={POST_AVOID_LEAVE_TOP_GOAL_X:+.3f} "
                    f"goal_y={POST_AVOID_LEAVE_TOP_GOAL_Y:+.3f} "
                    f"dx_to_goal={dx_to_goal} dy_to_goal={dy_to_goal} "
                    f"dist_to_goal={dist_to_goal} "
                    f"target_yaw={target_yaw:+.3f} "
                    f"fallback_reason={fallback_reason} "
                    f"heading_err={heading_error} "
                    f"cmd_lin=0.000 cmd_ang=0.000 "
                    f"{self._pose_text()}")
                self._start_avoid_retrigger_lockout("post_avoid_leave_top")
                self._set_state("STRAIGHT")
                self.get_logger().warn(
                    f"STRAIGHT resumed reason=post_avoid_leave_top "
                    f"front={front} {self._pose_text()}")
                return
            if (front is not None
                    and front < POST_AVOID_LEAVE_TOP_FRONT_DANGER_DIST):
                self._start_recover(
                    f"extreme front={front:.3f}m in POST_AVOID_LEAVE_TOP",
                    s,
                    front)
                return
            front_ready = (
                front is None
                or front >= POST_AVOID_LEAVE_TOP_FRONT_STOP_DIST
            )
            front_hold = final_down_clearance_active or not front_ready
            front_hold_reason = (
                clearance_reason
                if final_down_clearance_active
                else "front_below_stop"
                if front is not None and front < POST_AVOID_LEAVE_TOP_FRONT_STOP_DIST
                else "none"
            )
            post_timeout_reached = elapsed >= POST_AVOID_LEAVE_TOP_MAX_SEC
            timeout_ready = post_timeout_reached
            if (timeout_ready
                    and final_down_active
                    and final_down_elapsed is not None
                    and final_down_elapsed < POST_AVOID_LEAVE_TOP_FINAL_DOWN_MIN_SEC):
                timeout_ready = False
            final_down_progressing_for_extension = (
                final_down_active
                and final_down_y_progress is not None
                and final_down_y_progress >= POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_MIN_PROGRESS
                and current_y is not None
                and current_y > POST_AVOID_LEAVE_TOP_EXIT_Y
                and current_y <= POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_Y_MAX
                and recent_actual_dy_negative
                and front is not None
                and front >= POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_FRONT_MIN
            )
            final_down_extra_elapsed = max(
                0.0, elapsed - POST_AVOID_LEAVE_TOP_MAX_SEC)
            extension_active = (
                self._post_avoid_final_down_extension_until is not None
                and now < self._post_avoid_final_down_extension_until
            )
            post_extension_allowed = (
                timeout_ready
                and final_down_progressing_for_extension
                and self._post_avoid_final_down_extension_count
                < POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_MAX_COUNT
            )
            extension_reason = "none"
            exit_decision = "continue"
            if timeout_ready and extension_active:
                timeout_ready = False
                extension_reason = "extension_active"
                exit_decision = "extend_final_down_progressing"
            if (timeout_ready
                    and post_extension_allowed):
                self._post_avoid_final_down_extension_count += 1
                self._post_avoid_final_down_extension_until = (
                    now + POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_SEC)
                timeout_ready = False
                extension_reason = "extend_final_down_progressing"
                exit_decision = "extend_final_down_progressing"
                self.get_logger().warn(
                    f"POST_AVOID_LEAVE_TOP extend reason=extend_final_down_progressing "
                    f"elapsed={elapsed:.2f}s extension={POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_SEC:.1f}s "
                    f"final_down_extension_count={self._post_avoid_final_down_extension_count} "
                    f"final_down_extra_elapsed={final_down_extra_elapsed:.2f}s "
                    f"front={front} final_down_y_progress={final_down_y_progress} "
                    f"recent_actual_dy_min={recent_actual_dy_min} "
                    f"y={current_y:+.3f} target_exit_y={POST_AVOID_LEAVE_TOP_EXIT_Y:+.3f} "
                    f"final_down_subphase={final_down_subphase} "
                    f"clearance_reason={clearance_reason} "
                    f"post_timeout_reached={post_timeout_reached} "
                    f"post_extension_allowed={post_extension_allowed} "
                    f"exit_decision={exit_decision} "
                    f"retry_count={self._post_avoid_top_exit_retry_count} "
                    f"{self._pose_text()}",
                    throttle_duration_sec=0.5)
            if timeout_ready and self._try_reenter_top_exit(
                    "post_timeout", front, y_progress,
                    final_down_y_progress):
                return
            near_exit_timeout_not_done = (
                timeout_ready
                and current_y is not None
                and current_y > POST_AVOID_LEAVE_TOP_EXIT_Y
                and current_y <= POST_AVOID_LEAVE_TOP_PROGRESS_EXTENSION_Y_MAX
            )
            if near_exit_timeout_not_done:
                timeout_ready = False
                exit_decision = "continue_post_timeout_near_exit_not_done"
                self.get_logger().warn(
                    f"POST_AVOID_LEAVE_TOP continue reason=post_timeout_near_exit_not_done "
                    f"post_phase={phase} phase={phase} "
                    f"final_down_subphase={final_down_subphase} "
                    f"clearance_reason={clearance_reason} "
                    f"clearance_elapsed={clearance_elapsed} "
                    f"elapsed={elapsed:.2f}s front={front} "
                    f"post_timeout_reached={post_timeout_reached} "
                    f"post_extension_allowed={post_extension_allowed} "
                    f"extension_reason={extension_reason} "
                    f"final_down_extension_count={self._post_avoid_final_down_extension_count} "
                    f"final_down_extra_elapsed={final_down_extra_elapsed:.2f}s "
                    f"recent_actual_dy_min={recent_actual_dy_min} "
                    f"start_x={start_x} start_y={start_y} "
                    f"previous_x={previous_x} previous_y={previous_y} "
                    f"actual_dx={actual_dx} actual_dy={actual_dy} "
                    f"actual_motion_yaw={actual_motion_yaw} "
                    f"x_progress={x_progress} y_progress={y_progress} "
                    f"final_down_entry_reason={final_down_entry_reason} "
                    f"final_down_elapsed={final_down_elapsed} "
                    f"final_down_start_x={final_down_start_x} "
                    f"final_down_start_y={final_down_start_y} "
                    f"final_down_y_progress={final_down_y_progress} "
                    f"target_yaw={target_yaw:+.3f} "
                    f"heading_err={heading_error} "
                    f"front_hold={front_hold} "
                    f"front_hold_reason={front_hold_reason} "
                    f"retry_count={self._post_avoid_top_exit_retry_count} "
                    f"exit_decision={exit_decision} "
                    f"continue_control=True {self._pose_text()}",
                    throttle_duration_sec=0.5)
            if timeout_ready:
                exit_decision = "timeout_to_straight"
                self.get_logger().error(
                    f"POST_AVOID_LEAVE_TOP exit reason=timeout "
                    f"post_phase={phase} phase={phase} "
                    f"final_down_subphase={final_down_subphase} "
                    f"clearance_reason={clearance_reason} "
                    f"clearance_elapsed={clearance_elapsed} "
                    f"elapsed={elapsed:.2f}s front={front} "
                    f"start_x={start_x} start_y={start_y} "
                    f"previous_x={previous_x} previous_y={previous_y} "
                    f"actual_dx={actual_dx} actual_dy={actual_dy} "
                    f"actual_motion_yaw={actual_motion_yaw} "
                    f"x_progress={x_progress} y_progress={y_progress} "
                    f"final_down_entry_reason={final_down_entry_reason} "
                    f"final_down_elapsed={final_down_elapsed} "
                    f"final_down_start_x={final_down_start_x} "
                    f"final_down_start_y={final_down_start_y} "
                    f"final_down_y_progress={final_down_y_progress} "
                    f"start_dist_to_goal={start_dist_to_goal} "
                    f"dist_progress={dist_progress} "
                    f"goal_x={POST_AVOID_LEAVE_TOP_GOAL_X:+.3f} "
                    f"goal_y={POST_AVOID_LEAVE_TOP_GOAL_Y:+.3f} "
                    f"dx_to_goal={dx_to_goal} dy_to_goal={dy_to_goal} "
                    f"dist_to_goal={dist_to_goal} "
                    f"target_yaw={target_yaw:+.3f} "
                    f"fallback_reason={fallback_reason} "
                    f"heading_err={heading_error} "
                    f"front_hold={front_hold} "
                    f"front_hold_reason={front_hold_reason} "
                    f"post_timeout_reached={post_timeout_reached} "
                    f"post_extension_allowed={post_extension_allowed} "
                    f"extension_reason={extension_reason} "
                    f"final_down_extension_count={self._post_avoid_final_down_extension_count} "
                    f"final_down_extra_elapsed={final_down_extra_elapsed:.2f}s "
                    f"recent_actual_dy_min={recent_actual_dy_min} "
                    f"retry_count={self._post_avoid_top_exit_retry_count} "
                    f"exit_decision={exit_decision} "
                    f"cmd_lin=0.000 cmd_ang=0.000 {self._pose_text()}")
                self._start_avoid_retrigger_lockout("post_avoid_leave_top_timeout")
                self._set_state("STRAIGHT")
                self.get_logger().error(
                    f"STRAIGHT resumed reason=post_avoid_leave_top_timeout "
                    f"front={front} {self._pose_text()}")
                return
            no_waypoint_progress = (
                elapsed > POST_AVOID_LEAVE_TOP_STUCK_SEC
                and y_progress is not None
                and y_progress < POST_AVOID_LEAVE_TOP_STUCK_PROGRESS_DIST
            )
            if (no_waypoint_progress
                    and final_down_active
                    and final_down_subphase == "normal_down"
                    and down_path_clear is False):
                self._start_post_clearance_escape(
                    "stuck_no_waypoint_progress_without_geometric_clearance",
                    "normal_down")
                if current_x is not None:
                    self._post_avoid_leave_top_last_x = current_x
                if current_y is not None:
                    self._post_avoid_leave_top_last_y = current_y
                return
            if no_waypoint_progress:
                self.get_logger().error(
                    f"POST_AVOID_LEAVE_TOP tick reason=stuck_no_waypoint_progress "
                    f"phase={phase} final_down_subphase={final_down_subphase} "
                    f"clearance_reason={clearance_reason} "
                    f"clearance_elapsed={clearance_elapsed} "
                    f"elapsed={elapsed:.2f}s front={front} "
                    f"start_x={start_x} start_y={start_y} "
                    f"previous_x={previous_x} previous_y={previous_y} "
                    f"actual_dx={actual_dx} actual_dy={actual_dy} "
                    f"actual_motion_yaw={actual_motion_yaw} "
                    f"x_progress={x_progress} y_progress={y_progress} "
                    f"final_down_entry_reason={final_down_entry_reason} "
                    f"final_down_elapsed={final_down_elapsed} "
                    f"final_down_start_x={final_down_start_x} "
                    f"final_down_start_y={final_down_start_y} "
                    f"final_down_y_progress={final_down_y_progress} "
                    f"start_dist_to_goal={start_dist_to_goal} "
                    f"dist_progress={dist_progress} "
                    f"goal_x={POST_AVOID_LEAVE_TOP_GOAL_X:+.3f} "
                    f"goal_y={POST_AVOID_LEAVE_TOP_GOAL_Y:+.3f} "
                    f"dx_to_goal={dx_to_goal} dy_to_goal={dy_to_goal} "
                    f"dist_to_goal={dist_to_goal} "
                    f"target_yaw={target_yaw:+.3f} "
                    f"fallback_reason={fallback_reason} "
                    f"heading_err={heading_error} "
                    f"cmd_lin={POST_AVOID_LEAVE_TOP_WAYPOINT_SPEED:.3f} "
                    f"cmd_ang={angular:+.3f} {self._pose_text()}",
                    throttle_duration_sec=0.8)
            if fallback_reason is not None:
                self.get_logger().error(
                    f"POST_AVOID_LEAVE_TOP progress_watchdog "
                    f"reason={fallback_reason} phase={phase} "
                    f"final_down_subphase={final_down_subphase} "
                    f"clearance_reason={clearance_reason} "
                    f"clearance_elapsed={clearance_elapsed} "
                    f"elapsed={elapsed:.2f}s front={front} "
                    f"start_x={start_x} start_y={start_y} "
                    f"previous_x={previous_x} previous_y={previous_y} "
                    f"actual_dx={actual_dx} actual_dy={actual_dy} "
                    f"actual_motion_yaw={actual_motion_yaw} "
                    f"x_progress={x_progress} y_progress={y_progress} "
                    f"final_down_entry_reason={final_down_entry_reason} "
                    f"final_down_elapsed={final_down_elapsed} "
                    f"final_down_start_x={final_down_start_x} "
                    f"final_down_start_y={final_down_start_y} "
                    f"final_down_y_progress={final_down_y_progress} "
                    f"start_dist_to_goal={start_dist_to_goal} "
                    f"dist_progress={dist_progress} "
                    f"goal_x={POST_AVOID_LEAVE_TOP_GOAL_X:+.3f} "
                    f"goal_y={POST_AVOID_LEAVE_TOP_GOAL_Y:+.3f} "
                    f"target_yaw={target_yaw:+.3f} "
                    f"heading_err={heading_error} "
                    f"cmd_ang={angular:+.3f} {self._pose_text()}",
                    throttle_duration_sec=0.8)
            if finish_down_after_clearance_active:
                if heading_error is None or heading_error > POST_AVOID_LEAVE_TOP_FINAL_DOWN_ALIGN_ERR:
                    linear = 0.0
                    reason = "finish_down_align"
                else:
                    linear = POST_AVOID_LEAVE_TOP_FINISH_DOWN_SPEED
                    reason = "finish_down_after_clearance"
            elif final_down_clearance_active:
                if clearance_reason == "front_hold_clearance_downleft":
                    linear = POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_SPEED
                elif clearance_reason == "finish_down_after_clearance":
                    linear = POST_AVOID_LEAVE_TOP_FINISH_DOWN_SPEED
                elif front is not None and front >= POST_AVOID_LEAVE_TOP_FRONT_STOP_DIST:
                    linear = POST_AVOID_LEAVE_TOP_FINAL_DOWN_CLEARANCE_WAIT_SPEED
                else:
                    linear = 0.0
                reason = clearance_reason
            elif not front_ready:
                linear = 0.0
                reason = "front_hold"
            elif final_down_active and (
                    heading_error is None
                    or heading_error > POST_AVOID_LEAVE_TOP_FINAL_DOWN_ALIGN_ERR):
                linear = 0.0
                reason = "final_down_align"
            elif final_down_active and heading_error > POST_AVOID_LEAVE_TOP_FINAL_DOWN_SLOW_ERR:
                linear = POST_AVOID_LEAVE_TOP_FINAL_DOWN_CREEP_SPEED
                reason = "final_down_creep"
            elif final_down_active:
                linear = (
                    POST_AVOID_TOP_EXIT_RETRY_SPEED
                    if top_exit_retry_active
                    else POST_AVOID_LEAVE_TOP_FINAL_DOWN_SPEED
                )
                reason = "final_down_drive"
            elif (
                    heading_error is None
                    or heading_error > POST_AVOID_LEAVE_TOP_WAYPOINT_ALIGN_ERR):
                linear = 0.0
                reason = "align_only"
                if heading_error is None:
                    linear = 0.0
            elif heading_error > POST_AVOID_LEAVE_TOP_WAYPOINT_SLOW_ERR:
                linear = POST_AVOID_LEAVE_TOP_WAYPOINT_SLOW_SPEED
                reason = "creep_align"
            else:
                linear = (
                    POST_AVOID_LEAVE_TOP_WAYPOINT_FALLBACK_SPEED
                    if fallback_reason is not None
                    else POST_AVOID_LEAVE_TOP_WAYPOINT_SPEED
                )
                reason = "drive_aligned"
            if fallback_reason is not None and reason != "front_hold":
                reason = f"{fallback_reason}:{reason}"
            expected_world_vy = (
                None if self._yaw is None
                else linear * math.sin(self._yaw)
            )
            if (linear > 0.0
                    and expected_world_vy is not None
                    and expected_world_vy < 0.0
                    and down_path_clear is False):
                self._start_post_clearance_escape(
                    f"downward_publish_guard:{reason}",
                    final_down_subphase)
                if current_x is not None:
                    self._post_avoid_leave_top_last_x = current_x
                if current_y is not None:
                    self._post_avoid_leave_top_last_y = current_y
                return
            final_down_drive_aligned = (
                final_down_active
                and reason == "final_down_drive"
                and linear >= 0.08
                and heading_error is not None
                and heading_error <= POST_AVOID_LEAVE_TOP_FINAL_DOWN_SLOW_ERR
            )
            if final_down_drive_aligned:
                if self._post_avoid_final_down_drive_start_t is None:
                    self._post_avoid_final_down_drive_start_t = self._now()
                final_down_drive_elapsed = (
                    self._now() - self._post_avoid_final_down_drive_start_t
                )
            else:
                self._post_avoid_final_down_drive_start_t = None
                final_down_drive_elapsed = None
            if (
                    final_down_drive_aligned
                    and final_down_drive_elapsed is not None
                    and final_down_drive_elapsed >= POST_AVOID_LEAVE_TOP_FINAL_DOWN_BLOCKED_SEC
                    and final_down_y_progress is not None
                    and final_down_y_progress < POST_AVOID_LEAVE_TOP_FINAL_DOWN_BLOCKED_PROGRESS
                    and not self._post_avoid_final_down_motion_blocked_logged):
                self._post_avoid_final_down_motion_blocked_logged = True
                self.get_logger().error(
                    f"POST_AVOID_LEAVE_TOP tick "
                    f"reason=final_down_drive_no_y_progress post_phase={phase} "
                    f"final_down_subphase={final_down_subphase} "
                    f"clearance_reason={clearance_reason} "
                    f"clearance_elapsed={clearance_elapsed} "
                    f"elapsed={elapsed:.2f}s final_down_elapsed={final_down_elapsed} "
                    f"final_down_drive_elapsed={final_down_drive_elapsed} "
                    f"front={front} front_hold={front_hold} "
                    f"front_hold_reason={front_hold_reason} "
                    f"start_x={start_x} start_y={start_y} "
                    f"previous_x={previous_x} previous_y={previous_y} "
                    f"actual_dx={actual_dx} actual_dy={actual_dy} "
                    f"actual_motion_yaw={actual_motion_yaw} "
                    f"x_progress={x_progress} y_progress={y_progress} "
                    f"final_down_entry_reason={final_down_entry_reason} "
                    f"final_down_start_x={final_down_start_x} "
                    f"final_down_start_y={final_down_start_y} "
                    f"final_down_y_progress={final_down_y_progress} "
                    f"target_yaw={target_yaw:+.3f} "
                    f"heading_err={heading_signed_error} "
                    f"abs_heading_err={heading_error} "
                    f"cmd_lin={linear:.3f} cmd_ang={angular:+.3f} "
                    f"state={self._state} {self._pose_text()}")
            self._publish(linear, angular)
            self.get_logger().warn(
                f"POST_AVOID_LEAVE_TOP tick post_phase={phase} phase={phase} "
                f"final_down_subphase={final_down_subphase} "
                f"clearance_reason={clearance_reason} "
                f"clearance_elapsed={clearance_elapsed} "
                f"reason={reason} "
                f"elapsed={elapsed:.2f}s front={front} "
                f"start_x={start_x} start_y={start_y} "
                f"previous_x={previous_x} previous_y={previous_y} "
                f"actual_dx={actual_dx} actual_dy={actual_dy} "
                f"actual_motion_yaw={actual_motion_yaw} "
                f"x_progress={x_progress} y_progress={y_progress} "
                f"final_down_entry_reason={final_down_entry_reason} "
                f"final_down_elapsed={final_down_elapsed} "
                f"final_down_start_x={final_down_start_x} "
                f"final_down_start_y={final_down_start_y} "
                f"final_down_y_progress={final_down_y_progress} "
                f"final_down_drive_elapsed={final_down_drive_elapsed} "
                f"start_dist_to_goal={start_dist_to_goal} "
                f"dist_progress={dist_progress} "
                f"goal_x={POST_AVOID_LEAVE_TOP_GOAL_X:+.3f} "
                f"goal_y={POST_AVOID_LEAVE_TOP_GOAL_Y:+.3f} "
                f"dx_to_goal={dx_to_goal} dy_to_goal={dy_to_goal} "
                f"dist_to_goal={dist_to_goal} "
                f"target_yaw={target_yaw:+.3f} "
                f"fallback_reason={fallback_reason} "
                f"heading_err={heading_signed_error} "
                f"abs_heading_err={heading_error} "
                f"front_hold={front_hold} "
                f"front_hold_reason={front_hold_reason} "
                f"post_timeout_reached={post_timeout_reached} "
                f"post_extension_allowed={post_extension_allowed} "
                f"extension_reason={extension_reason} "
                f"final_down_extension_count={self._post_avoid_final_down_extension_count} "
                f"final_down_extra_elapsed={final_down_extra_elapsed:.2f}s "
                f"recent_actual_dy_min={recent_actual_dy_min} "
                f"exit_decision={exit_decision} "
                f"retry_count={self._post_avoid_top_exit_retry_count} "
                f"state={self._state} "
                f"cmd_lin={linear:.3f} cmd_ang={angular:+.3f} "
                f"{self._pose_text()}",
                throttle_duration_sec=0.35)
            if current_x is not None:
                self._post_avoid_leave_top_last_x = current_x
            if current_y is not None:
                self._post_avoid_leave_top_last_y = current_y
            return

        if self._state == "RECOVER":
            elapsed = self._elapsed()
            angular, heading_error = self._heading_hold_angular(
                POST_TURN_HEADING_MAX_ANGULAR)
            bottom_context = self._bottom_blue_line_recover_context()
            if bottom_context:
                front_clear = self._front_is_clear(
                    front, POST_TURN_CLEAR_FRONT_DIST)
                reverse_complete = (
                    elapsed >= RECOVER_STOP_SEC + RECOVER_BACKUP_SEC)
                if reverse_complete and front_clear:
                    self._stop()
                    self.get_logger().warn(
                        f"recover_clear recover_source_state={self._recover_source_state} "
                        f"bottom_context=True recover_phase=reverse_turn "
                        f"elapsed={elapsed:.2f}s front={front} "
                        f"front_clear=True issued_linear=0.000 issued_angular=+0.000 "
                        f"entry_yaw={self._bottom_recover_entry_yaw} "
                        f"previous_target_yaw={self._bottom_recover_previous_target_yaw} "
                        f"target_yaw={self._turn_target_yaw} "
                        f"heading_err={heading_error} clear_reason=front_clear "
                        f"-> POST_TURN {self._pose_text()}")
                    self._set_state("POST_TURN", "bottom_recover_clear", front)
                    return
                if elapsed >= RECOVER_TIMEOUT_SEC:
                    previous_linear, previous_angular = (
                        self._phase5q_last_published_command)
                    self._start_bottom_fallback_route(
                        front, scan_stamp_ns, elapsed,
                        previous_linear, previous_angular)
                    return
                if elapsed < RECOVER_STOP_SEC:
                    recover_phase = "stop"
                    issued_linear = 0.0
                    issued_angular = 0.0
                    self._stop()
                else:
                    recover_phase = "reverse_turn"
                    issued_linear = RECOVER_BACKUP_SPEED
                    issued_angular = angular
                    self._publish(issued_linear, issued_angular)
                self.get_logger().warn(
                    f"RECOVER bottom recover_source_state={self._recover_source_state} "
                    f"bottom_context=True recover_phase={recover_phase} "
                    f"elapsed={elapsed:.2f}/{RECOVER_TIMEOUT_SEC:.2f}s "
                    f"front={front} front_clear={front_clear} "
                    f"issued_linear={issued_linear:.3f} "
                    f"issued_angular={issued_angular:+.3f} "
                    f"entry_yaw={self._bottom_recover_entry_yaw} "
                    f"previous_target_yaw={self._bottom_recover_previous_target_yaw} "
                    f"target_yaw={self._turn_target_yaw} "
                    f"heading_err={heading_error} "
                    f"selected_angular_sign="
                    f"{1 if issued_angular > 0.0 else -1 if issued_angular < 0.0 else 0:+d} "
                    f"{self._pose_text()}",
                    throttle_duration_sec=0.4)
                return
            upper_pass_recover_context = (
                self._recover_source_state == "AVOID_PASS_OBSTACLE"
                and not bottom_context)
            if upper_pass_recover_context:
                front_clear = self._front_is_clear(
                    front, POST_TURN_CLEAR_FRONT_DIST)
                if front_clear:
                    if not self._upper_pass_recover_egress_seen:
                        self._upper_pass_recover_egress_seen = True
                        self._start_recover_egress_route(front)
                        return
                    self._stop()
                    if (self._xy is not None
                            and self._xy[1] > POST_AVOID_LEAVE_TOP_EXIT_Y
                            and self._recent_obstacle_avoidance_active()):
                        self.get_logger().warn(
                            f"RECOVER clear front={front} -> POST_AVOID_LEAVE_TOP "
                            f"reason=recover_clear_upper_zone_reenter_top_exit "
                            f"recover_count={self._recover_count} "
                            f"recover_failed_count={self._recover_failed_count} "
                            f"{self._pose_text()}")
                        if self._try_reenter_top_exit(
                                "recover_clear_upper_zone_reenter_top_exit",
                                front):
                            return
                    self.get_logger().warn(
                        f"RECOVER clear front={front} -> POST_TURN {self._pose_text()}")
                    self._set_state("POST_TURN", "recover_clear", front)
                    return
                if elapsed >= RECOVER_TIMEOUT_SEC:
                    self._stop()
                    self._set_state("RECOVER_FAILED")
                    self.get_logger().error(
                        f"RECOVER_FAILED front={front} heading_err={heading_error} "
                        f"elapsed={elapsed:.2f}s {self._pose_text()}")
                    return
                if elapsed < RECOVER_STOP_SEC:
                    self._stop()
                else:
                    self._publish(RECOVER_BACKUP_SPEED, angular)
                return
            if elapsed < RECOVER_STOP_SEC:
                self._stop()
            elif elapsed < RECOVER_STOP_SEC + RECOVER_BACKUP_SEC:
                self._publish(RECOVER_BACKUP_SPEED, angular)
            else:
                self._publish(RECOVER_FORWARD_SPEED, angular)
            if self._front_is_clear(front, POST_TURN_CLEAR_FRONT_DIST):
                if (self._xy is not None
                        and self._xy[1] > POST_AVOID_LEAVE_TOP_EXIT_Y
                        and self._recent_obstacle_avoidance_active()):
                    self.get_logger().warn(
                        f"RECOVER clear front={front} -> POST_AVOID_LEAVE_TOP "
                        f"reason=recover_clear_upper_zone_reenter_top_exit "
                        f"recover_count={self._recover_count} "
                        f"recover_failed_count={self._recover_failed_count} "
                        f"{self._pose_text()}")
                    if self._try_reenter_top_exit(
                            "recover_clear_upper_zone_reenter_top_exit",
                            front):
                        return
                self.get_logger().warn(
                    f"RECOVER clear front={front} -> POST_TURN {self._pose_text()}")
                self._set_state("POST_TURN", "recover_clear", front)
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
