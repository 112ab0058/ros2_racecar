#!/usr/bin/env python3
"""Passive Phase 7E-S saved-map localization acceptance probe.

The node publishes exactly one configured initial pose when explicitly armed.
It never publishes velocity commands.  ParticleCloud QoS is discovered from
the sole publisher before the subscription is created, and TF publisher GIDs
are captured directly from each callback's Jazzy message metadata.
"""

import csv
import json
import math
import os
from pathlib import Path
import statistics
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage
import yaml


OUT = Path(
    os.environ.get("PHASE7_LOCALIZATION_OUT", "/tmp/phase7er_work")
)
STATUS_PATH = OUT / "localization_probe_status.json"
FINAL_PATH = OUT / "localization_probe_final.json"
SAMPLES_PATH = OUT / "localization_error_samples.csv"
PRE_MOTION_PATH = OUT / "pre_motion_convergence_samples.csv"
PARTICLE_ENDPOINT_PATH = OUT / "particle_endpoint_inventory.csv"
TF_ENDPOINT_PATH = OUT / "tf_endpoint_inventory.csv"
TF_MESSAGE_PATH = OUT / "tf_message_gid_inventory.csv"
MAP_ODOM_PATH = OUT / "map_odom_samples.csv"
INITIAL_POSE_LOG = OUT / "initial_pose_publish_log.txt"
PUBLISH_INITIAL_POSE = OUT / "publish_initial_pose"
TRACKING_STARTED = OUT / "tracking_started"
ROUTE_COMPLETE = OUT / "route_complete"
STOP_FLAG = OUT / "stop_localization_probe"
CAUSAL_STAGE_PATH = OUT / "causal_stage.txt"
CAUSAL_OWNERSHIP_PASS = OUT / "causal_ownership_pass"
TRAVERSAL_STATUS = Path(
    os.environ.get(
        "PHASE7_TRAVERSAL_STATUS",
        "/tmp/phase7dr_work/traversal_status.json",
    )
)

SAMPLE_FIELDS = [
    "index",
    "sim_seconds",
    "phase",
    "waypoint_index",
    "tf_x",
    "tf_y",
    "tf_yaw",
    "amcl_x",
    "amcl_y",
    "amcl_yaw",
    "truth_x",
    "truth_y",
    "truth_yaw",
    "odom_x",
    "odom_y",
    "odom_yaw",
    "position_error_m",
    "yaw_error_rad",
    "covariance_x",
    "covariance_y",
    "covariance_yaw",
    "covariance_trace",
    "covariance_finite_nonnegative",
    "map_cell_state",
    "map_to_odom_stamp_ns",
]


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


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


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def finite_nonnegative(values):
    return all(math.isfinite(value) and value >= 0.0 for value in values)


def gid_key(value):
    """Normalize rclpy endpoint/callback GIDs without guessing their shape."""
    if value is None:
        return "unknown"
    if isinstance(value, str):
        return value.lower()
    if hasattr(value, "data"):
        value = value.data
    if isinstance(value, memoryview):
        value = value.tobytes()
    try:
        return bytes(value).hex()
    except (TypeError, ValueError):
        return str(value)


def callback_gid(message_info):
    """Support Jazzy's dict metadata and attribute-style test doubles."""
    if message_info is None:
        return "unknown"
    if isinstance(message_info, dict):
        return gid_key(message_info.get("publisher_gid"))
    return gid_key(getattr(message_info, "publisher_gid", None))


def enum_name(value):
    name = getattr(value, "name", None)
    if name:
        return str(name)
    text = str(value)
    return text.rsplit(".", 1)[-1]


def qos_record(profile):
    if profile is None:
        return {
            "history": None,
            "depth": None,
            "reliability": None,
            "durability": None,
        }
    return {
        "history": enum_name(getattr(profile, "history", None)),
        "depth": getattr(profile, "depth", None),
        "reliability": enum_name(
            getattr(profile, "reliability", None)
        ),
        "durability": enum_name(
            getattr(profile, "durability", None)
        ),
    }


def validate_particle_qos(record):
    """Return a normalized supported QoS contract or a rejection reason."""
    required = ("history", "depth", "reliability", "durability")
    missing = [
        field
        for field in required
        if record.get(field) is None or record.get(field) == "None"
    ]
    if missing:
        return None, f"missing_fields:{','.join(missing)}"
    history = str(record["history"]).upper()
    reliability = str(record["reliability"]).upper()
    durability = str(record["durability"]).upper()
    try:
        depth = int(record["depth"])
    except (TypeError, ValueError):
        return None, "invalid_depth"
    if history != "KEEP_LAST":
        return None, f"unsupported_history:{history}"
    if depth <= 0:
        return None, "invalid_depth"
    if reliability not in {"RELIABLE", "BEST_EFFORT"}:
        return None, f"unsupported_reliability:{reliability}"
    if durability not in {"VOLATILE", "TRANSIENT_LOCAL"}:
        return None, f"unsupported_durability:{durability}"
    return {
        "history": history,
        "depth": depth,
        "reliability": reliability,
        "durability": durability,
    }, None


def particle_qos_profile(record):
    normalized, reason = validate_particle_qos(record)
    if reason is not None:
        raise ValueError(reason)
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=normalized["depth"],
        reliability=(
            ReliabilityPolicy.RELIABLE
            if normalized["reliability"] == "RELIABLE"
            else ReliabilityPolicy.BEST_EFFORT
        ),
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if normalized["durability"] == "TRANSIENT_LOCAL"
            else DurabilityPolicy.VOLATILE
        ),
    )


def select_particle_endpoint(records):
    """Select one complete ParticleCloud endpoint without ambiguity."""
    if not records:
        return None, "waiting_for_single_publisher"
    if len(records) != 1:
        return None, f"publisher_count:{len(records)}"
    record = records[0]
    if record.get("topic_type") != "nav2_msgs/msg/ParticleCloud":
        return None, f"unexpected_type:{record.get('topic_type')}"
    _normalized, reason = validate_particle_qos(record)
    if reason is not None:
        return None, reason
    return record, None


def compose_planar(first, second):
    """Compose parent->middle and middle->child planar transforms."""
    cosine = math.cos(first[2])
    sine = math.sin(first[2])
    return (
        first[0] + cosine * second[0] - sine * second[1],
        first[1] + sine * second[0] + cosine * second[1],
        wrap_angle(first[2] + second[2]),
    )


class MetricsCore:
    """Pure fixed-rate TF sampling and localization-error accounting."""

    def __init__(self):
        self.samples = []
        self.pre_motion_attempt_count = 0
        self.dynamic_attempt_count = 0
        self.dynamic_invalid_count = 0
        self.time_regression_count = 0
        self.invalid_reasons = {}
        self._last_attempt_sim = None
        self._sequence = []
        self.convergence = None

    def invalid_attempt(self, sim_seconds, phase, reason):
        if phase == "pre_motion":
            self.pre_motion_attempt_count += 1
            self._sequence = []
        elif phase in {"dynamic", "post_stop"}:
            self.dynamic_attempt_count += 1
            self.dynamic_invalid_count += 1
        self.invalid_reasons[reason] = (
            self.invalid_reasons.get(reason, 0) + 1
        )
        if (
            self._last_attempt_sim is not None
            and sim_seconds is not None
            and sim_seconds < self._last_attempt_sim
        ):
            self.time_regression_count += 1
        if sim_seconds is not None:
            self._last_attempt_sim = sim_seconds

    def record(
        self,
        sim_seconds,
        phase,
        tf_pose,
        amcl_pose,
        truth_pose,
        covariance,
        *,
        map_cell_state,
        map_to_odom_stamp_ns,
        waypoint_index=None,
    ):
        if phase == "pre_motion":
            self.pre_motion_attempt_count += 1
        elif phase in {"dynamic", "post_stop"}:
            self.dynamic_attempt_count += 1
        if (
            self._last_attempt_sim is not None
            and sim_seconds < self._last_attempt_sim
        ):
            self.time_regression_count += 1
        self._last_attempt_sim = sim_seconds
        position_error = math.hypot(
            tf_pose[0] - truth_pose[0],
            tf_pose[1] - truth_pose[1],
        )
        yaw_error = abs(wrap_angle(tf_pose[2] - truth_pose[2]))
        covariance_finite = finite_nonnegative(covariance)
        values = [
            sim_seconds,
            *tf_pose,
            *amcl_pose,
            *truth_pose,
            *covariance,
            position_error,
            yaw_error,
        ]
        values_finite = all(math.isfinite(value) for value in values)
        sample = {
            "index": len(self.samples),
            "sim_seconds": sim_seconds,
            "phase": phase,
            "waypoint_index": waypoint_index,
            "tf_x": tf_pose[0],
            "tf_y": tf_pose[1],
            "tf_yaw": tf_pose[2],
            "amcl_x": amcl_pose[0],
            "amcl_y": amcl_pose[1],
            "amcl_yaw": amcl_pose[2],
            "truth_x": truth_pose[0],
            "truth_y": truth_pose[1],
            "truth_yaw": truth_pose[2],
            "odom_x": truth_pose[0],
            "odom_y": truth_pose[1],
            "odom_yaw": truth_pose[2],
            "position_error_m": position_error,
            "yaw_error_rad": yaw_error,
            "covariance_x": covariance[0],
            "covariance_y": covariance[1],
            "covariance_yaw": covariance[2],
            "covariance_trace": sum(covariance),
            "covariance_finite_nonnegative": covariance_finite,
            "map_cell_state": map_cell_state,
            "map_to_odom_stamp_ns": map_to_odom_stamp_ns,
        }
        self.samples.append(sample)

        good = (
            values_finite
            and position_error <= 0.100
            and yaw_error <= 0.200
            and covariance_finite
            and map_cell_state == "free"
        )
        if phase == "pre_motion":
            if good:
                if (
                    self._sequence
                    and sim_seconds <= self._sequence[-1]["sim_seconds"]
                ):
                    self.time_regression_count += 1
                    self._sequence = []
                self._sequence.append(sample)
                stamps = [
                    row["sim_seconds"] for row in self._sequence
                ]
                duration = stamps[-1] - stamps[0]
                gaps = [
                    right - left
                    for left, right in zip(stamps, stamps[1:])
                ]
                rate = (
                    (len(stamps) - 1) / duration
                    if duration > 0.0
                    else 0.0
                )
                if (
                    self.convergence is None
                    and duration >= 1.0
                    and len(stamps) >= 11
                    and rate >= 10.0
                    and (not gaps or max(gaps) <= 0.150)
                ):
                    window = list(self._sequence)
                    self.convergence = {
                        "start_sim": stamps[0],
                        "end_sim": stamps[-1],
                        "duration_sim_seconds": duration,
                        "sample_count": len(window),
                        "query_rate_hz": rate,
                        "maximum_query_gap_sim_seconds": (
                            max(gaps) if gaps else 0.0
                        ),
                        "position_error_max_m": max(
                            row["position_error_m"] for row in window
                        ),
                        "yaw_error_max_rad": max(
                            row["yaw_error_rad"] for row in window
                        ),
                        "map_to_odom_stamp_update_count": len(
                            {
                                row["map_to_odom_stamp_ns"]
                                for row in window
                            }
                        ),
                        "sample_start_index": window[0]["index"],
                        "sample_end_index": window[-1]["index"],
                    }
            else:
                self._sequence = []
        return sample

    def summary(self):
        dynamic = [
            sample
            for sample in self.samples
            if sample["phase"] in {"dynamic", "post_stop"}
        ]
        positions = [sample["position_error_m"] for sample in dynamic]
        yaws = [sample["yaw_error_rad"] for sample in dynamic]
        final_samples = [
            sample
            for sample in self.samples
            if sample["phase"] == "post_stop"
        ]
        valid_dynamic = len(dynamic)
        return {
            "sample_count": len(self.samples),
            "pre_motion_attempt_count": self.pre_motion_attempt_count,
            "dynamic_attempt_count": self.dynamic_attempt_count,
            "dynamic_sample_count": valid_dynamic,
            "dynamic_invalid_count": self.dynamic_invalid_count,
            "sample_coverage_ratio": (
                valid_dynamic / self.dynamic_attempt_count
                if self.dynamic_attempt_count
                else None
            ),
            "converged": self.convergence is not None,
            "convergence": self.convergence,
            "time_regression_count": self.time_regression_count,
            "invalid_reasons": self.invalid_reasons,
            "position_error_mean_m": (
                statistics.fmean(positions) if positions else None
            ),
            "position_error_median_m": (
                statistics.median(positions) if positions else None
            ),
            "position_error_p95_m": percentile(positions, 0.95),
            "position_error_max_m": max(positions) if positions else None,
            "yaw_error_mean_rad": (
                statistics.fmean(yaws) if yaws else None
            ),
            "yaw_error_median_rad": (
                statistics.median(yaws) if yaws else None
            ),
            "yaw_error_p95_rad": percentile(yaws, 0.95),
            "yaw_error_max_rad": max(yaws) if yaws else None,
            "position_within_0p150_ratio": (
                sum(value <= 0.150 for value in positions) / len(positions)
                if positions
                else None
            ),
            "yaw_within_0p300_ratio": (
                sum(value <= 0.300 for value in yaws) / len(yaws)
                if yaws
                else None
            ),
            "map_cell_violation_count": sum(
                sample["map_cell_state"] != "free" for sample in dynamic
            ),
            "covariance_invalid_count": sum(
                not sample["covariance_finite_nonnegative"]
                for sample in dynamic
            ),
            "final_sample": final_samples[-1] if final_samples else None,
        }


class LocalizationProbe(Node):
    def __init__(self, config_path):
        super().__init__("phase7er_localization_probe")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        document = yaml.safe_load(Path(config_path).read_text())
        self.config = document["phase7_localization_validation"]
        OUT.mkdir(parents=True, exist_ok=True)
        self.metrics = MetricsCore()
        self.clock_seconds = None
        self.last_clock_seconds = None
        self.clock_regression_count = 0
        self.latest_odom = None
        self.latest_amcl = None
        self.latest_transforms = {}
        self.last_sample_sim = None
        self.initial_pose_publish_count = 0
        self.initial_pose_publish_sim = None
        self.initial_pose_first_amcl_sim = None
        self.first_map_to_odom_after_initial_sim = None
        self.map_to_odom_updates_at_initial = 0
        self.map_to_odom_stamp_ns = None
        self.map_to_odom_stamp_update_count = 0
        self.map_to_odom_stamp_regression_count = 0
        self.map_to_odom_receive_sim = None
        self.map_to_odom_stage_counts = {}
        self.start_wall_ns = time.monotonic_ns()
        self.topic_records = {}
        self.particle_message_count = 0
        self.last_particle_count = 0
        self.last_particle_weight_sum = None
        self.last_particle_spread = None
        self.particle_subscription = None
        self.particle_qos_state = "waiting_for_single_publisher"
        self.particle_qos_rejection_reason = None
        self.particle_endpoint_records = []
        self.particle_selected_endpoint = None
        self.map_metadata = None
        self.map_grid = None
        self.tf_edges = {}
        self.tf_callback_unknown_gid_count = 0
        self.publisher_names_by_gid = {}
        self.endpoint_records = {}
        self.publisher_history = {
            "/map": [],
            "/cmd_vel": [],
            "/tf": [],
            "/tf_static": [],
            "/particle_cloud": [],
        }
        self.publisher_max = {
            topic: 0 for topic in self.publisher_history
        }
        self.disallowed_nodes_seen = set()
        self.odom_covariance_all_zero = True
        self.cmd_vel_nonzero_count = 0
        self.post_stop_nonzero_count = 0
        self.last_command = [0.0, 0.0]
        self.amcl_dynamic_sims = []
        self.tracking_start_sim = None
        self.route_complete_sim = None
        self.cached_waypoint_index = None
        self.last_traversal_read_wall = 0.0
        self.exception = None

        initial_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", initial_qos
        )
        self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10,
        )
        self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.create_subscription(
            TFMessage, "/tf", self.tf_callback, tf_qos
        )
        self.create_subscription(
            TFMessage,
            "/tf_static",
            self.tf_static_callback,
            tf_static_qos,
        )
        self.create_timer(0.05, self.sample_timer)
        self.create_timer(0.10, self.control_timer)
        self.create_timer(0.25, self.graph_timer)
        self.create_timer(0.20, self.write_status)

        self.initialize_csv_files()

    def initialize_csv_files(self):
        with SAMPLES_PATH.open("w", newline="") as stream:
            csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS).writeheader()
        with PRE_MOTION_PATH.open("w", newline="") as stream:
            csv.DictWriter(
                stream,
                fieldnames=["valid", "reason", *SAMPLE_FIELDS],
            ).writeheader()
        with PARTICLE_ENDPOINT_PATH.open("w", newline="") as stream:
            csv.DictWriter(
                stream,
                fieldnames=[
                    "node_name",
                    "node_namespace",
                    "qualified_name",
                    "gid",
                    "topic_type",
                    "history",
                    "depth",
                    "reliability",
                    "durability",
                    "selected",
                    "state",
                ],
            ).writeheader()
        with TF_ENDPOINT_PATH.open("w", newline="") as stream:
            csv.DictWriter(
                stream,
                fieldnames=[
                    "topic",
                    "node_name",
                    "node_namespace",
                    "qualified_name",
                    "gid",
                    "topic_type",
                    "history",
                    "depth",
                    "reliability",
                    "durability",
                ],
            ).writeheader()
        with TF_MESSAGE_PATH.open("w", newline="") as stream:
            csv.DictWriter(
                stream,
                fieldnames=[
                    "index",
                    "sim_seconds",
                    "topic",
                    "callback_gid",
                    "resolved_publisher",
                    "transform_count",
                    "transforms",
                ],
            ).writeheader()
        with MAP_ODOM_PATH.open("w", newline="") as stream:
            csv.DictWriter(
                stream,
                fieldnames=[
                    "index",
                    "sim_seconds",
                    "wall_monotonic_ns",
                    "causal_stage",
                    "transform_stamp_ns",
                    "x",
                    "y",
                    "yaw",
                    "callback_gid",
                ],
            ).writeheader()

    def current_causal_stage(self):
        try:
            value = CAUSAL_STAGE_PATH.read_text().strip()
        except OSError:
            return "UNSET"
        return value or "UNSET"

    def record_topic(self, name):
        record = self.topic_records.setdefault(
            name,
            {
                "count": 0,
                "first_wall_ns": None,
                "last_wall_ns": None,
                "first_sim": None,
                "last_sim": None,
            },
        )
        now_ns = time.monotonic_ns()
        record["count"] += 1
        if record["first_wall_ns"] is None:
            record["first_wall_ns"] = now_ns
            record["first_sim"] = self.clock_seconds
        record["last_wall_ns"] = now_ns
        record["last_sim"] = self.clock_seconds

    def clock_callback(self, message):
        value = float(message.clock.sec) + float(message.clock.nanosec) / 1e9
        if self.last_clock_seconds is not None and value < self.last_clock_seconds:
            self.clock_regression_count += 1
        self.last_clock_seconds = value
        self.clock_seconds = value
        self.record_topic("/clock")

    def scan_callback(self, _message):
        self.record_topic("/scan")

    def joint_states_callback(self, _message):
        self.record_topic("/joint_states")

    def odom_callback(self, message):
        self.record_topic("/odom")
        pose = message.pose.pose
        covariance = list(message.pose.covariance)
        self.odom_covariance_all_zero = (
            self.odom_covariance_all_zero
            and all(abs(value) <= 1e-15 for value in covariance)
            and all(
                abs(value) <= 1e-15 for value in message.twist.covariance
            )
        )
        self.latest_odom = {
            "wall_ns": time.monotonic_ns(),
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": quaternion_yaw(pose.orientation),
        }

    def map_callback(self, message):
        self.record_topic("/map")
        self.map_metadata = {
            "frame_id": message.header.frame_id,
            "resolution": float(message.info.resolution),
            "width": int(message.info.width),
            "height": int(message.info.height),
            "origin": [
                float(message.info.origin.position.x),
                float(message.info.origin.position.y),
                quaternion_yaw(message.info.origin.orientation),
            ],
        }
        self.map_grid = list(message.data)

    def map_cell_state(self, x, y):
        if self.map_metadata is None or self.map_grid is None:
            return "unavailable"
        resolution = self.map_metadata["resolution"]
        origin_x, origin_y, _ = self.map_metadata["origin"]
        column = int(math.floor((x - origin_x) / resolution))
        row = int(math.floor((y - origin_y) / resolution))
        width = self.map_metadata["width"]
        height = self.map_metadata["height"]
        if column < 0 or row < 0 or column >= width or row >= height:
            return "outside"
        value = self.map_grid[row * width + column]
        if value < 0:
            return "unknown"
        if value >= 65:
            return "occupied"
        return "free"

    def current_phase(self):
        if ROUTE_COMPLETE.exists():
            if self.route_complete_sim is None:
                self.route_complete_sim = self.clock_seconds
            return "post_stop"
        if TRACKING_STARTED.exists():
            if self.tracking_start_sim is None:
                self.tracking_start_sim = self.clock_seconds
            return "dynamic"
        return "pre_motion"

    def amcl_callback(self, message):
        self.record_topic("/amcl_pose")
        pose = message.pose.pose
        covariance = list(message.pose.covariance)
        self.latest_amcl = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": quaternion_yaw(pose.orientation),
            "covariance": [
                float(covariance[0]),
                float(covariance[7]),
                float(covariance[35]),
            ],
            "sim_seconds": self.clock_seconds,
            "wall_ns": time.monotonic_ns(),
        }
        if self.initial_pose_publish_count == 1:
            if self.initial_pose_first_amcl_sim is None:
                self.initial_pose_first_amcl_sim = self.clock_seconds
            if self.current_phase() in {"dynamic", "post_stop"}:
                self.amcl_dynamic_sims.append(self.clock_seconds)

    def particle_callback(self, message):
        self.record_topic("/particle_cloud")
        self.particle_message_count += 1
        self.last_particle_count = len(message.particles)
        weights = [float(particle.weight) for particle in message.particles]
        self.last_particle_weight_sum = sum(weights)
        if not message.particles:
            self.last_particle_spread = None
            return
        if self.last_particle_weight_sum > 0.0:
            normalized = [
                weight / self.last_particle_weight_sum for weight in weights
            ]
        else:
            normalized = [1.0 / len(weights)] * len(weights)
        mean_x = sum(
            weight * float(particle.pose.position.x)
            for weight, particle in zip(normalized, message.particles)
        )
        mean_y = sum(
            weight * float(particle.pose.position.y)
            for weight, particle in zip(normalized, message.particles)
        )
        variance = sum(
            weight
            * (
                (float(particle.pose.position.x) - mean_x) ** 2
                + (float(particle.pose.position.y) - mean_y) ** 2
            )
            for weight, particle in zip(normalized, message.particles)
        )
        self.last_particle_spread = math.sqrt(max(0.0, variance))

    def cmd_vel_callback(self, message):
        self.record_topic("/cmd_vel")
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        self.last_command = [linear, angular]
        if abs(linear) > 1e-12 or abs(angular) > 1e-12:
            self.cmd_vel_nonzero_count += 1
            if ROUTE_COMPLETE.exists():
                self.post_stop_nonzero_count += 1

    def transform_tuple(self, transform):
        value = transform.transform
        return (
            float(value.translation.x),
            float(value.translation.y),
            quaternion_yaw(value.rotation),
        )

    def record_tf(self, message, message_info, topic):
        self.record_topic(topic)
        source_gid = callback_gid(message_info)
        if source_gid == "unknown":
            self.tf_callback_unknown_gid_count += 1
        descriptions = []
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            edge = f"{parent}->{child}"
            stamp_ns = (
                int(transform.header.stamp.sec) * 1_000_000_000
                + int(transform.header.stamp.nanosec)
            )
            descriptions.append(f"{edge}@{stamp_ns}")
            record = self.tf_edges.setdefault(
                edge,
                {"topic": topic, "count": 0, "source_gids": []},
            )
            record["count"] += 1
            if source_gid not in record["source_gids"]:
                record["source_gids"].append(source_gid)
            self.latest_transforms[edge] = {
                "pose": self.transform_tuple(transform),
                "stamp_ns": stamp_ns,
                "receive_sim": self.clock_seconds,
                "source_gid": source_gid,
            }
            if edge == "map->odom":
                stage = self.current_causal_stage()
                self.map_to_odom_stage_counts[stage] = (
                    self.map_to_odom_stage_counts.get(stage, 0) + 1
                )
                with MAP_ODOM_PATH.open("a", newline="") as stream:
                    csv.DictWriter(
                        stream,
                        fieldnames=[
                            "index",
                            "sim_seconds",
                            "wall_monotonic_ns",
                            "causal_stage",
                            "transform_stamp_ns",
                            "x",
                            "y",
                            "yaw",
                            "callback_gid",
                        ],
                    ).writerow(
                        {
                            "index": sum(
                                self.map_to_odom_stage_counts.values()
                            ),
                            "sim_seconds": self.clock_seconds,
                            "wall_monotonic_ns": time.monotonic_ns(),
                            "causal_stage": stage,
                            "transform_stamp_ns": stamp_ns,
                            "x": self.latest_transforms[edge]["pose"][0],
                            "y": self.latest_transforms[edge]["pose"][1],
                            "yaw": self.latest_transforms[edge]["pose"][2],
                            "callback_gid": source_gid,
                        }
                    )
                if (
                    self.map_to_odom_stamp_ns is not None
                    and stamp_ns < self.map_to_odom_stamp_ns
                ):
                    self.map_to_odom_stamp_regression_count += 1
                if (
                    self.map_to_odom_stamp_ns is None
                    or stamp_ns > self.map_to_odom_stamp_ns
                ):
                    self.map_to_odom_stamp_update_count += 1
                    self.map_to_odom_stamp_ns = stamp_ns
                self.map_to_odom_receive_sim = self.clock_seconds
                if (
                    self.initial_pose_publish_count == 1
                    and self.first_map_to_odom_after_initial_sim is None
                ):
                    self.first_map_to_odom_after_initial_sim = (
                        self.clock_seconds
                    )
        with TF_MESSAGE_PATH.open("a", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "index",
                    "sim_seconds",
                    "topic",
                    "callback_gid",
                    "resolved_publisher",
                    "transform_count",
                    "transforms",
                ],
            )
            writer.writerow(
                {
                    "index": sum(
                        record["count"] for record in self.tf_edges.values()
                    ),
                    "sim_seconds": self.clock_seconds,
                    "topic": topic,
                    "callback_gid": source_gid,
                    "resolved_publisher": self.publisher_names_by_gid.get(
                        source_gid, "unresolved"
                    ),
                    "transform_count": len(message.transforms),
                    "transforms": "|".join(descriptions),
                }
            )

    def tf_callback(self, message, message_info):
        self.record_tf(message, message_info, "/tf")

    def tf_static_callback(self, message, message_info):
        self.record_tf(message, message_info, "/tf_static")

    def publish_initial_pose_once(self):
        if self.initial_pose_publish_count != 0:
            return
        self.metrics = MetricsCore()
        self.initialize_sample_csvs()
        self.latest_amcl = None
        self.particle_message_count = 0
        self.map_to_odom_updates_at_initial = (
            self.map_to_odom_stamp_update_count
        )
        contract = self.config["initial_pose"]
        message = PoseWithCovarianceStamped()
        message.header.frame_id = contract["frame_id"]
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(contract["x"])
        message.pose.pose.position.y = float(contract["y"])
        yaw = float(contract["yaw"])
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = float(contract["covariance_x"])
        message.pose.covariance[7] = float(contract["covariance_y"])
        message.pose.covariance[35] = float(contract["covariance_yaw"])
        self.initial_pose_publisher.publish(message)
        self.initial_pose_publish_count = 1
        self.initial_pose_publish_sim = self.clock_seconds
        INITIAL_POSE_LOG.write_text(
            "publish_count=1\n"
            f"sim_seconds={self.clock_seconds}\n"
            f"frame_id={contract['frame_id']}\n"
            f"x={float(contract['x'])}\n"
            f"y={float(contract['y'])}\n"
            f"yaw={yaw}\n"
            f"covariance_x={float(contract['covariance_x'])}\n"
            f"covariance_y={float(contract['covariance_y'])}\n"
            f"covariance_yaw={float(contract['covariance_yaw'])}\n"
        )

    def initialize_sample_csvs(self):
        with SAMPLES_PATH.open("w", newline="") as stream:
            csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS).writeheader()
        with PRE_MOTION_PATH.open("w", newline="") as stream:
            csv.DictWriter(
                stream,
                fieldnames=["valid", "reason", *SAMPLE_FIELDS],
            ).writeheader()

    def append_sample(self, sample):
        with SAMPLES_PATH.open("a", newline="") as stream:
            csv.DictWriter(
                stream, fieldnames=SAMPLE_FIELDS
            ).writerow(sample)
        if sample["phase"] == "pre_motion":
            with PRE_MOTION_PATH.open("a", newline="") as stream:
                csv.DictWriter(
                    stream,
                    fieldnames=["valid", "reason", *SAMPLE_FIELDS],
                ).writerow({"valid": True, "reason": "", **sample})

    def append_invalid_pre_motion(self, reason):
        if self.current_phase() != "pre_motion":
            return
        with PRE_MOTION_PATH.open("a", newline="") as stream:
            csv.DictWriter(
                stream,
                fieldnames=["valid", "reason", *SAMPLE_FIELDS],
            ).writerow(
                {
                    "valid": False,
                    "reason": reason,
                    "sim_seconds": self.clock_seconds,
                    "phase": "pre_motion",
                }
            )

    def waypoint_index(self):
        now = time.monotonic()
        if now - self.last_traversal_read_wall < 0.20:
            return self.cached_waypoint_index
        self.last_traversal_read_wall = now
        try:
            status = json.loads(TRAVERSAL_STATUS.read_text())
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return self.cached_waypoint_index
        self.cached_waypoint_index = status.get("waypoint_index")
        return self.cached_waypoint_index

    def sample_timer(self):
        if self.initial_pose_publish_count != 1 or self.clock_seconds is None:
            return
        if (
            self.last_sample_sim is not None
            and self.clock_seconds <= self.last_sample_sim
        ):
            return
        self.last_sample_sim = self.clock_seconds
        phase = self.current_phase()
        reason = None
        if self.latest_odom is None:
            reason = "missing_truth_odom"
        elif time.monotonic_ns() - self.latest_odom["wall_ns"] > 300_000_000:
            reason = "stale_truth_odom"
        elif self.latest_amcl is None:
            reason = "missing_amcl_pose"
        elif "map->odom" not in self.latest_transforms:
            reason = "missing_map_to_odom"
        elif "odom->base_footprint" not in self.latest_transforms:
            reason = "missing_odom_to_base_footprint"
        else:
            map_tf = self.latest_transforms["map->odom"]
            odom_tf = self.latest_transforms["odom->base_footprint"]
            if (
                map_tf["receive_sim"] is None
                or self.clock_seconds - map_tf["receive_sim"] > 0.350
            ):
                reason = "stale_map_to_odom"
            elif (
                odom_tf["receive_sim"] is None
                or self.clock_seconds - odom_tf["receive_sim"] > 0.250
            ):
                reason = "stale_odom_to_base_footprint"
        if reason is not None:
            self.metrics.invalid_attempt(self.clock_seconds, phase, reason)
            self.append_invalid_pre_motion(reason)
            return
        tf_pose = compose_planar(map_tf["pose"], odom_tf["pose"])
        amcl_pose = (
            self.latest_amcl["x"],
            self.latest_amcl["y"],
            self.latest_amcl["yaw"],
        )
        truth_pose = (
            self.latest_odom["x"],
            self.latest_odom["y"],
            self.latest_odom["yaw"],
        )
        sample = self.metrics.record(
            self.clock_seconds,
            phase,
            tf_pose,
            amcl_pose,
            truth_pose,
            self.latest_amcl["covariance"],
            map_cell_state=self.map_cell_state(tf_pose[0], tf_pose[1]),
            map_to_odom_stamp_ns=map_tf["stamp_ns"],
            waypoint_index=self.waypoint_index(),
        )
        self.append_sample(sample)

    def control_timer(self):
        if (
            PUBLISH_INITIAL_POSE.exists()
            and self.initial_pose_publish_count == 0
            and self.initial_pose_publisher.get_subscription_count() > 0
            and self.clock_seconds is not None
        ):
            self.publish_initial_pose_once()
        if STOP_FLAG.exists():
            self.write_status(final=True)
            rclpy.shutdown()

    def qualified_name(self, endpoint):
        namespace = endpoint.node_namespace.rstrip("/")
        if namespace:
            return f"{namespace}/{endpoint.node_name}"
        return f"/{endpoint.node_name}"

    def endpoint_record(self, topic, endpoint):
        qos = qos_record(endpoint.qos_profile)
        return {
            "topic": topic,
            "node_name": endpoint.node_name,
            "node_namespace": endpoint.node_namespace,
            "qualified_name": self.qualified_name(endpoint),
            "gid": gid_key(endpoint.endpoint_gid),
            "topic_type": endpoint.topic_type,
            **qos,
        }

    def write_endpoint_inventories(self):
        particle_rows = []
        selected_gid = (
            self.particle_selected_endpoint.get("gid")
            if self.particle_selected_endpoint
            else None
        )
        for record in self.particle_endpoint_records:
            particle_rows.append(
                {
                    **{key: record.get(key) for key in (
                        "node_name",
                        "node_namespace",
                        "qualified_name",
                        "gid",
                        "topic_type",
                        "history",
                        "depth",
                        "reliability",
                        "durability",
                    )},
                    "selected": record.get("gid") == selected_gid,
                    "state": self.particle_qos_state,
                }
            )
        with PARTICLE_ENDPOINT_PATH.open("w", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "node_name",
                    "node_namespace",
                    "qualified_name",
                    "gid",
                    "topic_type",
                    "history",
                    "depth",
                    "reliability",
                    "durability",
                    "selected",
                    "state",
                ],
            )
            writer.writeheader()
            writer.writerows(particle_rows)
        tf_rows = [
            record
            for topic in ("/tf", "/tf_static")
            for record in self.endpoint_records.get(topic, [])
        ]
        with TF_ENDPOINT_PATH.open("w", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "topic",
                    "node_name",
                    "node_namespace",
                    "qualified_name",
                    "gid",
                    "topic_type",
                    "history",
                    "depth",
                    "reliability",
                    "durability",
                ],
            )
            writer.writeheader()
            writer.writerows(tf_rows)

    def discover_particle_qos(self):
        endpoints = self.get_publishers_info_by_topic("/particle_cloud")
        self.particle_endpoint_records = [
            self.endpoint_record("/particle_cloud", endpoint)
            for endpoint in endpoints
        ]
        if self.particle_subscription is not None:
            return
        selected, reason = select_particle_endpoint(
            self.particle_endpoint_records
        )
        if reason == "waiting_for_single_publisher":
            self.particle_qos_state = "waiting_for_single_publisher"
            self.particle_qos_rejection_reason = None
            return
        if reason is not None:
            self.particle_qos_state = "rejected"
            self.particle_qos_rejection_reason = reason
            return
        self.particle_subscription = self.create_subscription(
            ParticleCloud,
            "/particle_cloud",
            self.particle_callback,
            particle_qos_profile(selected),
        )
        self.particle_selected_endpoint = selected
        self.particle_qos_state = "subscribed_exact"
        self.particle_qos_rejection_reason = None

    def graph_timer(self):
        for topic in (
            "/map",
            "/cmd_vel",
            "/tf",
            "/tf_static",
            "/particle_cloud",
        ):
            endpoints = self.get_publishers_info_by_topic(topic)
            records = [
                self.endpoint_record(topic, endpoint)
                for endpoint in endpoints
            ]
            self.endpoint_records[topic] = records
            names = sorted({record["qualified_name"] for record in records})
            for record in records:
                self.publisher_names_by_gid[record["gid"]] = record[
                    "qualified_name"
                ]
            self.publisher_max[topic] = max(
                self.publisher_max[topic], len(names)
            )
            history = self.publisher_history[topic]
            if not history or history[-1]["names"] != names:
                history.append(
                    {
                        "sim_seconds": self.clock_seconds,
                        "names": names,
                        "count": len(names),
                    }
                )
        self.discover_particle_qos()
        self.write_endpoint_inventories()
        disallowed = {
            "slam_toolbox",
            "planner_server",
            "controller_server",
            "bt_navigator",
            "behavior_server",
            "velocity_smoother",
            "robot_localization",
            "line_follower",
        }
        for name, namespace in self.get_node_names_and_namespaces():
            if name in disallowed or any(
                token in name for token in ("costmap", "ackermann", "stm32")
            ):
                self.disallowed_nodes_seen.add(
                    f"{namespace.rstrip('/')}/{name}"
                )

    def topic_rates(self):
        result = {}
        for topic, record in self.topic_records.items():
            count = record["count"]
            first = record["first_wall_ns"]
            last = record["last_wall_ns"]
            if count > 1 and first is not None and last is not None:
                duration = (last - first) / 1e9
                rate = (count - 1) / duration if duration > 0.0 else 0.0
            else:
                rate = 0.0
            result[topic] = {**record, "rate_hz": rate}
        return result

    def resolved_tf_edges(self):
        result = {}
        for edge, record in self.tf_edges.items():
            result[edge] = {
                **record,
                "publishers": sorted(
                    {
                        self.publisher_names_by_gid.get(gid, "unresolved")
                        for gid in record["source_gids"]
                    }
                ),
            }
        return result

    def covariance_summary(self):
        values = [
            (
                sample["covariance_x"],
                sample["covariance_y"],
                sample["covariance_yaw"],
            )
            for sample in self.metrics.samples
        ]
        traces = [sum(row) for row in values]
        head = traces[:10]
        tail = traces[-10:]
        stable = (
            bool(traces)
            and all(finite_nonnegative(row) for row in values)
            and max(traces) <= 0.25
            and statistics.fmean(tail)
            <= max(0.05, 4.0 * statistics.fmean(head))
        )
        return {
            "first_trace": traces[0] if traces else None,
            "final_trace": traces[-1] if traces else None,
            "maximum_trace": max(traces) if traces else None,
            "head_mean_trace": (
                statistics.fmean(head) if head else None
            ),
            "tail_mean_trace": (
                statistics.fmean(tail) if tail else None
            ),
            "stable": stable,
        }

    def direct_tf_ownership(self):
        record = self.tf_edges.get("map->odom", {})
        unknown_map_to_odom = (
            "unknown" in record.get("source_gids", [])
        )
        gids = sorted(
            gid
            for gid in record.get("source_gids", [])
            if gid != "unknown"
        )
        owners = sorted(
            {
                self.publisher_names_by_gid.get(gid, "unresolved")
                for gid in gids
            }
        )
        return {
            "callback_gid_available": (
                bool(gids) and not unknown_map_to_odom
            ),
            "map_to_odom_gids": gids,
            "map_to_odom_gid_count": len(gids),
            "resolved_publishers": owners,
            "single_amcl_gid": len(gids) == 1 and owners == ["/amcl"],
            "unknown_map_to_odom_gid": unknown_map_to_odom,
            "unknown_callback_gid_count": self.tf_callback_unknown_gid_count,
        }

    def amcl_motion_rate(self):
        stamps = [
            value for value in self.amcl_dynamic_sims if value is not None
        ]
        boundaries = [
            value
            for value in (self.tracking_start_sim, self.route_complete_sim)
            if value is not None
        ]
        if len(stamps) >= 2 and stamps[-1] > stamps[0]:
            rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        else:
            rate = 0.0
        gap_points = sorted(boundaries + stamps)
        gaps = [
            right - left
            for left, right in zip(gap_points, gap_points[1:])
        ]
        return {
            "message_count": len(stamps),
            "rate_hz": rate,
            "maximum_gap_sim_seconds": max(gaps) if gaps else None,
            "tracking_start_sim": self.tracking_start_sim,
            "route_complete_sim": self.route_complete_sim,
        }

    def convergence_status(self):
        summary = self.metrics.summary()
        convergence = summary.get("convergence") or {}
        covariance = self.covariance_summary()
        updates_after_initial = (
            self.map_to_odom_stamp_update_count
            - self.map_to_odom_updates_at_initial
        )
        direct = self.direct_tf_ownership()
        result = {
            "initial_pose_publish_count": self.initial_pose_publish_count,
            "valid_amcl_pose_seen": (
                self.initial_pose_first_amcl_sim is not None
            ),
            "particle_message_seen": self.particle_message_count > 0,
            "particle_qos_state": self.particle_qos_state,
            "tf_window": convergence,
            "map_to_odom_updates_after_initial": updates_after_initial,
            "map_to_odom_stamp_regression_count": (
                self.map_to_odom_stamp_regression_count
            ),
            "clock_regression_count": self.clock_regression_count,
            "covariance_stable": covariance["stable"],
            "direct_tf_single_amcl_gid": direct["single_amcl_gid"],
            "causal_ownership_pass": CAUSAL_OWNERSHIP_PASS.exists(),
        }
        result["pass"] = bool(
            self.initial_pose_publish_count == 1
            and result["valid_amcl_pose_seen"]
            and result["particle_message_seen"]
            and self.particle_qos_state == "subscribed_exact"
            and summary.get("converged") is True
            and convergence.get("sample_count", 0) >= 11
            and convergence.get("query_rate_hz", 0.0) >= 10.0
            and convergence.get(
                "maximum_query_gap_sim_seconds", math.inf
            )
            <= 0.150
            and convergence.get("position_error_max_m", math.inf) <= 0.100
            and convergence.get("yaw_error_max_rad", math.inf) <= 0.200
            and convergence.get(
                "map_to_odom_stamp_update_count", 0
            )
            >= 2
            and updates_after_initial >= 2
            and self.map_to_odom_stamp_regression_count == 0
            and self.clock_regression_count == 0
            and summary.get("time_regression_count") == 0
            and covariance["stable"]
            and result["causal_ownership_pass"]
        )
        return result

    def status(self):
        return {
            "clock_seconds": self.clock_seconds,
            "initial_pose_publish_count": self.initial_pose_publish_count,
            "initial_pose_publish_sim": self.initial_pose_publish_sim,
            "initial_pose_first_amcl_sim": self.initial_pose_first_amcl_sim,
            "first_map_to_odom_after_initial_sim": (
                self.first_map_to_odom_after_initial_sim
            ),
            "metrics": self.metrics.summary(),
            "pre_motion_convergence": self.convergence_status(),
            "particle_message_count": self.particle_message_count,
            "last_particle_count": self.last_particle_count,
            "last_particle_weight_sum": self.last_particle_weight_sum,
            "last_particle_spread_m": self.last_particle_spread,
            "particle_qos_state": self.particle_qos_state,
            "particle_qos_rejection_reason": (
                self.particle_qos_rejection_reason
            ),
            "particle_selected_endpoint": self.particle_selected_endpoint,
            "particle_endpoint_records": self.particle_endpoint_records,
            "map_metadata": self.map_metadata,
            "tf_edges": self.resolved_tf_edges(),
            "direct_tf_ownership": self.direct_tf_ownership(),
            "map_to_odom_stamp_ns": self.map_to_odom_stamp_ns,
            "map_to_odom_stamp_update_count": (
                self.map_to_odom_stamp_update_count
            ),
            "map_to_odom_stamp_regression_count": (
                self.map_to_odom_stamp_regression_count
            ),
            "map_to_odom_stage_counts": self.map_to_odom_stage_counts,
            "publisher_history": self.publisher_history,
            "publisher_max": self.publisher_max,
            "endpoint_records": self.endpoint_records,
            "topic_rates": self.topic_rates(),
            "amcl_motion_rate": self.amcl_motion_rate(),
            "disallowed_nodes_seen": sorted(self.disallowed_nodes_seen),
            "odom_covariance_all_zero": self.odom_covariance_all_zero,
            "covariance_summary": self.covariance_summary(),
            "cmd_vel_nonzero_count": self.cmd_vel_nonzero_count,
            "post_stop_nonzero_count": self.post_stop_nonzero_count,
            "last_command": self.last_command,
            "route_tracking_started": TRACKING_STARTED.exists(),
            "route_complete_marker": ROUTE_COMPLETE.exists(),
            "clock_regression_count": self.clock_regression_count,
            "exception": self.exception,
        }

    def write_status(self, final=False):
        payload = json.dumps(
            self.status(), indent=2, sort_keys=True
        ) + "\n"
        target = FINAL_PATH if final else STATUS_PATH
        temporary = target.with_suffix(target.suffix + ".tmp")
        temporary.write_text(payload)
        temporary.replace(target)


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    arguments, ros_arguments = parser.parse_known_args(argv)
    rclpy.init(args=ros_arguments)
    node = LocalizationProbe(arguments.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exception:
        node.exception = repr(exception)
        raise
    finally:
        try:
            node.write_status(final=True)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
