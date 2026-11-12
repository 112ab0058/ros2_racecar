"""
Decide what to drive from one lidar scan. Pure numpy, no ROS (bc_node wraps it).

Two policies answer left / straight / right:
  model   the imitation network from bc_train
  center  hand-written baseline: keep the walls on both sides equally far, and
          when something is ahead, turn to the side with more room

A third one, corner (corner.py, Ray's corner technique), reads the scan itself and
also uses full lock (FULL_LEFT / FULL_RIGHT) and odometry.

Driver puts safety rules before every policy (first one that fires wins):
  too_close  a lidar point inside the car outline + stop_margin -> stop, latched
             (stays stopped until bc_node is restarted)
  no_scan    no scan yet, or the newest is older than scan_timeout -> stop
  bad_scan   the scan does not start at 0 deg and cover 360 deg   -> stop
  no_odom    a policy that needs odometry has none newer than scan_timeout -> stop
base_node's CommandGate still applies afterwards (0.3 s timeout, speed limits).
"""
import math

import numpy as np

from .corner import FULL_LEFT, FULL_RIGHT
from .features import (BIN_DEG, HALF_FOV_DEG, LABELS, LEFT, MAX_RANGE, N_BINS, RIGHT,
                       scan_features, STRAIGHT)

# Car outline seen from the lidar (m). R550_HARDWARE.md: rear axle to the front 0.24 m,
# to the back 0.031 m, width 0.18 m; urdf: lidar 0.13 m in front of the rear axle.
FRONT = 0.24 - 0.13
REAR = 0.031 + 0.13
HALF_WIDTH = 0.09

STOP = -1
OK = 'ok'
TOO_CLOSE = 'too_close'
NO_SCAN = 'no_scan'
BAD_SCAN = 'bad_scan'
NO_ODOM = 'no_odom'


def inside_outline(ranges, angle_min, angle_increment, margin):
    """Return True if any lidar point lies inside the car outline grown by margin."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    seen = np.isfinite(r)
    x, y = r[seen] * np.cos(a[seen]), r[seen] * np.sin(a[seen])
    return bool(((x < FRONT + margin) & (x > -REAR - margin)
                 & (np.abs(y) < HALF_WIDTH + margin)).any())


def bins_between(lo_deg, hi_deg):
    """Return the slice of feature bins from lo_deg to hi_deg (0 = front, + = left)."""
    return slice(int(round((lo_deg + HALF_FOV_DEG) / BIN_DEG)),
                 int(round((hi_deg + HALF_FOV_DEG) / BIN_DEG)))


FRONT_BINS = bins_between(-15.0, 15.0)
LEFT_BINS = bins_between(30.0, 90.0)
RIGHT_BINS = bins_between(-90.0, -30.0)


def center_rule(x, front_dist=0.5, side_margin=0.05):
    """Return the baseline's label for each row of features x (see the module docstring)."""
    d = np.atleast_2d(x) * MAX_RANGE
    front = d[:, FRONT_BINS].min(axis=1)
    left = d[:, LEFT_BINS].min(axis=1)
    right = d[:, RIGHT_BINS].min(axis=1)
    blocked = front < front_dist
    margin = np.where(blocked, 0.0, side_margin)    # blocked: any difference decides
    out = np.full(len(d), STRAIGHT)
    out[left > right + margin] = LEFT
    out[right > left + margin] = RIGHT
    return out


def center_policy(front_dist=0.5, side_margin=0.05):
    """Return decide(features) -> (label, None) for the baseline."""
    def decide(x):
        return int(center_rule(x, front_dist, side_margin)[0]), None
    return decide


def model_policy(model):
    """Return decide(features) -> (label, probabilities) for a trained network."""
    def decide(x):
        p = model.predict_proba(x)[0]
        return int(p.argmax()), p
    return decide


def check_model(model, meta):
    """Raise ValueError if the network was trained on other inputs than features.py makes."""
    expected = {'half_fov_deg': HALF_FOV_DEG, 'bin_deg': BIN_DEG, 'max_range': MAX_RANGE,
                'labels': list(LABELS)}
    wrong = [k for k, v in expected.items() if meta.get(k) != v]
    if model.w[0].shape[0] != N_BINS or model.w[-1].shape[1] != len(LABELS):
        wrong.append('layer sizes')
    if wrong:
        raise ValueError('model does not match features.py (%s); train it again with bc_train'
                         % ', '.join(wrong))


class Driver:
    """Safety rules + policy -> (vx, wz). Times are seconds from one monotonic clock."""

    def __init__(self, decide, speed, turn_radius, stop_margin=0.04, scan_timeout=0.3,
                 full_turn_radius=None):
        """full_turn_radius: radius of FULL_LEFT / FULL_RIGHT (default: turn_radius)."""
        full_turn_radius = full_turn_radius or turn_radius
        if (speed <= 0 or turn_radius <= 0 or full_turn_radius <= 0 or stop_margin < 0
                or scan_timeout <= 0):
            raise ValueError('speed, turn radii, scan_timeout must be > 0, stop_margin >= 0')
        self.decide = decide
        self.speed = speed
        self.wz = speed / turn_radius
        self.full_wz = speed / full_turn_radius
        self.stop_margin = stop_margin
        self.scan_timeout = scan_timeout
        self.needs_odom = hasattr(decide, 'on_odom')
        self.odom_time = None
        self.scan_time = None
        self.label, self.probs = STOP, None
        self.reason = NO_SCAN
        self.latched = False

    def on_odom(self, t, x, y, yaw):
        """Pass odometry to a policy that uses it (corner); others ignore it."""
        self.odom_time = t
        if self.needs_odom:
            self.decide.on_odom(x, y, yaw)

    def on_scan(self, t, ranges, angle_min, angle_increment):
        """Use one scan: check it, check the car outline, then ask the policy."""
        r = np.asarray(ranges, dtype=np.float32)
        self.scan_time = t
        self.label, self.probs = STOP, None
        if (len(r) == 0 or abs(angle_min) > 1e-3
                or abs(angle_increment * len(r) - 2.0 * math.pi) > 1e-3):
            self.reason = BAD_SCAN
            return
        if self.latched or inside_outline(r, angle_min, angle_increment, self.stop_margin):
            self.latched = True
            self.reason = TOO_CLOSE
            return
        if self.needs_odom and (self.odom_time is None
                                or t - self.odom_time > self.scan_timeout):
            self.reason = NO_ODOM
            return
        if getattr(self.decide, 'takes_ranges', False):
            self.label, self.probs = self.decide(r, angle_min, angle_increment)
        else:
            self.label, self.probs = self.decide(scan_features(r))
        self.reason = OK

    def command(self, t):
        """Return (vx, wz, decision, reason) to publish now; decision STOP means 0, 0."""
        if self.latched:
            reason = TOO_CLOSE
        elif self.scan_time is None or t - self.scan_time > self.scan_timeout:
            reason = NO_SCAN
        elif self.needs_odom and (self.odom_time is None
                                  or t - self.odom_time > self.scan_timeout):
            reason = NO_ODOM
        else:
            reason = self.reason
        if reason != OK:
            return 0.0, 0.0, STOP, reason
        wz = {LEFT: self.wz, STRAIGHT: 0.0, RIGHT: -self.wz,
              FULL_LEFT: self.full_wz, FULL_RIGHT: -self.full_wz}[self.label]
        return self.speed, wz, self.label, OK
