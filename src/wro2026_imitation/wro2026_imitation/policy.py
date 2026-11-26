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
             (stays stopped until reset(): the phone's 重置 button, accepted only when
             the newest scan is fresh and clear; or bc_node is restarted). Points that
             scan_memory_node filled in (remembered, up to ~1.7 cm off) use memory_margin
             (default 0: only inside the outline itself): on 2026-11-12 a remembered
             point 2.5 cm from the side was drawn 2.0 cm away and latched the car on the
             stand.
             With predict (2026-11-26, Ray: "lock only when the computed car positions
             of the turn are sure to touch, not at some distance"), the margins apply to
             the closest the outline comes along the path the commands already sent will
             drive: horizon m (the car stops within ~2-3 cm), or the rest of the corner
             rule's turn (planned_turn), instead of to the outline where it is now. A wall
             0.5 cm beside a car driving along it does not stop it; a turn whose outer
             corner will reach the wall ahead stops it at once. On the field's 0.38 m
             corridor the island passed the turning car at a measured 1.6-1.9 cm and the
             2 cm stop latched three left turns that the 左滿舵 key then drove through.
  no_scan    no scan yet, or the newest is older than scan_timeout -> stop
  bad_scan   the scan does not start at 0 deg and cover 360 deg   -> stop
  no_odom    a policy that needs odometry has none newer than scan_timeout -> stop
base_node's CommandGate still applies afterwards (0.3 s timeout, speed limits).
"""
import math

import numpy as np

from .corner import DRIVEN, FULL_LEFT, FULL_RIGHT
from .features import (BIN_DEG, HALF_FOV_DEG, LABELS, LABELS_3, LEFT, MAX_RANGE, N_BINS,
                       RIGHT, scan_features, STRAIGHT)
from .turn_predict import LASER_X, TurnPredictor

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


def outline_gap(ranges, angle_min, angle_increment):
    """
    Per beam: how far the point lies outside the car outline (m); inf = no return.

    A point is inside the outline grown by margin when its gap < margin (< 0 = inside
    the car itself). The outline grows by the same margin on every side (a rectangle).
    """
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    gap = np.full(len(r), np.inf)
    seen = np.isfinite(r)
    x, y = r[seen] * np.cos(a[seen]), r[seen] * np.sin(a[seen])
    gap[seen] = np.maximum.reduce([x - FRONT, -REAR - x, np.abs(y) - HALF_WIDTH])
    return gap


def inside_outline(ranges, angle_min, angle_increment, margin):
    """Return True if any lidar point lies inside the car outline grown by margin."""
    return bool((outline_gap(ranges, angle_min, angle_increment) < margin).any())


# lidar directions (deg, 0 = ahead, + = left) as words for the phone page
SIDES = ['前', '左前', '左', '左後', '後', '右後', '右', '右前']


def side_word(deg):
    """Name the direction deg (0 ahead, + left) with one of eight words."""
    return SIDES[int(math.floor(((deg + 22.5) % 360.0) / 45.0)) % 8]


def point_text(point):
    """'右後 離車身 2.5 cm（記憶的點）' for a point (deg, gap m, remembered)."""
    deg, gap, remembered = point
    where = '車身內' if gap < 0 else '離車身 %.1f cm' % (100.0 * gap)
    return '%s %s%s' % (side_word(deg), where, '（記憶的點）' if remembered else '')


def path_text(point):
    """'左前的牆（記憶的點）' for a point the predicted path touches (deg, gap m, remembered)."""
    deg, _, remembered = point
    return '%s的牆%s' % (side_word(deg), '（記憶的點）' if remembered else '')


REASON_TEXT = {'no_scan': '沒有光達資料，停', 'bad_scan': '光達資料格式不對，停',
               'no_odom': '沒有里程計（EKF），停'}
CORNER_TEXT = {'follow': '保持中線', 'delay': '等著打滿', 'turn': '打滿轉彎',
               'settle': '轉完回正', 'done': '跑完了，停在起點',
               'blocked': '這個位置過不了彎，停：用按鍵倒車或把車放回走道中間'}


DIRECTION_TEXT = {'ccw': '彎道往左（逆時針）', 'cw': '彎道往右（順時針）',
                  'auto': '方向還沒判斷（到第一個彎才知道）'}


def state_text(reason, driver, policy_state=None, corners=0, direction=None):
    """
    One Traditional Chinese line for the phone page: what the car does and why.

    direction: corner policy only, 'ccw' / 'cw' / 'auto' (not decided yet).
    """
    if reason == TOO_CLOSE:
        # the phone page shows its 重置 button while this text contains 鎖住
        if getattr(driver, 'predicted', False):
            where = path_text(driver.close_point) if driver.close_point else ''
            now = ('現在往前開還會碰到：%s，先用按鍵開離' % path_text(driver.close_now)
                   if driver.close_now else '現在往前開不會碰到，可以按「重置」')
            return '照現在的轉法開下去會碰到，鎖住停車：%s｜%s' % (where, now)
        where = point_text(driver.close_point) if driver.close_point else ''
        now = ('現在還有：%s，先用按鍵開離' % point_text(driver.close_now) if driver.close_now
               else '現在周圍淨空，可以按「重置」')
        return '太靠近，鎖住停車：%s｜%s' % (where, now)
    if reason != OK:
        return REASON_TEXT.get(reason, reason)
    text = ('%s，過了 %d 個彎' % (CORNER_TEXT.get(policy_state, policy_state), corners)
            if policy_state else '正常')
    if direction:
        text += '｜%s' % DIRECTION_TEXT.get(direction, direction)
    if driver.nearest:
        text += '｜最近：%s' % point_text(driver.nearest)
    return text


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
    """
    Raise ValueError if the network was trained on other inputs than features.py makes.

    Labels: the 5 of features.LABELS (with full lock, 2026-11-25) or the first 3 (older
    models, never full lock); the numbers are the same, so both drive the same way.
    """
    expected = {'half_fov_deg': HALF_FOV_DEG, 'bin_deg': BIN_DEG, 'max_range': MAX_RANGE}
    wrong = [k for k, v in expected.items() if meta.get(k) != v]
    labels = meta.get('labels')
    if labels not in (list(LABELS), list(LABELS_3)):
        wrong.append('labels')
    elif model.w[0].shape[0] != N_BINS or model.w[-1].shape[1] != len(labels):
        wrong.append('layer sizes')
    if wrong:
        raise ValueError('model does not match features.py (%s); train it again with bc_train'
                         % ', '.join(wrong))


class Driver:
    """Safety rules + policy -> (vx, wz). Times are seconds from one monotonic clock."""

    def __init__(self, decide, speed, turn_radius, stop_margin=0.04, scan_timeout=0.3,
                 full_turn_radius=None, memory_margin=0.0, predict=False, horizon=0.05,
                 driven=None):
        """
        full_turn_radius: radius of FULL_LEFT / FULL_RIGHT (default: turn_radius).

        memory_margin: stop margin for remembered points (on_scan's remembered mask).

        predict: apply the margins along the path ahead (see the module docstring), with
        the measured steering (turn_predict.TurnPredictor at this speed) and driven, the
        curvature (1/m) the car drives for each label (default corner.DRIVEN, measured);
        horizon: how far that path goes when no turn is in progress (m).
        """
        full_turn_radius = full_turn_radius or turn_radius
        if (speed <= 0 or turn_radius <= 0 or full_turn_radius <= 0 or stop_margin < 0
                or memory_margin < 0 or scan_timeout <= 0):
            raise ValueError('speed, turn radii, scan_timeout must be > 0, margins >= 0')
        self.memory_margin = memory_margin
        self.close_point = None     # (deg, gap m, remembered) that latched too_close
        self.close_now = None       # same, the closest point inside the margins now (or None)
        self.nearest = None         # (deg, gap m, remembered) closest point of the last scan
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
        self.predictor = TurnPredictor(speed=speed) if predict else None
        self.horizon = horizon
        self.driven = dict(DRIVEN if driven is None else driven)
        self.odo = 0.0              # driven distance (m, odometry)
        self.xy = None
        self.k_now = 0.0            # curvature the car drives (1/m, odometry twist)
        self.sent = []              # (driven distance, label) of the recent commands
        self.predicted = False      # close_point came from the path ahead

    def on_odom(self, t, x, y, yaw, vx=None, wz=None):
        """
        Pass odometry to a policy that uses it (corner); others ignore it.

        vx, wz: the odometry's twist (m/s, rad/s), if known.
        """
        self.odom_time = t
        if self.xy is not None:
            self.odo += math.hypot(x - self.xy[0], y - self.xy[1])
        self.xy = (x, y)
        if vx is not None and wz is not None and abs(vx) > 0.02:
            self.k_now = wz / vx    # stopped: the steering stays where it was
        if self.needs_odom:
            self.decide.on_odom(x, y, yaw, vx, wz)

    def path_gap(self, r, angle_min, angle_increment):
        """
        Return each beam's smallest gap to the outline along the path ahead (m; inf: none).

        The path the commands already sent drive (TurnPredictor.drive_on): at least
        horizon m, and while the policy turns a corner (planned_turn() -> (sign, heading
        still to turn, rad)) the rest of that turn. Latched: horizon only (after 重置 the
        corner rule follows again). Last command STOP (not driving): where it is now.
        """
        a = angle_min + angle_increment * np.arange(len(r))
        ok = np.isfinite(r)
        rr = np.where(ok, r, 0.0)
        px = np.where(ok, rr * np.cos(a) + LASER_X, np.nan)
        py = np.where(ok, rr * np.sin(a), np.nan)
        if not self.sent or self.sent[-1][1] == STOP:
            return self.predictor.gaps(px, py, np.zeros((1, 3)))
        queued = [(self.odo - o, self.driven[lab]) for o, lab in self.sent if lab != STOP]
        plan = None if self.latched else getattr(self.decide, 'planned_turn', lambda: None)()
        sign, sweep = plan if plan else (0.0, None)
        poses = self.predictor.drive_on(self.k_now, queued=queued, length=self.horizon,
                                        sign=sign, sweep=sweep)
        return self.predictor.gaps(px, py, poses)

    def _sent(self, label):
        """Remember the command for path_gap: those still on their way and the one before."""
        if not self.sent or self.sent[-1][1] != label:
            self.sent.append((self.odo, label))
        while len(self.sent) > 1 and self.odo - self.sent[1][0] > 0.1:
            self.sent.pop(0)

    def on_scan(self, t, ranges, angle_min, angle_increment, remembered=None):
        """
        Use one scan: check it, check the car outline, then ask the policy.

        remembered: optional bool per beam, True = filled in by scan_memory_node.
        """
        r = np.asarray(ranges, dtype=np.float32)
        self.scan_time = t
        self.label, self.probs = STOP, None
        if (len(r) == 0 or abs(angle_min) > 1e-3
                or abs(angle_increment * len(r) - 2.0 * math.pi) > 1e-3):
            self.reason = BAD_SCAN
            return
        gap = outline_gap(r, angle_min, angle_increment)
        mem = (np.zeros(len(r), dtype=bool) if remembered is None
               else np.asarray(remembered, dtype=bool))
        k = int(np.argmin(gap))
        self.nearest = (None if not np.isfinite(gap[k]) else
                        (math.degrees(angle_min + angle_increment * k), float(gap[k]),
                         bool(mem[k])))
        if self.predictor is not None:
            gap = self.path_gap(r, angle_min, angle_increment)
        hit = np.nonzero(gap < np.where(mem, self.memory_margin, self.stop_margin))[0]
        self.close_now = None
        if len(hit):
            k = int(hit[np.argmin(gap[hit])])
            self.close_now = (math.degrees(angle_min + angle_increment * k), float(gap[k]),
                              bool(mem[k]))
        if not self.latched and self.close_now:
            self.close_point = self.close_now
            self.predicted = self.predictor is not None
            self.latched = True
        if self.latched:
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
        self._sent(self.label)
        self.reason = OK

    def reset(self, t):
        """
        Unlatch too_close (the phone's 重置); return (done, why in Chinese).

        Done only when the newest scan is fresh (scan_timeout) and has no point inside the
        margins; then the car waits for the next scan before it may drive again, and still
        only while the phone's 自動 button is held.
        """
        if not self.latched:
            return False, '沒有鎖住'
        if (self.scan_time is None or t - self.scan_time > self.scan_timeout
                or self.reason != TOO_CLOSE):
            return False, '沒有新的光達資料'
        if self.close_now:
            return False, '還太靠近：%s' % point_text(self.close_now)
        self.latched = False
        self.close_point = None
        self._sent(STOP)                # stopped: the next scan's check starts from here
        self.scan_time = None           # drive again only after the next scan
        self.reason = NO_SCAN
        self.label, self.probs = STOP, None
        return True, '已解除鎖住'

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
        if self.label == STOP:              # the policy itself stops (corner: laps done)
            return 0.0, 0.0, STOP, OK
        wz = {LEFT: self.wz, STRAIGHT: 0.0, RIGHT: -self.wz,
              FULL_LEFT: self.full_wz, FULL_RIGHT: -self.full_wz}[self.label]
        return self.speed, wz, self.label, OK
