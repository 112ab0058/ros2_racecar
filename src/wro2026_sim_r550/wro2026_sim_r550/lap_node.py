"""
Drive full laps of the 160 cm field in the SIMULATION: pure pursuit on the EKF pose.

    ros2 run wro2026_sim_r550 lap_node --ros-args -p use_sim_time:=true -p laps:=1 -p out:=DIR

Path (WRO style, counter-clockwise): a STRAIGHT along every island side and a 90 deg TURN
in every corner section, nothing else. Straights on x, y = +-0.55 m (35 cm from the island,
25 cm from the outer wall), 0.26 m long; corners are arcs of `corner_radius` 0.42 m (car
minimum 0.35 m: 20 % steering margin) that start 7 cm before the island corner line.
0.385 m corners (turning exactly at the island corner) needed full lock all the time and the
steering lag threw the car into the outer wall in the sim (2026-10-24).
(2026-10-24, Ray: no circle-like line; clear straights and corners like WRO.)
Path options (2026-10-27 campaign, defaults = the path above): `transition` clothoid length
at both ends of every corner (curvature ramps instead of jumping), `inset` turns every side
line so the car leaves each corner closer to the island (against the front overhang swinging
out toward the next wall), `obstacles` [x1, y1, x2, y2, ...] moves the line of a side with an
obstacle to `pass_inside` / `pass_outside` (obstacle on the outer / inner half of the lane).
Pose: /odometry/filtered (odom frame) + the known start pose (start_x/y/yaw = the spawn pose
of sim.launch.py) -> field frame. Same idea works on the real car if it starts at a known spot.
Control (`controller` feedforward, default): steer with the PATH curvature (0 on the
straights, 1/R in the corners, read speed x 0.35 s ahead for the steering lag) plus a small
correction for the lateral and heading error, so the wheels stay straight on the straights
and turn exactly at the entry point. (`controller` pure_pursuit, lookahead 0.30 m, starts
the turn 0.3 m early and rounds the corners off.) Curvature limited to the minimum radius.
More controllers (campaign): lag_ff = feedforward that models the steering lag (`steer_tau`,
`steer_delay`) and leads it, stanley (front axle on the path), rpp (regulated pure pursuit
like Nav2: speed-scaled lookahead, speed regulated by the path curvature).
Speed: `speed` 0.30 m/s on the straights, `corner_speed` 0.15 m/s from `brake_dist` before a
corner to its end, ramped with `accel` 0.4 / `decel` 0.8 m/s^2: brake, turn, speed up.
/cmd_vel (Twist, 20 Hz) goes through the real CommandGate (sim_base_node) like on the car;
zeros for 1 s at the end.
Checks against /sim/ground_truth (not used for control by default): cross-track error
(counted from the moment the car first reaches the line: the spawn pose is in the lane
corner, off the rounded path), wall clearance of the measured footprint, EKF vs truth,
lap time, true speed on the straights vs in the corners, CPU time of this node.
`pose_source` truth = DIAGNOSIS ONLY (2026-10-27 campaign): the controller reads the TRUE
pose from /sim/ground_truth to separate localization error from steering / control error.
That topic only exists in the simulation; never use it for anything that drives a real car.
Writes DIR/lap.txt (PASS / FAIL per check), DIR/lap.csv and DIR/path.csv.
SIMULATION ONLY: refuses to run if a real chassis node is on the ROS graph.
"""
import math
import os
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

HALF_SIDE = 0.55           # straight sides of the path, m from the field centre
CORNER_RADIUS = 0.42       # path corners (car minimum 0.35 m: 20 % steering margin)
WALL = 0.8                 # inner face of the outer walls
ISLAND = 0.2               # half size of the central island
WHEELBASE = 0.144          # m, firmware Akm_axlespacing (stanley steers the front axle)
# measured footprint around base_link (rear axle centre): docs/real/R550_HARDWARE.md
FOOTPRINT = ((-0.031, -0.095), (-0.031, 0.095), (0.24, 0.095), (0.24, -0.095))
MAX_XTE = 0.10             # m, PASS limit for the cross-track error of the true pose
MIN_CLEARANCE = 0.03       # m, PASS limit footprint <-> any wall
ON_LINE = 0.03             # m, the car has reached the path once its cross-track error is below
OBSTACLE_HALF = 0.025      # m, half size of the sim obstacles (sim.launch.py obstacle_size)
LANE_MIDDLE = (WALL + ISLAND) / 2.0
# travel direction of the sides, counter-clockwise: south, east, north, west
SIDE_DIRECTIONS = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))
CONTROLLERS = ('feedforward', 'pure_pursuit', 'lag_ff', 'stanley', 'rpp')
# LapController settings = ROS parameters of the same name, with their defaults
SETTINGS = {
    'speed': 0.30, 'corner_speed': 0.15, 'accel': 0.4, 'decel': 0.8,
    'brake_dist': 0.15,                 # m before a corner; CommandGate brakes 0.5 m/s^2
    'lookahead': 0.30,                  # pure_pursuit
    'min_radius': 0.35,                 # m, the car's minimum turn radius
    'k_y': 16.0,                        # 1/m^2, lateral error gain (feedforward, lag_ff)
    'k_h': 6.4,                         # 1/m, heading error gain (feedforward, lag_ff)
    'preview_time': 0.35,               # s, feedforward: path curvature read ahead
    'k_s': 2.0,                         # 1/s, stanley cross-track gain
    'la_time': 1.2, 'la_min': 0.25, 'la_max': 0.50,              # rpp lookahead (s, m, m)
    'reg_radius': 0.9, 'reg_min_speed': 0.10, 'reg_dist': 0.4,   # rpp speed regulation
    'steer_tau': 1.0,                   # s, lag_ff: steering time constant (model)
    'steer_delay': 0.05,                # s, lag_ff: dead time until a command acts
    'response_time': 0.3,               # s, lag_ff: wanted steering response time
}


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def corner_shape(radius, transition, turn, step=0.01, ds=0.0005):
    """
    Return (points, tangent) of a left turn by `turn` rad from (0, 0), heading +x.

    Plain arc (transition 0), or clothoid (curvature 0 -> 1 / radius over `transition` m),
    arc, clothoid back to 0. points: one every ~`step` m, the end point excluded. tangent:
    distance from the start to the vertex where the straight lines before and after meet
    (= radius for a plain 90 deg arc).
    """
    if transition <= 0.0:
        n = max(1, int(round(radius * turn / step)))
        a0 = -math.pi / 2
        return ([(radius * math.cos(a0 + turn * i / n),
                  radius + radius * math.sin(a0 + turn * i / n)) for i in range(n)],
                radius * math.tan(turn / 2))
    arc = radius * turn - transition      # the two clothoids turn transition / radius together
    if arc < 0.0:
        raise ValueError('transition %.3f m too long for a %.0f deg turn of radius %.3f m'
                         % (transition, math.degrees(turn), radius))
    total = 2.0 * transition + arc

    def theta(s):
        if s < transition:
            return s * s / (2.0 * transition * radius)
        if s < transition + arc:
            return transition / (2.0 * radius) + (s - transition) / radius
        u = total - s
        return turn - u * u / (2.0 * transition * radius)

    n = max(1, int(round(total / step)))
    m = max(1, int(math.ceil(total / n / ds)))      # integration sub-steps per sample
    h = total / n / m
    pts, x, y = [], 0.0, 0.0
    for i in range(n):
        pts.append((x, y))
        s0 = total * i / n
        for j in range(m):
            a = theta(s0 + (j + 0.5) * h)
            x += h * math.cos(a)
            y += h * math.sin(a)
    return pts, x - y / math.tan(turn)


def side_lines(offsets, inset=0.0):
    """
    Return [(point, unit direction)] of the 4 side lines (south, east, north, west).

    A side line runs at offsets[k] from the field centre; with `inset` it is turned so it is
    offset - inset where the side starts (island corner) and offset where it ends.
    """
    lines = []
    tilt = inset / (2.0 * ISLAND)
    for (dx, dy), off in zip(SIDE_DIRECTIONS, offsets):
        nx, ny = dy, -dx                                  # outward normal (right of travel)
        ux, uy = dx + tilt * nx, dy + tilt * ny
        norm = math.hypot(ux, uy)
        lines.append(((nx * (off - inset / 2.0), ny * (off - inset / 2.0)),
                      (ux / norm, uy / norm)))
    return lines


def make_path(half_side=HALF_SIDE, radius=CORNER_RADIUS, step=0.01, transition=0.0,
              offsets=None, inset=0.0, straights=None, ends=None):
    """
    Return the closed counter-clockwise path, from the west end of the south side.

    WRO style: a straight along every island side and a left turn in every corner section,
    one point every ~`step` m. offsets: distance of the south, east, north, west side lines
    from the field centre (default all half_side); transition, inset: see the module text.
    straights / ends: lists that get, per side, the straight's length (m) / its
    ((x0, y0), (x1, y1)) end points appended.
    Raises ValueError when two corners would overlap (no straight left on a side).
    """
    offs = [half_side] * 4 if offsets is None else [float(o) for o in offsets]
    if len(offs) != 4:
        raise ValueError('offsets: south, east, north, west')
    lines = side_lines(offs, inset)

    def meet(a, b):
        (p, u), (q, v) = a, b
        den = u[0] * v[1] - u[1] * v[0]
        t = ((q[0] - p[0]) * v[1] - (q[1] - p[1]) * v[0]) / den
        return p[0] + t * u[0], p[1] + t * u[1]

    corners = []                                     # (start, end, points) at each vertex
    for k in range(4):
        (_, u), (_, v) = lines[k], lines[(k + 1) % 4]
        vx, vy = meet(lines[k], lines[(k + 1) % 4])
        a_in = math.atan2(u[1], u[0])
        turn = wrap(math.atan2(v[1], v[0]) - a_in)
        local, tangent = corner_shape(radius, transition, turn, step)
        sx, sy = vx - tangent * u[0], vy - tangent * u[1]
        c, s = math.cos(a_in), math.sin(a_in)
        corners.append(((sx, sy), (vx + tangent * v[0], vy + tangent * v[1]),
                        [(sx + c * px - s * py, sy + s * px + c * py) for px, py in local]))
    pts = []
    for k in range(4):
        (x0, y0), (x1, y1) = corners[k - 1][1], corners[k][0]
        u = lines[k][1]
        length = (x1 - x0) * u[0] + (y1 - y0) * u[1]
        if length < -1e-9:
            raise ValueError('corners overlap on side %d (straight %.3f m)' % (k, length))
        if straights is not None:
            straights.append(max(0.0, length))
        if ends is not None:
            ends.append(((x0, y0), (x1, y1)))
        n = int(round(math.hypot(x1 - x0, y1 - y0) / step))
        pts.extend((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n))
        pts.extend(corners[k][2])
    return pts


def side_of(x, y):
    """Return (side index south, east, north, west; distance from the field centre line)."""
    if abs(y) >= abs(x):
        return (0 if y < 0 else 2), abs(y)
    return (1 if x > 0 else 3), abs(x)


def plan_offsets(obstacles, default, pass_inside, pass_outside):
    """
    Return the line offset of every side: pass obstacles on the other half of the lane.

    obstacles: [(x, y), ...] in the field frame. An obstacle on the inner half of a side's
    lane (closer to the island) -> that side's line moves out to pass_outside, on the outer
    half -> in to pass_inside. Sides without obstacles keep `default`.
    """
    offs = [default] * 4
    for x, y in obstacles:
        side, dist = side_of(x, y)
        offs[side] = pass_outside if dist < LANE_MIDDLE else pass_inside
    return offs


def curvature(path, k=3):
    """Return 1 / radius (1/m) at every point of the closed path, from points i-k, i, i+k."""
    n = len(path)
    out = []
    for i in range(n):
        a, b, c = path[i - k], path[i], path[(i + k) % n]
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        d = math.dist(a, b) * math.dist(b, c) * math.dist(a, c)
        out.append(2.0 * area2 / d if d > 1e-12 else 0.0)
    return out


def nearest(path, x, y, start=None, window=60):
    """Return the index of the path point closest to x, y (only `window` points after start)."""
    n = len(path)
    idx = range(n) if start is None else ((start + k) % n for k in range(window))
    return min(idx, key=lambda i: (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2)


def pure_pursuit(pose, path, i_near, lookahead, speed, min_radius):
    """Return (wz, target index) for pose (x, y, yaw) following the closed path."""
    x, y, yaw = pose
    n = len(path)
    j = i_near
    for _ in range(n):
        if math.hypot(path[j][0] - x, path[j][1] - y) >= lookahead:
            break
        j = (j + 1) % n
    dx, dy = path[j][0] - x, path[j][1] - y
    alpha = wrap(math.atan2(dy, dx) - yaw)
    curvature = 2.0 * math.sin(alpha) / max(math.hypot(dx, dy), 1e-6)
    limit = 1.0 / min_radius
    return speed * max(-limit, min(limit, curvature)), j


def path_errors(path, i, x, y, yaw):
    """Return (lateral error, + = left of the path; heading error car - path) at index i."""
    n = len(path)
    (px, py), (qx, qy) = path[i], path[(i + 1) % n]
    heading = math.atan2(qy - py, qx - px)
    lateral = -math.sin(heading) * (x - px) + math.cos(heading) * (y - py)
    return lateral, wrap(yaw - heading)


def feedforward(pose, path, curv, i, speed, min_radius, preview=7, k_y=16.0, k_h=6.4):
    """Return wz: path curvature `preview` points ahead + lateral / heading correction."""
    x, y, yaw = pose
    n = len(path)
    lateral, heading_error = path_errors(path, i, x, y, yaw)
    k = curv[(i + preview) % n] - k_y * lateral - k_h * math.sin(heading_error)
    limit = 1.0 / min_radius
    return speed * max(-limit, min(limit, k))


def stanley(pose, path, curv, i_front, speed, min_radius, k_s=2.0, soft=0.05,
            wheelbase=WHEELBASE):
    """
    Return wz: Stanley steering of the FRONT axle.

    Heading error + atan(k e / v) + the path curvature as feedforward; `i_front` = path index
    nearest the front axle.
    """
    x, y, yaw = pose
    fx, fy = x + wheelbase * math.cos(yaw), y + wheelbase * math.sin(yaw)
    lateral, heading_error = path_errors(path, i_front, fx, fy, yaw)
    delta = (-heading_error - math.atan2(k_s * lateral, speed + soft)
             + math.atan(wheelbase * curv[i_front]))
    delta = max(-1.2, min(1.2, delta))
    limit = 1.0 / min_radius
    return speed * max(-limit, min(limit, math.tan(delta) / wheelbase))


def clearance(pose):
    """Smallest distance (m) from the footprint corners to the outer walls and the island."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    best = float('inf')
    for fx, fy in FOOTPRINT:
        px, py = x + c * fx - s * fy, y + s * fx + c * fy
        outer = min(WALL - abs(px), WALL - abs(py))
        ox, oy = max(abs(px) - ISLAND, 0.0), max(abs(py) - ISLAND, 0.0)
        island = math.hypot(ox, oy) if (ox or oy) else -min(ISLAND - abs(px), ISLAND - abs(py))
        best = min(best, outer, island)
    return best


def footprint_corners(pose):
    """Return the 4 footprint corners (field frame) of the car at pose (x, y, yaw)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * fx - s * fy, y + s * fx + c * fy) for fx, fy in FOOTPRINT]


def outer_clearance(pose):
    """Return the smallest distance (m) from the footprint to the OUTER walls (< 0: through)."""
    return min(WALL - max(abs(px), abs(py)) for px, py in footprint_corners(pose))


def box_clearance(pose, cx, cy, hx, hy):
    """
    Return the distance (m) between the footprint and an axis-aligned box.

    Box centre cx, cy, half sizes hx, hy; 0 when they touch or overlap (a corner of either
    shape inside the other). Exact for two rectangles of these sizes.
    """
    car = footprint_corners(pose)
    box = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]

    def seg_dist(p, a, b):
        ax, ay = b[0] - a[0], b[1] - a[1]
        t = ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / max(ax * ax + ay * ay, 1e-12)
        t = max(0.0, min(1.0, t))
        return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)

    def inside(p, poly):
        sign = 0
        for k in range(4):
            a, b = poly[k], poly[(k + 1) % 4]
            cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
            if cross != 0:
                if sign and (cross > 0) != (sign > 0):
                    return False
                sign = cross
        return True

    if any(inside(p, box) for p in car) or any(inside(p, car) for p in box):
        return 0.0
    best = float('inf')
    for poly_a, poly_b in ((car, box), (box, car)):
        for p in poly_a:
            for k in range(4):
                best = min(best, seg_dist(p, poly_b[k], poly_b[(k + 1) % 4]))
    return best


def island_clearance(pose):
    """Return the distance (m) between the footprint and the central island (0 = contact)."""
    return box_clearance(pose, 0.0, 0.0, ISLAND, ISLAND)


def obstacle_clearance(pose, obstacles, half=OBSTACLE_HALF):
    """Return the smallest distance (m) from the footprint to the obstacles (inf if none)."""
    return min((box_clearance(pose, x, y, half, half) for x, y in obstacles),
               default=float('inf'))


def lateral_error(path, i, x, y):
    """Return the signed distance (m) of x, y from the path at index i: + = left (island)."""
    n = len(path)
    (px, py), (qx, qy) = path[i], path[(i + 1) % n]
    heading = math.atan2(qy - py, qx - px)
    return -math.sin(heading) * (x - px) + math.cos(heading) * (y - py)


class LapController:
    """
    One control tick without ROS: speed schedule + steering law.

    Used by the Lap node and by the offline simulator tools/analysis/lap_sim.py, so both run
    the same code.
    """

    def __init__(self, path, controller='feedforward', dt=0.05, **settings):
        if controller not in CONTROLLERS:
            raise ValueError('controller must be one of %s' % ', '.join(CONTROLLERS))
        unknown = set(settings) - set(SETTINGS)
        if unknown:
            raise ValueError('unknown settings: %s' % ', '.join(sorted(unknown)))
        for name, value in dict(SETTINGS, **settings).items():
            setattr(self, name, float(value))
        self.path, self.dt, self.controller = path, dt, controller
        self.curv = curvature(path)                          # left turns: all >= 0
        self.corner = [k > 1.25 for k in self.curv]          # radius < 0.8 m
        self.v = 0.0
        self.i_near = self.i_front = None
        self.progress = 0          # path points passed since the start
        self.k_est = 0.0           # lag_ff: modelled curvature the car drives now
        self.queue = [0.0] * max(0, int(round(self.steer_delay / dt)))   # lag_ff: not acting yet

    def start(self, pose):
        """Set the path indices from the first pose."""
        x, y, yaw = pose
        self.i_near = nearest(self.path, x, y)
        self.i_front = nearest(self.path, x + WHEELBASE * math.cos(yaw),
                               y + WHEELBASE * math.sin(yaw))

    def step(self, pose):
        """Return (vx, wz) for this tick (pose = x, y, yaw in the field frame)."""
        n = len(self.path)
        i = nearest(self.path, pose[0], pose[1], self.i_near)
        self.progress += (i - self.i_near) % n
        self.i_near = i
        if self.controller == 'rpp':      # Nav2 RPP: speed regulated by the curvature ahead
            k_ahead = self.curv[(i + int(round(self.reg_dist / 0.01))) % n]
            target = self.speed
            if k_ahead > 1.0 / self.reg_radius:
                target = max(self.reg_min_speed, self.speed / (k_ahead * self.reg_radius))
        else:
            ahead = any(self.corner[(i + k) % n]
                        for k in range(int(self.brake_dist / 0.01) + 1))
            target = self.corner_speed if ahead else self.speed
        dv = (self.accel if target > self.v else self.decel) * self.dt
        self.v = min(target, self.v + dv) if target > self.v else max(target, self.v - dv)
        if self.controller == 'pure_pursuit':
            wz, _ = pure_pursuit(pose, self.path, i, self.lookahead, self.v, self.min_radius)
        elif self.controller == 'rpp':
            la = max(self.la_min, min(self.la_max, self.v * self.la_time))
            wz, _ = pure_pursuit(pose, self.path, i, la, self.v, self.min_radius)
        elif self.controller == 'stanley':
            x, y, yaw = pose
            self.i_front = nearest(self.path, x + WHEELBASE * math.cos(yaw),
                                   y + WHEELBASE * math.sin(yaw), self.i_front)
            wz = stanley(pose, self.path, self.curv, self.i_front, self.v, self.min_radius,
                         self.k_s)
        elif self.controller == 'lag_ff':
            wz = self._lag_ff(pose, i)
        else:
            preview = max(1, round(self.v * self.preview_time / 0.01))
            wz = feedforward(pose, self.path, self.curv, i, self.v, self.min_radius, preview,
                             self.k_y, self.k_h)
        return self.v, wz

    def _lag_ff(self, pose, i):
        """
        Return wz from the feedforward with a model of the steering.

        The model: first-order lag steer_tau after a dead time steer_delay. Predicts the pose
        over the dead time, then leads the lag so the modelled curvature reaches the wanted one
        within response_time.
        """
        n = len(self.path)
        limit = 1.0 / self.min_radius
        a = 1.0 - math.exp(-self.dt / max(self.steer_tau, 1e-3))
        x, y, yaw = pose
        k = self.k_est
        for k_cmd in self.queue:               # commands already sent, not acting yet
            yaw_mid = yaw + 0.5 * self.v * k * self.dt
            x += self.v * self.dt * math.cos(yaw_mid)
            y += self.v * self.dt * math.sin(yaw_mid)
            yaw += self.v * k * self.dt
            k += a * (k_cmd - k)
        j = nearest(self.path, x, y, i)
        lateral, heading_error = path_errors(self.path, j, x, y, yaw)
        preview = max(1, round(self.v * self.response_time / 0.01))
        want = (self.curv[(j + preview) % n] - self.k_y * lateral
                - self.k_h * math.sin(heading_error))
        if self.steer_tau > 0.0 and self.response_time > 0.0:
            k_cmd = k + (want - k) * min(self.steer_tau / self.response_time, 20.0)
        else:
            k_cmd = want
        k_cmd = max(-limit, min(limit, k_cmd))
        self.queue.append(k_cmd)
        self.k_est += a * (self.queue.pop(0) - self.k_est)
        return self.v * k_cmd


class Lap(Node):
    def __init__(self):
        super().__init__('r550_sim_lap')
        p = self.declare_parameter
        self.out_dir = p('out', '.').value
        self.laps = int(p('laps', 1).value)
        self.pose_source = p('pose_source', 'ekf').value      # truth: DIAGNOSIS ONLY
        if self.pose_source not in ('ekf', 'truth'):
            raise ValueError('pose_source must be ekf or truth')
        self.start_pose = (float(p('start_x', -0.60).value), float(p('start_y', -0.55).value),
                           float(p('start_yaw', 0.0).value))
        flat = [float(v) for v in p('obstacles', [0.0]).value]      # [x1, y1, x2, y2, ...]
        self.obstacles = list(zip(flat[0::2], flat[1::2])) if len(flat) >= 2 else []
        half_side = float(p('half_side', HALF_SIDE).value)
        offsets = plan_offsets(self.obstacles, half_side, float(p('pass_inside', 0.38).value),
                               float(p('pass_outside', 0.60).value))
        self.straights = []
        self.path = make_path(half_side=half_side,
                              radius=float(p('corner_radius', CORNER_RADIUS).value),
                              transition=float(p('transition', 0.0).value),
                              offsets=offsets, inset=float(p('inset', 0.0).value),
                              straights=self.straights)
        settings = {name: float(p(name, value).value) for name, value in SETTINGS.items()}
        self.controller = p('controller', 'feedforward').value
        self.ctl = LapController(self.path, self.controller, **settings)
        self.speed, self.corner_speed = settings['speed'], settings['corner_speed']
        self.curv, self.corner = self.ctl.curv, self.ctl.corner
        closed = self.path[1:] + self.path[:1]
        self.length = sum(math.dist(a, b) for a, b in zip(self.path, closed))
        os.makedirs(self.out_dir, exist_ok=True)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.last = {}
        for name, topic in (('truth', 'sim/ground_truth'), ('ekf', 'odometry/filtered')):
            self.create_subscription(Odometry, topic,
                                     lambda m, n=name: self.last.__setitem__(n, m), 10)
        self.t0 = self.t_end = self.t_line = None
        self.i_truth = None
        self.rows = []
        self.done = False
        self.cpu0 = self.wall0 = None
        with open(os.path.join(self.out_dir, 'path.csv'), 'w') as f:
            f.write('x,y,curvature,corner\n')
            for (x, y), k, c in zip(self.path, self.curv, self.corner):
                f.write('%.5f,%.5f,%.4f,%d\n' % (x, y, k, c))
        self.create_timer(0.05, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self, name):
        m = self.last[name]
        q = m.pose.pose
        return q.position.x, q.position.y, yaw_of(q.orientation)

    def _stamp(self, name):
        s = self.last[name].header.stamp
        return s.sec + s.nanosec * 1e-9

    def _control_pose(self):
        """Return the pose the controller uses: the EKF, or the TRUE pose (diagnosis only)."""
        return self._pose('truth') if self.pose_source == 'truth' else self._ekf_field()

    def _ekf_field(self):
        """EKF pose (odom frame, starts at 0) -> field frame via the known start pose."""
        x, y, yaw = self._pose('ekf')
        x0, y0, a0 = self.start_pose
        return (x0 + math.cos(a0) * x - math.sin(a0) * y,
                y0 + math.sin(a0) * x + math.cos(a0) * y, wrap(a0 + yaw))

    def _send(self, vx, wz):
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        self.pub.publish(msg)

    def _tick(self):
        tick_start = time.perf_counter()
        now = self._now()
        if now <= 0.0 or self.done:
            return
        if self.t0 is None:
            if any(n.startswith('r550_base') or n == 'r550_feedback'
                   for n in self.get_node_names()):
                self.get_logger().error('a REAL chassis node is running: refusing to drive')
                self.done = True
                return
            if not all(k in self.last for k in ('truth', 'ekf')):
                return
            self.t0 = now
            self.cpu0, self.wall0 = time.process_time(), time.monotonic()
            pose = self._control_pose()
            self.ctl.start(pose)
            self.i_truth = self.ctl.i_near
            self.get_logger().info('lap: %d x %.2f m, %.2f m/s straights, %.2f m/s corners, '
                                   'start index %d, %s on the %s pose'
                                   % (self.laps, self.length, self.speed, self.corner_speed,
                                      self.ctl.i_near, self.controller, self.pose_source))
        t = now - self.t0
        if self.t_end is not None:             # 1 s of zeros, then report
            self._send(0.0, 0.0)
            if t - self.t_end > 1.0:
                self.done = True
                self._report(t)
            return
        pose = self._control_pose()
        n = len(self.path)
        v, wz = self.ctl.step(pose)
        timeout = self.laps * self.length / self.corner_speed * 2.0 + 20.0
        if self.ctl.progress >= self.laps * n or t > timeout:
            self.t_end = t
            self._send(0.0, 0.0)
            return
        self._send(v, wz)
        truth = self._pose('truth')
        tw = self.last['truth'].twist.twist
        self.i_truth = nearest(self.path, truth[0], truth[1], self.i_truth)
        xte = math.dist(self.path[self.i_truth], truth[:2])
        if self.t_line is None and xte < ON_LINE:
            self.t_line = t
        self.rows.append((t, *truth, *self._ekf_field(), xte, clearance(truth), v, wz,
                          math.hypot(tw.linear.x, tw.linear.y),
                          float(self.corner[self.i_truth]),
                          lateral_error(self.path, self.i_truth, truth[0], truth[1]),
                          outer_clearance(truth), island_clearance(truth),
                          self._stamp('truth'), self._stamp('ekf'), tw.angular.z,
                          min(obstacle_clearance(truth, self.obstacles), 9.9999),
                          (time.perf_counter() - tick_start) * 1e3))

    def _report(self, t):
        lines = []
        done = self.ctl.progress >= self.laps * len(self.path)
        lap_time = self.t_end
        lines.append('laps: %d x %.2f m, finished %s in %.1f s (%.3f m/s)  %s'
                     % (self.laps, self.length, 'yes' if done else 'NO (timeout)', lap_time,
                        self.laps * self.length / max(lap_time, 1e-6), 'PASS' if done else 'FAIL'))
        lines.append('path: straights %s m (total %.2f m per lap), %d obstacles  INFO'
                     % (' / '.join('%.2f' % s for s in self.straights), sum(self.straights),
                        len(self.obstacles)))
        if self.t_line is None:
            lines.append('true pose never got within %.2f m of the lane centre  FAIL' % ON_LINE)
        else:
            xtes = [r[7] for r in self.rows if r[0] >= self.t_line]
            rms = math.sqrt(sum(e * e for e in xtes) / len(xtes))
            lines.append('true pose vs lane centre (from %.1f s, on the line): max %.3f m, '
                         'RMS %.3f m (limit %.2f)  %s'
                         % (self.t_line, max(xtes), rms, MAX_XTE,
                            'PASS' if max(xtes) < MAX_XTE else 'FAIL'))
        if self.rows:
            clear = min(r[8] for r in self.rows)
            lines.append('closest footprint corner to a wall: %.3f m (limit %.2f)  %s'
                         % (clear, MIN_CLEARANCE, 'PASS' if clear > MIN_CLEARANCE else 'FAIL'))
        if self.obstacles and self.rows:
            obst = min(r[19] for r in self.rows)
            lines.append('closest footprint point to an obstacle: %.3f m  %s'
                         % (obst, 'PASS' if obst > 0.0 else 'FAIL (contact)'))
        on_line = [r for r in self.rows if self.t_line is not None and r[0] >= self.t_line]
        st = [r[11] for r in on_line if not r[12]]
        co = [r[11] for r in on_line if r[12]]
        if st and co:
            lines.append('true speed: straights max %.3f mean %.3f m/s, corners mean %.3f m/s  '
                         '%s' % (max(st), sum(st) / len(st), sum(co) / len(co),
                                 'PASS' if max(st) > 1.5 * sum(co) / len(co) else 'FAIL'))
        if self.rows:
            lats = sorted(abs(r[13]) for r in on_line) or [0.0]
            ekf_err = sorted(math.dist(r[1:3], r[4:6]) for r in self.rows)
            lines.append('true pose lateral error: p95 %.3f m, max %.3f m; outer wall %.3f m, '
                         'island %.3f m; ekf position error p95 %.3f m  INFO'
                         % (lats[int(0.95 * (len(lats) - 1))], lats[-1],
                            min(r[14] for r in self.rows), min(r[15] for r in self.rows),
                            ekf_err[int(0.95 * (len(ekf_err) - 1))]))
            ticks = [r[20] for r in self.rows]
            cpu = (time.process_time() - self.cpu0) / max(time.monotonic() - self.wall0, 1e-6)
            lines.append('this node: CPU %.1f %% of one core, control tick mean %.2f ms, '
                         'max %.2f ms  INFO' % (100.0 * cpu, sum(ticks) / len(ticks), max(ticks)))
        truth, ekf = self._pose('truth'), self._ekf_field()
        lines.append('ekf vs truth at the end: position %.3f m, yaw %+.2f deg  INFO'
                     % (math.dist(truth[:2], ekf[:2]), math.degrees(wrap(ekf[2] - truth[2]))))
        text = '\n'.join(['== sim lap (%s on the %s pose%s) =='
                          % (self.controller, self.pose_source,
                             ', DIAGNOSIS ONLY' if self.pose_source == 'truth' else '')]
                         + lines)
        print(text, flush=True)
        with open(os.path.join(self.out_dir, 'lap.txt'), 'w') as f:
            f.write(text + '\n')
        with open(os.path.join(self.out_dir, 'lap.csv'), 'w') as f:
            f.write('t,truth_x,truth_y,truth_yaw,ekf_x,ekf_y,ekf_yaw,xte,clearance,vx,wz,'
                    'truth_v,corner,lat,outer,island,truth_stamp,ekf_stamp,truth_wz,obstacle,'
                    'tick_ms\n')
            for r in self.rows:
                f.write(','.join('%.4f' % v for v in r) + '\n')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = Lap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
