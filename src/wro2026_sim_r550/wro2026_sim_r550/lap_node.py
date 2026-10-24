"""
Drive full laps of the 160 cm field in the SIMULATION: pure pursuit on the EKF pose.

    ros2 run wro2026_sim_r550 lap_node --ros-args -p use_sim_time:=true -p laps:=1 -p out:=DIR

Path (WRO style, counter-clockwise): a STRAIGHT along every island side and a 90 deg TURN
in every corner section, nothing else. Straights on x, y = +-0.55 m (35 cm from the island,
25 cm from the outer wall), 0.26 m long; corners are arcs of `corner_radius` 0.42 m (car
minimum 0.35 m: 20 % steering margin) that start 7 cm before the island corner line. 0.385 m corners (turning
exactly at the island corner) needed full lock all the time and the steering lag threw the
car into the outer wall in the sim (2026-10-24).
(2026-10-24, Ray: no circle-like line; clear straights and corners like WRO.)
Pose: /odometry/filtered (odom frame) + the known start pose (start_x/y/yaw = the spawn pose
of sim.launch.py) -> field frame. Same idea works on the real car if it starts at a known spot.
Control (`controller` feedforward, default): steer with the PATH curvature (0 on the
straights, 1/R in the corners, read speed x 0.35 s ahead for the steering lag) plus a small
correction for the lateral and heading error, so the wheels stay straight on the straights
and turn exactly at the entry point. (`controller` pure_pursuit, lookahead 0.30 m, starts
the turn 0.3 m early and rounds the corners off.) Curvature limited to the minimum radius.
Speed: `speed` 0.30 m/s on the straights, `corner_speed` 0.15 m/s from `brake_dist` before a
corner to its end, ramped with `accel` 0.4 / `decel` 0.8 m/s^2: brake, turn, speed up.
/cmd_vel (Twist, 20 Hz) goes through the real CommandGate (sim_base_node) like on the car;
zeros for 1 s at the end.
Checks against /sim/ground_truth (NEVER used for control): cross-track error (counted from
the moment the car first reaches the line: the spawn pose is in the lane corner, off the
rounded path), wall clearance of the measured footprint, EKF vs truth at the end, lap time,
true speed on the straights vs in the corners.
Writes DIR/lap.txt (PASS / FAIL per check) and DIR/lap.csv.
SIMULATION ONLY: refuses to run if a real chassis node is on the ROS graph.
"""
import math
import os
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

HALF_SIDE = 0.55           # straight sides of the path, m from the field centre
CORNER_RADIUS = 0.42       # path corners (car minimum 0.35 m: 20 % steering margin)
WALL = 0.8                 # inner face of the outer walls
ISLAND = 0.2               # half size of the central island
# measured footprint around base_link (rear axle centre): docs/real/R550_HARDWARE.md
FOOTPRINT = ((-0.031, -0.095), (-0.031, 0.095), (0.24, 0.095), (0.24, -0.095))
MAX_XTE = 0.10             # m, PASS limit for the cross-track error of the true pose
MIN_CLEARANCE = 0.03       # m, PASS limit footprint <-> any wall
ON_LINE = 0.03             # m, the car has reached the path once its cross-track error is below


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def make_path(half_side=HALF_SIDE, radius=CORNER_RADIUS, step=0.01):
    """Return the closed counter-clockwise rounded square, from the west end of the south side."""
    c = half_side - radius
    pts = []

    def line(x0, y0, x1, y1):
        n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
        pts.extend((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n))

    def arc(cx, cy, a0):
        n = max(1, int(round(radius * math.pi / 2 / step)))
        pts.extend((cx + radius * math.cos(a0 + math.pi / 2 * i / n),
                    cy + radius * math.sin(a0 + math.pi / 2 * i / n)) for i in range(n))

    line(-c, -half_side, c, -half_side)
    arc(c, -c, -math.pi / 2)
    line(half_side, -c, half_side, c)
    arc(c, c, 0.0)
    line(c, half_side, -c, half_side)
    arc(-c, c, math.pi / 2)
    line(-half_side, c, -half_side, -c)
    arc(-c, -c, math.pi)
    return pts


def curvature(path, k=3):
    """Return the curvature (1/m) at every point of the closed path, from points i-k, i, i+k."""
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


def feedforward(pose, path, curv, i, speed, min_radius, preview=7, k_y=16.0, k_h=6.4):
    """Return wz: path curvature `preview` points ahead + lateral / heading correction."""
    x, y, yaw = pose
    n = len(path)
    (px, py), (qx, qy) = path[i], path[(i + 1) % n]
    heading = math.atan2(qy - py, qx - px)
    lateral = -math.sin(heading) * (x - px) + math.cos(heading) * (y - py)   # + = left of path
    k = curv[(i + preview) % n] - k_y * lateral - k_h * math.sin(wrap(yaw - heading))
    limit = 1.0 / min_radius
    return speed * max(-limit, min(limit, k))


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


class Lap(Node):
    def __init__(self):
        super().__init__('r550_sim_lap')
        p = self.declare_parameter
        self.out_dir = p('out', '.').value
        self.laps = int(p('laps', 1).value)
        self.speed = float(p('speed', 0.30).value)
        self.corner_speed = float(p('corner_speed', 0.15).value)
        self.accel = float(p('accel', 0.4).value)
        self.decel = float(p('decel', 0.8).value)
        self.brake_dist = float(p('brake_dist', 0.15).value)   # CommandGate brakes 0.5 m/s^2
        self.lookahead = float(p('lookahead', 0.30).value)
        self.min_radius = float(p('min_radius', 0.35).value)
        self.controller = p('controller', 'feedforward').value
        self.start_pose = (float(p('start_x', -0.60).value), float(p('start_y', -0.55).value),
                           float(p('start_yaw', 0.0).value))
        self.path = make_path(half_side=float(p('half_side', HALF_SIDE).value),
                              radius=float(p('corner_radius', CORNER_RADIUS).value))
        self.curv = curvature(self.path)                         # left turns: all >= 0
        self.corner = [k > 1.25 for k in self.curv]              # radius < 0.8 m
        self.v = 0.0
        closed = self.path[1:] + self.path[:1]
        self.length = sum(math.dist(a, b) for a, b in zip(self.path, closed))
        os.makedirs(self.out_dir, exist_ok=True)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.last = {}
        for name, topic in (('truth', 'sim/ground_truth'), ('ekf', 'odometry/filtered')):
            self.create_subscription(Odometry, topic,
                                     lambda m, n=name: self.last.__setitem__(n, m), 10)
        self.t0 = self.t_end = self.t_line = None
        self.i_near = self.i_truth = None
        self.progress = 0          # path points passed since the start
        self.rows = []
        self.done = False
        self.create_timer(0.05, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self, name):
        m = self.last[name]
        q = m.pose.pose
        return q.position.x, q.position.y, yaw_of(q.orientation)

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
            pose = self._ekf_field()
            self.i_near = nearest(self.path, pose[0], pose[1])
            self.i_truth = self.i_near
            self.get_logger().info('lap: %d x %.2f m, %.2f m/s straights, %.2f m/s corners, '
                                   'start index %d' % (self.laps, self.length, self.speed,
                                                       self.corner_speed, self.i_near))
        t = now - self.t0
        if self.t_end is not None:             # 1 s of zeros, then report
            self._send(0.0, 0.0)
            if t - self.t_end > 1.0:
                self.done = True
                self._report(t)
            return
        pose = self._ekf_field()
        n = len(self.path)
        i = nearest(self.path, pose[0], pose[1], self.i_near)
        self.progress += (i - self.i_near) % n
        self.i_near = i
        ahead = any(self.corner[(i + k) % n] for k in range(int(self.brake_dist / 0.01) + 1))
        target = self.corner_speed if ahead else self.speed
        dv = (self.accel if target > self.v else self.decel) * 0.05
        self.v = min(target, self.v + dv) if target > self.v else max(target, self.v - dv)
        if self.controller == 'pure_pursuit':
            wz, _ = pure_pursuit(pose, self.path, i, self.lookahead, self.v, self.min_radius)
        else:
            preview = max(3, round(self.v * 0.35 / 0.01))       # steering lag ~0.35 s
            wz = feedforward(pose, self.path, self.curv, i, self.v, self.min_radius, preview)
        timeout = self.laps * self.length / self.corner_speed * 2.0 + 20.0
        if self.progress >= self.laps * n or t > timeout:
            self.t_end = t
            self._send(0.0, 0.0)
            return
        self._send(self.v, wz)
        truth = self._pose('truth')
        tw = self.last['truth'].twist.twist.linear
        self.i_truth = nearest(self.path, truth[0], truth[1], self.i_truth)
        xte = math.dist(self.path[self.i_truth], truth[:2])
        if self.t_line is None and xte < ON_LINE:
            self.t_line = t
        self.rows.append((t, *truth, *pose, xte, clearance(truth), self.v, wz,
                          math.hypot(tw.x, tw.y), float(self.corner[self.i_truth])))

    def _report(self, t):
        lines = []
        done = self.progress >= self.laps * len(self.path)
        lap_time = self.t_end
        lines.append('laps: %d x %.2f m, finished %s in %.1f s (%.3f m/s)  %s'
                     % (self.laps, self.length, 'yes' if done else 'NO (timeout)', lap_time,
                        self.laps * self.length / max(lap_time, 1e-6), 'PASS' if done else 'FAIL'))
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
        on_line = [r for r in self.rows if self.t_line is not None and r[0] >= self.t_line]
        st = [r[11] for r in on_line if not r[12]]
        co = [r[11] for r in on_line if r[12]]
        if st and co:
            lines.append('true speed: straights max %.3f mean %.3f m/s, corners mean %.3f m/s  '
                         '%s' % (max(st), sum(st) / len(st), sum(co) / len(co),
                                 'PASS' if max(st) > 1.5 * sum(co) / len(co) else 'FAIL'))
        truth, ekf = self._pose('truth'), self._ekf_field()
        lines.append('ekf vs truth at the end: position %.3f m, yaw %+.2f deg  INFO'
                     % (math.dist(truth[:2], ekf[:2]), math.degrees(wrap(ekf[2] - truth[2]))))
        text = '\n'.join(['== sim lap (pure pursuit on the EKF pose) =='] + lines)
        print(text, flush=True)
        with open(os.path.join(self.out_dir, 'lap.txt'), 'w') as f:
            f.write(text + '\n')
        with open(os.path.join(self.out_dir, 'lap.csv'), 'w') as f:
            f.write('t,truth_x,truth_y,truth_yaw,ekf_x,ekf_y,ekf_yaw,xte,clearance,vx,wz,'
                    'truth_v,corner\n')
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
