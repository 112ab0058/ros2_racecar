"""
Laps of the 160 cm field with Nav2 in the SIMULATION (NavigateThroughPoses), then checks.

    ros2 run wro2026_sim_r550 nav_lap_node --ros-args -p use_sim_time:=true -p laps:=1 -p out:=DIR

Needs sim.launch.py + wro2026_bringup nav2.launch.py (map maps/field_sim.yaml, field frame).
`goals` (what Nav2 gets):
  corners (default)  4 poses on the lane centre line after each corner, counter-clockwise,
      sent as two NavigateThroughPoses legs per lap (east + north, then west + south): a single
      leg ending at the south lane goal "reached" it right after the start, because the goal
      checker only looks at the distance to the last pose and the lap starts next to it.
  straights  8 poses: the start and the end of every straight of the reference path (below),
      so Smac drives straight along each island side and turns in the corner sections (with
      only 4 goals it planned the shortest way round, a near circle; 2026-10-24). Two legs per
      lap (the 6 goals from the south end to the west start, then the last 2).
  path  no planner: the reference path itself, two FollowPath halves per lap, so Nav2's RPP
      controller tracks exactly the line lap_node tracks (2026-10-27 campaign, stage D).
Reference path = lap_node's path with the same options (half_side, corner_radius, transition,
inset). Checks against /sim/ground_truth (NEVER sent to Nav2): result, time, cross-track
error to the reference path (from the moment the car is on it), footprint wall clearance,
turning on the spot (|wz| > 0.05 with |vx| < 0.01 in /cmd_vel_out), recoveries, and Nav2's
own pose (TF map -> base_link: AMCL + EKF) vs the truth.
Writes DIR/nav_lap.txt (PASS / FAIL per check) and DIR/nav_lap.csv.
SIMULATION ONLY: refuses to run if a real chassis node is on the ROS graph.
"""
import math
import os
import sys
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .lap_node import (clearance, CORNER_RADIUS, curvature, HALF_SIDE, island_clearance,
                       lateral_error, make_path, MAX_XTE, MIN_CLEARANCE, nearest, ON_LINE,
                       outer_clearance, yaw_of)

# lane centre, after each corner (field frame, counter-clockwise from the south lane)
CORNER_GOALS = [(0.50, -0.10, math.pi / 2), (0.10, 0.50, math.pi),
                (-0.50, 0.10, -math.pi / 2), (-0.10, -0.50, 0.0)]
GOAL_MODES = ('corners', 'straights', 'path')


def straight_goals(ends):
    """Return 8 poses (x, y, yaw): start and end of every straight, south first."""
    goals = []
    for (x0, y0), (x1, y1) in ends:
        yaw = math.atan2(y1 - y0, x1 - x0)
        goals += [(x0, y0, yaw), (x1, y1, yaw)]
    return goals


class NavLap(BasicNavigator):
    def __init__(self):
        super().__init__('r550_sim_nav_lap')
        p = self.declare_parameter
        self.out_dir = p('out', '.').value
        self.laps = int(p('laps', 1).value)
        self.start_pose = (float(p('start_x', -0.60).value), float(p('start_y', -0.55).value),
                           float(p('start_yaw', 0.0).value))
        self.mode = p('goals', 'corners').value
        if self.mode not in GOAL_MODES:
            raise ValueError('goals must be one of %s' % ', '.join(GOAL_MODES))
        self.ends = []
        self.path = make_path(float(p('half_side', HALF_SIDE).value),
                              float(p('corner_radius', CORNER_RADIUS).value),
                              transition=float(p('transition', 0.0).value),
                              inset=float(p('inset', 0.0).value), ends=self.ends)
        os.makedirs(self.out_dir, exist_ok=True)
        self.corner = [k > 1.25 for k in curvature(self.path)]   # radius < 0.8 m
        self.truth = None
        self.cmd = (0.0, 0.0)
        self.rows = []
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(Odometry, 'sim/ground_truth',
                                 lambda m: setattr(self, 'truth', m), 10)
        self.create_subscription(Twist, 'cmd_vel_out',
                                 lambda m: setattr(self, 'cmd', (m.linear.x, m.angular.z)), 10)

    def goal(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def nav_pose(self):
        """Return Nav2's own pose (TF map -> base_link), or None."""
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
        except Exception:          # not available yet
            return None
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, yaw_of(q)

    def path_msg(self, i0, i1):
        """Return the reference points i0 .. i1 (wrapping) as a Path, headings along it."""
        n = len(self.path)
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for k in range(i0, i1 + 1):
            (x, y), (qx, qy) = self.path[k % n], self.path[(k + 1) % n]
            msg.poses.append(self.goal(x, y, math.atan2(qy - y, qx - x)))
        return msg

    def legs(self):
        """Return [(kind, payload)] for all laps."""
        if self.mode == 'corners':
            return [('poses', CORNER_GOALS[:2]), ('poses', CORNER_GOALS[2:])] * self.laps
        if self.mode == 'straights':
            g = straight_goals(self.ends)     # S0 S1 E0 E1 N0 N1 W0 W1
            # the lap starts mid south side: south end ... west start, then west end + south start
            return [('poses', g[1:7]), ('poses', g[7:] + g[:1])] * self.laps
        n = len(self.path)
        i0 = nearest(self.path, self.start_pose[0], self.start_pose[1])
        return [('path', (i0 + k * n // 2, i0 + (k + 1) * n // 2))
                for k in range(2 * self.laps)]

    def run(self):
        if any(n.startswith('r550_base') or n == 'r550_feedback' for n in self.get_node_names()):
            self.error('a REAL chassis node is running: refusing to drive')
            return 1
        # BasicNavigator re-publishes its initial pose until AMCL answers; without this it
        # would send (0, 0, 0) and move AMCL to the island
        self.setInitialPose(self.goal(*self.start_pose))
        self.waitUntilNav2Active()
        # that first pose can arrive before AMCL is active; AMCL then starts from the pose in
        # nav2_r550.yaml (the default spawn), 0.6 m off for a start mid south side (2026-10-27
        # campaign). Send it again and wait until Nav2's own pose agrees.
        self.setInitialPose(self.goal(*self.start_pose))
        t_wait = time.monotonic()
        while time.monotonic() - t_wait < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            loc = self.nav_pose()
            if loc is not None and math.dist(loc[:2], self.start_pose[:2]) < 0.03:
                break
        else:
            self.error('Nav2 pose did not reach the start pose: %s' % (self.nav_pose(),))
            return 1
        # scans taken with the wrong pose left phantom walls in the costmaps ("Start occupied",
        # RPP "collision ahead"): clear them and let the lidar fill them again
        self.clearAllCostmaps()
        t_wait = time.monotonic()
        while time.monotonic() - t_wait < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        while self.truth is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        t0 = self.now()
        cpu0, wall0 = time.process_time(), time.monotonic()
        recoveries, i_truth, t_line, result = 0, None, None, TaskResult.UNKNOWN
        for kind, payload in self.legs():
            if kind == 'poses':
                self.goThroughPoses([self.goal(*g) for g in payload])
            else:
                self.followPath(self.path_msg(*payload))
            leg_recoveries = 0
            while not self.isTaskComplete():        # spins this node ~10 Hz
                fb = self.getFeedback()
                if fb is not None and hasattr(fb, 'number_of_recoveries'):
                    leg_recoveries = max(leg_recoveries, fb.number_of_recoveries)
                q = self.truth.pose.pose
                pose = (q.position.x, q.position.y, yaw_of(q.orientation))
                i_truth = nearest(self.path, pose[0], pose[1], i_truth)
                xte = math.dist(self.path[i_truth], pose[:2])
                if t_line is None and xte < ON_LINE:
                    t_line = self.now() - t0
                tw = self.truth.twist.twist.linear
                loc = self.nav_pose() or (float('nan'),) * 3
                self.rows.append((self.now() - t0, *pose, xte, clearance(pose), *self.cmd,
                                  math.hypot(tw.x, tw.y), float(self.corner[i_truth]),
                                  lateral_error(self.path, i_truth, pose[0], pose[1]),
                                  outer_clearance(pose), island_clearance(pose), *loc))
                if self.now() - t0 > 120.0 * self.laps:
                    self.cancelTask()
                    break
            recoveries += leg_recoveries
            result = self.getResult()
            if result != TaskResult.SUCCEEDED:
                break
        cpu = (time.process_time() - cpu0) / max(time.monotonic() - wall0, 1e-6)
        self.report(result, recoveries, t_line, self.now() - t0, cpu)
        return 0

    def report(self, result, recoveries, t_line, t_end, cpu):
        lines = []
        ok = result == TaskResult.SUCCEEDED
        lines.append('Nav2 through %d goals (%d lap, %s): %s in %.1f s  %s'
                     % (len(self.legs()), self.laps, self.mode, result, t_end,
                        'PASS' if ok else 'FAIL'))
        if t_line is None:
            lines.append('true pose never got within %.2f m of the lane centre  FAIL' % ON_LINE)
        else:
            xtes = [r[4] for r in self.rows if r[0] >= t_line]
            lines.append('true pose vs lane centre (from %.1f s): max %.3f m, RMS %.3f m '
                         '(limit %.2f)  %s'
                         % (t_line, max(xtes), math.sqrt(sum(e * e for e in xtes) / len(xtes)),
                            MAX_XTE, 'PASS' if max(xtes) < MAX_XTE else 'FAIL'))
        if self.rows:
            clear = min(r[5] for r in self.rows)
            lines.append('closest footprint corner to a wall: %.3f m (limit %.2f)  %s'
                         % (clear, MIN_CLEARANCE, 'PASS' if clear > MIN_CLEARANCE else 'FAIL'))
            spin = sum(1 for r in self.rows if abs(r[6]) < 0.01 and abs(r[7]) > 0.05)
            lines.append('turn-on-the-spot commands out of CommandGate: %d  %s'
                         % (spin, 'PASS' if spin == 0 else 'FAIL'))
        on_line = [r for r in self.rows if t_line is not None and r[0] >= t_line]
        st = [r[8] for r in on_line if not r[9]]
        co = [r[8] for r in on_line if r[9]]
        if st and co:
            lines.append('true speed: straights max %.3f mean %.3f m/s, corners mean %.3f m/s  '
                         '%s' % (max(st), sum(st) / len(st), sum(co) / len(co),
                                 'PASS' if max(st) > 1.5 * sum(co) / len(co) else 'FAIL'))
        lines.append('recoveries: %d; this node CPU %.1f %% of one core  INFO'
                     % (recoveries, 100.0 * cpu))
        text = '\n'.join(['== sim Nav2 lap =='] + lines)
        print(text, flush=True)
        with open(os.path.join(self.out_dir, 'nav_lap.txt'), 'w') as f:
            f.write(text + '\n')
        with open(os.path.join(self.out_dir, 'nav_lap.csv'), 'w') as f:
            f.write('t,truth_x,truth_y,truth_yaw,xte,clearance,cmd_vx,cmd_wz,truth_v,corner,'
                    'lat,outer,island,loc_x,loc_y,loc_yaw\n')
            for r in self.rows:
                f.write(','.join('%.4f' % v for v in r) + '\n')
        with open(os.path.join(self.out_dir, 'path.csv'), 'w') as f:     # the reference
            f.write('x,y,curvature,corner\n')
            for (x, y), c in zip(self.path, self.corner):
                f.write('%.5f,%.5f,0,%d\n' % (x, y, c))


def main(args=None):
    rclpy.init(args=args)
    node = NavLap()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    time.sleep(0.1)
    return code


if __name__ == '__main__':
    sys.exit(main())
