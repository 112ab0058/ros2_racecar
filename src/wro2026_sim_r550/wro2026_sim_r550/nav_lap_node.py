"""
Laps of the 160 cm field with Nav2 in the SIMULATION (NavigateThroughPoses), then checks.

    ros2 run wro2026_sim_r550 nav_lap_node --ros-args -p use_sim_time:=true -p laps:=1 -p out:=DIR

Needs sim.launch.py + wro2026_bringup nav2.launch.py (map maps/field_sim.yaml, field frame).
Goals: 4 poses on the lane centre line after each corner, counter-clockwise, sent as two
NavigateThroughPoses legs per lap (east + north, then west + south): a single leg ending at
the south lane goal "reached" it right after the start, because the goal checker only looks
at the distance to the last pose and the lap starts next to it.
Checks against /sim/ground_truth (NEVER sent to Nav2): result, time, cross-track error to
the lane centre line (lap_node's path; from the moment the car is on it), footprint wall
clearance, turning on the spot (|wz| > 0.05 with |vx| < 0.01 in /cmd_vel_out), recoveries.
Writes DIR/nav_lap.txt (PASS / FAIL per check) and DIR/nav_lap.csv.
SIMULATION ONLY: refuses to run if a real chassis node is on the ROS graph.
"""
import math
import os
import sys
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
import rclpy

from .lap_node import clearance, make_path, MAX_XTE, MIN_CLEARANCE, nearest, ON_LINE, yaw_of

# lane centre, after each corner (field frame, counter-clockwise from the south lane)
CORNER_GOALS = [(0.50, -0.10, math.pi / 2), (0.10, 0.50, math.pi),
                (-0.50, 0.10, -math.pi / 2), (-0.10, -0.50, 0.0)]


class NavLap(BasicNavigator):
    def __init__(self):
        super().__init__('r550_sim_nav_lap')
        self.out_dir = self.declare_parameter('out', '.').value
        self.laps = int(self.declare_parameter('laps', 1).value)
        self.start_pose = (float(self.declare_parameter('start_x', -0.60).value),
                           float(self.declare_parameter('start_y', -0.55).value),
                           float(self.declare_parameter('start_yaw', 0.0).value))
        os.makedirs(self.out_dir, exist_ok=True)
        self.path = make_path()
        self.truth = None
        self.cmd = (0.0, 0.0)
        self.rows = []
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

    def run(self):
        if any(n.startswith('r550_base') or n == 'r550_feedback' for n in self.get_node_names()):
            self.error('a REAL chassis node is running: refusing to drive')
            return 1
        # BasicNavigator re-publishes its initial pose until AMCL answers; without this it
        # would send (0, 0, 0) and move AMCL to the island
        self.setInitialPose(self.goal(*self.start_pose))
        self.waitUntilNav2Active()
        while self.truth is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        legs = [CORNER_GOALS[:2], CORNER_GOALS[2:]] * self.laps
        t0 = self.now()
        recoveries, i_truth, t_line, result = 0, None, None, TaskResult.UNKNOWN
        for leg in legs:
            self.goThroughPoses([self.goal(*g) for g in leg])
            leg_recoveries = 0
            while not self.isTaskComplete():        # spins this node ~10 Hz
                fb = self.getFeedback()
                if fb is not None:
                    leg_recoveries = max(leg_recoveries, fb.number_of_recoveries)
                q = self.truth.pose.pose
                pose = (q.position.x, q.position.y, yaw_of(q.orientation))
                i_truth = nearest(self.path, pose[0], pose[1], i_truth)
                xte = math.dist(self.path[i_truth], pose[:2])
                if t_line is None and xte < ON_LINE:
                    t_line = self.now() - t0
                self.rows.append((self.now() - t0, *pose, xte, clearance(pose), *self.cmd))
                if self.now() - t0 > 120.0 * self.laps:
                    self.cancelTask()
                    break
            recoveries += leg_recoveries
            result = self.getResult()
            if result != TaskResult.SUCCEEDED:
                break
        self.report(result, recoveries, t_line, self.now() - t0)
        return 0

    def report(self, result, recoveries, t_line, t_end):
        lines = []
        ok = result == TaskResult.SUCCEEDED
        lines.append('Nav2 through %d goals (%d lap, 2 legs each): %s in %.1f s  %s'
                     % (len(CORNER_GOALS) * self.laps, self.laps, result, t_end,
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
        lines.append('recoveries: %d  INFO' % recoveries)
        text = '\n'.join(['== sim Nav2 lap =='] + lines)
        print(text, flush=True)
        with open(os.path.join(self.out_dir, 'nav_lap.txt'), 'w') as f:
            f.write(text + '\n')
        with open(os.path.join(self.out_dir, 'nav_lap.csv'), 'w') as f:
            f.write('t,truth_x,truth_y,truth_yaw,xte,clearance,cmd_vx,cmd_wz\n')
            for r in self.rows:
                f.write(','.join('%.4f' % v for v in r) + '\n')


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
