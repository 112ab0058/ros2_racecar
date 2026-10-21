"""
Scripted drive in the SIMULATION, then compare /odom and the EKF with the true pose.

    ros2 run wro2026_sim_r550 drive_test_node --ros-args -p use_sim_time:=true -p out:=DIR

Course (simulation time), starting at the default spawn (-0.60, -0.55, yaw 0):
   3 s  still                     (ZUPT learns the simulated gyro bias)
   4 s  straight 0.15 m/s          east along the south lane
   5.24 s  left turn 0.15 m/s, wz 0.30 (radius 0.5 m, 90 deg)
   2 s  straight 0.15 m/s          north along the east lane
   3 s  stop (nothing published)  -> CommandGate timeout must stop the car
   2 s  wz 0.5 with vx 0          -> must NOT turn (no turning on the spot)
   2 s  stop
Writes DIR/drive_test.txt (PASS / WARN per check) and DIR/drive_test.csv.
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

# (duration s, vx, wz, publish?)
COURSE = [(3.0, 0.0, 0.0, False), (4.0, 0.15, 0.0, True), (5.236, 0.15, 0.30, True),
          (2.0, 0.15, 0.0, True), (3.0, 0.0, 0.0, False), (2.0, 0.0, 0.5, True),
          (2.0, 0.0, 0.0, False)]
EXPECTED_PATH_M = 0.15 * (4.0 + 5.236 + 2.0)


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class DriveTest(Node):
    def __init__(self):
        super().__init__('r550_sim_drive_test')
        self.out_dir = self.declare_parameter('out', '.').value
        os.makedirs(self.out_dir, exist_ok=True)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.last = {}
        self.rows = []
        for name, topic in (('truth', 'sim/ground_truth'), ('odom', 'odom'),
                            ('ekf', 'odometry/filtered')):
            self.create_subscription(Odometry, topic,
                                     lambda m, n=name: self.last.__setitem__(n, m), 10)
        self.t0 = None
        self.start = {}
        self.marks = {}
        self.done = False
        self.create_timer(0.05, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self, name):
        m = self.last.get(name)
        if m is None:
            return None
        p = m.pose.pose.position
        return p.x, p.y, yaw_of(m.pose.pose.orientation)

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
            if not all(k in self.last for k in ('truth', 'odom')):
                return
            self.t0 = now
            for k in ('truth', 'odom', 'ekf'):
                self.start[k] = self._pose(k)
        t = now - self.t0
        seg_end, vx, wz, publish, idx = 0.0, 0.0, 0.0, False, len(COURSE)
        for i, (d, v, w, p) in enumerate(COURSE):
            seg_end += d
            if t < seg_end:
                vx, wz, publish, idx = v, w, p, i
                break
        if idx not in self.marks:
            self.marks[idx] = {k: self._pose(k) for k in ('truth', 'odom', 'ekf')}
        if publish:
            msg = Twist()
            msg.linear.x, msg.angular.z = vx, wz
            self.pub.publish(msg)
        row = [t]
        for k in ('truth', 'odom', 'ekf'):
            row += list(self._pose(k) or (float('nan'),) * 3)
        tm = self.last['truth'].twist.twist
        row += [math.hypot(tm.linear.x, tm.linear.y), tm.angular.z]
        self.rows.append(row)
        if idx == len(COURSE):
            self.done = True
            self._report()

    def _rel(self, k, pose):
        """Pose relative to the start pose of source k, in the start frame."""
        x0, y0, a0 = self.start[k]
        dx, dy = pose[0] - x0, pose[1] - y0
        return (math.cos(a0) * dx + math.sin(a0) * dy,
                -math.sin(a0) * dx + math.cos(a0) * dy, wrap(pose[2] - a0))

    def _report(self):
        lines = []
        path = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(self.rows, self.rows[1:]))
        ok = abs(path - EXPECTED_PATH_M) < 0.15 * EXPECTED_PATH_M
        lines.append('true path %.3f m (commanded %.3f m)  %s'
                     % (path, EXPECTED_PATH_M, 'PASS' if ok else 'WARN'))
        end_drive = self.marks.get(4)          # start of the 3 s stop
        truth_end = self._rel('truth', self.marks[5]['truth'])
        for k in ('odom', 'ekf'):
            if self.start.get(k) is None or self.marks[5].get(k) is None:
                lines.append('%s: no data  WARN' % k)
                continue
            e = self._rel(k, self.marks[5][k])
            dist = math.hypot(e[0] - truth_end[0], e[1] - truth_end[1])
            dyaw = math.degrees(wrap(e[2] - truth_end[2]))
            lines.append('%-4s vs truth after the course: position error %.3f m '
                         '(%.1f %% of path), yaw error %+.2f deg  INFO'
                         % (k, dist, 100 * dist / max(path, 1e-6), dyaw))
        if end_drive:
            a = self._rel('truth', end_drive['truth'])
            b = truth_end
            lines.append('stop: moved %.3f m during the 3 s after the last command  %s'
                         % (math.hypot(b[0] - a[0], b[1] - a[1]),
                            'PASS' if math.hypot(b[0] - a[0], b[1] - a[1]) < 0.10 else 'WARN'))
        spin = math.degrees(abs(wrap(self._rel('truth', self.marks[len(COURSE)]['truth'])[2]
                                     - truth_end[2])))
        lines.append('no turning on the spot: yaw changed %.2f deg with vx 0, wz 0.5  %s'
                     % (spin, 'PASS' if spin < 2.0 else 'WARN'))
        text = '\n'.join(['== sim drive test =='] + lines)
        print(text, flush=True)
        with open(os.path.join(self.out_dir, 'drive_test.txt'), 'w') as f:
            f.write(text + '\n')
        with open(os.path.join(self.out_dir, 'drive_test.csv'), 'w') as f:
            f.write('t,truth_x,truth_y,truth_yaw,odom_x,odom_y,odom_yaw,ekf_x,ekf_y,ekf_yaw,'
                    'truth_v,truth_wz\n')
            for r in self.rows:
                f.write(','.join('%.4f' % v for v in r) + '\n')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = DriveTest()
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
