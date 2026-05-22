#!/usr/bin/env python3
"""
Independent WRO 2026 lawn-mower mapping tool.

This node subscribes to /odom and /scan, then publishes motion commands on
/cmd_vel while it executes a fixed coverage pattern. Do not run it at the same
time as wro2026_sim/scripts/line_follower.py, because both nodes can publish
/cmd_vel. Step 6/7 official line-triggered race control still uses
wro2026_sim/scripts/line_follower.py.
"""

import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    Twist,
    PoseStamped,
    PoseWithCovarianceStamped
)


class WRO2026Mapper(Node):

    def __init__(self):
        super().__init__('wro2026_mapper')

        # parameters
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')

        # publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_ready = False

        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0
        self.scan_ready = False

        self.front_min = float('inf')

        # limits
        self.max_lin = 0.06
        self.max_ang = 0.35

        self.front_deg = 20
        self.front_thresh = 0.25

        # subscribers
        self.odom_sub = None
        self.scan_sub = None

        self.setup_subscriptions()

        # movement plan
        self.actions = self.build_actions(
            line_length=1.6,
            side_shift=0.6,
            passes=4
        )

    # =========================================================
    # BUILD ACTIONS
    # =========================================================

    def build_actions(self, line_length=1.6, side_shift=0.6, passes=4):

        actions = []

        for i in range(passes):

            # move forward
            actions.append(('move', line_length))

            # last pass no turn
            if i == passes - 1:
                break

            # lawn mower pattern
            if i % 2 == 0:

                actions.append(('turn', math.pi / 2))
                actions.append(('move', side_shift))
                actions.append(('turn', math.pi / 2))

            else:

                actions.append(('turn', -math.pi / 2))
                actions.append(('move', side_shift))
                actions.append(('turn', -math.pi / 2))

        return actions

    # =========================================================
    # SUBSCRIPTIONS
    # =========================================================

    def setup_subscriptions(self):

        odom_topic = self.get_parameter('odom_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        topics = dict(self.get_topic_names_and_types())

        # -----------------------------
        # odom source
        # -----------------------------

        odom_assigned = False

        if odom_topic in topics:

            types = topics[odom_topic]

            if 'nav_msgs/msg/Odometry' in types:

                self.odom_sub = self.create_subscription(
                    Odometry,
                    odom_topic,
                    self.odom_cb,
                    10
                )

                odom_assigned = True

            elif 'geometry_msgs/msg/PoseStamped' in types:

                self.odom_sub = self.create_subscription(
                    PoseStamped,
                    odom_topic,
                    self.pose_stamped_cb,
                    10
                )

                odom_assigned = True

            elif 'geometry_msgs/msg/PoseWithCovarianceStamped' in types:

                self.odom_sub = self.create_subscription(
                    PoseWithCovarianceStamped,
                    odom_topic,
                    self.pose_with_cov_cb,
                    10
                )

                odom_assigned = True

        # fallback
        if not odom_assigned:

            for name, types in topics.items():

                if 'nav_msgs/msg/Odometry' in types:

                    self.odom_sub = self.create_subscription(
                        Odometry,
                        name,
                        self.odom_cb,
                        10
                    )

                    odom_topic = name
                    odom_assigned = True
                    break

        if not odom_assigned:

            self.odom_sub = self.create_subscription(
                Odometry,
                odom_topic,
                self.odom_cb,
                10
            )

            print(f'Waiting for odom topic: {odom_topic}')

        else:

            print(f'Using odom source: {odom_topic}')

        # -----------------------------
        # scan source
        # -----------------------------

        scan_assigned = False

        if scan_topic in topics:

            if 'sensor_msgs/msg/LaserScan' in topics[scan_topic]:

                self.scan_sub = self.create_subscription(
                    LaserScan,
                    scan_topic,
                    self.scan_cb,
                    10
                )

                scan_assigned = True

        if not scan_assigned:

            for name, types in topics.items():

                if 'sensor_msgs/msg/LaserScan' in types:

                    self.scan_sub = self.create_subscription(
                        LaserScan,
                        name,
                        self.scan_cb,
                        10
                    )

                    scan_topic = name
                    scan_assigned = True
                    break

        if not scan_assigned:

            self.scan_sub = self.create_subscription(
                LaserScan,
                scan_topic,
                self.scan_cb,
                10
            )

            print(f'Waiting for scan topic: {scan_topic}')

        else:

            print(f'Using scan source: {scan_topic}')

    # =========================================================
    # CALLBACKS
    # =========================================================

    def quaternion_to_yaw(self, q):

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        return math.atan2(siny_cosp, cosy_cosp)

    def pose_stamped_cb(self, msg):

        self.odom_x = msg.pose.position.x
        self.odom_y = msg.pose.position.y
        self.odom_yaw = self.quaternion_to_yaw(msg.pose.orientation)

        self.odom_ready = True

    def pose_with_cov_cb(self, msg):

        p = msg.pose.pose

        self.odom_x = p.position.x
        self.odom_y = p.position.y
        self.odom_yaw = self.quaternion_to_yaw(p.orientation)

        self.odom_ready = True

    def odom_cb(self, msg):

        p = msg.pose.pose

        self.odom_x = p.position.x
        self.odom_y = p.position.y
        self.odom_yaw = self.quaternion_to_yaw(p.orientation)

        self.odom_ready = True

    def scan_cb(self, msg):

        self.scan_ranges = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

        self.scan_ready = True

        self.compute_front_min()

    # =========================================================
    # LASER
    # =========================================================

    def compute_front_min(self):

        if not self.scan_ready:

            self.front_min = float('inf')
            return

        front_min = float('inf')

        for i, r in enumerate(self.scan_ranges):

            if math.isinf(r) or math.isnan(r):
                continue

            angle = (
                self.scan_angle_min
                + i * self.scan_angle_increment
            )

            if abs(angle) <= math.radians(self.front_deg):

                if r < front_min:
                    front_min = r

        self.front_min = front_min

    # =========================================================
    # UTIL
    # =========================================================

    def publish_zero(self):

        t = Twist()

        t.linear.x = 0.0
        t.angular.z = 0.0

        self.cmd_pub.publish(t)

    def normalize_angle(self, ang):

        return math.atan2(
            math.sin(ang),
            math.cos(ang)
        )

    # =========================================================
    # MOVE
    # =========================================================

    def move_distance(self, dist):

        start_x = self.odom_x
        start_y = self.odom_y
        start_yaw = self.odom_yaw

        target_x = start_x + dist * math.cos(start_yaw)
        target_y = start_y + dist * math.sin(start_yaw)

        print(
            f'CURRENT x={self.odom_x:.3f} '
            f'y={self.odom_y:.3f} '
            f'yaw={self.odom_yaw:.3f}'
        )

        print(
            f'TARGET  x={target_x:.3f} '
            f'y={target_y:.3f}'
        )

        self.publish_zero()
        time.sleep(1.0)

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.01)

            dx = target_x - self.odom_x
            dy = target_y - self.odom_y

            remain = math.hypot(dx, dy)

            if remain < 0.03:
                break

            # safety stop
            if self.front_min < self.front_thresh:

                self.publish_zero()

                print(
                    f'EMERGENCY STOP '
                    f'front={self.front_min:.3f}'
                )

                return False

            target_yaw = math.atan2(dy, dx)

            yaw_error = self.normalize_angle(
                target_yaw - self.odom_yaw
            )

            lin = min(
                self.max_lin,
                remain * 0.5
            )

            ang = max(
                -self.max_ang,
                min(self.max_ang, yaw_error)
            )

            if abs(yaw_error) > 0.35:
                lin = 0.0

            t = Twist()

            t.linear.x = lin
            t.angular.z = ang

            self.cmd_pub.publish(t)

            time.sleep(0.05)

        self.publish_zero()
        time.sleep(1.0)

        return True

    # =========================================================
    # ROTATE
    # =========================================================

    def rotate_angle(self, angle):

        start_yaw = self.odom_yaw

        target_yaw = self.normalize_angle(
            start_yaw + angle
        )

        print(
            f'ROTATE target_yaw={target_yaw:.3f}'
        )

        self.publish_zero()
        time.sleep(1.0)

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.01)

            error = self.normalize_angle(
                target_yaw - self.odom_yaw
            )

            if abs(error) < 0.02:
                break

            if self.front_min < self.front_thresh:

                self.publish_zero()

                print(
                    f'EMERGENCY STOP '
                    f'front={self.front_min:.3f}'
                )

                return False

            ang = max(
                -self.max_ang,
                min(self.max_ang, error * 1.2)
            )

            t = Twist()

            t.linear.x = 0.0
            t.angular.z = ang

            self.cmd_pub.publish(t)

            time.sleep(0.05)

        self.publish_zero()
        time.sleep(1.0)

        return True

    # =========================================================
    # READY CHECK
    # =========================================================

    def wait_ready(self, timeout=15.0):

        start = time.time()

        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0.05)

            if self.odom_ready and self.scan_ready:
                return True

            if time.time() - start > timeout:
                break

            time.sleep(0.05)

        print('ERROR: odom or scan not ready')

        return False

    # =========================================================
    # MAIN MAPPING
    # =========================================================

    def run_mapping(self):

        if not self.wait_ready():

            print('Failed to receive sensors')
            return

        print('Starting mapping sequence')

        for act, val in self.actions:

            if act == 'move':

                ok = self.move_distance(val)

            else:

                ok = self.rotate_angle(val)

            if not ok:

                print('Mapping aborted')
                break

        self.publish_zero()

        print('Mapping finished')


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = WRO2026Mapper()

    try:

        node.run_mapping()

    finally:

        node.publish_zero()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
