#!/usr/bin/env python3
"""
Step 6: Visual PID straight-line controller
Subscribes to /line_detection, publishes /cmd_vel to keep car centered.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time

# ─── Tunable parameters ───────────────────────────────────────────────
IMAGE_WIDTH   = 640
CENTER_X      = IMAGE_WIDTH / 2.0

Kp = 0.003
Ki = 0.0000
Kd = 0.000            # 先設 0，確認轉向方向正確後再調

LINEAR_SPEED  = 0.12
MAX_ANGULAR   = 0.8

INTEGRAL_CLAMP  = 300.0
DEAD_ZONE_PX    = 10.0

# Step 6 單獨測試設 False；Step 8 整合狀態機後改 True
STOP_ON_TRIGGER = False
# ──────────────────────────────────────────────────────────────────────


class LinePIDFollower(Node):
    def __init__(self):
        super().__init__('line_pid_follower')

        self.sub = self.create_subscription(
            Float32MultiArray,
            '/line_detection',
            self.detection_cb,
            10
        )
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._prev_error = 0.0
        self._integral   = 0.0
        self._last_time  = time.time()

        self.get_logger().info('Line PID Follower started')

    def _pick_cx(self, data) -> float | None:
        """橘線優先（面積較大穩定）→ 藍線 → None"""
        if data[0] and data[1] >= 0:
            return data[1]
        if data[4] and data[5] >= 0:
            return data[5]
        return None

    def _pid(self, error: float) -> float:
        now = time.time()
        dt  = now - self._last_time
        if dt < 1e-6:
            dt = 1e-6
        self._last_time = now

        self._integral += error * dt
        self._integral  = max(-INTEGRAL_CLAMP,
                               min(INTEGRAL_CLAMP, self._integral))

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        pid_output = Kp * error + Ki * self._integral + Kd * derivative
        return max(-MAX_ANGULAR, min(MAX_ANGULAR, pid_output))

    def detection_cb(self, msg: Float32MultiArray):
        data = msg.data

        if len(data) < 8:
            self.get_logger().warn(
                f'/line_detection length too short: {len(data)}',
                throttle_duration_sec=1.0
            )
            self.pub.publish(Twist())
            return

        orange_trigger = bool(data[3])
        blue_trigger   = bool(data[7])

        if STOP_ON_TRIGGER and (orange_trigger or blue_trigger):
            self.pub.publish(Twist())
            return

        cx = self._pick_cx(data)
        twist = Twist()

        if cx is None:
            twist.linear.x  = LINEAR_SPEED * 0.4
            twist.angular.z = 0.0
            self.pub.publish(twist)
            return

        error = cx - CENTER_X   # 正=線在畫面右側，車需右轉

        if abs(error) < DEAD_ZONE_PX:
            error = 0.0

        pid_output = self._pid(error)
        twist.linear.x  = LINEAR_SPEED
        twist.angular.z = -pid_output  # error正 → pid正 → angular_z負 → 右轉

        self.get_logger().info(
            f'cx={cx:.0f}  err={error:+.0f}  angular_z={twist.angular.z:+.3f}',
            throttle_duration_sec=0.5
        )
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LinePIDFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
