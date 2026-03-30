import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException, Buffer, TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


WORLD_SIZE  = 11.08
WALL_MARGIN = 1.2
WALL_ESCAPE = 0.3
DELAY_SEC   = 5.0  # 追幾秒前的位置


class TimeTravelListener(Node):

    def __init__(self):
        super().__init__('time_travel_listener')

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher   = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)

        self.k_v = 1.2
        self.k_w = 2.5

        self.stop_distance = 0.15
        self.max_lin = 2.0
        self.max_ang = 2.5

        self.turtle2_x = 5.5
        self.turtle2_y = 5.5

        self.timer = self.create_timer(0.05, self.on_timer)
        self.get_logger().info(f'⏰ 時間旅行模式：追 {DELAY_SEC} 秒前的 carrot1')

    def on_timer(self):
        try:
            t2 = self.tf_buffer.lookup_transform('world', 'turtle2', rclpy.time.Time())
            self.turtle2_x = t2.transform.translation.x
            self.turtle2_y = t2.transform.translation.y
        except TransformException:
            pass

        msg = Twist()

        escape = self._escape_wall(self.turtle2_x, self.turtle2_y)
        if escape is not None:
            msg.linear.x  = escape[0]
            msg.angular.z = escape[1]
            self.publisher.publish(msg)
            return

        try:
            # ✅ 時間旅行：查 DELAY_SEC 秒前 carrot1 的位置
            when = self.get_clock().now() - rclpy.duration.Duration(seconds=DELAY_SEC)

            trans = self.tf_buffer.lookup_transform_full(
                target_frame='turtle2',
                target_time=rclpy.time.Time(),   # turtle2 用現在位置
                source_frame='carrot1',
                source_time=when,                 # carrot1 用過去位置
                fixed_frame='world',
                timeout=rclpy.duration.Duration(seconds=0.05))

            x = trans.transform.translation.x
            y = trans.transform.translation.y

            distance    = math.sqrt(x*x + y*y)
            angle_error = normalize_angle(math.atan2(y, x))
            wall_scale  = self._wall_scale(self.turtle2_x, self.turtle2_y)

            if distance < self.stop_distance:
                msg.linear.x  = 0.0
                msg.angular.z = 0.0
            else:
                linear  = self.k_v * distance * abs(math.cos(angle_error)) * wall_scale
                angular = self.k_w * angle_error
                msg.linear.x  = max(min(linear,  self.max_lin), -self.max_lin)
                msg.angular.z = max(min(angular,  self.max_ang), -self.max_ang)

            self.publisher.publish(msg)

        except TransformException:
            # 前5秒還沒有歷史資料，正常現象
            self.publisher.publish(Twist())

    def _escape_wall(self, wx, wy):
        hitting = (wx < WALL_ESCAPE or wx > WORLD_SIZE - WALL_ESCAPE or
                   wy < WALL_ESCAPE or wy > WORLD_SIZE - WALL_ESCAPE)
        if not hitting:
            return None
        cx, cy = WORLD_SIZE / 2, WORLD_SIZE / 2
        angle_to_center = math.atan2(cy - wy, cx - wx)
        return (0.8, normalize_angle(angle_to_center) * 3.0)

    def _wall_scale(self, wx, wy):
        dist_to_wall = min(wx, wy, WORLD_SIZE - wx, WORLD_SIZE - wy)
        if dist_to_wall >= WALL_MARGIN:
            return 1.0
        return max(0.2, dist_to_wall / WALL_MARGIN)


def main():
    rclpy.init()
    rclpy.spin(TimeTravelListener())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
