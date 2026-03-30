import rclpy, math
from rclpy.node import Node
from geometry_msgs.msg import Point
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TurtleBroadcaster(Node):
    def __init__(self):
        super().__init__('turtle_tf2_broadcaster')
        self.turtlename = self.declare_parameter('turtlename', 'turtle1').get_parameter_value().string_value
        self.br = TransformBroadcaster(self)

        # ✅ 記錄上一幀位置，用來估算偏航角
        self.last_x = None
        self.last_y = None
        self.last_yaw = 0.0

        topic = "t1_pos_raw" if self.turtlename == "turtle1" else "t2_pos_raw"
        self.create_subscription(Point, topic, self.handle_pose, 10)
        self.get_logger().info(f"🚀 廣播器已就緒：[{self.turtlename}] 監聽 [{topic}]")

    def handle_pose(self, msg):
        # ✅ 修正3：根據移動方向估算偏航角
        if self.last_x is not None:
            dx = msg.x - self.last_x
            dy = msg.y - self.last_y
            if abs(dx) > 1e-4 or abs(dy) > 1e-4:
                self.last_yaw = math.atan2(dy, dx)

        self.last_x = msg.x
        self.last_y = msg.y

        # 偏航角轉四元數
        yaw = self.last_yaw
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # ✅ 修正3：廣播實際偏航角，不再用單位四元數
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(TurtleBroadcaster())
    rclpy.shutdown()

if __name__ == '__main__':
    main()