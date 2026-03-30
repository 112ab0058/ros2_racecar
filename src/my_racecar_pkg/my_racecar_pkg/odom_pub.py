import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_pub')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.vel_callback, 10)
        
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()

    def vel_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        
        # 積分計算位置
        vx, vth = msg.linear.x, msg.angular.z
        self.x += vx * math.cos(self.th) * dt
        self.y += vx * math.sin(self.th) * dt
        self.th += vth * dt

        # 1. 發布 TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # 2. 發布 Odometry 訊息 (標準規範)
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('里程計停止中...')
    finally:
        node.destroy_node()
        rclpy.shutdown()