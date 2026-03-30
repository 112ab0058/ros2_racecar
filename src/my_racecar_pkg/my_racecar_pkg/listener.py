import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'馬達接收指令 -> 前進: {msg.linear.x} m/s, 轉向: {msg.angular.z} rad/s')

def main(args=None):
    rclpy.init(args=args)
    # ✅ 修正：這裡要對應上面的 class
    node = MotorDriver() 
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('馬達驅動器停止中...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()