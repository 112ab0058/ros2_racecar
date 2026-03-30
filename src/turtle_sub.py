import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_subscriber_node')
        # 建立訂閱者，主題名稱必須完全一致
        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info('接收端啟動成功，正在監聽指令頻道...')

    def listener_callback(self, msg):
        # 收到訊息時的回呼函式
        self.get_logger().info('接收到 -> 線速度: %.1f, 角速度: %.1f' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSubscriber()
    try:
        rclpy.spin(node) # 讓程式停在這裡等待訊息
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()