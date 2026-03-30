import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_control_node')
        # 建立發送者，發送到小烏龜的指令主題
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 設定計時器，每 0.1 秒執行一次 timer_callback
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('發送端已啟動，開始指揮小烏龜！')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0  # 前進速度
        msg.angular.z = 1.0 # 轉彎速度
        self.publisher_.publish(msg)
        # 這一行很重要，能讓你看到程式真的有在發送
        self.get_logger().info('正在發送指令：前進=2.0, 轉彎=1.0')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControl()
    try:
        rclpy.spin(node) # 讓程式持續運行
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()