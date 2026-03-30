import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SpeedPublisher(Node):
    def __init__(self):
        super().__init__('speed_publisher')
        self.publisher_ = self.create_publisher(String, 'car_speed', 10)
        # 確保下面這一行的 self.timer_callback 對應到下面的函數名稱
        self.timer = self.create_timer(0.1, self.timer_callback) 

    def timer_callback(self):
        msg = String()
        msg.data = 'Speed: 20km/h'
        self.publisher_.publish(msg)
        self.get_logger().info('正在發送賽車時速...')

def main(args=None):
    rclpy.init(args=args)
    node = SpeedPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()