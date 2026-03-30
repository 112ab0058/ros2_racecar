import rclpy
import math
import random
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSim(Node):
    def __init__(self):
        super().__init__('lidar_sim')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp    = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min       = -math.pi
        scan.angle_max       =  math.pi
        scan.angle_increment =  2 * math.pi / 360
        scan.time_increment  = 0.0
        scan.range_min       = 0.1
        scan.range_max       = 10.0
        scan.ranges = [random.uniform(0.1, 5.0) for _ in range(360)]
        self.publisher_.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('雷達模擬器停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
