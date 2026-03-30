import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random

class LidarSim(Node):
    def __init__(self):
        super().__init__('lidar_sim')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.range_min = 0.1
        scan.range_max = 10.0
        # 產生 360 個隨機距離，模擬真實掃描環境
        scan.ranges = [random.uniform(0.1, 5.0) for _ in range(360)]
        self.publisher_.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    # ✅ 修正：這裡要對應上面的 class 
    node = LidarSim() 
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('雷達模擬器停止中...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()