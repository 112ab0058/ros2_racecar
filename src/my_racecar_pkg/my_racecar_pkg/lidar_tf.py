import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class LidarTFBroadcaster(Node):
    def __init__(self):
        super().__init__('lidar_tf_broadcaster')
        self.tf_publisher = StaticTransformBroadcaster(self)
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'    # 父座標：車身中心
        t.child_frame_id = 'laser_frame'   # 子座標：雷達
        t.transform.translation.x = 0.1    # 雷達在前 10cm
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2    # 雷達高 20cm
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_publisher.sendTransform(t)
        self.get_logger().info('已發布雷達座標轉換: base_link -> laser_frame')

def main():
    rclpy.init()
    node = LidarTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
