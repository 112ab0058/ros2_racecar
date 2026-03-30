import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class CarrotBroadcaster(Node):
    def __init__(self):
        super().__init__('carrot_broadcaster')
        self.br = TransformBroadcaster(self)
        # 每 0.05 秒更新一次胡蘿蔔的位置
        self.timer = self.create_timer(0.05, self.broadcast_carrot)

    def broadcast_carrot(self):
        t = TransformStamped()
        seconds, _ = self.get_clock().now().seconds_nanoseconds()
        
        # 讓胡蘿蔔繞著 turtle1 轉，半徑 2.0 公尺
        # 利用時間當角度，達成旋轉效果
        angle = seconds * 0.5 

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'  # 它的爸爸是紅色海龜
        t.child_frame_id = 'carrot1'   # 它叫胡蘿蔔
        
        # 定義相對於爸爸的位置
        t.transform.translation.x = 2.0 * math.sin(angle)
        t.transform.translation.y = 2.0 * math.cos(angle)
        t.transform.translation.z = 0.0
        
        # 旋轉設為 0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(CarrotBroadcaster())
    rclpy.shutdown()

if __name__ == '__main__':
    main()