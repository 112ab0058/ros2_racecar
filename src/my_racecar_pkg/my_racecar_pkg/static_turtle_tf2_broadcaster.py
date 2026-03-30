import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai, aj, ak = ai/2.0, aj/2.0, ak/2.0
    ci, si, cj, sj, ck, sk = math.cos(ai), math.sin(ai), math.cos(aj), math.sin(aj), math.cos(ak), math.sin(ak)
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs if 'sc' in locals() else si*cj*ck - ci*sj*sk # 簡化修正版
    q[0], q[1], q[2], q[3] = si*cj*ck - ci*sj*sk, ci*sj*ck + si*cj*sk, ci*cj*sk - si*sj*ck, ci*cj*ck + si*sj*sk
    return q

class StaticFramePublisher(Node):
    def __init__(self, transformation):
        super().__init__('static_turtle_tf2_broadcaster')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms(transformation)

    def make_transforms(self, transformation):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = transformation[1]
        t.transform.translation.x = float(transformation[2])
        t.transform.translation.y = float(transformation[3])
        t.transform.translation.z = float(transformation[4])
        quat = quaternion_from_euler(float(transformation[5]), float(transformation[6]), float(transformation[7]))
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info(f'廣播成功: {t.header.frame_id} -> {t.child_frame_id}')

def main():
    # 修正重點：放寬參數檢查，避免 Docker 環境變數導致秒退
    if len(sys.argv) < 8:
        print('用法: python3 ... child_frame x y z roll pitch yaw')
        sys.exit(1)

    rclpy.init()
    node = StaticFramePublisher(sys.argv)
    try:
        rclpy.spin(node) # 修正重點：確保進入 spin 狀態
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()