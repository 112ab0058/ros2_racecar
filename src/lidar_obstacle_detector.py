import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs  # noqa
from tf2_ros import Buffer, TransformListener


class ObstacleDetector(Node):
    OBSTACLE_THRESHOLD = 2.0
    SECTORS = {
        'FRONT':       (-30,   30),
        'FRONT_LEFT':  ( 30,   90),
        'LEFT':        ( 90,  150),
        'FRONT_RIGHT': (-90,  -30),
        'RIGHT':       (-150, -90),
    }

    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 手動實作 MessageFilter 概念：
        # 收到 scan 後先確認 transform 存在才處理，否則丟棄
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.warning_pub = self.create_publisher(String, '/obstacle_warning', 10)
        self.get_logger().info('障礙物偵測器啟動，等待 laser_frame → base_link ...')

    def scan_callback(self, scan_msg):
        # ── MessageFilter 核心概念：先確認 transform 可用 ──────────────────
        try:
            self.tf_buffer.lookup_transform(
                'base_link',
                scan_msg.header.frame_id,
                scan_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except tf2_ros.TransformException:
            # transform 還沒準備好，丟棄這筆資料（跟 MessageFilter 行為一致）
            return

        # ── 逐點轉換 ───────────────────────────────────────────────────────
        obstacles = []
        angle = scan_msg.angle_min
        for distance in scan_msg.ranges:
            angle_deg = math.degrees(angle)
            angle += scan_msg.angle_increment

            if (distance < scan_msg.range_min or distance > scan_msg.range_max
                    or math.isnan(distance) or math.isinf(distance)):
                continue
            if distance > self.OBSTACLE_THRESHOLD:
                continue

            p = PointStamped()
            p.header.stamp    = scan_msg.header.stamp
            p.header.frame_id = scan_msg.header.frame_id
            p.point.x = distance * math.cos(math.radians(angle_deg))
            p.point.y = distance * math.sin(math.radians(angle_deg))
            p.point.z = 0.0

            try:
                pb = self.tf_buffer.transform(
                    p, 'base_link',
                    timeout=rclpy.duration.Duration(seconds=0.05))
                obs_ang = math.degrees(math.atan2(pb.point.y, pb.point.x))
                obstacles.append((obs_ang, distance))
            except tf2_ros.TransformException:
                continue

        if not obstacles:
            return

        # ── 扇區判斷 ───────────────────────────────────────────────────────
        triggered = []
        for name, (lo, hi) in self.SECTORS.items():
            for ang, dist in obstacles:
                if lo <= ang <= hi:
                    triggered.append(f'{name}({dist:.2f}m)')
                    break
        for ang, dist in obstacles:
            if ang >= 150 or ang <= -150:
                triggered.append(f'REAR({dist:.2f}m)')
                break

        if triggered:
            warning = f'⚠ 障礙物：{", ".join(triggered)}'
            self.get_logger().warn(warning)
            msg = String()
            msg.data = warning
            self.warning_pub.publish(msg)


def main():
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
