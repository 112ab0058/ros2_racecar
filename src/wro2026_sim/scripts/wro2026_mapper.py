#!/usr/bin/env python3
import math, time, subprocess, os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WROMapper(Node):
    def __init__(self):
        super().__init__('wro2026_mapper')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_x = 0.0; self.odom_y = 0.0; self.odom_yaw = 0.0
        self.odom_ready = False; self.scan_ready = False
        self.front_min = float('inf')
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.max_lin = 0.10; self.max_ang = 0.40

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.odom_x = p.position.x; self.odom_y = p.position.y
        q = p.orientation
        self.odom_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.odom_ready = True

    def scan_cb(self, msg):
        self.scan_ready = True
        front = [r for i,r in enumerate(msg.ranges)
                 if not(math.isinf(r) or math.isnan(r))
                 and abs(msg.angle_min+i*msg.angle_increment) < math.radians(15)]
        self.front_min = min(front) if front else float('inf')

    def stop(self):
        self.cmd_pub.publish(Twist())

    def norm(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def move_to(self, tx, ty, label=""):
        self.get_logger().info(f"→ ({tx:.2f},{ty:.2f}) {label}")
        self.stop(); time.sleep(0.3)
        for _ in range(2000):
            rclpy.spin_once(self, timeout_sec=0.02)
            dx = tx - self.odom_x; dy = ty - self.odom_y
            dist = math.hypot(dx, dy)
            if dist < 0.06: break
            target_yaw = math.atan2(dy, dx)
            yaw_err = self.norm(target_yaw - self.odom_yaw)
            lin = min(self.max_lin, dist*0.8) if abs(yaw_err) < 0.4 else 0.0
            ang = max(-self.max_ang, min(self.max_ang, yaw_err*2.5))
            t = Twist(); t.linear.x = float(lin); t.angular.z = float(ang)
            self.cmd_pub.publish(t)
            time.sleep(0.05)
        self.stop(); time.sleep(0.3)
        self.get_logger().info(f"  實際({self.odom_x:.2f},{self.odom_y:.2f})")
        return True

    def rotate_to(self, target_yaw):
        self.stop(); time.sleep(0.3)
        for _ in range(500):
            rclpy.spin_once(self, timeout_sec=0.02)
            err = self.norm(target_yaw - self.odom_yaw)
            if abs(err) < 0.03: break
            ang = max(-self.max_ang, min(self.max_ang, err*2.5))
            t = Twist(); t.angular.z = float(ang)
            self.cmd_pub.publish(t)
            time.sleep(0.05)
        self.stop(); time.sleep(0.3)

    def wait_ready(self):
        self.get_logger().info("等待感測器...")
        for _ in range(400):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom_ready and self.scan_ready:
                self.get_logger().info(f"起點({self.odom_x:.3f},{self.odom_y:.3f}) yaw={math.degrees(self.odom_yaw):.1f}°")
                return True
            time.sleep(0.05)
        return False

    def save_map(self):
        path = os.path.expanduser("~/ros2_ws/src/wro2026_sim/maps/wro2026_map")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        r = subprocess.run(["ros2","run","nav2_map_server","map_saver_cli",
             "-f",path,"--ros-args","-p","save_map_timeout:=10.0"],
            capture_output=True, text=True, timeout=30)
        if r.returncode==0: self.get_logger().info("✅ 存檔成功")
        else: self.get_logger().error(f"❌ 失敗\n{r.stderr}")

    def run(self):
        if not self.wait_ready(): return

        # 外圈走道中心座標
        # 東側 x=1.1, 西側 x=-1.1, 北側 y=1.1, 南側 y=-1.1
        # 起點 (0, 0.9) 朝東
        #
        # 關鍵：所有路徑點都在外圈走道內，不穿過內圈
        # 內牆在 x=±0.6, y=±0.6
        # 外圈走道：x=0.6~1.6（東）, x=-1.6~-0.6（西）, y=0.6~1.6（北）, y=-1.6~-0.6（南）
        #
        # 路徑：沿外圈走道邊緣繞，不走中間
        # 起點(0,0.9) → 東北角(1.1,1.1) → 東南角(1.1,-1.1)
        #             → 西南角(-1.1,-1.1) → 西北角(-1.1,1.1) → 回東北角
        # 第二圈重複
        # 注意：(0,0.9) 到 (1.1,1.1) 是斜線，全在北側走道內，安全

        corners = [
            ( 1.1,  1.1, "東北角"),
            ( 1.1, -1.1, "東南角"),
            (-1.1, -1.1, "西南角"),
            (-1.1,  1.1, "西北角"),
            ( 1.1,  1.1, "東北角 [第1圈完]"),
            ( 1.1, -1.1, "東南角"),
            (-1.1, -1.1, "西南角"),
            (-1.1,  1.1, "西北角"),
            ( 1.1,  1.1, "東北角 [第2圈完]"),
        ]

        for tx, ty, label in corners:
            self.move_to(tx, ty, label)

        # 回中心原地轉掃柱子
        self.move_to(0.0, 0.0, "回中心")
        self.get_logger().info("原地轉360°")
        start = self.odom_yaw
        for i in range(1, 9):
            self.rotate_to(self.norm(start + i*math.pi/4))

        time.sleep(1.0)
        self.save_map()

def main():
    rclpy.init()
    node = WROMapper()
    try:
        node.run()
    except KeyboardInterrupt:
        node.stop()
        node.save_map()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
