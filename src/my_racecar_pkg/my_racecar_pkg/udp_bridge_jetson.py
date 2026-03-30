import rclpy, socket, json, threading
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class JetsonUDPBridge(Node):
    def __init__(self):
        super().__init__('jetson_udp_bridge')
        self.pc_ip = '192.168.0.181'
        self.pc_port = 6666
        self.jetson_port = 6665   # ✅ 改這裡

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(1.0)
        self.sock.bind(('0.0.0.0', self.jetson_port))
        self.sock_lock = threading.Lock()

        self.pub1 = self.create_publisher(Point, 't1_pos_raw', 10)
        self.pub2 = self.create_publisher(Point, 't2_pos_raw', 10)
        self.create_subscription(Twist, 'turtle2/cmd_vel', self.cmd_cb, 10)

        threading.Thread(target=self.receive_loop, daemon=True).start()
        self.get_logger().info(f"🚀 Jetson UDP 橋接器已啟動，監聽 Port {self.jetson_port}")

    def cmd_cb(self, msg):
        data = json.dumps({"type": "cmd_vel", "linear": msg.linear.x, "angular": msg.angular.z}).encode('utf-8')
        with self.sock_lock:
            try:
                self.sock.sendto(data, (self.pc_ip, self.pc_port))
            except Exception as e:
                self.get_logger().error(f"發送失敗: {e}")

    def receive_loop(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))
                if msg.get("type") == "pose":
                    p = Point(x=float(msg["x"]), y=float(msg["y"]), z=0.0)
                    if msg["name"] == "t1": self.pub1.publish(p)
                    elif msg["name"] == "t2": self.pub2.publish(p)
            except socket.timeout:
                continue
            except json.JSONDecodeError as e:
                self.get_logger().warn(f"JSON 解析失敗: {e}")
            except Exception as e:
                self.get_logger().warn(f"receive_loop 錯誤: {e}")

def main():
    rclpy.init()
    rclpy.spin(JetsonUDPBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
