import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan  # 接收雷達數據
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RacePlanner(Node):
    def __init__(self):
        super().__init__('race_planner')
        
        # 1. 宣告 YAML 參數
        self.declare_parameter('max_speed', 1.0)
        
        # 2. 初始化 TF2 監聽器 (骨架感知)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. 建立雷達訂閱者 (感測感知)
        self.scan_sub = self.create_subscription(
            LaserScan, 
            'scan', 
            self.scan_callback, 
            10)
        self.latest_dist = 10.0  # 初始設定為前方無障礙

        # 4. 建立指令發布者
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 縮短計時器時間 (0.1s 檢查一次)，讓避障反應更快
        self.timer = self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        """ 進化版：找出全範圍內最近的障礙物 """
        # 真實雷達數據包含無窮大(inf)或無效值，需要過濾
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        
        if valid_ranges:
            # 取得全周最近距離，而不只是正前方
            self.latest_dist = min(valid_ranges)
        else:
            self.latest_dist = msg.range_max # 沒看到東西就假設是安全的

    def control_loop(self):
        """ 決策大腦：綜合參數、TF 與雷達數據 """
        msg = Twist()
        
        try:
            # 獲取座標轉換 (確認雷達還在車上)
            self.tf_buffer.lookup_transform('base_link', 'laser_frame', rclpy.time.Time())
            
            # 讀取 YAML 速度參數
            base_speed = self.get_parameter('max_speed').value
            
            # --- 避障邏輯判斷 ---
            if self.latest_dist < 1.0:  # 如果障礙物小於 1 公尺
                msg.linear.x = 0.0      # 煞車
                msg.angular.z = 1.0     # 開始原地旋轉找路
                status = "⚠️ 發現障礙！緊急避讓中"
            else:
                msg.linear.x = base_speed # 正常前進
                msg.angular.z = 0.0
                status = "✅ 前方淨空"
            
            self.publisher_.publish(msg)
            self.get_logger().info(f'{status} | 距離: {self.latest_dist:.2f}m | 速度: {msg.linear.x}')

        except TransformException as ex:
            self.get_logger().warn(f'等待 TF 數據中: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = RacePlanner()
    
    try:
        rclpy.spin(node)  # 這裡會持續執行直到被 Ctrl-C
    except KeyboardInterrupt:
        # 當使用者按下 Ctrl-C 時，會跳到這裡執行，而不是直接噴紅字
        node.get_logger().info('收到停止指令，正在關閉賽車大腦...')
    finally:
        # 無論如何都會執行的清理動作
        node.destroy_node()
        rclpy.shutdown()