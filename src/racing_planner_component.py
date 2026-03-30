import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RacingPlannerComponent(Node):
    def __init__(self, options=None):
        # 組件化節點必須支援 options 參數
        super().__init__('racing_planner', parameter_overrides=options)
        self.subscription = self.create_subscription(
            String, 'sensor_data', self.listener_callback, 10)
        self.get_logger().info('賽車路徑規劃組件已啟動！')

    def listener_callback(self, msg):
        self.get_logger().info(f'收到感測器數據: "{msg.data}" -> 正在運算避障路徑...')

# 注意：組件通常由 Manager 載入，不需要 main()，但為了測試可以保留