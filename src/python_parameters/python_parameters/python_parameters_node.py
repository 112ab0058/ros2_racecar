import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        # 宣告參數，名稱為 'my_parameter'，預設值為 'world'
        self.declare_parameter('my_parameter', 'world')
        # 每秒執行一次回呼函數
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        # 取得目前的參數值
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info('Hello %s!' % my_param)

        # (選擇性) 這裡示範如何在程式碼內部把參數設回 'world'
        # 但在賽車專案中，我們通常會讓參數保持由外部控制
        new_param = rclpy.parameter.Parameter('my_parameter', rclpy.Parameter.Type.STRING, 'world')
        self.set_parameters([new_param])

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
