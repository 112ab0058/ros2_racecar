from example_interfaces.srv import AddTwoInts # 導入服務格式：兩個整數輸入，一個總和輸出
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        # 建立服務，名稱為 'add_two_ints'
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b # 執行運算
        self.get_logger().info(f'收到請求: a={request.a}, b={request.b} -> 回傳和: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()