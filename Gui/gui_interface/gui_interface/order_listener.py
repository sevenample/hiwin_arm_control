import rclpy
from rclpy.node import Node
from hiwin_msgs.msg import OrderArray, CatchArray
import threading
from rclpy.executors import MultiThreadedExecutor

class OrderListener(Node):
    def __init__(self):
        super().__init__('order_listener')
        
        # 訂閱 OrderArray 類型的訊息
        self.order_subscription = self.create_subscription(
            OrderArray,
            'order_list',
            self.order_callback,
            10)
        
        # 訂閱 CatchArray 類型的訊息
        self.catch_subscription = self.create_subscription(
            CatchArray,
            'catch_list',
            self.catch_callback,
            10)
        
        # 初始化統計數據
        self.order_count = []
        self.catch_count = []

    def order_callback(self, msg):
        self.order_count = []  # 清空之前的計數
        for count in msg.quantities:
            self.order_count.append(count)
        print("Order received:", self.order_count)

    def catch_callback(self, msg):
        self.catch_count=msg.items
        print("Catch received:", self.catch_count)

    def start_spin_in_thread(self):
        # 使用 MultiThreadedExecutor 來處理多個回調
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        
        # 開啟執行緒來運行 executor
        spin_thread = threading.Thread(target=executor.spin)
        spin_thread.start()
        spin_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = OrderListener()
    node.start_spin_in_thread()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
