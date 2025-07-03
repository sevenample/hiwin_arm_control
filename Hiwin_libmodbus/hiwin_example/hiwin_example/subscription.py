import rclpy
from rclpy.node import Node
from hiwin_msgs.msg import OrderArray, catchArray
import threading

class OrderListener(Node):
    def __init__(self):
        super().__init__('order_listener')
        
        # 訂閱 OrderArray 類型的訊息
        self.order_subscription = self.create_subscription(
            OrderArray,
            'order_list',
            self.order_callback,
            10)
        
        # 訂閱 catchArray 類型的訊息
        self.catch_subscription = self.create_subscription(
            catchArray,
            'catch_list',
            self.catch_callback,
            10)
        
        # 初始化統計數據
        self.order_count = []
        self.catch_count = []

    def order_callback(self, msg):
        for count in msg.quantities:
            self.order_count.append(count)
        print("Order received:", self.order_count)

    def catch_callback(self, msg):
        for count in msg.quantities:
            self.catch_count.append(count)
        print("Catch received:", self.catch_count)

    def start_spin_in_thread(self):
        # 在不同的線程中啟動兩個訂閱者的 spin
        order_thread = threading.Thread(target=self.spin_order)
        catch_thread = threading.Thread(target=self.spin_catch)
        
        order_thread.start()
        catch_thread.start()
        order_thread.join()  # 等待訂單處理線程完成
        catch_thread.join()  # 等待夾取處理線程完成
        return order_thread, catch_thread

    def spin_order(self):
        """用於處理訂單訊息的spin"""
        while rclpy.ok():
            rclpy.spin_once(self)  # 只處理訂單訊息

    def spin_catch(self):
        """用於處理夾取訊息的spin"""
        while rclpy.ok():
            rclpy.spin_once(self)  # 只處理夾取訊息

def main(args=None):
    rclpy.init(args=args)
    node = OrderListener()
    # 啟動兩個獨立的線程
    node.start_spin_in_thread()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
