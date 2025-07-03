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

