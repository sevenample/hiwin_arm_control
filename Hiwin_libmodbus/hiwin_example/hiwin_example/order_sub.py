import rclpy
from rclpy.node import Node
from hiwin_msgs.msg import OrderArray
import threading

class OrderListener(Node):
    def __init__(self):
        super().__init__('order_listener')
        self.subscription = self.create_subscription(
            OrderArray,
            'order_list',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.order_count = []
        self.last_timestamp = None

    def listener_callback(self, msg):
        self.order_count = []
        now = self.get_clock().now().nanoseconds
        if self.last_timestamp is None or now - self.last_timestamp > 500_000_000:
            self.order_count = [1]  # reset count for new submission
        else:
            self.order_count.append(1)

        self.last_timestamp = now

        for count in msg.quantities:
            self.order_count.append(count)
        print(self.order_count)

    def start_spin_in_thread(self):
        # 在獨立的線程中啟動 spin
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,))
        spin_thread.daemon = True  # 當主程式結束時，線程會自動結束
        spin_thread.start()
        spin_thread.join()  # 確保 spin 完成後再結束主程式
