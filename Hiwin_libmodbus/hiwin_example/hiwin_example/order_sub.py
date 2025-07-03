import rclpy
from rclpy.node import Node
from hiwin_msgs.msg import CatchArray
import threading

Sorting_area_base = [
    ([300.0, 427.0, 185.0, -180.0, 0.00, 90.00], [225.0, 427.0, 185.0, -180.0, 0.00, 90.00],[150.0, 427.0, 185.0, -180.0, 0.00, 90.00],[75.0, 427.0, 185.0, -180.0, 0.00, 90.00]),  # A row
    ([300.0, 566.0, 235.0, -180.0, 0.00, 90.00],[225.0, 566.0, 235.0, -180.0, 0.00, 90.00],[150.0, 566.0, 235.0, -180.0, 0.00, 90.00],[75.0, 566.0, 235.0, -180.0, 0.00, 90.00]), # B row
    ([0.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-75.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-150.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-225.0, 427.0, 185.0, -180.0, 0.00, 90.00]),  # C row
    ([0.0, 566.0, 235.0, -180.0, 0.00, 90.00],[-75.0, 566.0, 235.0, -180.0, 0.00, 90.00],[-150.0, 566.0, 235.0, -180.0, 0.00, 90.00],[-225.0, 566.0, 235.0, -180.0, 0.00, 90.00])   # D row
]
class OrderListener(Node):
    def __init__(self):
        super().__init__('order_listener')
        self.catch_subscription = self.create_subscription(
            CatchArray,
            'catch_list',
            self.catch_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.order_count = []
        self.last_timestamp = None
        self.sort_count_map = {'A': 0, 'B': 0, 'C': 0, 'D': 0}
        self.sort_order_map = {'A': 0, 'B': 1, 'C': 2, 'D': 3}
    def catch_callback(self, msg):
        self.order_count = []
        for j, item in enumerate(self.order_count):
            row = self.sort_order_map[item]
            col = self.sort_count_map[item]


            # 計算位置（加上列的基礎座標 + 欄位間隔）
            # x,y,z,rx,ry,rz = Sorting_area_base[row]
            # x = x - col * 75.0

            x,y,z,rx,ry,rz = Sorting_area_base[row][col]

            if j == 0:
                x -= 75.0
                print("👉 第一個物體：夾具偏移 (x - 50)")
            elif j == 2:
                x += 75.0
                print("🔁 第三個物體：夾具偏移 (x + 50)")
            self.sort_count_map[item] += 1
            self.Order_palce.append([x,y,z,rx,ry,rz])
            self.Order_palce_DOWN.append([x,y,z-50,rx,ry,rz])


def main(args=None):
    rclpy.init(args=args)
    stratery = OrderListener()
    rclpy.spin(stratery)
    rclpy.spin(stratery)
    
    rclpy.shutdown()
