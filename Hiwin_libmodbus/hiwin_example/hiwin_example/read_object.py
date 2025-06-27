import rclpy
from rclpy.node import Node
from hiwin_msgs.msg import OrderArray, CatchArray
import serial

class TCRT5000SerialPublisher(Node):
    def __init__(self):
        super().__init__('tcrt5000_serial_publisher')
        self.publisher_ = self.create_publisher(CatchArray, 'catch_list', 10)

        # 初始化 Serial 連接
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # 請根據實際調整埠號
        self.get_logger().info('🔌 Serial 連接成功，開始接收資料')
        self.same_count = 0
        self.prev_labels = ['無', '無', '無']
        self.same_counts = [0, 0, 0]
        self.detected_objects = ['無', '無', '無']  # 確認後的結果
        self.prev_detected_objects = [None, None, None]  # 上次偵測到的物體

        # 定時讀取 Serial
        self.timer = self.create_timer(0.2, self.timer_callback)
    def timer_callback(self):

        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        data_list = [int(x.strip()) for x in line.strip().split('哈')]
        # print(data_list)
        data_matrix = [data_list[i*5:(i+1)*5] for i in range(3)]
        labels = [self.detect_shape(row) for row in data_matrix]
        for i in range(3):
            if labels[i] == self.prev_labels[i]:
                self.same_counts[i] += 1
            else:
                self.same_counts[i] = 1
                self.prev_labels[i] = labels[i]
            if self.same_counts[i] == 5:
                self.same_counts[i] = 0  # 重置計數
                self.detected_objects[i] = str(labels[i])
                print(f"✅ 位置更新: {i+1}")

        
        if self.detected_objects == self.prev_detected_objects:
            print("🔄 偵測結果未變化，跳過發布")
        else:
            print("📦 偵測結果已變化，發布新消息")
            print(f"偵測到的物體: {self.detected_objects}")
            self.prev_detected_objects = self.detected_objects.copy()
            msg = CatchArray()
            msg.items = self.detected_objects
            self.publisher_.publish(msg)
            
        
    # 偵測形狀邏輯
    def detect_shape(self,row):
        if all(row[i] < 500 for i in [0, 1, 2, 3, 4]):
            return "正方體"
        elif all(row[i] < 500 for i in [1, 2, 3]):
            return "圓柱"
        elif all(row[i] < 500 for i in [0, 1, 3]) or all(row[i] < 500 for i in [2, 3, 4]):
            return "長方體"
        elif row[2] < 500:
            return "三角柱"
        else:
            return "無"
def main(args=None):
    rclpy.init(args=args)
    node = TCRT5000SerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()