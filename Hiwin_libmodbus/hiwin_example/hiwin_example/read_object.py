import rclpy
from rclpy.node import Node
import serial
from hiwin_msgs.msg import CatchArray  # 根據實際套件名稱調整

class ShapeClassifier(Node):
    def __init__(self):
        super().__init__('shape_classifier')
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.publisher_ = self.create_publisher(CatchArray, 'detected_shapes', 10)
        self.timer = self.create_timer(0.1, self.read_serial_data)

        self.shape_map = {
            '大立方體': 'A',
            '中立方體': 'B',
            '小立方體': 'C',
            '圓柱': 'D',
            '三角柱': 'E',
            '六角柱': 'F',
            '無法分類': 'NONE'
        }

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            try:
                adc_str, b1, b2, b3, b4 = line.split(',')
                adc = int(adc_str)
                b1 = int(b1)
                b2 = int(b2)
                b3 = int(b3)
                b4 = int(b4)

                shape_name = self.classify(adc, b1, b2, b3, b4)
                shape_code = self.shape_map.get(shape_name, 'NONE')

                msg = CatchArray()
                msg.items = [shape_code]  # 發送代碼
                self.publisher_.publish(msg)
                self.get_logger().info(f"辨識為: {shape_name} -> {shape_code} (ADC: {adc})")
            except Exception as e:
                self.get_logger().warn(f"解析錯誤: {e} -> '{line}'")

    def classify(self, sensorValue, b1, b2, b3, b4):
        if sensorValue >= 1000:
            if b1 == 0:
                return "大立方體"
            elif b2 == 0:
                return "大長方體"  # 不在對照表內，將視為 NONE
            else:
                return "小長方體"  # 不在對照表內，將視為 NONE
        elif sensorValue >= 870:
            return "六角柱"
        elif 720 <= sensorValue <= 820:
            return "中立方體" if b2 == 0 else "三角柱"
        elif 600 <= sensorValue <= 670:
            if b1 == 0:
                return "六角柱"
            elif b3 == 0:
                return "大長方體"  # 不在表內
            else:
                return "圓柱"
        elif 400 <= sensorValue <= 460:
            return "三角柱"
        elif 250 <= sensorValue <= 370:
            return "小長方體"  # 不在表內
        elif 70 <= sensorValue <= 125:
            return "小立方體"
        elif sensorValue < 30:
            return "小長方體"  # 不在表內
        else:
            return "無法分類"

def main(args=None):
    rclpy.init(args=args)
    node = ShapeClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
