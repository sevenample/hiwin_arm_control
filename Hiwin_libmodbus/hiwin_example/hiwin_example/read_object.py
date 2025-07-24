import sys
import serial
import rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets, QtCore
from hiwin_msgs.msg import CatchArray

class ShapeClassifier(Node):
    def __init__(self):
        super().__init__('shape_classifier')
        self.ser = serial.Serial('/dev/ttyUSB2', 9600, timeout=1)
        self.publisher_ = self.create_publisher(CatchArray, 'detected_shapes', 10)
        self.timer = self.create_timer(0.1, self.read_serial_data)

        self.latest_label = "等待資料中..."
        self.latest_code = "NONE"

        self.shape_code_map = {
            '大立方體': 'A', '中立方體': 'B', '小立方體': 'C',
            '圓柱': 'D', '三角柱（躺）': 'E', '三角柱（立）': 'E',
            '六角柱（立）': 'F', '六角柱（躺）': 'F',
            '大長方體': 'G', '小長方體': 'G', '小長方體（立）': 'G',
            '小長方體（立19）': 'G', '無法分類': 'NONE'
        }

        self.code_to_chinese = {
            'A': '大立方體', 'B': '中立方體', 'C': '小立方體',
            'D': '圓柱', 'E': '三角柱', 'F': '六角柱',
            'G': '長方體（異常）', 'NONE': '無法分類'
        }

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            try:
                parts = line.split(',')
                if len(parts) != 10:
                    raise ValueError(f"資料長度不符: 預期10個數值，收到{len(parts)} -> '{line}'")

                adc1, adc2, b1, b2, b3, b4, b5, b6, b7, b8 = map(int, parts)
                code1 = self.classify_shape(adc1, b1, b2, b3, b4)
                code2 = self.classify_shape(adc2, b5, b6, b7, b8)

                label1 = self.code_to_chinese.get(code1, '未知')
                label2 = self.code_to_chinese.get(code2, '未知')

                msg = CatchArray()
                msg.items = [code2, code1]
                self.publisher_.publish(msg)

                # 儲存最近結果（GUI 按下按鈕時才顯示）
                self.latest_code = f"{code1}, {code2}"
                self.latest_label = f"{label1} 與 {label2}"

                self.get_logger().info(
                    f"ADC1={adc1}, Buttons=({b1},{b2},{b3},{b4}) -> {code1} ({label1}); "
                    f"ADC2={adc2}, Buttons=({b5},{b6},{b7},{b8}) -> {code2} ({label2})"
                )
            except Exception as e:
                self.get_logger().warn(f"解析錯誤: {e} -> '{line}'")


    def classify_shape(self, adc, b1, b2, b3, b4):
        if adc >= 1000:
            if b1 != 0 and b3 != 0:
                return 'F' # 六角柱
            else:
                return 'A' # 大立方體
            #return 'A' if b1 == 0 else 'G' # 大立方體或長方體
        elif adc >= 870:
            return 'F' # 六角柱
        elif 690 <= adc <= 820:
            return 'B' #if b2 == 0 else 'E' # 中立方體或三角柱
        elif 480 <= adc <= 680:
            if b4 == 0 or b2 == 0 or b3 == 0:
                return 'G' # 長方體（異常）
            # elif b1 == 0 and b3 != 0:
            #     return 'G' # 長方體（異常）
            # elif b2 == 0 and b3 == 0:
            #     return 'F' # 六角柱（躺）
            else:
                return 'D' # 圓柱
            
        elif 370 <= adc <= 460:
             return 'E' # 三角柱
        elif 200 <= adc <= 360:
            return 'G' # 長方體（異常）
        elif 25 <= adc <= 150:                                              
            return 'C' # 小立方體
        elif adc < 25 and (b3 == 0 or b4 == 0):
            return 'G' # 小長方體（異常）
        else:
            return 'NONE'

class ShapeGUI(QtWidgets.QWidget):
    def __init__(self, ros_node: ShapeClassifier):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("物件辨識即時顯示")
        self.resize(600, 300)

        self.layout = QtWidgets.QVBoxLayout()

        self.label = QtWidgets.QLabel("等待資料中...", self)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 64px; color: black;")
        self.layout.addWidget(self.label)

        self.button = QtWidgets.QPushButton("顯示最新辨識結果")
        self.button.setStyleSheet("font-size: 28px; padding: 10px;")
        self.button.clicked.connect(self.display_latest)
        self.layout.addWidget(self.button)

        self.setLayout(self.layout)

    def display_latest(self):
        label = self.ros_node.latest_label
        code = self.ros_node.latest_code
        self.label.setText(f"檢測到：{label} \n（代碼：{code}）")

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    node = ShapeClassifier()
    gui = ShapeGUI(node)

    def ros_spin_once():
        rclpy.spin_once(node, timeout_sec=0)

    timer = QtCore.QTimer()
    timer.timeout.connect(ros_spin_once)
    timer.start(10)

    gui.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
