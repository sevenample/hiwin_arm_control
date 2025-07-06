from PyQt5 import QtWidgets, QtGui, QtCore
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json  # 用來傳送陣列

class OrderPublisher(Node):
    def __init__(self):
        super().__init__('order_publisher')
        self.publisher_ = self.create_publisher(String, 'order_matrix', 10)

    def publish_matrix(self, matrix):
        msg = String()
        msg.data = json.dumps(matrix)  # 轉成字串發送
        self.publisher_.publish(msg)
        self.get_logger().info(f'✅ 已發送訂單陣列：{msg.data}')


class MultiOrderTrayWindow(QtWidgets.QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("智慧裝配 - 訂單與盤子位置輸入")
        self.setGeometry(300, 300, 1000, 700)
        self.ros_node = ros_node

        layout = QtWidgets.QHBoxLayout()
        self.setLayout(layout)

        # 左側圖片
        self.tray_widget = QtWidgets.QLabel()
        pixmap = QtGui.QPixmap("/home/sevenample/ros2_ws/src/Gui/gui_interface/gui_interface/pan.jpg")
        scaled_pixmap = pixmap.scaled(400, 400, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.tray_widget.setPixmap(scaled_pixmap)
        layout.addWidget(self.tray_widget)

        # 右側訂單
        right_layout = QtWidgets.QVBoxLayout()
        self.orders = []
        zones = ["A", "B", "C", "D", "E"]
        items = [
            ("NONE", "無"),
            ("A", "大立方體"),
            ("B", "中立方體"),
            ("C", "小立方體"),
            ("D", "圓柱"),
            ("E", "三角柱"),
            ("F", "六角柱")
        ]

        for i in range(3):
            group_box = QtWidgets.QGroupBox(f"訂單 {i+1}")
            form_layout = QtWidgets.QFormLayout()
            order_inputs = {}
            for zone in zones:
                combo_box = QtWidgets.QComboBox()
                for code, name in items:
                    combo_box.addItem(f"{code} - {name}", code)
                form_layout.addRow(f"區域 {zone}", combo_box)
                order_inputs[zone] = combo_box
            group_box.setLayout(form_layout)
            right_layout.addWidget(group_box)
            self.orders.append(order_inputs)

        self.submit_btn = QtWidgets.QPushButton("✅ 提交並發送")
        self.submit_btn.clicked.connect(self.submit_orders)
        right_layout.addWidget(self.submit_btn)

        self.result_panel = QtWidgets.QTextEdit()
        self.result_panel.setReadOnly(True)
        right_layout.addWidget(self.result_panel)

        layout.addLayout(right_layout)

    def submit_orders(self):
        result = ""
        matrix = []  # 2 x n 陣列
        for idx, order in enumerate(self.orders):
            col = 1
            result += f"🧾 訂單 {idx+1}:\n"
            row = []
            for zone, cb in order.items():
                item = cb.currentData()
                row.append(item)
                result += f"  區域 {zone}: {cb.currentText()}\n"
                if col ==2 :
                    matrix.append(row)
                    row = []
                    col = 1
                else:
                    col +=1 
            row.append('NONE')
            matrix.append(row)
            result += "\n"
        self.result_panel.setText(result)
        # 發送 matrix 給 ROS2
        self.ros_node.publish_matrix(matrix)


def main():
    rclpy.init()
    ros_node = OrderPublisher()

    app = QtWidgets.QApplication(sys.argv)
    win = MultiOrderTrayWindow(ros_node)
    win.show()

    # 建立 ROS spin 執行緒
    from threading import Thread
    def ros_spin():
        rclpy.spin(ros_node)

    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
