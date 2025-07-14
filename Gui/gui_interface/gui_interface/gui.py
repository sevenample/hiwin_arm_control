from PyQt5 import QtWidgets, QtGui, QtCore
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hiwin_msgs.msg import OrderArray
import json

class OrderPublisher(Node):
    def __init__(self):
        super().__init__('order_publisher')
        self.publisher_ = self.create_publisher(OrderArray, 'order_list', 10)

    def publish_matrix(self, matrix):
        flat_data = [item for pair in matrix for item in pair]
        msg = OrderArray()
        msg.item_names = flat_data
        self.publisher_.publish(msg)
        self.get_logger().info(f'{msg.item_names}')


class MultiOrderTrayWindow(QtWidgets.QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("智慧裝配 - 訂單與盤子位置輸入")
        self.setGeometry(300, 300, 1100, 700)
        self.ros_node = ros_node
        self.warnings_active = False

        self.setStyleSheet("""
        QWidget {
            font-family: -apple-system, "San Francisco", "Helvetica Neue", Helvetica, Arial, sans-serif;
            font-size: 15px;
            background-color: #f9f9f9;
        }
        QTextEdit {
            border: none;
            background: rgba(245, 245, 245, 0.9);
            border-radius: 10px;
            padding: 10px;
        }
        QComboBox {
            border: 1px solid #d0d0d0;
            border-radius: 8px;
            padding: 6px 10px;
            background-color: white;
            min-width: 140px;
            selection-background-color: #e0e0e0;
            font-size: 15px;
        }
        QComboBox QAbstractItemView {
            border: 1px solid #d0d0d0;
            background-color: #f2f2f2;
            border-radius: 10px;
            padding: 4px;
            font-size: 15px;
            selection-background-color: #ddeeff;
            selection-color: black;
            outline: none;
        }
        QComboBox::drop-down {
            border: none;
            width: 30px;
        }
        QComboBox::down-arrow {
            image: url(:/qt-project.org/styles/commonstyle/images/arrowdown-16.png);
            width: 12px;
            height: 12px;
            margin-right: 6px;
        }
        QPushButton {
            background-color: #007AFF;
            color: white;
            border: none;
            border-radius: 8px;
            padding: 10px 20px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #005FCC;
        }
        QGroupBox {
            border: 1px solid #e0e0e0;
            border-radius: 10px;
            margin-top: 10px;
            padding: 10px;
        }
        QGroupBox:title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
        }
        """)

        self.setContentsMargins(20, 20, 20, 20)
        layout = QtWidgets.QHBoxLayout()
        layout.setSpacing(20)
        self.setLayout(layout)

        left_layout = QtWidgets.QVBoxLayout()
        left_layout.setSpacing(15)
        layout.addLayout(left_layout, 1)

        self.summary_panel = QtWidgets.QTextEdit()
        self.summary_panel.setReadOnly(True)
        self.summary_panel.setPlaceholderText("📦 即時物件數量統計")
        self.summary_panel.setMaximumHeight(120)
        left_layout.addWidget(self.summary_panel)

        self.tray_widget = QtWidgets.QLabel()
        pixmap = QtGui.QPixmap("pan.jpg")  # 圖片檔名請確認
        scaled_pixmap = pixmap.scaled(480, 371, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.tray_widget.setPixmap(scaled_pixmap)
        self.tray_widget.setAlignment(QtCore.Qt.AlignCenter)
        self.tray_widget.setScaledContents(True)
        self.tray_widget.setStyleSheet("border-radius: 10px;")
        left_layout.addWidget(self.tray_widget)

        self.result_panel = QtWidgets.QTextEdit()
        self.result_panel.setReadOnly(True)
        self.result_panel.setMinimumHeight(200)
        left_layout.addWidget(self.result_panel)

        right_layout = QtWidgets.QVBoxLayout()
        right_layout.setSpacing(15)
        layout.addLayout(right_layout, 1)

        self.orders = []
        zones = ['A', 'B', 'C', 'D', 'E']
        items = [
            ('NONE', '無'),
            ('A', '大立方體'),
            ('B', '中立方體'),
            ('C', '小立方體'),
            ('D', '圓柱'),
            ('E', '三角柱'),
            ('F', '六角柱')
        ]

        for i in range(3):
            group_box = QtWidgets.QGroupBox(f"訂單 {i+1}")
            form_layout = QtWidgets.QFormLayout()
            order_inputs = {}
            for zone in zones:
                combo_box = QtWidgets.QComboBox()
                for code, name in items:
                    combo_box.addItem(f"{name}", code)  # 顯示不帶 A-
                combo_box.currentIndexChanged.connect(self.update_summary)
                form_layout.addRow(f"區域 {zone}", combo_box)
                order_inputs[zone] = combo_box
            group_box.setLayout(form_layout)
            right_layout.addWidget(group_box)
            self.orders.append(order_inputs)

        self.submit_btn = QtWidgets.QPushButton("✅ 提交")
        self.submit_btn.clicked.connect(self.submit_orders)
        right_layout.addWidget(self.submit_btn)

    def update_summary(self):
        item_counter = {}
        item_names = {
            "A": "大立方體",
            "B": "中立方體",
            "C": "小立方體",
            "D": "圓柱",
            "E": "三角柱",
            "F": "六角柱"
        }

        for order in self.orders:
            for cb in order.values():
                item = cb.currentData()
                if item != 'NONE':
                    item_counter[item] = item_counter.get(item, 0) + 1

        summary_html = "<b>📦 即時物件數量統計：</b><br>"
        warnings = []

        for item_code, count in item_counter.items():
            name = item_names.get(item_code, item_code)
            if count > 3:
                summary_html += f"{name}：{count} 個 <span style='color:red'>⚠️ 超過 3 個</span><br>"
                warnings.append(f"「{name}」超過 3 個！")
            else:
                summary_html += f"{name}：{count} 個<br>"

        self.summary_panel.setHtml(summary_html)

        if warnings:
            self.warnings_active = True
            self.submit_btn.setEnabled(False)
            self.submit_btn.setStyleSheet("background-color: #ccc; color: #666;")
        else:
            self.warnings_active = False
            self.submit_btn.setEnabled(True)
            self.submit_btn.setStyleSheet("background-color: #007AFF; color: white;")

    def submit_orders(self):
        if self.warnings_active:
            return

        result = ""
        matrix = []
        for idx, order in enumerate(self.orders):
            col = 1
            result += f"🧾 訂單 {idx+1}:\n"
            row = []
            for zone, cb in order.items():
                item = cb.currentData()
                row.append(item)
                result += f"  區域 {zone}: {cb.currentText()}\n"
                if col == 2:
                    matrix.append(row)
                    row = []
                    col = 1
                else:
                    col += 1
            row.append('NONE')
            matrix.append(row)
            result += "\n"

        self.result_panel.setText(result)
        self.ros_node.publish_matrix(matrix)


def main():
    rclpy.init()
    ros_node = OrderPublisher()

    app = QtWidgets.QApplication(sys.argv)
    win = MultiOrderTrayWindow(ros_node)
    win.show()

    from threading import Thread
    def ros_spin():
        rclpy.spin(ros_node)

    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
