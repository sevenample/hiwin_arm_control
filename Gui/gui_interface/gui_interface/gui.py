from PyQt5 import QtWidgets, QtGui, QtCore
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hiwin_msgs.msg import OrderArray
import json


ITEMS = [
    ('NONE', '無'),
    ('A', '大立方體'),
    ('B', '中立方體'),
    ('C', '小立方體'),
    ('D', '圓柱'),
    ('E', '三角柱'),
    ('F', '六角柱')
]

ZONE_POSITIONS = {
    'A': QtCore.QRectF(290, 160, 130, 130),
    'B': QtCore.QRectF(290, 25, 130, 130),
    'C': QtCore.QRectF(170, 30, 120, 120),
    'D': QtCore.QRectF(30, 25, 130, 130),
    'E': QtCore.QRectF(160, 160, 130, 130),
    'F': QtCore.QRectF(30, 160, 130, 130),
}


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

class TrayComboOverlay(QtWidgets.QWidget):
    def __init__(self, update_callback):
        super().__init__()
        self.combo_boxes = []
        self.tray_count = 3
        self.combo_width = 90
        self.combo_height = 30
        self.update_callback = update_callback
        self.init_combos()

    def init_combos(self):
        self.combo_boxes.clear()
        
        for tray_index in range(self.tray_count):
            y_offset = tray_index * 300
            tray_combos = {}
            for zone, rect in ZONE_POSITIONS.items():
                combo = QtWidgets.QComboBox(self)
                for code, name in ITEMS:
                    combo.addItem(name, code)
                combo.setGeometry(
                    int(rect.x() + (rect.width() - self.combo_width) / 2),
                    int(rect.y() + y_offset + (rect.height() - self.combo_height) / 2),
                    self.combo_width,
                    self.combo_height
                )
                combo.currentIndexChanged.connect(self.update_callback)
                combo.setMinimumContentsLength(4)   # 最少顯示幾個字元長度
                combo.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
                tray_combos[zone] = combo
            self.combo_boxes.append(tray_combos)



    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtGui.QPen(QtCore.Qt.black, 2))
        painter.setBrush(QtGui.QBrush(QtCore.Qt.lightGray))
        for i in range(self.tray_count):
            self.draw_tray(painter, i * 300)

    def draw_tray(self, painter, y_offset):
        painter.drawRoundedRect(QtCore.QRectF(20, 20 + y_offset, 400,280), 15, 15)
        painter.drawRoundedRect(QtCore.QRectF(50, 40 + y_offset, 90, 90), 20, 20)
        painter.drawEllipse(QtCore.QPointF(230, 90 + y_offset), 50, 50)
        painter.drawRoundedRect(QtCore.QRectF(310, 40 + y_offset, 90, 90), 20, 20)
        painter.drawRoundedRect(QtCore.QRectF(40, 180 + y_offset, 240, 90), 20, 20)
        painter.drawRoundedRect(QtCore.QRectF(310, 180 + y_offset, 90, 90), 20, 20)


class MultiOrderTrayWindow(QtWidgets.QWidget):
    def __init__(self,ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("智慧裝配 - 視覺化訂單輸入")
        self.setGeometry(300, 100, 800, 1000)

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
            font-size: 18px;                   
        }
        QComboBox {
            border: 1px solid #d0d0d0;
            border-radius: 8px;
            padding: 6px 10px;
            background-color: white;
            min-width: 0px;
            selection-background-color: #e0e0e0;
            font-size: 12px;
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
        """)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setSpacing(20)

        # 左邊資訊區
        left_layout = QtWidgets.QVBoxLayout()
        layout.addLayout(left_layout, 1)

        self.summary_panel = QtWidgets.QTextEdit()
        self.summary_panel.setReadOnly(True)
        self.summary_panel.setPlaceholderText("📦 即時物件數量統計")
        self.summary_panel.setMaximumHeight(280)
        left_layout.addWidget(self.summary_panel)



        self.result_panel = QtWidgets.QTextEdit()
        self.result_panel.setReadOnly(True)
        self.result_panel.setMinimumHeight(200)
        left_layout.addWidget(self.result_panel)

        # 右側為餐盤 + 按鈕
        # 右側為餐盤 + 按鈕
        right_layout = QtWidgets.QVBoxLayout()
        layout.addLayout(right_layout, 1)

        # 滾動容器區
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)

        self.tray_zone_area = TrayComboOverlay(self.update_summary)

        # 包裝 tray_zone_area 進 QWidget 才能設 QVBoxLayout
        tray_container = QtWidgets.QWidget()
        tray_layout = QtWidgets.QVBoxLayout(tray_container)
        tray_layout.setContentsMargins(0, 0, 0, 0)
        tray_layout.addWidget(self.tray_zone_area)
        scroll_area.setWidget(tray_container)

        right_layout.addWidget(scroll_area)

        # 右下角提交按鈕
        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addStretch()
        self.submit_btn = QtWidgets.QPushButton("✅ 提交")
        self.submit_btn.setFixedWidth(430)
        self.submit_btn.clicked.connect(self.submit_orders)
        btn_layout.addWidget(self.submit_btn)
        right_layout.addLayout(btn_layout)


        self.update_summary()

    def update_summary(self):
        o = 0
        counter = {}
        for tray in self.tray_zone_area.combo_boxes:
            for cb in tray.values():
                val = cb.currentData()
                if val != "NONE":
                    counter[val] = counter.get(val, 0) + 1
                    o +=1

        item_names = {code: name for code, name in ITEMS if code != "NONE"}
        summary_html = "<b>📦 即時物件數量統計：</b><br>"
        summary_html += f"總共：{o} 個<br>"

        warnings = []

        for code, name in item_names.items():
            count = counter.get(code, 0)
            if count > 3:
                summary_html += f"{name}：{count} 個 <span style='color:red'>⚠️ 超過 3 個</span><br>"
                warnings.append(name)
            else:
                summary_html += f"{name}：{count} 個<br>"

        self.summary_panel.setHtml(summary_html)

        if warnings:
            self.submit_btn.setEnabled(False)
            self.submit_btn.setStyleSheet("background-color: #ccc; color: #666;")
        else:
            self.submit_btn.setEnabled(True)
            self.submit_btn.setStyleSheet("background-color: #007AFF; color: white;")

    def submit_orders(self):
        result = ""
        matrix = []
        for idx, tray in enumerate(self.tray_zone_area.combo_boxes):
            result += f"🧾 訂單 {idx + 1}:\n"
            row = []
            col = 1
            for zone, cb in tray.items():
                name = cb.currentText()
                val = cb.currentData()
                row.append(val)
                result += f"  區域 {zone}: {name}\n"
                if col == 2:
                    matrix.append(row)
                    row = []
                    col = 1
                else:
                    col += 1
            if row:
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
