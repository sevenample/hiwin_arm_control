from PyQt5 import QtWidgets, QtGui, QtCore
import sys

class MultiOrderTrayWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("智慧裝配 - 訂單與盤子位置輸入")
        self.setGeometry(300, 300, 1000, 700)

        layout = QtWidgets.QHBoxLayout()
        self.setLayout(layout)

        # 左側：顯示圖片，等比例縮小
        self.tray_widget = QtWidgets.QLabel()
        pixmap = QtGui.QPixmap("pan.jpg")
        scaled_pixmap = pixmap.scaled(400, 400, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.tray_widget.setPixmap(scaled_pixmap)
        layout.addWidget(self.tray_widget)

        # 右側：三個訂單，每個訂單固定ABCDE區輸入 + 顯示對應編號和物品名稱
        right_layout = QtWidgets.QVBoxLayout()
        self.orders = []
        zones = ["A", "B", "C", "D", "E"]
        items = [
            ("0", "無"),
            ("1", "大立方體"),
            ("2", "中立方體"),
            ("3", "小立方體"),
            ("4", "圓柱"),
            ("5", "三角柱"),
            ("6", "六角柱")
        ]

        for i in range(3):
            group_box = QtWidgets.QGroupBox(f"訂單 {i+1}")
            form_layout = QtWidgets.QFormLayout()
            order_inputs = {}
            for zone in zones:
                combo_box = QtWidgets.QComboBox()
                for number, name in items:
                    combo_box.addItem(f"{number} - {name}", number)
                form_layout.addRow(f"區域 {zone}", combo_box)
                order_inputs[zone] = combo_box
            group_box.setLayout(form_layout)
            right_layout.addWidget(group_box)
            self.orders.append(order_inputs)

        self.submit_btn = QtWidgets.QPushButton("✅ 提交")
        self.submit_btn.clicked.connect(self.submit_orders)
        right_layout.addWidget(self.submit_btn)

        self.result_panel = QtWidgets.QTextEdit()
        self.result_panel.setReadOnly(True)
        right_layout.addWidget(self.result_panel)

        layout.addLayout(right_layout)

    def submit_orders(self):
        result = ""
        for idx, order in enumerate(self.orders):
            result += f"🧾 訂單 {idx+1}:\n"
            for zone, cb in order.items():
                result += f"  區域 {zone}: {cb.currentText()}\n"
            result += "\n"
        self.result_panel.setText(result)
def main(args=None):
    app = QtWidgets.QApplication(sys.argv)
    win = MultiOrderTrayWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()