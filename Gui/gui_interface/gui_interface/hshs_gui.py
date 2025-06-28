from PyQt5 import QtWidgets, QtCore
import sys
import rclpy
from rclpy.node import Node
from hiwin_msgs.msg import OrderArray


class ROSPublisher(Node):
    def __init__(self):
        super().__init__('order_gui_node')
        self.publisher_ = self.create_publisher(OrderArray, 'order_list', 10)

    def publish_order(self, order_data):
        msg = OrderArray()
        msg.item_names = list(order_data.keys())
        msg.quantities = list(order_data.values())
        self.publisher_.publish(msg)
        self.get_logger().info(f'📤 發送：{msg.item_names} - {msg.quantities}')


class OrderForm(QtWidgets.QGroupBox):
    def __init__(self, index, remove_callback):
        super().__init__(f"訂單 {index}")
        self.inputs = {}
        self.remove_callback = remove_callback

        layout = QtWidgets.QVBoxLayout()

        # 刪除按鈕（置於右上角）
        close_btn = QtWidgets.QPushButton("❌ 刪除")
        close_btn.setFixedWidth(60)
        close_btn.clicked.connect(self._on_remove)
        close_layout = QtWidgets.QHBoxLayout()
        close_layout.addStretch()
        close_layout.addWidget(close_btn)
        layout.addLayout(close_layout)

        for name in ["大正方體","中正方體","小正方體", "六邊柱", "三角柱", "圓柱體"]:
            h = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(f"{name} 數量：")
            spin = QtWidgets.QSpinBox()
            spin.setRange(0, 1000)
            self.inputs[name] = spin
            h.addWidget(label)
            h.addWidget(spin)
            layout.addLayout(h)

        self.setLayout(layout)

    def _on_remove(self):
        if self.remove_callback:
            self.remove_callback(self)

    def get_data(self):
        return {name: box.value() for name, box in self.inputs.items()}


class MultiOrderWindow(QtWidgets.QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("智慧裝配 - 多筆訂單輸入")
        self.setGeometry(300, 300, 900, 500)

        self.ros_node = ros_node
        self.order_count = 0
        self.orders = []

        main_layout = QtWidgets.QHBoxLayout(self)

        # 左側
        left_layout = QtWidgets.QVBoxLayout()
        self.scroll_area = QtWidgets.QScrollArea()
        self.scroll_widget = QtWidgets.QWidget()
        self.order_layout = QtWidgets.QVBoxLayout()
        self.scroll_widget.setLayout(self.order_layout)
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.scroll_widget)
        left_layout.addWidget(self.scroll_area)

        btn_layout = QtWidgets.QHBoxLayout()
        add_btn = QtWidgets.QPushButton("➕ 新增訂單")
        add_btn.clicked.connect(self.add_order)
        submit_btn = QtWidgets.QPushButton("✅ 提交全部訂單")
        submit_btn.clicked.connect(self.submit_all_orders)
        btn_layout.addWidget(add_btn)
        btn_layout.addWidget(submit_btn)
        left_layout.addLayout(btn_layout)

        left_container = QtWidgets.QWidget()
        left_container.setLayout(left_layout)
        left_container.setMinimumWidth(450)
        main_layout.addWidget(left_container)

        # 右側
        self.right_panel = QtWidgets.QTextEdit()
        self.right_panel.setReadOnly(True)
        self.right_panel.setStyleSheet("background-color: #f4f4f4; font-family: Courier;")
        main_layout.addWidget(self.right_panel)

        self.add_order()

    def add_order(self):
        self.order_count += 1
        form = OrderForm(self.order_count, self.remove_order)
        self.orders.append(form)
        self.order_layout.addWidget(form)
        self.renumber_orders()

    def remove_order(self, form):
        self.order_layout.removeWidget(form)
        form.deleteLater()
        self.orders.remove(form)
        self.renumber_orders()

    def renumber_orders(self):
        for i, form in enumerate(self.orders):
            form.setTitle(f"訂單 {i + 1}")

    def submit_all_orders(self):
        text = "📦 所有訂單內容如下：\n"
        for i, order in enumerate(self.orders):
            data = order.get_data()
            if all(v == 0 for v in data.values()):
                continue  # 跳過全為 0 的訂單
            text += f"\n🧾 訂單 {i + 1}：\n"
            for k, v in data.items():
                text += f"  - {k}：{v} 個\n"
            self.ros_node.publish_order(data)
        self.right_panel.setText(text)


def main():
    rclpy.init()
    node = ROSPublisher()
    app = QtWidgets.QApplication(sys.argv)
    win = MultiOrderWindow(node)
    win.show()
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(100)
    sys.exit(app.exec_())
    node.destroy_node()
    rclpy.shutdown()
