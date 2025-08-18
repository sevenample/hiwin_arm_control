from PyQt5 import QtWidgets, QtGui, QtCore
import sys
import rclpy
from rclpy.node import Node
from hiwin_msgs.msg import OrderArray
import json
from pathlib import Path

ITEMS = [
    ('NONE', '無'),
    ('A', '大立方體'),
    ('B', '中立方體'),
    ('C', '小立方體'),
    ('D', '圓柱'),
    ('E', '三角柱'),
    ('F', '六角柱')
]

# 建議固定一個區域順序，避免 dict 迭代順序造成不一致
ZONE_ORDER = ['A', 'B', 'C', 'D', 'E', 'F']

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
    def __init__(self, update_callback, tray_count=3):
        super().__init__()
        self.combo_boxes = []
        self.tray_count = tray_count  # 允許外部指定托盤數
        self.combo_width = 90
        self.combo_height = 30
        self.update_callback = update_callback
        self.init_combos()

    def init_combos(self):
        self.combo_boxes.clear()
        for tray_index in range(self.tray_count):
            y_offset = tray_index * 300
            tray_combos = {}
            # 依固定順序建立
            for zone in ZONE_ORDER:
                rect = ZONE_POSITIONS[zone]
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
                combo.setMinimumContentsLength(4)
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
        painter.drawRoundedRect(QtCore.QRectF(20, 20 + y_offset, 400, 280), 15, 15)
        painter.drawRoundedRect(QtCore.QRectF(50, 40 + y_offset, 90, 90), 20, 20)
        painter.drawEllipse(QtCore.QPointF(230, 90 + y_offset), 50, 50)
        painter.drawRoundedRect(QtCore.QRectF(310, 40 + y_offset, 90, 90), 20, 20)
        painter.drawRoundedRect(QtCore.QRectF(40, 180 + y_offset, 240, 90), 20, 20)
        painter.drawRoundedRect(QtCore.QRectF(310, 180 + y_offset, 90, 90), 20, 20)

class MultiOrderTrayWindow(QtWidgets.QWidget):
    def __init__(self, ros_node):
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

        # 左側
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

        # 右側
        right_layout = QtWidgets.QVBoxLayout()
        layout.addLayout(right_layout, 1)

        # 滾動容器
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)

        # 托盤區（可改 tray_count）
        self.tray_zone_area = TrayComboOverlay(self._on_combo_changed, tray_count=3)

        tray_container = QtWidgets.QWidget()
        tray_layout = QtWidgets.QVBoxLayout(tray_container)
        tray_layout.setContentsMargins(0, 0, 0, 0)
        tray_layout.addWidget(self.tray_zone_area)
        scroll_area.setWidget(tray_container)
        right_layout.addWidget(scroll_area)

        # 右下角按鈕列
        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addStretch()
        self.submit_btn = QtWidgets.QPushButton("✅ 提交")
        self.submit_btn.setFixedWidth(430)
        self.submit_btn.clicked.connect(self.submit_orders)
        btn_layout.addWidget(self.submit_btn)
        right_layout.addLayout(btn_layout)

        # 啟動→先讀檔、套回 UI
        self._load_orders()
        # 再跑一次 summary（會做檢查與按鈕 enable/disable）
        self.update_summary()

    # ====== 路徑、序列化 ======
    def _orders_path(self) -> Path:
        cfg = Path.home() / ".config" / "hiwin_example"
        try:
            cfg.mkdir(parents=True, exist_ok=True)
        except Exception:
            cfg = Path.cwd()
        return cfg / "orders.json"

    def _orders_from_ui(self):
        """把 UI 目前狀態轉成我們要儲存/發佈的資料結構。"""
        # 產生你原本 publish 時用的 matrix（每兩個一組）
        matrix = []
        row = []
        for tray in self.tray_zone_area.combo_boxes:
            for zone in ZONE_ORDER:
                cb = tray[zone]
                row.append(cb.currentData())
                if len(row) == 2:
                    matrix.append(row)
                    row = []
        if row:  # 萬一最後不是偶數，兜一組（理論上不會發生）
            matrix.append(row + ['NONE']*(2-len(row)))

        # 同時也存成對應每個托盤各區域的 mapping（下次載入可還原 UI）
        trays = []
        for tray in self.tray_zone_area.combo_boxes:
            tray_map = {zone: tray[zone].currentData() for zone in ZONE_ORDER}
            trays.append(tray_map)

        return {"matrix": matrix, "trays": trays, "zones": ZONE_ORDER}

    def _apply_orders_to_ui(self, trays_payload):
        """把載入的 trays（list of dict）套回 UI。"""
        if not isinstance(trays_payload, list):
            return
        for t_idx, tray_map in enumerate(trays_payload):
            if t_idx >= len(self.tray_zone_area.combo_boxes):
                break
            tray = self.tray_zone_area.combo_boxes[t_idx]
            if not isinstance(tray_map, dict):
                continue
            for zone in ZONE_ORDER:
                if zone in tray_map:
                    code = tray_map[zone]
                    cb = tray[zone]
                    # 找到 data == code 的 index
                    idx = cb.findData(code)
                    if idx >= 0:
                        # 阻止觸發多次自動存檔：先暫時斷訊號
                        try:
                            cb.blockSignals(True)
                            cb.setCurrentIndex(idx)
                        finally:
                            cb.blockSignals(False)

    def _save_orders(self):
        try:
            payload = self._orders_from_ui()
            self._orders_path().write_text(json.dumps(payload, ensure_ascii=False, indent=2))
        except Exception as e:
            # 失敗就略過，不影響操作
            print(f"[orders] save failed: {e}")

    def _load_orders(self):
        p = self._orders_path()
        if not p.exists():
            print(f"[orders] no previous orders file: {p}")
            return
        try:
            data = json.loads(p.read_text())
            trays = data.get("trays")
            self._apply_orders_to_ui(trays)
        except Exception as e:
            print(f"[orders] load failed: {e}")

    # ====== 事件處理 ======
    def _on_combo_changed(self):
        # 任何 combo 變更：更新摘要 + 自動存檔
        self.update_summary()
        self._save_orders()

    def update_summary(self):
        o = 0
        counter = {}
        for tray in self.tray_zone_area.combo_boxes:
            for zone in ZONE_ORDER:
                cb = tray[zone]
                val = cb.currentData()
                if val != "NONE":
                    counter[val] = counter.get(val, 0) + 1
                    o += 1

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

    def _matrix_for_publish(self):
        # 與你原先 submit_orders 裡的 matrix 產生規則一致（每兩個一組）
        matrix = []
        row = []
        for tray in self.tray_zone_area.combo_boxes:
            for zone in ZONE_ORDER:
                val = tray[zone].currentData()
                row.append(val)
                if len(row) == 2:
                    matrix.append(row)
                    row = []
        if row:
            matrix.append(row + ['NONE']*(2-len(row)))
        return matrix

    def submit_orders(self):
        result = ""
        matrix = []
        row = []
        for idx, tray in enumerate(self.tray_zone_area.combo_boxes):
            result += f"🧾 訂單 {idx + 1}:\n"
            for zone in ZONE_ORDER:
                cb = tray[zone]
                name = cb.currentText()
                val = cb.currentData()
                row.append(val)
                result += f"  區域 {zone}: {name}\n"
                if len(row) == 2:
                    matrix.append(row)
                    row = []
            if row:
                matrix.append(row)
                row = []
            result += "\n"

        self.result_panel.setText(result)
        # 送 ROS topic
        self.ros_node.publish_matrix(matrix)
        # 再存一次（雙保險）
        self._save_orders()

    def closeEvent(self, event):
        # 視窗關閉時保存（第三道保險）
        self._save_orders()
        super().closeEvent(event)

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