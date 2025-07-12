from PyQt5 import QtWidgets, QtGui, QtCore
import sys

class MultiOrderTrayWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("智慧裝配 - 訂單與盤子位置輸入")
        self.setGeometry(300, 300, 1100, 700)
        self.warnings_active = False  # 控制提交按鈕啟用

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
                border-radius: 10px;  /* ✅ 額外選配圓角 */
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

        # 左側佔 1/2：圖片 + 統計 + 結果
        left_layout = QtWidgets.QVBoxLayout()
        left_layout.setSpacing(15)
        layout.addLayout(left_layout, 1)  # 1/2 寬

        self.summary_panel = QtWidgets.QTextEdit()
        self.summary_panel.setReadOnly(True)
        self.summary_panel.setPlaceholderText("📦 即時物件數量統計")
        self.summary_panel.setMaximumHeight(120)
        left_layout.addWidget(self.summary_panel)

        self.tray_widget = QtWidgets.QLabel()
        pixmap = QtGui.QPixmap("pan.jpg")  # 請改成你自己的圖檔名稱
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

        # 右側佔 1/2：訂單區 + 提交按鈕
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
                    combo_box.addItem(f"{name}", code)
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

        # 按鈕狀態處理
        if warnings:
            self.warnings_active = True
            self.submit_btn.setEnabled(False)
            self.submit_btn.setStyleSheet("""
                QPushButton {
                    background-color: #ccc;
                    color: #666;
                    border-radius: 8px;
                    padding: 10px 20px;
                    font-weight: bold;
                }
            """)
        else:
            self.warnings_active = False
            self.submit_btn.setEnabled(True)
            self.submit_btn.setStyleSheet("""
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



        if warnings:
            msg = QtWidgets.QMessageBox(self)
            msg.setIcon(QtWidgets.QMessageBox.Warning)
            msg.setWindowTitle("⚠️ 數量過多警告")
            msg.setText("\n".join(warnings))
            msg.setStyleSheet("""
                QMessageBox {
                    font-size: 18px;
                    padding: 20px;
                }
                QPushButton {
                    font-size: 16px;
                    padding: 6px 20px;
                }
            """)
            msg.exec_()

    def submit_orders(self):
        if self.warnings_active:
            return  # 停止提交，避免執行
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
        print("提交結果 matrix：", matrix)


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MultiOrderTrayWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
