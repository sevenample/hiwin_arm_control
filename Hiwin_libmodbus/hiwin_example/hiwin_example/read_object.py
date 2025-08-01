#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ShapeClassifier 範例
--------------------
1. 自動掃描 /dev/ttyUSB*，並在斷線後自動重連（SerialManager 類別）。
2. 保留您原本用來除錯的『備註』，全部以 # 註解形式保存在原位置，方便之後開關或調整。
3. 仍採用單執行緒 Qt + rclpy.timer 的方式，讓 GUI 與 ROS2 共存。
"""

import sys
import time
import serial
from serial.tools import list_ports

import rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets, QtCore

from hiwin_msgs.msg import CatchArray


class SerialManager:
    """處理 USB‑序列埠的自動搜尋、開啟與掉線重連。"""

    def __init__(self,
                 baudrate: int = 9600,
                 timeout: float = 1,
                 preferred_vid_pid=None,          # e.g. [(0x10C4, 0xEA60)]
                 preferred_desc_keywords=None):   # e.g. ["CH340", "Silicon Labs"]
        self.baudrate = baudrate
        self.timeout = timeout
        self.preferred_vid_pid = preferred_vid_pid or []
        self.preferred_desc_keywords = preferred_desc_keywords or []
        self.ser = None
        self.open_or_reconnect()

    # ---------- 公開 API ----------
    def readline(self):
        """讀一行，若掉線則自動重連。"""
        while True:
            try:
                return self.ser.readline()
            except serial.SerialException:
                self._log("SerialException ‑ 嘗試重連…")
                self.open_or_reconnect()

    @property
    def in_waiting(self):
        try:
            return self.ser.in_waiting
        except serial.SerialException:
            self._log("SerialException ‑ 嘗試重連…")
            self.open_or_reconnect()
            return 0

    # ---------- 私有方法 ----------
    def open_or_reconnect(self):
        while True:
            port = self._find_port()
            if port is None:
                self._log("找不到符合條件的 /dev/ttyUSB*，1 秒後重試…")
                time.sleep(1)
                continue
            try:
                self.ser = serial.Serial(port, self.baudrate, timeout=self.timeout)
                self._log(f"已連線 {port}")
                return
            except serial.SerialException as e:
                self._log(f"開啟 {port} 失敗：{e}，1 秒後重試…")
                time.sleep(1)

    def _find_port(self):
        """尋找第一個符合條件的 /dev/ttyUSB*，找不到回傳 None。"""
        candidates = []
        for p in list_ports.comports():
            if not p.device.startswith('/dev/ttyUSB'):
                continue
            # 視需要鎖定 VID/PID 或描述字
            if self.preferred_vid_pid and (p.vid, p.pid) not in self.preferred_vid_pid:
                continue
            if self.preferred_desc_keywords and not any(k.lower() in (p.description or '').lower()
                                                        for k in self.preferred_desc_keywords):
                continue
            candidates.append(p.device)
        return sorted(candidates)[0] if candidates else None

    @staticmethod
    def _log(msg):
        print(f"[SerialManager] {msg}")


class ShapeClassifier(Node):
    def __init__(self):
        super().__init__('shape_classifier')

        # ------- ① 自動尋找並連線 USB‑序列埠 -------
        self.serial_mgr = SerialManager(
            baudrate=9600,
            timeout=1,
            # preferred_vid_pid=[(0x2341, 0x0043)],      # ← 可依實際裝置設定 VID/PID
            # preferred_desc_keywords=["CH340"],        # ← 或者用描述字
        )

        # ------- ② ROS2 publisher -------
        self.publisher_ = self.create_publisher(CatchArray, 'detected_shapes', 10)
        self.timer = self.create_timer(0.1, self.read_serial_data)

        # ------- ③ 其他屬性 -------
        self.latest_label = "等待資料中..."
        self.latest_code = "NONE"

        # ----- 傳入/傳出代碼對照表 -----
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

    # ---------------- Serial 讀取 & 分類 ----------------
    def read_serial_data(self):
        if self.serial_mgr.in_waiting == 0:
            return

        line_raw = self.serial_mgr.readline()
        if not line_raw:
            return
        line = line_raw.decode('utf-8', errors='replace').strip()

        try:
            parts = line.split(',')
            if len(parts) != 10:
                # ↓↓↓↓↓↓ 這段備註留著方便日後查看長度錯誤 ↓↓↓↓↓↓
                raise ValueError(f"資料長度不符: 預期10個數值，收到{len(parts)} -> '{line}'")

            adc1, adc2, b1, b2, b3, b4, b5, b6, b7, b8 = map(int, parts)
            code1 = self.classify_shape(adc1, b1, b2, b3, b4)
            code2 = self.classify_shape(adc2, b5, b6, b7, b8)

            label1 = self.code_to_chinese.get(code1, '未知')
            label2 = self.code_to_chinese.get(code2, '未知')

            msg = CatchArray()
            msg.items = [code2, code1]  # 注意順序
            self.publisher_.publish(msg)

            # 儲存最近結果（GUI 按下按鈕時才顯示）
            self.latest_code = f"{code1}, {code2}"
            self.latest_label = f"{label1} 與 {label2}"

            # 仍保留原本詳細 log，方便對照 ADC 與按鈕狀態
            self.get_logger().info(
                f"ADC1={adc1}, Buttons=({b1},{b2},{b3},{b4}) -> {code1} ({label1}); "
                f"ADC2={adc2}, Buttons=({b5},{b6},{b7},{b8}) -> {code2} ({label2})"
            )
        except Exception as e:
            # 保留除錯訊息
            self.get_logger().warn(f"解析錯誤: {e} -> '{line}'")

    # ---- 分類規則 ----
    # 以下留存您原本的備註（含被註解掉的邏輯），日後可隨時開啟/調整
    def classify_shape(self, adc, b1, b2, b3, b4):
        if adc >= 970:
            if b1 != 0 and b3 != 0:
                return 'F'  # 六角柱
            else:
                return 'A'  # 大立方體
            # return 'A' if b1 == 0 else 'G'  # 大立方體或長方體
        elif adc >= 850:
            return 'F'  # 六角柱
        elif 690 <= adc <= 820:
            return 'B'  # if b2 == 0 else 'E'  # 中立方體或三角柱
        elif 480 <= adc <= 680:
            if b4 == 0 or b2 == 0 or b3 == 0:
                return 'G'  # 長方體（異常）
            # elif b1 == 0 and b3 != 0:
            #     return 'G'  # 長方體（異常）
            # elif b2 == 0 and b3 == 0:
            #     return 'F'  # 六角柱（躺）
            else:
                return 'D'  # 圓柱
        elif 370 <= adc <= 460:
            return 'E'  # 三角柱
        elif 200 <= adc <= 360:
            if b2 == 0 and adc > 330:
                return 'E'
            else:
                return 'G'  # 長方體（異常）
        elif 25 <= adc <= 150:
            return 'C'  # 小立方體
        elif adc < 25 and (b3 == 0 or b4 == 0):
            return 'G'  # 小長方體（異常）
        else:
            return 'NONE'


# ------------------------- GUI -------------------------
class ShapeGUI(QtWidgets.QWidget):
    def __init__(self, ros_node: ShapeClassifier):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("物件辨識即時顯示")
        self.resize(600, 300)

        layout = QtWidgets.QVBoxLayout(self)

        self.label = QtWidgets.QLabel("等待資料中...", self)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 64px; color: black;")
        layout.addWidget(self.label)

        btn = QtWidgets.QPushButton("顯示最新辨識結果")
        btn.setStyleSheet("font-size: 28px; padding: 10px;")
        btn.clicked.connect(self.display_latest)
        layout.addWidget(btn)

    def display_latest(self):
        self.label.setText(
            f"檢測到：{self.ros_node.latest_label}\n（代碼：{self.ros_node.latest_code}）")


# ------------------------- main -------------------------

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    node = ShapeClassifier()
    gui = ShapeGUI(node)

    # Qt timer ‑> spin_once 讓 ROS 與 GUI 共用同一執行緒
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10)

    gui.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
