#!/usr/bin/env python3
import time
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from rclpy.task import Future
from typing import NamedTuple
from hiwin_msgs.msg import OrderArray, CatchArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from hiwin_interfaces.srv import Motioncmd
from hiwin_interfaces.srv import Digitalcmd
from hiwin_interfaces.srv import Readcmd
import serial
import os
import cv2
import numpy as np
import random
import time
# from YoloDetector import YoloDetectorActionClient

# === 新增：GUI、執行緒鎖、與持久化 ===
import threading
import json
import atexit
from pathlib import Path
try:
    import tkinter as tk
    from tkinter import ttk
except Exception:
    tk = None  # 若無圖形環境，GUI 自動停用

DEFAULT_VELOCITY = 70
DEFAULT_ACCELERATION = 70
LINE_VELOCITY = 100
LINE_ACCELERATION = 100


HOME_POSE = [0.00, 368.00, 293.00, -180.00, 0.00, 90.000]
# 抓取物件數
Number_of_grips = 2


# IO切換
IO = {1:1,
      2:2,
      3:5,
      4:4}
correction = True # 是否開啟校正

# IO接反
# IO = {1:2,
#       2:1,
#       3:4,
#       4:5}

IO_STATE = [Digitalcmd.Request.DIGITAL_OFF,
            Digitalcmd.Request.DIGITAL_ON]

relay_point =[756.0, -52.0,  3.0,  -180.0, 0.00, 89.00]

ERROR_relay_point =[-45.0,  0.0,  3.0,    -180.00, 0.00, 89.00]

FRIST_POSE = [86.0,  -186.0,  240.0,    -180.00, 0.00, 89.00]
x_offset = 0.0
y_offset = -2.0
z_offset = 0.0

Sorting_area_base = [
    ([672.0 + x_offset, 145.0 + y_offset, 123.0 + z_offset, -180.0, 0.00, 89.00],
     [588.0 + x_offset, 145.0 + y_offset, 123.0 + z_offset, -180.0, 0.00, 89.00],
     [504.0 + x_offset, 145.0 + y_offset, 123.0 + z_offset, -180.0, 0.00, 89.00],
     [420.0 + x_offset, 145.0 + y_offset, 123.0 + z_offset, -180.0, 0.00, 89.00]),  # A row

    ([756.0 + x_offset, 40.0 + y_offset, 63.0 + z_offset, -180.0, 0.00, 89.00],
     [672.0 + x_offset, 40.0 + y_offset, 63.0 + z_offset, -180.0, 0.00, 89.00],
     [588.0 + x_offset, 40.0 + y_offset, 63.0 + z_offset, -180.0, 0.00, 89.00],
     [504.0 + x_offset, 40.0 + y_offset, 63.0 + z_offset, -180.0, 0.00, 89.00]),  # B row

    ([840.0 + x_offset, -52.0 + y_offset, 3.0 + z_offset, -180.0, 0.00, 89.00],
     [756.0 + x_offset, -52.0 + y_offset, 3.0 + z_offset, -180.0, 0.00, 89.00],
     [672.0 + x_offset, -52.0 + y_offset, 3.0 + z_offset, -180.0, 0.00, 89.00],
     [588.0 + x_offset, -52.0 + y_offset, 3.0 + z_offset, -180.0, 0.00, 89.00]),  # C row

    ([337.0 + x_offset, 145.0 + y_offset, 123.0 + z_offset, -180.0, 0.00, 89.00],
     [253.0 + x_offset, 145.0 + y_offset, 123.0 + z_offset, -180.0, 0.00, 89.00],
     [169.0 + x_offset, 145.0 + y_offset, 123.0 + z_offset, -180.0, 0.00, 89.00],
     [85.0 + x_offset, 145.0 + y_offset, 123.0 + z_offset, -180.0, 0.00, 89.00]),  # D row

    ([420.0 + x_offset, 40.0 + y_offset, 63.0 + z_offset, -180.0, 0.00, 89.00],
     [337.0 + x_offset, 40.0 + y_offset, 63.0 + z_offset, -180.0, 0.00, 89.00],
     [253.0 + x_offset, 40.0 + y_offset, 63.0 + z_offset, -180.0, 0.00, 89.00],
     [169.0 + x_offset, 40.0 + y_offset, 80.0 + z_offset, -180.0, 0.00, 89.00]),  # E row

    ([504.0 + x_offset, -52.0 + y_offset, 3.0 + z_offset, -180.0, 0.00, 89.00],
     [420.0 + x_offset, -52.0 + y_offset, 3.0 + z_offset, -180.0, 0.00, 89.00],
     [337.0 + x_offset, -52.0 + y_offset, 3.0 + z_offset, -180.0, 0.00, 89.00],
     [253.0 + x_offset, -52.0 + y_offset, 3.0 + z_offset, -180.0, 0.00, 89.00]),  # F row

    ([-45.0 + x_offset, 163.0 + y_offset, 123.0 + z_offset, -180.0, 0.00, 89.00],) * 200  # G row
]

OBJECT_POSES = [    
    ([  86.0 + x_offset,  -186.0 + y_offset,  3.0 + z_offset, -180.00, 0.00, 89.00]),
    ([  86.0 + x_offset,   -88.0 + y_offset,  3.0 + z_offset, -180.00, 0.00, 89.00]),
    ([  86.0 + x_offset,    10.0 + y_offset,  3.0 + z_offset, -180.00, 0.00, 89.00]),
    ([ -82.0 + x_offset,  -184.5 + y_offset,  3.0 + z_offset, -180.00, 0.00, 89.00]),
    ([ -82.0 + x_offset,   -86.5 + y_offset,  3.0 + z_offset, -180.00, 0.00, 89.00]),
    ([ -82.0 + x_offset,    11.5 + y_offset,  3.0 + z_offset, -180.00, 0.00, 89.00]),

    ([-170.0 + x_offset,  -184.5 + y_offset,  3.0 + z_offset, -180.00, 0.00, 89.00]),
    ([-170.0 + x_offset,   -86.5 + y_offset,  3.0 + z_offset, -180.00, 0.00, 89.00]),
    ([-170.0 + x_offset,    11.5 + y_offset,  3.0 + z_offset, -180.00, 0.00, 89.00]),
]


#訂單放置
x_order_offset = 0.0
y_order_offset = 0.0
z_order_offset = 0.0

ORDER_POSES = [
    [ 76.0 + x_order_offset,  417.0 + y_order_offset,  25.7 + z_order_offset, -180.0, 0.0, 0.0],
    [-37.0 + x_order_offset,  420.0 + y_order_offset,  25.7 + z_order_offset, -180.0, 0.0, 89.0],
    [-37.0 + x_order_offset,  332.0 + y_order_offset,  25.7 + z_order_offset, -180.0, 0.0, 89.0],

    [ 76.0 + x_order_offset,  202.0 + y_order_offset,  25.7 + z_order_offset, -180.0, 0.0, 0.0],
    [-37.0 + x_order_offset,  207.0 + y_order_offset,  25.7 + z_order_offset, -180.0, 0.0, 89.0],
    [-37.0 + x_order_offset,  119.0 + y_order_offset,  25.7 + z_order_offset, -180.0, 0.0, 89.0],

    [ 76.0 + x_order_offset,  -13.0 + y_order_offset,  25.7 + z_order_offset, -180.0, 0.0, 0.0],
    [-37.0 + x_order_offset,    4.0 + y_order_offset,  25.7 + z_order_offset, -180.0, 0.0, 89.0],
    [-37.0 + x_order_offset, -103.0 + y_order_offset,  25.7 + z_order_offset, -180.0, 0.0, 89.0],
]

class States(Enum):
    INIT = 0
    HOME_MOVE = 1
    FINISH = 2    
    CLOSE_ROBOT= 3
    CORRECTION = 20
    

    OBJECT_AREA =4
    CATCH_OBJECT=5
    READ_OBJECT = 6
    SORT_AREA = 7
    SORT_PLACE = 8
    F_ERROR = 17
    
    READ_ORDER = 9
    ORDER_OBJECT_AREA =10
    ORDER_OBJECT_PICK = 11
    ORDER_AREA = 12
    ORDER_PLACE = 13
    END_HOME_MOVE = 14

    ERROR_PLACE = 15
    ORDER_RELAY_POINT = 16
    TEST = 100


class ExampleStrategy(Node):

    def __init__(self):
        super().__init__('example_strategy')
        self.base_state = 0


        self.hiwin_client_mo = self.create_client(Motioncmd, 'motioncmd')
        self.hiwin_client_di = self.create_client(Digitalcmd, 'digitalcmd')
        self.hiwin_client_rd = self.create_client(Readcmd, 'readcmd')
        
        self.count_map = {'A': 1, 'B': 1, 'C': 1, 'D': 1, 'E': 1, 'F': 1, 'G': 1,  'H': 1 , 'I': 1}
        self.order_map = {'A': 0, 'B': 1, 'C': 2, 'D': 3, 'E': 4, 'F': 5, 'G': 6 , 'H': 6 , 'I': 6}

        self.catch_items = ['A'] * 3 + ['B'] * 3 + ['C'] * 3 + ['D'] * 3 + ['E'] * 3 + ['F'] * 3 + ['G'] * 3 
        random.shuffle(self.catch_items)

        self.order_area_num = 0
        self.item = ['NONE','NONE']
        self.oder_items = []   
        self.oder_item = []   

        self.Sorting_palce = []
        self.Order_palce = []
        

        self.catch_num = 0
        self.order_catch_palce_num = 0
        self.order_palce_num = 0

        self.same = 0
        self.F = 0
        self.res  = None
        

# ----------------------------------------------------
        self.order_base = 2
        self.base_state = 3
# -------------------------------------------------------


        self.sort_count_map = {'A': 1, 'B': 1, 'C': 1, 'D': 1,'E': 1,'F': 1,}
        # 訂閱 OrderArray 類型的訊息
        self.order_subscription = self.create_subscription(
            OrderArray,
            'order_list',
            self.order_callback,
            10)
        
        # 訂閱 CatchArray 類型的訊息
        self.catch_subscription = self.create_subscription(
            CatchArray,
            'detected_shapes',
            self.catch_callback,
            10)
        
        # 初始化統計數據
        self.catch_count = []

        # ===================== 新增：雙 Base 偏移控制 =====================
        # Base 3（分檢/來料）與 Base 2（訂單區）各自一組 ΔX/ΔY/ΔZ（mm）與開關
        self._delta_lock = threading.Lock()
        self.fine3_dx = 0.0
        self.fine3_dy = 0.0
        self.fine3_dz = 0.0
        self.use_fine3 = True

        self.fine2_dx = 0.0
        self.fine2_dy = 0.0
        self.fine2_dz = 0.0
        self.use_fine2 = True

        # === 新增：載入上次關閉前的偏移與啟用狀態 ===
        self._offset_file = self._offsets_path()
        self._load_offsets()  # 先嘗試載入（若檔案不存在就忽略）
        atexit.register(self._save_offsets_safe)  # 關閉前再存一次（雙保險）

        # 啟動左右並排 GUI（若有圖形環境）
        if tk is not None:
            threading.Thread(target=self._start_offset_gui, daemon=True).start()
        else:
            self.get_logger().info('No GUI environment detected; XYZ fine-tune window disabled.')
        
        self._debug_offsets = True  # 開/關：送動作時列印 base 與實際套用的 ΔXYZ

        # ===============================================================

# -----------------訂單--------------------------
    def order_callback(self, msg):
        flat_data = msg.item_names
        self.oder_items = [flat_data[i:i+2] for i in range(0, len(flat_data), 2)]
        print("Order received:", self.oder_items)

# -----------------辨識------------------------------
    def catch_callback(self, msg):
        self.catch_count=msg.items

# ---------------相同物體-------------------

    def same_thing (self,pose):
        if len(pose) >1:
            if pose[0] == pose[1]:
                del pose[0]
                return 1
        else :
            return 0
        
# ---------------上升(下降)-------------------

    # def down_pose(self, pose,state):
    #     new_pose = pose.copy()  # ← 建立一份新 list
    #     if state in ('Z', 'C', 'F'):
    #         new_pose[2] = Down_Offset[0]
    #     elif state in ('I'):
    #         new_pose[2] = Down_Offset[1]
    #     elif state in('A','D'):
    #         new_pose[2] = Down_Offset[2]
    #     elif state in('B' ,'E'):
    #         new_pose[2] = Down_Offset[3]
    #     elif state in ('G'):
    #         new_pose[2] = Down_Offset[4]
    #     return new_pose
# ------------------------------------------
    def up_pose(self, pose):
        new_pose = pose.copy()
        new_pose[2] = 220.0  
        return new_pose
    
    def order_up_pose(self, pose):
        new_pose = pose.copy()
        new_pose[2] = 120.0  
        return new_pose
    
    def sort_up_pose(self, pose):
        new_pose = pose.copy()
        new_pose[2] = 80.0  
        return new_pose
    
    def F_turn_pose(self, pose):
        new_pose = pose.copy()
        # new_pose[2] = 80.0
        new_pose[1] -= 57.0  
        return new_pose
    
    def sort_offset(self, pose):
        new_pose = pose.copy()
        new_pose[1] -= 5.0
        return new_pose
    def Catch_Place(self, state):
        res1 = self.digital_request_send(
            cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
            # digital_input_pin=1,
            digital_output_pin=IO[2],
            digital_output_cmd=IO_STATE[state],
            time_wait=0,
            holding=False
            )
        res1 = self.digital_request_send(
            cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
            # digital_input_pin=1,
            digital_output_pin=IO[4],
            digital_output_cmd=IO_STATE[state],
            time_wait=0,
            holding=True
            )

# ===================== 新增：左右並排 GUI（Base3 左、Base2 右） =====================
    def _start_offset_gui(self):
        root = tk.Tk()
        root.title("XYZ 微調 - 左 Base3（分檢/來料） / 右 Base2（訂單區）")
        root.geometry("660x300")

        step_var = tk.DoubleVar(value=1.0)
        enabled3_var = tk.BooleanVar(value=self.use_fine3)
        enabled2_var = tk.BooleanVar(value=self.use_fine2)

        # 上方工具列（共用步進）
        top = ttk.Frame(root); top.pack(fill="x", padx=8, pady=6)
        ttk.Label(top, text="步進 (mm)：").pack(side="left")
        ttk.Entry(top, textvariable=step_var, width=8).pack(side="left", padx=6)

        # 左/右區塊
        left = ttk.LabelFrame(root, text="Base 3（分檢/來料）"); left.pack(side="left", fill="both", expand=True, padx=8, pady=6)
        right = ttk.LabelFrame(root, text="Base 2（訂單區）");  right.pack(side="left", fill="both", expand=True, padx=8, pady=6)

        def apply_delta(base, axis, sign):
            step = step_var.get()
            with self._delta_lock:
                if base == 3:
                    if axis == 'x': self.fine3_dx += sign*step
                    elif axis == 'y': self.fine3_dy += sign*step
                    elif axis == 'z': self.fine3_dz += sign*step
                elif base == 2:
                    if axis == 'x': self.fine2_dx += sign*step
                    elif axis == 'y': self.fine2_dy += sign*step
                    elif axis == 'z': self.fine2_dz += sign*step
                self._save_offsets_locked()
            refresh_labels()

        def reset_delta(base):
            with self._delta_lock:
                if base == 3:
                    self.fine3_dx = self.fine3_dy = self.fine3_dz = 0.0
                elif base == 2:
                    self.fine2_dx = self.fine2_dy = self.fine2_dz = 0.0
                self._save_offsets_locked()
            refresh_labels()

        def toggle_enabled(base, var):
            with self._delta_lock:
                if base == 3:
                    self.use_fine3 = var.get()
                elif base == 2:
                    self.use_fine2 = var.get()
                self._save_offsets_locked()
            refresh_labels()

        # 共用的小組件：每邊的 UI
        def build_side(parent, base, enable_var):
            ttk.Checkbutton(parent, text="啟用微調", variable=enable_var,
                            command=lambda: toggle_enabled(base, enable_var)).grid(row=0, column=0, columnspan=3, sticky="w", padx=4, pady=2)

            ttk.Label(parent, text="X").grid(row=1, column=0)
            ttk.Button(parent, text="-", width=4, command=lambda: apply_delta(base, 'x', -1)).grid(row=1, column=1)
            ttk.Button(parent, text="+", width=4, command=lambda: apply_delta(base, 'x', +1)).grid(row=1, column=2)

            ttk.Label(parent, text="Y").grid(row=2, column=0)
            ttk.Button(parent, text="-", width=4, command=lambda: apply_delta(base, 'y', -1)).grid(row=2, column=1)
            ttk.Button(parent, text="+", width=4, command=lambda: apply_delta(base, 'y', +1)).grid(row=2, column=2)

            ttk.Label(parent, text="Z").grid(row=3, column=0)
            ttk.Button(parent, text="-", width=4, command=lambda: apply_delta(base, 'z', -1)).grid(row=3, column=1)
            ttk.Button(parent, text="+", width=4, command=lambda: apply_delta(base, 'z', +1)).grid(row=3, column=2)

            lbl_dx = ttk.Label(parent, text=""); lbl_dx.grid(row=4, column=0, columnspan=3, sticky="w")
            lbl_dy = ttk.Label(parent, text=""); lbl_dy.grid(row=5, column=0, columnspan=3, sticky="w")
            lbl_dz = ttk.Label(parent, text=""); lbl_dz.grid(row=6, column=0, columnspan=3, sticky="w")
            lbl_state = ttk.Label(parent, text=""); lbl_state.grid(row=7, column=0, columnspan=3, sticky="w")

            ttk.Button(parent, text="Reset", command=lambda: reset_delta(base)).grid(row=8, column=0, columnspan=3, pady=6)

            return lbl_dx, lbl_dy, lbl_dz, lbl_state

        lbl3_dx, lbl3_dy, lbl3_dz, lbl3_state = build_side(left, 3, enabled3_var)
        lbl2_dx, lbl2_dy, lbl2_dz, lbl2_state = build_side(right, 2, enabled2_var)

        def refresh_labels():
            with self._delta_lock:
                dx3, dy3, dz3, en3 = self.fine3_dx, self.fine3_dy, self.fine3_dz, self.use_fine3
                dx2, dy2, dz2, en2 = self.fine2_dx, self.fine2_dy, self.fine2_dz, self.use_fine2
            lbl3_dx.config(text=f"ΔX = {dx3:.3f} mm")
            lbl3_dy.config(text=f"ΔY = {dy3:.3f} mm")
            lbl3_dz.config(text=f"ΔZ = {dz3:.3f} mm")
            lbl3_state.config(text=f"狀態：{'啟用' if en3 else '停用'}")
            lbl2_dx.config(text=f"ΔX = {dx2:.3f} mm")
            lbl2_dy.config(text=f"ΔY = {dy2:.3f} mm")
            lbl2_dz.config(text=f"ΔZ = {dz2:.3f} mm")
            lbl2_state.config(text=f"狀態：{'啟用' if en2 else '停用'}")

        refresh_labels()
        root.mainloop()
# ============================================================================

# ===================== 新增：偏移檔案 存取函式 =====================
    def _offsets_path(self) -> Path:
        # 存在執行檔同目錄：offsets.json
        try:
            base = Path(__file__).resolve().parent
        except Exception:
            base = Path.cwd()
        return base / "offsets.json"

    def _load_offsets(self):
        p = self._offset_file
        if not p.exists():
            self.get_logger().info(f"No previous offsets file: {p}")
            return
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
            with self._delta_lock:
                self.fine3_dx = float(data.get("fine3_dx", self.fine3_dx))
                self.fine3_dy = float(data.get("fine3_dy", self.fine3_dy))
                self.fine3_dz = float(data.get("fine3_dz", self.fine3_dz))
                self.use_fine3 = bool(data.get("use_fine3", self.use_fine3))

                self.fine2_dx = float(data.get("fine2_dx", self.fine2_dx))
                self.fine2_dy = float(data.get("fine2_dy", self.fine2_dy))
                self.fine2_dz = float(data.get("fine2_dz", self.fine2_dz))
                self.use_fine2 = bool(data.get("use_fine2", self.use_fine2))
            self.get_logger().info(
                f"Offsets loaded from {p}: "
                f"base3 Δ=({self.fine3_dx},{self.fine3_dy},{self.fine3_dz}) en={self.use_fine3}; "
                f"base2 Δ=({self.fine2_dx},{self.fine2_dy},{self.fine2_dz}) en={self.use_fine2}"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to load offsets from {p}: {e}")

    def _save_offsets_locked(self):
        # 呼叫前已拿到 self._delta_lock
        p = self._offset_file
        data = {
            "fine3_dx": self.fine3_dx, "fine3_dy": self.fine3_dy, "fine3_dz": self.fine3_dz, "use_fine3": self.use_fine3,
            "fine2_dx": self.fine2_dx, "fine2_dy": self.fine2_dy, "fine2_dz": self.fine2_dz, "use_fine2": self.use_fine2,
        }
        try:
            p.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception as e:
            self.get_logger().warn(f"Failed to save offsets to {p}: {e}")

    def _save_offsets_safe(self):
        # 安全儲存（供 atexit 使用）
        with self._delta_lock:
            self._save_offsets_locked()
# ============================================================================

# ---------------------------------------------------

    def _state_machine(self, state: States) -> States:
        if state == States.INIT:
            res2 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.READ_DI,
                    digital_input_pin=2,
                    holding=False
                    )
            if res2.digital_state == 1:
                self.get_logger().info('INIT')
                nest_state = States.HOME_MOVE
            else :
                nest_state = States.INIT

# ------------------回家-------------------
        elif state == States.HOME_MOVE:
            

            self.get_logger().info('HOME_MOVE !!!!!')
            for i in range(1, 3):
                res1 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    # digital_input_pin=1
                    digital_output_pin=IO[i*2],
                    digital_output_cmd=Digitalcmd.Request.DIGITAL_OFF,
                    time_wait=0,
                    holding=False
                    )
                res = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    # digital_input_pin=1
                    digital_output_pin=IO[i*2-1],
                    digital_output_cmd=Digitalcmd.Request.DIGITAL_ON,
                    time_wait=0,
                    holding=False
                    )
                
            self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = 0,
                pose=HOME_POSE,
                holding=True,
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION
                )
            self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=FRIST_POSE,
                holding=True,
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION
                )

            nest_state = States.OBJECT_AREA
            # nest_state = States.READ_ORDER




# -------------------移動到來料區----------------
        elif state == States.OBJECT_AREA:
            if self.item[1] in ('G','H','I'):
                res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(ERROR_relay_point),
                    holding=False,
                    velocity=DEFAULT_VELOCITY,
                    acceleration=DEFAULT_ACCELERATION
                    )
            self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.up_pose(OBJECT_POSES[self.order_area_num if self.order_area_num <= 2 else self.order_area_num - 3]),
                holding=False,
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                )
            self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.sort_up_pose(OBJECT_POSES[self.order_area_num]),
                holding=False,
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                )
            nest_state = States.CATCH_OBJECT
            print("\n夾取第",self.order_area_num+1,"次來料區\n")

        
# -------------------夾取來料區---------------------
        elif state == States.CATCH_OBJECT:
            # -----------------下降--------------------
            if self.F == 0 :
                self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.LINE,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=OBJECT_POSES[self.order_area_num],
                    velocity=LINE_VELOCITY,
                    acceleration=LINE_ACCELERATION,
                    holding=False
                )
                self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.F_turn_pose(OBJECT_POSES[self.order_area_num]),
                    holding=True,
                    velocity=60,
                    acceleration=60,
                )
            # ---------------  氣閥夾取-----------------
            self.Catch_Place(1)

            nest_state = States.READ_OBJECT

 # ----------------------辨識物品-----------------------

        elif state == States.READ_OBJECT:
            if self.F != 0:
            # -------------------上升-------------------
                self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.LINE,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.sort_up_pose(OBJECT_POSES[self.order_area_num]),
                    velocity=LINE_VELOCITY,
                    acceleration=LINE_ACCELERATION,
                    holding=False
                )
                self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(OBJECT_POSES[self.order_area_num if self.order_area_num <= 2 else self.order_area_num - 3]),
                    holding=False,
                    velocity=DEFAULT_VELOCITY,
                    acceleration=DEFAULT_ACCELERATION,
                    )
            
            time.sleep(0.8)

            print("抓取物品",self.catch_count)
            self.item = self.catch_count
            for j, item in enumerate(self.item):
                if item == 'NONE':
                    res2 = self.digital_request_send(
                        cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                        digital_output_pin=IO[(j+1)*2],
                        digital_output_cmd=IO_STATE[0],
                        time_wait=0,
                        holding=False
                    )
                    if j == 0:
                        self.catch_num += 1
                else:
                    row = self.order_map[item]
                    col = self.count_map[item]
                    if col >3 and item != 'G': 
                        col = 3    
                        if self.F != 0 or item != 'F':
                            row = 6
                            self.item[j] = 'H'

                    if j == 1:
                        if item != 'G':
                            col -= 1
                    x,y,z,rx,ry,rz = Sorting_area_base[row][col]
                    self.count_map[item] += 1
                    self.Sorting_palce.append([x,y,z,rx,ry,rz])
            if self.F == 0 :
                nest_state = States.F_ERROR
                print(f"🔷 [第 {self.order_area_num+1} 次抓取]：{self.item}")

            else : 
                self.order_area_num += 1
                self.same = self.same_thing (self.Sorting_palce)
                self.F = 0
                print(f"🔷 [回正後第 {self.order_area_num+1} 次抓取]：{self.item}")

                nest_state = States.SORT_AREA


        elif state == States.F_ERROR:
            for j ,item in enumerate(self.item):
                if item  in ('F','G'):
                    self.F = 1
                    res1 = self.digital_request_send(
                        cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                        digital_output_pin=IO[(j+1)*2],
                        digital_output_cmd=IO_STATE[0],
                        time_wait=0,
                        holding=True
                    )

            if self.F > 0 :
                for j ,item in enumerate(self.item):
                    if item not in ('NONE', 'H'):
                        self.count_map[item] -= 1
                        self.Sorting_palce = []
                        self.catch_num = 0

                for i in range(1,3):
                    res = self.motion_request_send(
                        cmd_mode=Motioncmd.Request.PTP,
                        cmd_type=Motioncmd.Request.POSE_CMD,
                        base = self.base_state,
                        pose=self.F_turn_pose(OBJECT_POSES[self.order_area_num]),
                        holding=False,
                        velocity=50,
                        acceleration=50,
                        )
                    self.res = self.motion_request_send(
                        cmd_mode=Motioncmd.Request.PTP,
                        cmd_type=Motioncmd.Request.POSE_CMD,
                        base = self.base_state,
                        pose=OBJECT_POSES[self.order_area_num],
                        holding=True,
                        velocity=50,
                        acceleration=50,
                        )
                nest_state = States.CATCH_OBJECT

            else :      
                self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.LINE,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.sort_up_pose(self.F_turn_pose(OBJECT_POSES[self.order_area_num])),
                    holding=False,
                    velocity=LINE_VELOCITY,
                    acceleration=LINE_ACCELERATION
                )
                self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(OBJECT_POSES[self.order_area_num if self.order_area_num <= 2 else self.order_area_num - 3]),
                    holding=False,
                    velocity=DEFAULT_VELOCITY,
                    acceleration=DEFAULT_ACCELERATION,
                    )
                self.order_area_num += 1
                self.same = self.same_thing (self.Sorting_palce)
                nest_state = States.SORT_AREA


# --------------------移動到分檢區域---------------------
        elif state == States.SORT_AREA:
            del self.catch_items[:Number_of_grips]

            if self.item[self.catch_num] in ('H','G','I'):
                res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(ERROR_relay_point),
                    holding=False,
                    velocity=DEFAULT_VELOCITY,
                    acceleration=DEFAULT_ACCELERATION,
                    )
                self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.Sorting_palce[0],
                    holding=False,
                    velocity=DEFAULT_VELOCITY,
                    acceleration=DEFAULT_ACCELERATION,
                )

            else :
                self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.up_pose(self.Sorting_palce[0]),
                holding=False,
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                )
            nest_state = States.SORT_PLACE
            print("準備放置物品",self.catch_num+1)

# ---------------------放置物品--------------------
        elif state == States.SORT_PLACE:
            print(f"放置 {self.item [self.catch_num]}")
            self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.Sorting_palce[0],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            if self.same:
                self.Catch_Place(0)
            else:
                res2 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    digital_output_pin=IO[(self.catch_num+1)*2],
                    digital_output_cmd=IO_STATE[0],
                    time_wait=0,
                    holding=True
                    )    
            if self.item[self.catch_num] in ('H','G','I'):
                self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(ERROR_relay_point),
                    holding=False,
                    velocity=DEFAULT_VELOCITY,
                    acceleration=DEFAULT_ACCELERATION,
                    )
            else :                
                self.res = self.motion_request_send(
                        cmd_mode=Motioncmd.Request.PTP,
                        cmd_type=Motioncmd.Request.POSE_CMD,
                        base = self.base_state,
                        pose=self.sort_offset(self.Sorting_palce[0]),
                        velocity=DEFAULT_VELOCITY,
                        acceleration=DEFAULT_ACCELERATION,
                        holding=False
                        )        
                self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.LINE,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(self.Sorting_palce[0]),
                    holding=True,
                    velocity=LINE_VELOCITY,
                    acceleration=LINE_ACCELERATION
                )
            self.catch_num+=1
            print("放置完成")  
            del self.Sorting_palce[0]
            if self.Sorting_palce:
                print('下一個')
                nest_state = States.SORT_AREA
            else:
                if  self.order_area_num < 9:
                    self.Catch_Place(0)
                    self.Sorting_palce = []
                    self.catch_num = 0
                    nest_state = States.OBJECT_AREA
                else:
                    if self.item[1] =='G':
                        self.res = self.motion_request_send(
                            cmd_mode=Motioncmd.Request.PTP,
                            cmd_type=Motioncmd.Request.POSE_CMD,
                            base = self.base_state,
                            pose=self.up_pose(ERROR_relay_point),
                            velocity=DEFAULT_VELOCITY,
                            acceleration=DEFAULT_ACCELERATION,
                            holding=False
                            )
                    self.res = self.motion_request_send(
                        cmd_mode=Motioncmd.Request.PTP,
                        cmd_type=Motioncmd.Request.POSE_CMD,
                        base = 0,
                        pose=HOME_POSE,
                        velocity=DEFAULT_VELOCITY,
                        acceleration=DEFAULT_ACCELERATION,
                        holding=True
                        )
                    nest_state = States.READ_ORDER
                    self.get_logger().info('分檢完成')
                    self.catch_num = 0



# ----------------------讀取訂單----------------------
        elif state == States.READ_ORDER:
            print("進行訂單",self.oder_items)
            self.oder_item = self.oder_items[0]
            del self.oder_items[0]
            if self.oder_item == ['NONE', 'NONE']:
                self.order_palce_num += 1
                if self.oder_items :
                    print("沒有訂單，跳過\n")
                    nest_state = States.READ_ORDER
                else :
                    print('無訂單')
                    nest_state = States.END_HOME_MOVE
            else:
                for j, item in enumerate(self.oder_item):
                    if (self.oder_item[j] == 'NONE'):
                        if j == 0:
                            self.order_catch_palce_num+=1
                    else:
                        row = self.order_map[item]
                        col = self.sort_count_map[item]
                        if j == 1:
                            col -= 1
                        x,y,z,rx,ry,rz = Sorting_area_base[row][col]
                        self.sort_count_map[item] += 1
                        self.Order_palce.append([x,y,z,rx,ry,rz])
                        self.same = self.same_thing (self.Order_palce)
                nest_state = States.ORDER_OBJECT_AREA

# -------------------移動到訂單物品區域-------------------
        elif state == States.ORDER_OBJECT_AREA:
            self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.up_pose(self.Order_palce[0]),
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                holding=False
                )
            nest_state = States.ORDER_OBJECT_PICK
            print("抓取",self.order_catch_palce_num+1,"個物品")
        

# -------------------抓取訂單物品---------------------------
        elif state == States.ORDER_OBJECT_PICK:
            self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.LINE,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.Order_palce[0],
                    holding=True,
                    velocity=LINE_VELOCITY,
                    acceleration=LINE_ACCELERATION
                )
            if self.same:
                self.Catch_Place(1)
            else:
                res1 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    digital_output_pin=IO[(self.order_catch_palce_num+1)*2],
                    digital_output_cmd=IO_STATE[1],
                    time_wait=0,
                    holding=True
                    )
            self.res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.LINE,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(self.Order_palce[0]),
                    holding=True,
                    velocity=LINE_VELOCITY,
                    acceleration=LINE_ACCELERATION
                )
            print("抓取",self.order_catch_palce_num+1,"完畢")
            self.order_catch_palce_num += 1
            del self.Order_palce[0]
            if self.Order_palce:
                nest_state = States.ORDER_OBJECT_AREA
            else:
                nest_state = States.ORDER_RELAY_POINT

# ------------------------中計點---------------------
        elif state == States.ORDER_RELAY_POINT:
            self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.up_pose(relay_point),
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                holding=True
                )       
            nest_state = States.ORDER_AREA


# -------------------移動到訂單區域-------------------
        elif state == States.ORDER_AREA:
            self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.order_base,
                pose=self.order_up_pose(ORDER_POSES[self.order_palce_num]),
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                holding=False
                )
            print("移動至訂單區")
            nest_state = States.ORDER_PLACE


# -------------------放置訂單物品-------------------

        elif state == States.ORDER_PLACE:
            res2 = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.order_base,
                pose=ORDER_POSES[self.order_palce_num],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                ) 
            self.Catch_Place(0)
            res2 = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.order_base,
                pose=self.order_up_pose(ORDER_POSES[self.order_palce_num]),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION,
                )
            self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.up_pose(relay_point),
                velocity=DEFAULT_VELOCITY,
                acceleration=DEFAULT_ACCELERATION,
                holding=False
                )  
            self.order_palce_num+=1
            print("完成",self.order_palce_num+1)  
            if self.oder_items :
                print("Next order")
                self.order_palce = []
                self.order_catch_palce_num= 0
                nest_state = States.READ_ORDER
            else:
                nest_state = States.END_HOME_MOVE
                self.get_logger().info('All objects sorted, closing robot')
                
# -------------------回家-------------------
        elif state == States.END_HOME_MOVE:
            self.get_logger().info('End !!!!!')
            self.res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = 0,
                pose=HOME_POSE,
                velocity=100,
                acceleration=100,
                holding=True
                )
            nest_state = States.CLOSE_ROBOT



        elif state == States.CLOSE_ROBOT:
            self.get_logger().info('CLOSE_ROBOT')
            for i in range(1, 5):
                res1 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    digital_output_pin=IO[i],
                    digital_output_cmd=IO_STATE[0],
                    time_wait=0,
                    holding=True
                    )
            # 關閉前再存一次
            self._save_offsets_safe()
            self.res = self.motion_request_send(cmd_mode=Motioncmd.Request.CLOSE)
            nest_state = States.FINISH

        else:
            print("Error: Invalid state")
            nest_state = None
            self.get_logger().error('Input state not supported!')
        return nest_state

    def _main_loop(self):
        state = States.INIT
        print("Start button")
        while state != States.FINISH:
            state = self._state_machine(state)
            if state == None:
                break
        self.destroy_node()
        rclpy.shutdown()

    def _wait_for_future_done(self, future: Future, timeout=-1):
        time_start = time.time()
        while not future.done():
            time.sleep(0.01)
            if timeout > 0 and time.time() - time_start > timeout:
                self.get_logger().error('Wait for service timeout!')
                return False
        return True

    def motion_request_send(
            self,
            holding=True,
            cmd_mode=Motioncmd.Request.PTP,
            cmd_type=Motioncmd.Request.POSE_CMD,
            velocity=DEFAULT_VELOCITY,
            acceleration=DEFAULT_ACCELERATION,
            tool=0,
            base=0,
            pose=[float('inf')]*6,
            joints=[float('inf')]*6,
            circ_s=[],
            circ_end=[],
            jog_joint=6,
            jog_dir=0,
            move_dir = "z",
            move_dis = 0.01
            ):
        request = Motioncmd.Request()
        request.acceleration = acceleration
        
        request.velocity = velocity
        request.tool = tool
        request.base = base
        request.cmd_mode = cmd_mode
        request.cmd_type = cmd_type
        request.holding = holding

        # ================= 套用【分 Base】XYZ 偏移（mm） =================
        pose_to_send = pose
        applied_dx = applied_dy = applied_dz = 0.0  # 記錄實際套用的偏移
        if pose[0] != float('inf'):
            pose_to_send = pose.copy()
            with self._delta_lock:
                if base == 3 and self.use_fine3:
                    applied_dx, applied_dy, applied_dz = self.fine3_dx, self.fine3_dy, self.fine3_dz
                elif base == 2 and self.use_fine2:
                    applied_dx, applied_dy, applied_dz = self.fine2_dx, self.fine2_dy, self.fine2_dz
            pose_to_send[0] += applied_dx
            pose_to_send[1] += applied_dy
            pose_to_send[2] += applied_dz
        # ===============================================================

        # 新增：列印這一步實際使用的 base 與套用的 ΔXYZ 與 from->to
        if getattr(self, "_debug_offsets", False):
            try:
                print(
                    f"=({pose[0]:.1f},{pose[1]:.1f},{pose[2]:.1f}) -> =({pose_to_send[0]:.1f},{pose_to_send[1]:.1f},{pose_to_send[2]:.1f})"
                )
            except Exception:
                print(f"[motion] base={base} Δ=({applied_dx},{applied_dy},{applied_dz}) -> {pose_to_send[:3]}")

        pose_ = Twist()
        [pose_.linear.x, pose_.linear.y, pose_.linear.z] = pose_to_send[0:3]
        [pose_.angular.x, pose_.angular.y, pose_.angular.z] = pose_to_send[3:6]
        
        request.pose = pose_
        request.joints = joints
        request.circ_s = circ_s
        request.circ_end = circ_end
        request.jog_joint = jog_joint
        request.jog_dir = jog_dir
        request.move_dir = move_dir
        request.move_dis = move_dis

        while not self.hiwin_client_mo.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.hiwin_client_mo.call_async(request)
        if self._wait_for_future_done(future):
            res = future.result()
        else:
            res = None
        return pose

    def digital_request_send(
            self, 
            holding=True,
            cmd_mode=Digitalcmd.Request.READ_DI,
            digital_input_pin=0,
            digital_output_pin=0,
            digital_output_cmd=Digitalcmd.Request.DIGITAL_OFF,
            time_wait=0,

            ):
        request = Digitalcmd.Request()
        request.digital_input_pin = digital_input_pin
        request.digital_output_pin = digital_output_pin
        request.digital_output_cmd = digital_output_cmd
        request.do_timer = time_wait
        request.cmd_mode = cmd_mode
        request.holding = holding

        while not self.hiwin_client_di.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.hiwin_client_di.call_async(request)
        if self._wait_for_future_done(future):
            res = future.result()
        else:
            res = None
        return res
    def read_request_send(
            self, 
            holding=True,
            cmd_mode=Readcmd.Request.CHECK_JOINT,
            ):
        request = Readcmd.Request()
        request.cmd_mode = cmd_mode
        request.holding = holding

        while not self.hiwin_client_rd.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.hiwin_client_rd.call_async(request)
        if self._wait_for_future_done(future):
            res = future.result()
        else:
            res = None
        return res
    def start_main_loop_thread(self):
        self.main_loop_thread = Thread(target=self._main_loop)
        self.main_loop_thread.daemon = True
        self.main_loop_thread.start()

def main(args=None):
    rclpy.init(args=args)
    stratery = ExampleStrategy()
    stratery.start_main_loop_thread()
    rclpy.spin(stratery)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
