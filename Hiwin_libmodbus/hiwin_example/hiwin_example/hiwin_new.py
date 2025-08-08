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

DEFAULT_VELOCITY = 50
DEFAULT_ACCELERATION = 50
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

relay_point =[320.0, 271.0,  20.0,  -180.0, 0.00, 89.00]

Sorting_area_base = [
    ([320.0, 463.0,  142.0, -180.0, 0.00, 89.00],[235.0,  463.0,  142.0, -180.0, 0.00, 89.00],[ 152.0,  463.0,  142.0, -180.0, 0.00, 89.00],[ 70.0,  463.0,  142.0, -180.0, 0.00, 89.00]),  # A row
    ([404.0, 356.0,  78.0,  -180.0, 0.00, 89.00],[320.0, 356.0,  78.0,  -180.0, 0.00, 89.00],[235.0,  356.0,  78.0,  -180.0, 0.00, 89.00],[ 152.0,  356.0,  78.0,  -180.0, 0.00, 89.00]), # B row
    ([404.0, 271.0,  20.0,  -180.0, 0.00, 89.00],[320.0, 271.0,  20.0,  -180.0, 0.00, 89.00],[235.0,  271.0,  20.0,  -180.0, 0.00, 89.00],[ 152.0,  271.0,  20.0,  -180.0, 0.00, 89.00]),  # C row
    ([-14.0, 463.0,  142.0, -180.0, 0.00, 89.00],[-98.0,  463.0,  142.0, -180.0, 0.00, 89.00],[-182.0,  463.0,  142.0, -180.0, 0.00, 89.00],[-266.0,  356.0,  78.0,  -180.0, 0.00, 89.00]),   # D row
    ([70.0,  356.0,  78.0,  -180.0, 0.00, 89.00],[-14.0, 356.0,  78.0,  -180.0, 0.00, 89.00],[-98.0,  356.0,  78.0,  -180.0, 0.00, 89.00],[-182.0,  356.0, 98.0,  -180.0, 0.00, 89.00]),   # E row
    ([70.0,  271.0,  20.0,  -180.0, 0.00, 89.00],[-14.0, 271.0,  20.0,  -180.0, 0.00, 89.00],[-98.0,  271.0,  20.0,  -180.0, 0.00, 89.00],[-182.0,  271.0,  20.0,  -180.0, 0.00, 89.00]), # F row

    ([480.0, 18.0,  37.0,  -180.0, 0.00, 89.00],[480.0,  -56.0,  37.0,  -180.0, 0.00, 89.00],[480.0,  -130.0,  37.0,  -180.0, 0.00, 89.00],[480.0,  -204.0,  37.0,  -180.0, 0.00, 89.00],[480.0,  -279.0,  37.0,  -180.0, 0.00, 89.00],[480.0,  -353.0,  37.0,  -180.0, 0.00, 89.00])
      # G row
]

# 錯誤物料區域基礎座標

OBJECT_POSES = [    
    ([-262.0,  133.0,  20.0,    -180.00, 0.00, 89.00]),
    ([-262.0,  230.0,  20.0,    -180.00, 0.00, 89.00]),
    ([-262.0,  329.0,  20.0,    -180.00, 0.00, 89.00]),
    ([-430.0,  133.0,  20.0,    -180.00, 0.00, 89.00]),
    ([-430.0,  230.0,  20.0,    -180.00, 0.00, 89.00]),
    ([-430.0,  329.0,  20.0,    -180.00, 0.00, 89.00]),


    ([-511.0,  133.0,  20.0,    -180.00, 0.00, 89.00]),
    ([-511.0,  230.0,  20.0,    -180.00, 0.00, 89.00]),
    ([-511.0,  329.0,  20.0,    -180.00, 0.00, 89.00]),
]

#訂單放置
ORDER_POSES = [
    ([396.0,   0.0,   34.0,   -180.00, 0.00, 0.00]),
    ([288.0,   0.0,   34.0,   -180.00, 0.00, 89.00]),
    ([288.0,  -88.0,  34.0,   -180.00, 0.00, 89.00]),


    ([396.0,   -201.0,  34.0,   -180.00, 0.00, 0.00]),
    ([288.0,   -203.0,  34.0,   -180.00, 0.00, 89.00]),
    ([288.0,   -301.0,  34.0,   -180.00, 0.00, 89.00]),

    ([396.0,   -430.0,  34.0,   -180.00, 0.00, 0.00]),
    ([288.0,   -416.0,  34.0,   -180.00, 0.00, 89.00]),
    ([288.0,   -513.0,  34.0,   -180.00, 0.00, 89.00]),
]
TEST_POSE = [0.00, 368.00, 100.00, -180.00, 0.00, 90.000]

base_point = OBJECT_POSES[3][:3]
order_point = ORDER_POSES [1][:3]

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
        
        self.count_map = {'A': 1, 'B': 1, 'C': 1, 'D': 1, 'E': 1, 'F': 1, 'G': 1}
        self.order_map = {'A': 0, 'B': 1, 'C': 2, 'D': 3, 'E': 4, 'F': 5, 'G': 6}

        self.catch_items = ['A'] * 3 + ['B'] * 3 + ['C'] * 3 + ['D'] * 3 + ['E'] * 3 + ['F'] * 3  
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
        new_pose[2] = 210.0  
        return new_pose
    
    def order_up_pose(self, pose):
        new_pose = pose.copy()
        new_pose[2] = 110.0  
        return new_pose
    
    def sort_up_pose(self, pose):
        new_pose = pose.copy()
        new_pose[2] = 80.0  
        return new_pose
    
    def F_turn_pose(self, pose):
        new_pose = pose.copy()
        # new_pose[2] = 80.0
        new_pose[1] -= 30.0  
        return new_pose
# ------------------PTP-------------------------
    def OFFSET(self,pose,offset):
        new_pose = pose.copy()
        new_pose[2] += offset
        return new_pose

# ----------------PTP下降------------------------
    def PTP_down_up(self,pose,bas,hold,ex):
        # arm_state=self.read_request_send(
        #     holding=True,
        #     cmd_mode=Readcmd.Request.CHECK_POSE,
        #     )
        # arm = list(arm_state.current_position)

        if pose [2]-ex[2] < 0.0:
            symbol = -1
        else : 
            symbol = 1
        offset =  int((pose [2]-ex[2])/10*symbol) 
        for i in range ( offset ):
            print(offset)
            print(self.OFFSET(ex,i*10*symbol),)
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = bas,
                pose=self.OFFSET(ex,i*10*symbol),
                holding=False
                )
        res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = bas,
                pose=pose,
                holding=hold
                )



# ---------------校正-------------------
    def Correction_base(self, pose):
            pose[0] -= base_point[0]
            pose[1] -= base_point[1]
            pose[2] -= base_point[2]
    

    def Correction_order (self, pose):
            pose[0] -= order_point[0]
            pose[1] -= order_point[1]
            pose[2] -= order_point[2]

    def Correction_base_sorting_place(self, pose):
            for row in pose:
                for pose in row:
                    self.Correction_base(pose)

    def Correction_base_place(self, pose):
        for row in pose:
            self.Correction_base(row)
    def Correction_order_place(self, pose):
        for row in pose:
            self.Correction_base(row)
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
                nest_state = States.CORRECTION
            else :
                nest_state = States.INIT


        elif state == States.CORRECTION:
            if correction:
                self.base_state = 2
                self.get_logger().info('CORRECTION !!!!!')
                self.Correction_base_place(OBJECT_POSES)
                self.Correction_base_sorting_place(Sorting_area_base)
                self.Correction_order_place(ORDER_POSES)
                self.Correction_base(relay_point)
            nest_state = States.HOME_MOVE


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
                res2 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    # digital_input_pin=1
                    digital_output_pin=IO[i*2-1],
                    digital_output_cmd=Digitalcmd.Request.DIGITAL_ON,
                    time_wait=0,
                    holding=False
                    )
                
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = 0,
                pose=HOME_POSE,
                holding=True
                )
            arm_state=self.read_request_send(
            holding=True,
            cmd_mode=Readcmd.Request.CHECK_POSE,
            )
            nest_state = States.OBJECT_AREA



# -------------------移動到來料區----------------
        elif state == States.OBJECT_AREA:
            if self.item[1] =='G':
                es = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(relay_point),
                    holding=False
                    )
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.up_pose(OBJECT_POSES[self.order_area_num]),
                holding=False
                )
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.sort_up_pose(OBJECT_POSES[self.order_area_num]),
                holding=False
                )
            nest_state = States.CATCH_OBJECT
            print("\n夾取第",self.order_area_num+1,"次來料區\n")
        
# -------------------夾取來料區---------------------
        elif state == States.CATCH_OBJECT:

            # -----------------下降--------------------
            self.PTP_down_up(OBJECT_POSES[self.order_area_num],self.base_state,True,self.sort_up_pose(OBJECT_POSES[self.order_area_num]))
            # ---------------  氣閥夾取-----------------
            for i in range(1, 3):
                res1 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    # digital_input_pin=1,
                    digital_output_pin=IO[i*2],
                    digital_output_cmd=IO_STATE[1],
                    time_wait=0,
                    holding=True
                    )



            nest_state = States.READ_OBJECT

 # ----------------------辨識物品-----------------------

        elif state == States.READ_OBJECT:
            if self.F != 0:
            # -------------------上升-------------------
                self.PTP_down_up(self.sort_up_pose(OBJECT_POSES[self.order_area_num]),self.base_state,False,OBJECT_POSES[self.order_area_num])
                res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(OBJECT_POSES[self.order_area_num]),
                    holding=False
                    )
            else:
                time.sleep(0.8)

            print("抓取物品",self.catch_count)
            self.item = self.catch_count
            # self.item = self.catch_items[:Number_of_grips]
            # del self.catch_items[:Number_of_grips]
            print(f"🔷 [第 {self.order_area_num+1} 次抓取]：{self.item}")
            for j, item in enumerate(self.item):
                if item == 'NONE':
                    res2 = self.digital_request_send(
                        cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                        # digital_input_pin=1,
                        digital_output_pin=IO[(j+1)*2],
                        digital_output_cmd=IO_STATE[0],
                        time_wait=0,
                        holding=True
                    )
                    if j == 0:
                        self.catch_num += 1
                else:
                    row = self.order_map[item]
                    col = self.count_map[item]
                    if j == 1:
                        if item != 'G':
                            col -= 1
                    # 計算位置（加上列的基礎座標 + 欄位間隔）
                    x,y,z,rx,ry,rz = Sorting_area_base[row][col]
                    if j == 1 and item == 'G':
                        x += 85
                    self.count_map[item] += 1
                    self.Sorting_palce.append([x,y,z,rx,ry,rz])
            if self.F == 0 :
                nest_state = States.F_ERROR
            else : 
                self.order_area_num += 1
                self.same = self.same_thing (self.Sorting_palce)
                self.F = 0
                nest_state = States.SORT_AREA


        elif state == States.F_ERROR:
            for j ,item in enumerate(self.item):
                

                if item  in ('F', 'D'):

                    self.F = 1
                    res1 = self.digital_request_send(
                        cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                        # digital_input_pin=1,
                        digital_output_pin=IO[(j+1)*2],
                        digital_output_cmd=IO_STATE[0],
                        time_wait=0,
                        holding=True
                    )

            if self.F > 0 :
                for j ,item in enumerate(self.item):
                    if item != 'NONE':
                        self.count_map[item] -= 1
                        self.Sorting_palce = []
                        self.catch_num = 0

                for i in range(1,4):
                    res = self.motion_request_send(
                        cmd_mode=Motioncmd.Request.LINE,
                        cmd_type=Motioncmd.Request.POSE_CMD,
                        base = self.base_state,
                        pose=self.F_turn_pose(OBJECT_POSES[self.order_area_num]),
                        holding=True,
                        velocity=LINE_VELOCITY,
                        acceleration=LINE_ACCELERATION
                        )
                    res = self.motion_request_send(
                        cmd_mode=Motioncmd.Request.LINE,
                        cmd_type=Motioncmd.Request.POSE_CMD,
                        base = self.base_state,
                        pose=OBJECT_POSES[self.order_area_num],
                        holding=True,
                        velocity=LINE_VELOCITY,
                        acceleration=LINE_ACCELERATION
                        )
                nest_state = States.CATCH_OBJECT

            else :      
                # -------------------上升-------------------
                self.PTP_down_up(self.sort_up_pose(OBJECT_POSES[self.order_area_num]),self.base_state,False,OBJECT_POSES[self.order_area_num])
                res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(OBJECT_POSES[self.order_area_num]),
                    holding=False
                    )
                self.order_area_num += 1
                self.same = self.same_thing (self.Sorting_palce)
                nest_state = States.SORT_AREA


# --------------------移動到分檢區域---------------------
        elif state == States.SORT_AREA:
            if self.item[self.catch_num] =='G':
                es = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.up_pose(relay_point),
                    holding=False
                    )
                res = self.motion_request_send(
                    cmd_mode=Motioncmd.Request.PTP,
                    cmd_type=Motioncmd.Request.POSE_CMD,
                    base = self.base_state,
                    pose=self.order_up_pose(self.Sorting_palce[0]),
                    holding=False
                    )
            else :
                res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.up_pose(self.Sorting_palce[0]),
                holding=False
                )
            nest_state = States.SORT_PLACE
            print("準備放置物品",self.catch_num+1)

# ---------------------放置物品--------------------
        elif state == States.SORT_PLACE:
             # -----------------下降--------------------
            print(f"放置 {self.item [self.catch_num]}")
            self.PTP_down_up(self.Sorting_palce[0],self.base_state,True,self.up_pose(self.Sorting_palce[0]))
            # ---------------  氣閥放開-----------------
            if self.same:
                for i in range(1, 3):
                    res2 = self.digital_request_send(
                        cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                        # digital_input_pin=1,
                        digital_output_pin=IO[i*2],
                        digital_output_cmd=IO_STATE[0],
                        time_wait=0,
                        holding=True
                    )
            else:
                res2 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    # digital_input_pin=1,
                    digital_output_pin=IO[(self.catch_num+1)*2],
                    digital_output_cmd=IO_STATE[0],
                    time_wait=0,
                    holding=True
                    )    
            if self.item[self.catch_num] =='G' :
                self.PTP_down_up(self.order_up_pose(self.Sorting_palce[0]),self.base_state,False,self.Sorting_palce[0])
                if self.item[0] =='G' and self.item != ['G', 'G']:
                    es = self.motion_request_send(
                        cmd_mode=Motioncmd.Request.PTP,
                        cmd_type=Motioncmd.Request.POSE_CMD,
                        base = self.base_state,
                        pose=self.up_pose(relay_point),
                        holding=False
                        )
            else :                        
                # -------------------上升-------------------
                self.PTP_down_up(self.up_pose(self.Sorting_palce[0]),self.base_state,False,self.Sorting_palce[0])
           
            # ------------------判斷-------------------
            self.catch_num+=1
            print("放置完成")  
            del self.Sorting_palce[0]
            if self.Sorting_palce:
                print('下一個')
                nest_state = States.SORT_AREA
            else:
                if  self.order_area_num < 9:
                    for i in range(1, 3):
                        res2 = self.digital_request_send(
                            cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                            # digital_input_pin=1,
                            digital_output_pin=IO[i*2],
                            digital_output_cmd=IO_STATE[0],
                            time_wait=0,
                            holding=True
                        )
                    self.Sorting_palce = []
                    self.catch_num = 0
                    nest_state = States.OBJECT_AREA

                else:
                    res = self.motion_request_send(
                        cmd_mode=Motioncmd.Request.PTP,
                        cmd_type=Motioncmd.Request.POSE_CMD,
                        base = 0,
                        pose=HOME_POSE,
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
                    print("沒有訂單，跳過")
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
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.up_pose(self.Order_palce[0]),
                holding=False
                )
            nest_state = States.ORDER_OBJECT_PICK
            print("抓取",self.order_catch_palce_num+1,"個物品")
        

# -------------------抓取訂單物品---------------------------
        elif state == States.ORDER_OBJECT_PICK:

            # -----------------下降--------------------
            self.PTP_down_up(self.Order_palce[0],self.base_state,True,self.up_pose(self.Order_palce[0]),self.up_pose(self.Order_palce[0]))
    
            # ---------------  氣閥放開-----------------
            if self.same:
                for i in range(1, 3):
                    res1 = self.digital_request_send(
                        cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                        # digital_input_pin=1,
                        digital_output_pin=IO[i*2],
                        digital_output_cmd=IO_STATE[1],
                        time_wait=0,
                        holding=True
                    )
            else:
                res1 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    # digital_input_pin=1,
                    digital_output_pin=IO[(self.order_catch_palce_num+1)*2],
                    digital_output_cmd=IO_STATE[1],
                    time_wait=0,
                    holding=True
                    )
            # -------------------上升-------------------
            self.PTP_down_up(self.up_pose(self.Order_palce[0]),self.base_state,False,self.Order_palce[0])

            print("抓取",self.order_catch_palce_num+1,"完畢")
            self.order_catch_palce_num += 1
            del self.Order_palce[0]
            if self.Order_palce:
                nest_state = States.ORDER_OBJECT_AREA
            else:
                nest_state = States.ORDER_RELAY_POINT

# ------------------------中計點---------------------
        elif state == States.ORDER_RELAY_POINT:
            es = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.up_pose(relay_point),
                holding=False
                )       
            nest_state = States.ORDER_AREA


# -------------------移動到訂單區域-------------------
        elif state == States.ORDER_AREA:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.order_up_pose(ORDER_POSES[self.order_palce_num]),
                holding=False
                )
            print("移動至訂單區")
            nest_state = States.ORDER_PLACE


# -------------------放置訂單物品-------------------

        elif state == States.ORDER_PLACE:
            # -----------------下降--------------------
            # self.get_logger().info('down')
            self.PTP_down_up(ORDER_POSES[self.order_palce_num],self.base_state,True,self.order_up_pose(ORDER_POSES[self.order_palce_num]))   
            # ---------------  氣閥放開-----------------
            for i in range(1, 3):
                res1 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    # digital_input_pin=1,
                    digital_output_pin=IO[i*2],
                    digital_output_cmd=IO_STATE[0],
                    time_wait=0,
                    holding=True
                    )                
            # -------------------上升-------------------
            self.PTP_down_up(self.order_up_pose(ORDER_POSES[self.order_palce_num]),self.base_state,False,ORDER_POSES[self.order_palce_num])   

            res2 = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = self.base_state,
                pose=self.up_pose(ORDER_POSES[self.order_palce_num]),
                holding=False,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            
            self.order_palce_num+=1
            print("完成",self.order_palce_num+1)  
            # ----------判斷------------
            if self.oder_items :
                print("next order")
                self.order_palce = []
                self.order_catch_palce_num= 0
                nest_state = States.READ_ORDER

            else:
                nest_state = States.END_HOME_MOVE
                self.get_logger().info('All objects sorted, closing robot')
                print("All objects sorted, closing robot")
                
# -------------------回家-------------------
        elif state == States.END_HOME_MOVE:
            self.get_logger().info('end !!!!!')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                base = 0,
                pose=HOME_POSE,
                holding=True
                )
            nest_state = States.CLOSE_ROBOT



        elif state == States.CLOSE_ROBOT:
            self.get_logger().info('CLOSE_ROBOT')
            res1 = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=1,
                digital_output_cmd=IO_STATE[1],
                time_wait=0,
                holding=True
                )
            res2 = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=5,
                digital_output_cmd=IO_STATE[1],
                time_wait=0,
                holding=True
                )
            
            res = self.motion_request_send(cmd_mode=Motioncmd.Request.CLOSE)
            nest_state = States.FINISH

        else:
            print("Error: Invalid state")
            nest_state = None
            self.get_logger().error('Input state not supported!')
            # return
        return nest_state

    def _main_loop(self):
        state = States.INIT
        print("start button")
        while state != States.FINISH:
            state = self._state_machine(state)
            if state == None:
                break
        self.destroy_node()
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
            tool=1,
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
        pose_ = Twist()
        [pose_.linear.x, pose_.linear.y, pose_.linear.z] = pose[0:3]
        [pose_.angular.x, pose_.angular.y, pose_.angular.z] = pose[3:6]
        
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
        return res
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
    # rclpy.spin(stratery)
    rclpy.spin(stratery)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
