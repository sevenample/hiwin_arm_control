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
# 左右偏移量
Offset = 35.0
# 下降偏移量
# Down_Offset = [145.0,86.0,26.0,30.0]

Down_Offset = [140.0,75.0,21.0,29.0]

# 21 -31 -96


# IO切換
IO = {1:1,
      2:2,
      3:5,
      4:4}


# IO接反
# IO = {1:2,
#       2:1,
#       3:4,
#       4:5}

IO_STATE = [Digitalcmd.Request.DIGITAL_OFF,
            Digitalcmd.Request.DIGITAL_ON]

ERROR_POES = [471.00, 177.00, 230.00, -180.00, 0.00, 90.000]

Sorting_area_base = [
    ([404.0, 463.0,  230.0, -180.0, 0.00, 90.00],[318.0, 463.0,  230.0, -180.0, 0.00, 90.00],[236.0,  463.0,  230.0, -180.0, 0.00, 90.00],[152.0,  463.0,  230.0, -180.0, 0.00, 90.00]),  # A row
    ([404.0, 356.0,  230.0, -180.0, 0.00, 90.00],[318.0, 356.0,  230.0, -180.0, 0.00, 90.00],[236.0,  356.0,  230.0, -180.0, 0.00, 90.00],[152.0,  356.0,  230.0, -180.0, 0.00, 90.00]), # B row
    ([404.0, 265.0,  230.0, -180.0, 0.00, 90.00],[318.0, 265.0,  230.0, -180.0, 0.00, 90.00],[236.0,  265.0,  230.0, -180.0, 0.00, 90.00],[152.0,  265.0,  230.0, -180.0, 0.00, 90.00]),  # C row
    ([58.0,  463.0,  230.0, -180.0, 0.00, 90.00],[-16.0, 463.0,  230.0, -180.0, 0.00, 90.00],[-90.0, 463.0,  230.0, -180.0, 0.00, 90.00],[-176.0, 463.0,  230.0, -180.0, 0.00, 90.00]),   # D row
    ([58.0,  356.0,  230.0, -180.0, 0.00, 90.00],[-16.0, 356.0,  230.0, -180.0, 0.00, 90.00],[-90.0, 356.0,  230.0, -180.0, 0.00, 90.00],[-176.0, 356.0,  230.0, -180.0, 0.00, 90.00]),   # E row
    ([58.0,  265.0,  230.0, -180.0, 0.00, 90.00],[-16.0, 265.0,  230.0, -180.0, 0.00, 90.00],[-90.0, 265.0,  230.0, -180.0, 0.00, 90.00],[-176.0, 265.0,  230.0, -180.0, 0.00, 90.00]), # F row
    (ERROR_POES,ERROR_POES,ERROR_POES,ERROR_POES,ERROR_POES,ERROR_POES,ERROR_POES,ERROR_POES,ERROR_POES,ERROR_POES)  # G row
]

# 錯誤物料區域基礎座標

OBJECT_POSES = [    
    ([-262.0, 156.0, 230.0, -180.00, 0.00, 90.00]),
    ([-262.0, 253.0, 230.0, -180.00, 0.00, 90.00]),
    ([-262.0, 352.0, 230.0, -180.00, 0.00, 90.00]),
    ([-430.0, 156.0, 230.0, -180.00, 0.00, 90.00]),
    ([-430.0, 253.0, 230.0, -180.00, 0.00, 90.00]),
    ([-430.0, 352.0, 230.0, -180.00, 0.00, 90.00]),


    ([-511.0, 156.0, 230.0, -180.00, 0.00, 90.00]),
    ([-511.0, 253.0, 230.0, -180.00, 0.00, 90.00]),
    ([-511.0, 352.0, 210.0, -180.00, 0.00, 90.00]),

]

ORDER_POSES = [
    ([383.0, -455.0, 230.0, -180.00, 0.00, 90.00]),
    ([383.0, -360.0,  230.0, -180.00, 0.00, 90.00]),
    ([383.0, -266.0,  230.0, -180.00, 0.00, 90.00]),

    ([383.0, -160.0,  230.0, -180.00, 0.00, 90.00]),
    ([383.0, -60.0,  230.0, -180.00, 0.00, 90.00]),
    ([383.0, 20.0,  230.0, -180.00, 0.00, 90.00]),

    ([594.0, -160.0,  230.0, -180.00, 0.00, 90.00]),
    ([594.0, -60.0,  230.0, -180.00, 0.00, 90.00]),
    ([594.0, 20.0,  230.0, -180.00, 0.00, 90.00]),


]

class States(Enum):
    INIT = 0
    HOME_MOVE = 1
    FINISH = 2    
    CLOSE_ROBOT= 3

    OBJECT_AREA =4
    CATCH_OBJECT=5
    READ_OBJECT = 6
    SORT_AREA = 7
    SORT_PLACE = 8
    
    READ_ORDER = 9
    ORDER_OBJECT_AREA =10
    ORDER_OBJECT_PICK = 11
    ORDER_AREA = 12
    ORDER_PLACE = 13
    END_HOME_MOVE = 14

    ERROR_PLACE = 15

class ExampleStrategy(Node):

    def __init__(self):
        super().__init__('example_strategy')
        self.hiwin_client_mo = self.create_client(Motioncmd, 'motioncmd')
        self.hiwin_client_di = self.create_client(Digitalcmd, 'digitalcmd')
        self.hiwin_client_rd = self.create_client(Readcmd, 'readcmd')
        
        self.count_map = {'A': 1, 'B': 1, 'C': 1, 'D': 1,'E': 1,'F': 1,'G': 1}
        self.order_map = {'A': 0, 'B': 1, 'C': 2, 'D': 3,'E':4,'F':5, 'G':6}

        self.catch_items = ['A'] * 3 + ['B'] * 3 + ['C'] * 3 + ['D'] * 3 + ['E'] * 3 + ['F'] * 3 + ['G'] * 3 
        random.shuffle(self.catch_items)
        del self.catch_items[15:]

        self.order_area_num = 0
        self.item = []
        self.oder_items = []   
        self.oder_item = []   

        self.Sorting_palce = []
        self.Order_palce = []
        

        self.catch_num = 0
        self.order_catch_palce_num = 0
        self.order_palce_num = 0

        self.same = 0

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

    def order_callback(self, msg):
        flat_data = msg.item_names
        self.oder_items = [flat_data[i:i+2] for i in range(0, len(flat_data), 2)]

        print("Order received:", self.oder_items)

    def catch_callback(self, msg):
        self.catch_count=msg.items


    def same_thing (self,pose):
        if len(pose) >1:
            if pose[0] == pose[1]:
                del pose[0]
                return 1
        else :
            return 0

    def down_pose(self, pose,state):
        new_pose = pose.copy()  # ← 建立一份新 list
        if state in ('Z', 'C', 'F'):
            new_pose[2] = Down_Offset[2]
        elif state in ('I'):
            new_pose[2] = Down_Offset[3]

        elif state in('B' ,'E'):
            new_pose[2] = Down_Offset[1]
        elif state in('A','D'):
            new_pose[2] = Down_Offset[0]
        elif state in ('G'):
            new_pose[2] = 30.0
        return new_pose


    def _state_machine(self, state: States) -> States:
        if state == States.INIT:
            self.get_logger().info('INIT')
            nest_state = States.HOME_MOVE


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
                pose=HOME_POSE,
                holding=True
                )
            nest_state = States.OBJECT_AREA


        elif state == States.OBJECT_AREA:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=OBJECT_POSES[self.order_area_num],
                holding=False
                )
            nest_state = States.CATCH_OBJECT
            print("\n夾取第",self.order_area_num+1,"次來料區\n")
        

        elif state == States.CATCH_OBJECT:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.down_pose(OBJECT_POSES[self.order_area_num],'Z'),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            for i in range(1, 3):
                res1 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    # digital_input_pin=1,
                    digital_output_pin=IO[i*2],
                    digital_output_cmd=IO_STATE[1],
                    time_wait=0,
                    holding=True
                    )
            res2 = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=OBJECT_POSES[self.order_area_num],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION)
            nest_state = States.READ_OBJECT

        elif state == States.READ_OBJECT:
            print("抓取物品",self.catch_count)
            self.item = self.catch_count
            i = 0
            # self.item = self.catch_items[:Number_of_grips]
            # del self.catch_items[:Number_of_grips]
            print(f"\n🔷 [第 {self.order_area_num+1} 次抓取]：{self.item}")
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
                        col -= 1
                    # 計算位置（加上列的基礎座標 + 欄位間隔）
                    x,y,z,rx,ry,rz = Sorting_area_base[row][col]




                    self.count_map[item] += 1
                    self.Sorting_palce.append([x,y,z,rx,ry,rz])
            self.same = self.same_thing (self.Sorting_palce)
            self.order_area_num += 1
            nest_state = States.SORT_AREA
        
        elif state == States.SORT_AREA:

            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.Sorting_palce[0],
                holding=False
                )
            nest_state = States.SORT_PLACE
            print("準備放置物品",self.catch_num+1)

        elif state == States.SORT_PLACE:
            print(self.item [self.catch_num])
            res1 = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.down_pose(self.Sorting_palce[0],self.item [self.catch_num]),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
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
            res3 = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.Sorting_palce[0],
                holding=False,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            self.catch_num+=1
            print("放置物品",self.catch_num,"完成")  
            del self.Sorting_palce[0]
            if self.Sorting_palce:
                print('next')
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
                        pose=HOME_POSE,
                        holding=True
                        )
                    nest_state = States.READ_ORDER
                    self.get_logger().info('分檢完成')
                    self.catch_num = 0




        elif state == States.READ_ORDER:
            print("進行訂單",self.oder_items)
            self.oder_item = self.oder_items[0]
            del self.oder_items[0]
            if self.oder_item == ['NONE', 'NONE']:
                self.order_palce_num += 1
                print("沒有訂單，跳過")
                nest_state = States.READ_ORDER
            else:
                for j, item in enumerate(self.oder_item):
                    if (self.oder_item[j] == 'NONE'):
                        if j == 0:
                            self.order_catch_palce_num+=1
                        print("")
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

        elif state == States.ORDER_OBJECT_AREA:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.Order_palce[0],
                holding=True
                )
            nest_state = States.ORDER_OBJECT_PICK
            print("抓取",self.order_catch_palce_num+1,"個物品")
        

        elif state == States.ORDER_OBJECT_PICK:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.down_pose(self.Order_palce[0],self.oder_item [self.order_catch_palce_num]),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
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
            res2 = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.Order_palce[0],
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
                nest_state = States.ORDER_AREA

        elif state == States.ORDER_AREA:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=ORDER_POSES[self.order_palce_num],
                holding=True
                )
            print("移動至訂單區")
            nest_state = States.ORDER_PLACE

        elif state == States.ORDER_PLACE:
            # self.get_logger().info('down')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.down_pose(ORDER_POSES[self.order_palce_num],'I'),
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            for i in range(1, 3):
                res1 = self.digital_request_send(
                    cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                    # digital_input_pin=1,
                    digital_output_pin=IO[i*2],
                    digital_output_cmd=IO_STATE[0],
                    time_wait=0,
                    holding=True
                    )
            res2 = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=ORDER_POSES[self.order_palce_num],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            self.order_palce_num+=1
            print("完成",self.order_palce_num+1)  

            if self.oder_items :
                print("next order")
                self.order_palce = []
                self.order_catch_palce_num= 0
                nest_state = States.READ_ORDER

            else:
                nest_state = States.END_HOME_MOVE
                self.get_logger().info('All objects sorted, closing robot')
                print("All objects sorted, closing robot")
                self.num = 0
        elif state == States.END_HOME_MOVE:
            self.get_logger().info('end !!!!!')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=HOME_POSE,
                holding=True
                )
            nest_state = States.CLOSE_ROBOT



        elif state == States.CLOSE_ROBOT:
            self.get_logger().info('CLOSE_ROBOT')
            res1 = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=20,
                digital_output_cmd=IO_STATE[1],
                time_wait=0,
                holding=True
                )
            res2 = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=20,
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
