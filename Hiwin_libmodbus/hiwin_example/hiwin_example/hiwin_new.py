#!/usr/bin/env python3
import time
import rclpy
from enum import Enum
from threading import Thread
from rclpy.node import Node
from rclpy.task import Future
from typing import NamedTuple
from hiwin_msgs.msg import OrderArray, CatchArray
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

DEFAULT_VELOCITY = 20
DEFAULT_ACCELERATION = 20
LINE_VELOCITY = 80
LINE_ACCELERATION = 80
VACUUM1_PIN = 3
VACUUM2_PIN = 4

HOME_POSE = [0.00, 368.00, 293.00, -180.00, 0.00, 90.000]

Sorting_area_base = [
    ([300.0, 427.0, 185.0, -180.0, 0.00, 90.00], [225.0, 427.0, 185.0, -180.0, 0.00, 90.00],[150.0, 427.0, 185.0, -180.0, 0.00, 90.00],[75.0, 427.0, 185.0, -180.0, 0.00, 90.00]),  # A row
    ([300.0, 566.0, 235.0, -180.0, 0.00, 90.00],[225.0, 566.0, 235.0, -180.0, 0.00, 90.00],[150.0, 566.0, 235.0, -180.0, 0.00, 90.00],[75.0, 566.0, 235.0, -180.0, 0.00, 90.00]), # B row
    ([0.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-75.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-150.0, 427.0, 185.0, -180.0, 0.00, 90.00],[-225.0, 427.0, 185.0, -180.0, 0.00, 90.00]),  # C row
    ([0.0, 566.0, 235.0, -180.0, 0.00, 90.00],[-75.0, 566.0, 235.0, -180.0, 0.00, 90.00],[-150.0, 566.0, 235.0, -180.0, 0.00, 90.00],[-225.0, 566.0, 235.0, -180.0, 0.00, 90.00])   # D row
]
OBJECT_POSES = [
    ([-346.0, 229.0, 180.0, -180.00, 0.00, 90.00], [-346.0, 229.0, 290.0, -180.00, 0.00, 90.00]),
    ([-346.0, 309.0, 180.0, -180.00, 0.00, 90.00], [-346.0, 309.0, 290.0, -180.00, 0.00, 90.00]),
    ([-346.0, 368.0, 180.0, -180.00, 0.00, 90.00], [-346.0, 368.0, 290.0, -180.00, 0.00, 90.00]),
    ([-346.0, 229.0, 180.0, -180.00, 0.00, 90.00], [-346.0, 229.0, 290.0, -180.00, 0.00, 90.00]),
]
ORDER_POSES = [
    ([346.0, 229.0, 180.0, -180.00, 0.00, 90.00], [346.0, 229.0, 290.0, -180.00, 0.00, 90.00]),
    ([346.0, 309.0, 180.0, -180.00, 0.00, 90.00], [346.0, 309.0, 290.0, -180.00, 0.00, 90.00]),
    ([346.0, 368.0, 180.0, -180.00, 0.00, 90.00], [346.0, 368.0, 290.0, -180.00, 0.00, 90.00]),
    ([346.0, 229.0, 180.0, -180.00, 0.00, 90.00], [346.0, 229.0, 290.0, -180.00, 0.00, 90.00]),
]
# only for example as we don't use yolo here
# assume NUM_OBJECTS=5, then this process will loop 5 times
class States(Enum):
    INIT = 0
    HOME_MOVE = 1
    FINISH = 2    
    CLOSE_ROBOT= 3

    UP_MOVE = 4
    DOWN_MOVE = 5
    PICK_OBJECT = 6
    PLACE_OBJECT = 7
    READ_OBJECT = 8
    SORT_PLACE = 9
    SORT_PLACE_DOWN = 10
    SORT_PLACE_UP = 11
    CHECK_POSE = 12
    OBJECT_AREA = 13


    ORDER_OBJECT_PALCE = 14
    ORDER_OBJECT_PALCE_DOWN = 15
    ORDER_OBJECT_PALCE_UP = 16

    ORDER_PICK = 17
    ORDER_PLACE = 18
    ORDER_AREA = 19
    ORDER_AREA_DOWN = 20
    ORDER_AREA_UP = 21
    CHECK_ORDER = 22
    SORT_ORDER = 23
    TT = 24  # Temporary state for processing order items
    END_HOME = 25  # End home state after all orders are processed





class ExampleStrategy(Node):

    def __init__(self):
        super().__init__('example_strategy')
        self.hiwin_client_mo = self.create_client(Motioncmd, 'motioncmd')
        self.hiwin_client_di = self.create_client(Digitalcmd, 'digitalcmd')
        self.hiwin_client_rd = self.create_client(Readcmd, 'readcmd')
        
        self.count_map =        {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0}
        self.order_map =        {'A': 0, 'B': 1, 'C': 2, 'D': 3, 'E': 4, 'F': 5}
        self.sort_count_map =   {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0}
        self.sort_order_map =   {'A': 0, 'B': 1, 'C': 2, 'D': 3, 'E': 4, 'F': 5}
        
# --------------------測試-----------------------
        self.catch_items = ['A'] * 3 + ['B'] * 3 + ['C'] * 3 + ['D'] * 3
        random.shuffle(self.catch_items)

# ---------------------分檢變數-------------------------
        self.object_area_num = 0
        self.item = []
        self.Sorting_palce = []
        self.Sorting_palce_DOWN = [] 
        self.catch_num = 0

# ---------------訂單用變數------------------
        self.oder_items = []   
        self.oder_item = []  
        self.Order_palce = []
        self.Order_palce_DOWN = []
        self.order_catch_palce_num = 0
        self.order_palce_num = 0
# ---------------------------------------------

        # 訂閱 OrderArray 類型的訊息
        self.order_subscription = self.create_subscription(
            OrderArray,
            'order_list',
            self.order_callback,
            10)
        
        # 訂閱 CatchArray 類型的訊息
        self.catch_subscription = self.create_subscription(
            CatchArray,
            'catch_list',
            self.catch_callback,
            10)
        
        # 初始化統計數據
        self.order_count = []
        self.catch_count = []

    def order_callback(self, msg):
        items = []
        for count in msg.quantities:
            items.append(count)
        self.order_count.append(items)
        print("Order received:", self.order_count)

    def catch_callback(self, msg):
        self.catch_count=msg.items
        print("Catch received:", self.catch_count)

    def _state_machine(self, state: States) -> States:
        if state == States.INIT:
            self.get_logger().info('INIT')
            nest_state = States.HOME_MOVE


        elif state == States.HOME_MOVE:
            
            self.get_logger().info('HOME_MOVE !!!!!')
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
                pose=OBJECT_POSES[self.object_area_num][1],
                holding=True
                )
            nest_state = States.DOWN_MOVE
            print("Directly above the object:",self.object_area_num+1)
        

        elif state == States.DOWN_MOVE:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=OBJECT_POSES[self.object_area_num][0],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            nest_state = States.PICK_OBJECT
            print("Ready catch object:",self.object_area_num)


        elif state == States.PICK_OBJECT:
            res = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=1,
                digital_output_cmd=Digitalcmd.Request.DIGITAL_ON,
                time_wait=2,
                holding=True
                )
            res2 = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=2,
                digital_output_cmd=Digitalcmd.Request.DIGITAL_ON,
                time_wait=2,
                holding=True
                )
            res3 = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=3,
                digital_output_cmd=Digitalcmd.Request.DIGITAL_ON,
                time_wait=2,
                holding=True
                )
            nest_state = States.UP_MOVE
            print("PICK object")
            
        elif state == States.UP_MOVE:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=OBJECT_POSES[self.object_area_num][1],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            self.object_area_num += 1
            nest_state = States.READ_OBJECT

        elif state == States.READ_OBJECT:
            self.item = self.catch_items[:3]
            del self.catch_items[:3]
            print(f"\n🔷 [第 {self.object_area_num} 次抓取]：{self.item}")
            for j, item in enumerate(self.item):
                row = self.order_map[item]
                col = self.count_map[item]

                if col >= 3:
                    raise ValueError(f"{item} 類物體已放滿！")

                # 計算位置（加上列的基礎座標 + 欄位間隔）
                x,y,z,rx,ry,rz = Sorting_area_base[row][col]

                if j == 0:
                    x -= 75.0
                elif j == 2:
                    x += 75.0


                self.count_map[item] += 1
                self.Sorting_palce.append([x,y,z,rx,ry,rz])
                self.Sorting_palce_DOWN.append([x, y, z-50.0,rx,ry,rz])
            nest_state = States.SORT_PLACE

        elif state == States.SORT_PLACE:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.Sorting_palce[self.catch_num],
                holding=True
                )
            nest_state = States.SORT_PLACE_DOWN
            print("Sort object:",self.catch_num+1)

        elif state == States.SORT_PLACE_DOWN:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.Sorting_palce_DOWN[self.catch_num],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            nest_state = States.PLACE_OBJECT
            print("Sort object down:",self.catch_num+1) 
             
        elif state == States.PLACE_OBJECT:
            res = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=self.catch_num+1,
                digital_output_cmd=Digitalcmd.Request.DIGITAL_OFF,
                time_wait=2,
                holding=True
                )
            print("Place object:",self.catch_num+1)
            nest_state = States.SORT_PLACE_UP

        elif state == States.SORT_PLACE_UP:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.Sorting_palce[self.catch_num],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            self.catch_num+=1
            print("Sort object up:",self.catch_num+1)  
            if self.catch_num < 3:
                
                nest_state = States.SORT_PLACE
            else:
                if  self.object_area_num < 4:
                    self.Sorting_palce_DOWN =[]
                    self.Sorting_palce = []
                    self.catch_num = 0
                    nest_state = States.OBJECT_AREA

                else:
                    nest_state = States.TT
                    self.get_logger().info('All objects sorted')


        elif state == States.TT:
            col_counter = 1
            for row,col in enumerate(self.order_count, start=0):
                tt = []
                for i,counter in enumerate(col, start=0):
                    for j in range(counter):
                        if i == 0:
                            tt.append('A')
                        elif i == 1:
                            tt.append('B')
                        elif i == 2:    
                            tt.append('C')
                        elif i == 3:
                            tt.append('D')
                        elif i == 4:
                            tt.append('E')
                        elif i == 5:
                            tt.append('F')
                        if (col_counter)%3 == 0:
                            self.oder_items.append(tt)    
                            tt = []
                            col_counter = 1
                        else:
                            col_counter+=1
                col_counter = 1
                self.oder_items.append(tt)
            nest_state = States.SORT_ORDER

        elif state == States.SORT_ORDER:
            print("Sort order items:",self.oder_items)
            self.oder_item = self.oder_items[0]
            del self.oder_items[0]
            for j, item in enumerate(self.oder_item):
                row = self.sort_order_map[item]
                col = self.sort_count_map[item]


                # 計算位置（加上列的基礎座標 + 欄位間隔）
                # x,y,z,rx,ry,rz = Sorting_area_base[row]
                # x = x - col * 75.0

                x,y,z,rx,ry,rz = Sorting_area_base[row][col]

                if j == 0:
                    x -= 75.0
                    print("👉 第一個物體：夾具偏移 (x - 50)")
                elif j == 2:
                    x += 75.0
                    print("🔁 第三個物體：夾具偏移 (x + 50)")
                self.sort_count_map[item] += 1
                self.Order_palce.append([x,y,z,rx,ry,rz])
                self.Order_palce_DOWN.append([x,y,z-50,rx,ry,rz])

            print(f"🔷 {self.Order_palce}")
            nest_state = States.ORDER_OBJECT_PALCE

        elif state == States.ORDER_OBJECT_PALCE:
            # self.get_logger().info('Move to object of order')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.Order_palce[0],
                holding=True
                )
            nest_state = States.ORDER_OBJECT_PALCE_DOWN
            print("Directly above the object of order:",self.order_catch_palce_num+1)
        

        elif state == States.ORDER_OBJECT_PALCE_DOWN:
            # self.get_logger().info('Down')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.Order_palce_DOWN[0],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            nest_state = States.ORDER_PICK
            print("ready catch object of order:",self.order_catch_palce_num+1)


        elif state == States.ORDER_PICK:
            res = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=self.order_catch_palce_num+1,
                digital_output_cmd=Digitalcmd.Request.DIGITAL_ON,
                time_wait=2,
                holding=True
                )
            nest_state = States.ORDER_OBJECT_PALCE_UP
            print("PICK object of order:",self.order_catch_palce_num+1)
            
        elif state == States.ORDER_OBJECT_PALCE_UP:
            # self.get_logger().info(' Up  ')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=self.Order_palce[0],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            self.order_catch_palce_num += 1
            del self.Order_palce[0]
            del self.Order_palce_DOWN[0]
            if self.Order_palce:
                nest_state = States.ORDER_OBJECT_PALCE
            else:
                nest_state = States.ORDER_AREA

        elif state == States.ORDER_AREA:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=ORDER_POSES[self.order_palce_num][1],
                holding=True
                )
            nest_state = States.ORDER_AREA_DOWN

        elif state == States.ORDER_AREA_DOWN:
            # self.get_logger().info('down')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=ORDER_POSES[self.order_palce_num][0],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            nest_state = States.ORDER_PLACE
            
        elif state == States.ORDER_PLACE:
            res = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=1,
                digital_output_cmd=Digitalcmd.Request.DIGITAL_OFF,
                time_wait=2,
                holding=True
                )
            res1 = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=2,
                digital_output_cmd=Digitalcmd.Request.DIGITAL_OFF,
                time_wait=2,
                holding=True
                )
            res2 = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=3,
                digital_output_cmd=Digitalcmd.Request.DIGITAL_OFF,
                time_wait=2,
                holding=True
                )
            nest_state = States.ORDER_AREA_UP

        elif state == States.ORDER_AREA_UP:
            # self.get_logger().info('up')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=ORDER_POSES[self.order_palce_num][1],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            self.order_palce_num+=1
            print("Sort object up:",self.order_palce_num+1)  
            if not len(self.oder_items[0])==0:
                print("next order")
                self.order_palce_DOWN =[]
                self.order_palce = []
                self.order_catch_palce_num= 0
                nest_state = States.SORT_ORDER

            else:
                nest_state = States.END_HOME
                self.get_logger().info('All objects sorted, closing robot')
                print("All objects sorted, closing robot")

        elif state == States.END_HOME:
            self.get_logger().info('HOME_MOVE !!!!!')
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
                digital_output_cmd=Digitalcmd.Request.DIGITAL_OFF,
                time_wait=0,
                holding=True
                )
            res2 = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=20,
                digital_output_cmd=Digitalcmd.Request.DIGITAL_OFF,
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