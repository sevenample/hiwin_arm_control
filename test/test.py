    ORDER_OBJECT_PALCE = 14
    ORDER_OBJECT_PALCE_DOWN = 15
    ORDER_OBJECT_PALCE_UP = 15

    ORDER_PICK = 16
    ORDER_PLACE = 17
    ORDER_AREA = 18
    ORDER_AREA_DOWN = 19
    ORDER_AREA_UP = 20
    CHECK_ORDER = 21
        elif state == States.CHECK_ORDER:

        elif state == States.ORDER_OBJECT_PALCE:
            self.get_logger().info('Move to object of order')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=order_palce[0],
                holding=True
                )
            nest_state = States.ORDER_OBJECT_PALCE_DOWN
            print("Directly above the object of order:",self.num2)
        

        elif state == States.ORDER_OBJECT_PALCE_DOWN:
            self.get_logger().info('Down')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=order_palce_down[0],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            nest_state = States.ORDER_PICK
            print("ready catch object of order:",self.num2+1)


        elif state == States.ORDER_PICK:
            res = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=self.num2+1,
                digital_output_cmd=Digitalcmd.Request.DIGITAL_ON,
                time_wait=2,
                holding=True
                )
            nest_state = States.ORDER_OBJECT_PALCE_UP
            print("PICK object of order:",self.num2+1)
            
        elif state == States.ORDER_OBJECT_PALCE_UP:
            self.get_logger().info(' Up  ')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=order_palce_down[0],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            self.num2 += 1
            del self.Sorting_palce[0]
            del self.Sorting_palce_down[0]
            if self.Sorting_palce:
                nest_state = States.ORDER_OBJECT_PALCE
            else:
                nest_state = States.ORDER_AREA

        elif state == States.ORDER_AREA:
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.PTP,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=ORDER_POSES[self.num3][1],
                holding=True
                )
            nest_state = States.ORDER_AREA_DOWN
            print("Sort object:",self.num+1)

        elif state == States.ORDER_AREA_DOWN:
            self.get_logger().info('down')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=ORDER_POSES[self.num3][0],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            nest_state = States.ORDER_PLACE
            print("Sort object down:",self.num+1) 
             
        elif state == States.ORDER_PLACE:
            res = self.digital_request_send(
                cmd_mode=Digitalcmd.Request.DIGITAL_OUTPUT,
                # digital_input_pin=1,
                digital_output_pin=1
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
            print("Place object:",self.num+1)
            nest_state = States.ORDER_AREA_UP

        elif state == States.ORDER_AREA_UP:
            self.get_logger().info('up')
            res = self.motion_request_send(
                cmd_mode=Motioncmd.Request.LINE,
                cmd_type=Motioncmd.Request.POSE_CMD,
                pose=ORDER_POSES[self.num3][1],
                holding=True,
                velocity=LINE_VELOCITY,
                acceleration=LINE_ACCELERATION
                )
            self.num+=1
            print("Sort object up:",self.num+1)  
            if self.num < 2:
                
                nest_state = States.SORT_PLACE
            else:
                if self.items:
                    self.order_palce_DOWN =[]
                    self.order_palce = []
                    self.num2= 0
                    nest_state = States.OBJECT_AREA

                else:
                    nest_state = States.CLOSE_ROBOT
                    self.get_logger().info('All objects sorted, closing robot')
                    print("All objects sorted, closing robot")
                    self.num = 0