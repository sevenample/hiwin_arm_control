#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hiwin_interfaces.srv import Digitalcmd
from hiwin_interfaces.srv import RobotCommand
import time
from rclpy.task import Future
from threading import Thread




from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup


# include "hiwin_interfaces/srv/robot_command.hpp"
read_DI=1
write_DI=2






class MinimalService(Node):

    def __init__(self):
        super().__init__('inout_service')
        client_group = ReentrantCallbackGroup()
        server_group = client_group
        self.srv = self.create_service(Digitalcmd, 'inout_sever', self.inout_callback,callback_group=server_group)
        self.cli = self.create_client(RobotCommand, 'hiwinmodbus_service',callback_group=client_group)
        # self.srv = self.create_service(RobotCommand, 'inout_sever', self.inout_callback)
        # self.cli = self.create_client(RobotCommand, 'hiwinmodbus_service')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req=RobotCommand.Request()
        # self.response=RobotCommand.Response()
        self.flag=0
        
    def inout_callback(self, request, response):
        self.get_logger().info('heard : %d'%request.cmd_mode)
        response.arm_state+=1
        print("sssss=",response)
        if request.cmd_mode==request.READ_DI:
            self.req.cmd_mode=self.req.READ_DI
            self.req.digital_input_pin = request.digital_input_pin
            self.req.holding=False

            
        

        elif request.cmd_mode==request.DIGITAL_OUTPUT:
            self.req.cmd_mode=self.req.DIGITAL_OUTPUT
            # self.req.do_timer=request.do_timer
            self.req.digital_output_pin=request.digital_output_pin
            self.req.digital_output_cmd=request.digital_output_cmd

            self.req.holding=False
        # self.main_loop_thread = Thread(target=self.call_hiwin)
        # self.main_loop_thread.daemon = True
        # self.main_loop_thread.start()
        # self.main_loop_thread.join()
        response=self.call_hiwin(self.req)
        print("res=",response)
        return response



    # def _main_loop(self):
    #     while 1:
    #         pass
    #     self.destroy_node()
    def call_hiwin(self,req):
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        future = self.cli.call_async(req)
        self.get_logger().info('Waiting for future to complete...')
        rclpy.spin_until_future_complete(self,future)
        if future.done():
            response = future.result()
            print("qqqqqqq=",response)
            self.get_logger().info(f'Received from hiwinmodbus_service: {response.arm_state}')
            return response
        else:
            self.response = None
        # return res








    def _wait_for_future_done(self, future: Future, timeout=5):
        time_start = time.time()
        while not future.done():
            time.sleep(0.01)
            if timeout > 0 and time.time() - time_start > timeout:
                self.get_logger().error('Wait for service timeout!')
                return False
        return True
    # def start_main_loop_thread(self):
    #     self.main_loop_thread = Thread(target=self._main_loop)
    #     self.main_loop_thread.daemon = True
    #     self.main_loop_thread.start()

def main(args=None):
    rclpy.init(args=args)

    inout_service = MinimalService()
    executor = MultiThreadedExecutor()
    executor.add_node(inout_service)
    # try:
    inout_service.get_logger().info('Beginning client, shut down with CTRL-C')
    executor.spin()
    print("gffsdf")
    # except KeyboardInterrupt:
    #     inout_service.get_logger().info('Keyboard interrupt, shutting down.\n')
    # minimal_service.start_main_loop_thread()
    # rclpy.spin(inout_service)
    inout_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()