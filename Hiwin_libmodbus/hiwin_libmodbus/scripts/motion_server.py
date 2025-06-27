#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hiwin_interface.srv import Motioncmd



class MinimalService(Node):

    def __init__(self):
        super().__init__('motion_service')
        self.srv = self.create_service(Motioncmd, 'motion_sever', self.motion_callback)

    def motion_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()