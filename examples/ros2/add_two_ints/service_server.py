#!/usr/bin/env python3
"""
ROS 2 Service Server - Adds two integers

This example demonstrates the service server pattern in ROS 2.
The server receives requests with two integers and returns their sum.

Author: Physical AI & Humanoid Robotics Course
License: MIT
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """Service server that adds two integers"""

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Service server started - waiting for requests on /add_two_ints')

    def add_two_ints_callback(self, request, response):
        """
        Service callback - processes request and generates response

        Args:
            request: AddTwoInts.Request with fields .a and .b
            response: AddTwoInts.Response with field .sum

        Returns:
            response: Filled response object
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    server = AddTwoIntsServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Service server shutting down')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
