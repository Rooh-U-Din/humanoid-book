#!/usr/bin/env python3
"""
ROS 2 Listener Node - Subscribes to string messages

This example demonstrates the subscriber pattern in ROS 2.
The node listens to messages on the /chatter topic and logs them.

Author: Physical AI & Humanoid Robotics Course
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """Subscriber node that receives string messages"""

    def __init__(self):
        super().__init__('listener')

        # Create subscription to /chatter topic with queue size 10
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning
        self.subscription

        self.get_logger().info('Listener node started - subscribed to /chatter')

    def listener_callback(self, msg):
        """Callback function executed when message is received"""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function to initialize and spin the node"""
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    listener = ListenerNode()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(listener)
    except KeyboardInterrupt:
        listener.get_logger().info('Listener node shutting down (KeyboardInterrupt)')
    finally:
        # Clean shutdown
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
