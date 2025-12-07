#!/usr/bin/env python3
"""
ROS 2 Talker Node - Publishes string messages at 1Hz

This example demonstrates the publisher pattern in ROS 2.
The node publishes "Hello World: <count>" messages to the /chatter topic.

Author: Physical AI & Humanoid Robotics Course
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """Publisher node that sends string messages"""

    def __init__(self):
        super().__init__('talker')

        # Create publisher on /chatter topic with queue size 10
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create timer to publish at 1 Hz (every 1.0 seconds)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Message counter
        self.count = 0

        self.get_logger().info('Talker node started - publishing to /chatter at 1 Hz')

    def timer_callback(self):
        """Callback function executed by timer"""
        msg = String()
        msg.data = f'Hello World: {self.count}'

        # Publish the message
        self.publisher.publish(msg)

        # Log to console
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1


def main(args=None):
    """Main function to initialize and spin the node"""
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    talker = TalkerNode()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(talker)
    except KeyboardInterrupt:
        talker.get_logger().info('Talker node shutting down (KeyboardInterrupt)')
    finally:
        # Clean shutdown
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
