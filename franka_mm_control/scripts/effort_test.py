#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

class EffortPublisher(Node):

    def __init__(self):
        super().__init__('effort_publisher')
        self.publisher_left = self.create_publisher(
            Float64MultiArray, 
            '/joint_effort_left/commands',
            10
        )
        self.publisher_right = self.create_publisher(
            Float64MultiArray, 
            '/joint_effort_right/commands',
            10
        )

        self.timer_count = 0
        self.timer_period = 0.01  # seconds

        self.amplitude = 1.5
        self.frequency = 1

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        elapsed_time = self.timer_count * self.timer_period
        self.timer_count += 1

        if elapsed_time >= 10:
            self.get_logger().info('Stopping timer after 10 seconds.')
            self.timer.cancel()

            msg = Float64MultiArray()
            msg.data = [0.0] * 7
            self.publisher_left.publish(msg)
            self.publisher_right.publish(msg)
            self.get_logger().info(f'Sent: "{msg.data}"')
            return

        effort = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)

        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, effort, 0.0, 0.0]
        self.publisher_left.publish(msg)
        self.publisher_right.publish(msg)
        self.get_logger().info(f'Sent: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    publisher = EffortPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()