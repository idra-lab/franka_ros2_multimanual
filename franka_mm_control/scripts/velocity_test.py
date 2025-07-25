#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_left = self.create_publisher(
            Float64MultiArray, 
            '/joint_velocity_left/commands',
            10
        )
        self.publisher_right = self.create_publisher(
            Float64MultiArray, 
            '/joint_velocity_right/commands',
            10
        )

        self.timer_count = 0
        self.timer_period = 0.5  # seconds

        self.amplitude = 0.3
        self.frequency = 0.2

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        elapsed_time = self.timer_count * self.timer_period
        self.timer_count += 1

        if elapsed_time >= 5:
            self.get_logger().info('Stopping timer after 5 seconds.')
            self.timer.cancel()

            msg = Float64MultiArray()
            msg.data = [0.0] * 7
            self.publisher_left.publish(msg)
            self.publisher_right.publish(msg)
            self.get_logger().info(f'Sent: "{msg.data}"')
            return

        velocity = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)

        msg = Float64MultiArray()
        msg.data = [velocity] * 7
        self.publisher_left.publish(msg)
        self.publisher_right.publish(msg)
        self.get_logger().info(f'Sent: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    publisher = VelocityPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()