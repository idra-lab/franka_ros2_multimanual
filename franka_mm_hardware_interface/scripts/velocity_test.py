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
        timer_period = 0.5  # seconds
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.amplitude = 0.3
        self.frequency = 0.2

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9  # seconds

        if elapsed_time >= 5:
            self.get_logger().info('Stopping timer after 5 seconds.')
            self.timer.cancel()

            msg = Float64MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher_left.publish(msg)
            self.publisher_right.publish(msg)
            self.get_logger().info(f'Sent: "{msg.data}"')

            return

        velocity = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)

        msg = Float64MultiArray()
        msg.data = [velocity, velocity, velocity, velocity, velocity, velocity, velocity ]
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