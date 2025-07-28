#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_publisher')

        self.publisher_left = self.create_publisher(Float64MultiArray, '/joint_position_left/commands', 10)
        self.publisher_right = self.create_publisher(Float64MultiArray, '/joint_position_right/commands', 10)

        self.timer_period = 0.001  # seconds
        self.timer_count = 0

        self.amplitude = 0.5
        self.frequency = 0.001

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.base_position = []
        self.received_joint_state = False

        self.create_subscription(Float64MultiArray, '/current_position', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        if self.received_joint_state:
            return

        temp = list(msg.data)

        if len(temp) % 7 == 0:
            self.received_joint_state = True
            self.base_position = np.array(temp).reshape(-1, 7)
            self.get_logger().info(f"Base positions: {self.base_position}")
        else:
            self.get_logger().warn('Unexpected size of the array')


    def timer_callback(self):
        if not self.received_joint_state or len(self.base_position) < 2:
            return  # Still waiting for valid joint states

        elapsed_time = self.timer_count * self.timer_period
        self.timer_count += 1

        if elapsed_time >= 100:
            self.get_logger().info('Stopping timer after 5 seconds.')
            self.timer.cancel()

            # msg = Float64MultiArray()
            # msg.data = self.base_position
            # self.publisher_left.publish(msg)
            # self.publisher_right.publish(msg)
            # self.get_logger().info(f'Sent final position: {msg.data}')
            return

        position_offset = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)

        modulated_position_left = self.base_position[1].copy().tolist()
        modulated_position_left[3] += position_offset
        msg_left = Float64MultiArray()
        msg_left.data = modulated_position_left
        self.publisher_left.publish(msg_left)
        self.get_logger().info(f'Left: {msg_left.data}')

        modulated_position_right = self.base_position[0].copy().tolist()
        modulated_position_right[3] += position_offset
        msg_right = Float64MultiArray()
        msg_right.data = modulated_position_right
        # self.publisher_right.publish(msg_right)




def main(args=None):
    rclpy.init(args=args)
    publisher = PositionPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()