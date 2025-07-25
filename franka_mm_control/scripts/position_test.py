#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_publisher')

        self.publisher_left = self.create_publisher(Float64MultiArray, '/joint_position_left/commands', 10)
        self.publisher_right = self.create_publisher(Float64MultiArray, '/joint_position_right/commands', 10)

        self.timer_period = 0.01  # seconds
        self.timer_count = 0

        self.amplitude = 0.5
        self.frequency = 1 / 10

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.base_position = []
        self.received_joint_state = False

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def joint_state_callback(self, msg):
        if self.received_joint_state:
            return

        r_names = ["franka1", "franka2"]
        self.received_joint_state = True
        for i_name in range(len(r_names)):
            expected_joint_names = [f"{r_names[i_name]}_fr3_joint{i}" for i in range(1, 8)]

            name_to_index = {name: idx for idx, name in enumerate(msg.name)}

            # Check if all expected joints are present
            if all(joint in name_to_index for joint in expected_joint_names):
                # Extract joint positions in the correct order
                self.base_position.append([
                    msg.position[name_to_index[joint]] for joint in expected_joint_names
                ])
                self.get_logger().info(f"Base position for {r_names[i_name]} received: {self.base_position[i_name]}")
            else:
                missing = [j for j in expected_joint_names if j not in name_to_index]
                self.get_logger().warn(f"Waiting for joints: {missing}")

    def timer_callback(self):
        elapsed_time = self.timer_count * self.timer_period
        self.timer_count += 1

        if elapsed_time >= 10:
            self.get_logger().info('Stopping timer after 5 seconds.')
            self.timer.cancel()

            # msg = Float64MultiArray()
            # msg.data = self.base_position
            # self.publisher_left.publish(msg)
            # self.publisher_right.publish(msg)
            # self.get_logger().info(f'Sent final position: {msg.data}')
            return

        position_offset = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)

        msg = Float64MultiArray()
        modulated_position = self.base_position[0].copy()
        modulated_position[3] += position_offset  # Apply sinusoid to joint index 3
        msg.data = modulated_position
        self.publisher_left.publish(msg)

        msg = Float64MultiArray()
        modulated_position = self.base_position[1].copy()
        modulated_position[3] += position_offset  # Apply sinusoid to joint index 3
        msg.data = modulated_position
        self.publisher_right.publish(msg)
        self.get_logger().info(f'Sent: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    publisher = PositionPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()