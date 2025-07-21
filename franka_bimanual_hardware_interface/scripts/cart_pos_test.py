#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
import math
import numpy as np


class SmoothCartesianPosePublisher(Node):

    def __init__(self):
        super().__init__('smooth_cartesian_pose_publisher')

        self.current_pose_data = [0.0] * 16
        self.received_indices = set()
        self.initial_pose = None
        self.last_sent_pose = None

        # Subscribe to each of the 16 state interfaces
        topic = f'/initialpose'
        self.create_subscription(
            Float64MultiArray,
            topic,
            self.pose_callback,
            10
        )

        # Publishers
        self.publisher_left = self.create_publisher(Float64MultiArray, '/cartesian_pose_left/commands', 10)
        self.publisher_right = self.create_publisher(Float64MultiArray, '/cartesian_pose_right/commands', 10)

        # Motion parameters
        self.amplitude = 0.03  # max displacement along X [meters]
        self.frequency = 1 / 5  # Hz

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.timer_callback)

    def pose_callback(self, msg):
        if len(msg.data) != 16:
            self.get_logger().warn("Received pose message with wrong size")
            return
        # Convert to numpy 4x4 matrix (column-major)
        self.current_pose_data = np.array(msg.data).reshape((4, 4), order='F')

    def timer_callback(self):
        if self.current_pose_data is None:
            self.get_logger().warn("Waiting for first pose message...")
            return

        # On first valid pose, save initial pose
        if not hasattr(self, 'initial_pose'):
            self.initial_pose = np.copy(self.current_pose_data)
            self.start_time = self.get_clock().now()
        
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9

        # Initialize the initial_pose once
        if self.initial_pose is None:
            pose_matrix = np.array(self.current_pose_data).reshape((4, 4), order='F')
            self.initial_pose = np.copy(pose_matrix)
            self.last_sent_pose = np.copy(pose_matrix)

        # Generate smooth X displacement using cosine easing
        dx = self.amplitude * (1 - math.cos(2 * math.pi * self.frequency * elapsed_time)) / 2

        # Apply offset to initial pose
        new_pose = np.copy(self.initial_pose)
        new_pose[0, 3] += dx  # move along X

        # Publish in column-major format
        msg = Float64MultiArray()
        msg.data = new_pose.T.flatten().tolist()  # Transpose for column-major
        self.publisher_left.publish(msg)
        self.publisher_right.publish(msg)

        self.last_sent_pose = new_pose
        self.get_logger().info(f'Publishing dx={dx:.4f}')


def main(args=None):
    rclpy.init(args=args)
    node = SmoothCartesianPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
