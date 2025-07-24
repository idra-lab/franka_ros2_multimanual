#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import math


class SmoothCartesianPosePublisher(Node):

    def __init__(self):
        super().__init__('smooth_cartesian_pose_publisher')

        self.current_pose_data = None
        self.initial_pose = None

        # Subscribe to pose
        self.create_subscription(
            Float64MultiArray,
            '/current_pose',
            self.pose_callback,
            10
        )

        # Publishers (assumed to be different topics, otherwise remove one)
        self.publisher_left = self.create_publisher(Float64MultiArray, '/cartesian_pose_left/commands', 10)
        self.publisher_right = self.create_publisher(Float64MultiArray, '/cartesian_pose_right/commands', 10)

        # Motion parameters
        self.amplitude = 0.003  # [m]
        self.frequency = 1 / 100  # Hz

        self.start_time = None
        self.timer = self.create_timer(0.05, self.timer_callback)

    def pose_callback(self, msg):
        if len(msg.data) % 16 != 0:
            self.get_logger().error("Invalid pose message: length must be a multiple of 16.")
            return

        temp = np.array(msg.data, dtype=np.float64)
        self.current_pose_data = temp.reshape(-1, 16)

        if self.initial_pose is None:
            self.initial_pose = np.copy(self.current_pose_data)
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"Initial pose recorded: {self.initial_pose}")

    def timer_callback(self):
        if self.current_pose_data is None or self.initial_pose is None:
            self.get_logger().warn("Waiting for valid initial pose...")
            return

        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9

        dx = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)

        # Apply displacement to a copy of the initial pose
        new_pose = np.copy(self.initial_pose)
        new_pose[:, 12] += dx  # Apply to X column

        # Flatten and publish
        msg = Float64MultiArray()
        msg.data = new_pose[0].tolist()
        self.publisher_right.publish(msg)
        
        msg = Float64MultiArray()
        msg.data = new_pose[1].tolist()
        self.publisher_left.publish(msg)
    
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SmoothCartesianPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
