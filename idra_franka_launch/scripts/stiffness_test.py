#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from franka_msgs.srv import SetJointStiffness  # Replace with correct package

class StiffnessClient(Node):

    def __init__(self):
        super().__init__('stiffness_client')
        self.client = self.create_client(SetJointStiffness, '/franka2_service_server/set_joint_stiffness')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_joint_stiffness service...')

        # Prepare request
        request = SetJointStiffness.Request()
        request.joint_stiffness = [10.0] * 7  # Example stiffness values

        # Send request
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Stiffness set successfully: {response.message}')
            else:
                self.get_logger().warn(f'Failed to set stiffness: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

        # Optional: shutdown after service call completes
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = StiffnessClient()
    rclpy.spin(node)

if __name__ == '__main__':
    main()