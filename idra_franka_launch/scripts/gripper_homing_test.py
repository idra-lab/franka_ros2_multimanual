#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import Homing

# /frankaX/franka_gripper/grasp
# /frankaX/franka_gripper/homing
# /frankaX/franka_gripper/move

class GripperHomingClient(Node):

    def __init__(self):
        super().__init__('gripper_home_client')
        self.action_clients = []
        self.robot_names = ["franka1", "franka2"]

        for robot in self.robot_names:
            self.action_clients.append(ActionClient(self, Homing, f'/{robot}/franka_gripper/homing'))

    def send_goal(self):
        goal_msg = Homing.Goal()

        for client in self.action_clients:
            client.wait_for_server()
            self.get_logger().info('Sending homing goal...')
            self._send_goal_future = client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Grasp success: {result.success}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    gripper_client = GripperHomingClient()
    gripper_client.send_goal()
    rclpy.spin(gripper_client)

if __name__ == '__main__':
    main()
