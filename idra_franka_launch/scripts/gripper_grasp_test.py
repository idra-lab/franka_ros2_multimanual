#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import Grasp

# /frankaX/franka_gripper/grasp
# /frankaX/franka_gripper/gripper_action
# /frankaX/franka_gripper/homing
# /frankaX/franka_gripper/move

class GripperGraspClient(Node):

    def __init__(self):
        super().__init__('gripper_grasp_client')
        self._action_client = ActionClient(self, Grasp, 'franka1/franka_gripper/grasp')

    def send_goal(self, width, epsilon_inner, epsilon_outer, speed, force):
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.epsilon.inner = epsilon_inner
        goal_msg.epsilon.outer = epsilon_outer
        goal_msg.speed = speed
        goal_msg.force = force

        self._action_client.wait_for_server()
        self.get_logger().info('Sending grasp goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
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
    gripper_client = GripperGraspClient()

    # Example parameters for grasp
    width = 0.  # in meters
    epsilon_inner = 0.005
    epsilon_outer = 0.005
    speed = 0.1
    force = 20.0

    gripper_client.send_goal(width, epsilon_inner, epsilon_outer, speed, force)
    rclpy.spin(gripper_client)


if __name__ == '__main__':
    main()
