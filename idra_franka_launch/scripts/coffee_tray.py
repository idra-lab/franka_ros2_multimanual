#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

import threading

import rclpy
import time
from rclpy.node import Node, Publisher
from rclpy.action import ActionClient
from rclpy.task import Future

from std_msgs.msg import Float64MultiArray
from franka_msgs.action import Grasp

class CoffeeTray(Node):

    def __init__(self):
        super().__init__('gear_pick')

        # Initial position get
        self.initial_pose = None
        self.create_subscription(
            Float64MultiArray,
            '/current_q_pose',
            self.pose_callback,
            10
        )

        self.grasp_left    = ActionClient(self, Grasp, 'franka2/franka_gripper/grasp')
        self.control_left  = self.create_publisher(Float64MultiArray, '/cartesian_impedance_left/commands', 10)

        self.grasp_right   = ActionClient(self, Grasp, 'franka1/franka_gripper/grasp')
        self.control_right = self.create_publisher(Float64MultiArray, '/cartesian_impedance_right/commands', 10)

        self.tray_width = 0.450

    def pose_callback(self, msg):
        if len(msg.data) % 7 != 0:
            self.get_logger().error("Invalid pose message: length must be a multiple of 7.")
            return

        temp = np.array(msg.data, dtype=np.float64)
        self.current_pose_data = temp.reshape(-1, 7)

        if self.initial_pose is None:
            self.initial_pose = np.copy(self.current_pose_data)
            self.get_logger().info(f"Initial pose recorded: {self.initial_pose}")
            self.run() # START

    def get_rotation(self, x, y, z):
        # Step 1: Define rotation
        rotation = R.from_euler('zyx', [z, y, x], degrees=True)  # Order: x first, then y, then z

        # Step 2: Convert to quaternion
        quaternion = rotation.as_quat()  # Returns [x, y, z, w]

        return [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]

    def run(self):
        left_rotation  = self.get_rotation(-90, 0, 0)
        right_rotation = self.get_rotation( 90, 0, 0)

        self.get_logger().info(f"Motion start. Please pay attention")

        self.grasp(self.grasp_left, 0.5, 0.1, 0.0)
        self.grasp(self.grasp_right, 0.5, 0.1, 0.0)

        time.sleep(5)

        start = [
            0.33514574174014890000,
            0.02643892168232797000,
            0.46694372999730827000,
            .0,
            1.0,
            .0,
            .0,
        ]

        self.get_logger().info(f"Going to the start")
        f_start_left  = self.execute_trajectory(self.control_left, self.create_trajectory(self.initial_pose[1], start))
        f_start_right = self.execute_trajectory(self.control_right, self.create_trajectory(self.initial_pose[0], start))

        while not (f_start_left.done() and f_start_right.done()):
            pass

        left_pick_up = [
            0.308,
            0.065,
            0.125,
        ] + left_rotation

        right_pick_up = [
            0.308,
            -0.065,
            0.125,
        ] + right_rotation

        self.get_logger().info(f"Going down for pickup")
        f_first_left  = self.execute_trajectory(self.control_left, self.create_trajectory(start, left_pick_up))
        f_first_right = self.execute_trajectory(self.control_right, self.create_trajectory(start, right_pick_up))

        while not (f_first_left.done() and f_first_right.done()):
            pass
        
        left_ahead = [
            0.305,
            0.220,
            0.125,
        ] + left_rotation

        right_ahead = [
            0.305,
            -0.220,
            0.125,
        ] + right_rotation

        self.get_logger().info(f"Going ahead")
        f_ahead_left  = self.execute_trajectory(self.control_left, self.create_trajectory(left_pick_up, left_ahead))
        f_ahead_right = self.execute_trajectory(self.control_right, self.create_trajectory(right_pick_up, right_ahead))


        while not (f_ahead_left.done() and f_ahead_right.done()):
            pass

        time.sleep(1)

        self.get_logger().info(f"Grabbing tray")
        self.grasp(self.grasp_left,  0.0052, 0.1, 1.0)
        self.grasp(self.grasp_right, 0.0052, 0.1, 1.0)

        time.sleep(3)

        left_move_with_tray_1 = [
            0.320,
            -0.018,
            0.55,
        ] + left_rotation

        right_move_with_tray_1 = [
            0.320,
            right_ahead[1] - abs(left_ahead[1] - left_move_with_tray_1[1]),
            0.55,
        ] + right_rotation

        self.get_logger().info(f"Moving tray to position one")
        f_move1_left  = self.execute_trajectory(self.control_left, self.create_trajectory(left_ahead, left_move_with_tray_1))
        f_move1_right = self.execute_trajectory(self.control_right, self.create_trajectory(right_ahead, right_move_with_tray_1))

        while not (f_move1_left.done() and f_move1_right.done()):
            pass

        time.sleep(1)

        left_move_with_tray_2 = [
            0.355,
            0.470,
            0.30,
        ] + left_rotation

        right_move_with_tray_2 = [
            0.355,
            right_move_with_tray_1[1] + abs(left_move_with_tray_2[1] - left_move_with_tray_1[1]),
            0.30,
        ] + right_rotation

        self.get_logger().info(f"Moving tray to position two")
        f_move2_left  = self.execute_trajectory(self.control_left, self.create_trajectory(left_move_with_tray_1, left_move_with_tray_2))
        f_move2_right = self.execute_trajectory(self.control_right, self.create_trajectory(right_move_with_tray_1, right_move_with_tray_2))

        while not (f_move2_left.done() and f_move2_right.done()):
            pass

        time.sleep(1)

        left_move_with_tray_3 = [
            0.305,
            0.215,
            0.255,
        ] + left_rotation

        right_move_with_tray_3 = [
            0.305,
            -0.215,
            0.255,
        ] + right_rotation

        self.get_logger().info(f"Moving tray to position three")
        f_move2_left  = self.execute_trajectory(self.control_left, self.create_trajectory(left_move_with_tray_2, left_move_with_tray_3))
        f_move2_right = self.execute_trajectory(self.control_right, self.create_trajectory(right_move_with_tray_2, right_move_with_tray_3))

        while not (f_move2_left.done() and f_move2_right.done()):
            pass

        time.sleep(1)
        
        self.get_logger().info(f"Moving tray to the start")
        f_move_last_left  = self.execute_trajectory(self.control_left, self.create_trajectory(left_move_with_tray_3, left_ahead))
        f_move_last_right = self.execute_trajectory(self.control_right, self.create_trajectory(right_move_with_tray_3, right_ahead))

        while not (f_move_last_left.done() and f_move_last_right.done()):
            pass

        time.sleep(1)

        self.get_logger().info(f"Unloading tray")
        self.grasp(self.grasp_left, 0.5, 0.1, 0.0)
        self.grasp(self.grasp_right, 0.5, 0.1, 0.0)

        time.sleep(3)

        self.get_logger().info(f"Going back")
        f_return_left  = self.execute_trajectory(self.control_left, self.create_trajectory(left_ahead, left_pick_up))
        f_return_right = self.execute_trajectory(self.control_right, self.create_trajectory(right_ahead, right_pick_up))

        while not (f_return_left.done() and f_return_right.done()):
            pass

        self.get_logger().info(f"Returning to the start")
        f_return_left  = self.execute_trajectory(self.control_left, self.create_trajectory(left_pick_up, start))
        f_return_right = self.execute_trajectory(self.control_right, self.create_trajectory(right_pick_up, start))

        while not (f_return_left.done() and f_return_right.done()):
            pass

        self.get_logger().info(f"Motion end")




    def create_trajectory(self, start_pose, end_pose):
        # Number of points in trajectory
        num_steps = 1000
        times = np.linspace(0, 1, num_steps)

        # Interpolate positions (x, y, z)
        start_pos = np.array(start_pose[:3])
        end_pos = np.array(end_pose[:3])
        interp_positions = np.outer(1 - times, start_pos) + np.outer(times, end_pos)

        # Interpolate orientations (SLERP on quaternions)
        start_quat = [start_pose[4], start_pose[5], start_pose[6], start_pose[3]]  # [x, y, z, w]
        end_quat   = [end_pose[4],   end_pose[5],   end_pose[6],   end_pose[3]]    # [x, y, z, w]

        slerp = Slerp([0, 1], R.from_quat([start_quat, end_quat]))
        interp_rots = slerp(times)
        interp_quats = interp_rots.as_quat()  # [x, y, z, w]

        # Assemble full poses: [x, y, z, w, x, y, z]
        trajectory = []
        for i in range(num_steps):
            pos = interp_positions[i]
            quat = interp_quats[i]
            pose = [*pos, quat[3], quat[0], quat[1], quat[2]]  # back to [x, y, z, w, qx, qy, qz]
            trajectory.append(pose)

        # trajectory is a list of poses
        return trajectory
    
    def execute_trajectory(self, controller: Publisher, points):
        future = Future()

        def _run():
            for point in points:
                msg = Float64MultiArray()
                msg.data = point
                controller.publish(msg)
                time.sleep(0.001)
            future.set_result(True)
        
        threading.Thread(target=_run, daemon=True).start()

        return future
    
    def grasp(self, client, width, speed, force, epsilon_inner = 0.005, epsilon_outer = 0.005):
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.epsilon.inner = epsilon_inner
        goal_msg.epsilon.outer = epsilon_outer
        goal_msg.speed = speed
        goal_msg.force = force

        client.wait_for_server()
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
    node = CoffeeTray()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
