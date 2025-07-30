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

class GearPick(Node):

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
    
    def run(self):
        start = [
            0.33514574174014890000,
            0.02643892168232797000,
            0.46694372999730827000,
            -0.0026928836785990434,
            0.99974059619854730000,
            -0.0224713545721692700,
            0.00255478252665934660
        ]

        self.get_logger().info(f"Going to the start")
        f_start_left  = self.execute_trajectory(self.control_left, self.create_trajectory(self.initial_pose[1], start))
        f_start_right = self.execute_trajectory(self.control_right, self.create_trajectory(self.initial_pose[0], start))
        self.grasp(self.grasp_left, 0.5, 0.1, 0.0)
        self.grasp(self.grasp_right, 0.5, 0.1, 0.0)

        while not (f_start_left.done() and f_start_right.done()):
            pass

        left_up_the_gear = [
            0.42088686323506140000,
            0.33349253942267134000,
            0.11039421677369335000,
            -0.0005802710366792942,
            0.99784736565249750000,
            -0.0130311792355438330,
            0.06426886115083995000
        ]
        right_ready_to_pick = [
            0.402447645215346100,
            -0.19306420917867967,
            0.577725860808667400,
            0.526584894593899200,
            0.499276859177045060,
            0.492634594164184500,
            -0.48035624624701860
        ]
        
        self.get_logger().info(f"Reach pickup position")
        f_first_left  = self.execute_trajectory(self.control_left, self.create_trajectory(start, left_up_the_gear))
        f_first_right = self.execute_trajectory(self.control_right, self.create_trajectory(start, right_ready_to_pick))

        while not (f_first_left.done() and f_first_right.done()):
            pass

        self.get_logger().info(f"Going down for the gear")
        left_go_down = [
            0.4236736703618876000,
            0.2983066031134850000,
            0.0170,
            0.0030718538427738865, 
            0.9998604424359376000, 
            0.0137148361264623800, 
            -0.009031203452697641
        ]
        f_down_left = self.execute_trajectory(self.control_left, self.create_trajectory(left_up_the_gear, left_go_down))
        while not (f_down_left.done()):
            pass

        time.sleep(5)

        self.get_logger().info(f"Left Grasping")
        self.grasp(self.grasp_left, .06167, 0.05, 30.0)

        time.sleep(5)

        self.get_logger().info(f"Left going up")
        left_go_up = Float64MultiArray()
        left_go_up.data = left_go_down
        left_go_up.data[2] = left_go_down[2] + 0.050
        self.control_left.publish(left_go_up)

        time.sleep(5)

        self.get_logger().info(f"Left going to swap position")
        left_swap_position = [
            0.4108836431452997400,
            0.4518144023683745600,
            0.5796994071815261000,
            0.6977028039282266000,
            -0.716232739812557200,
            0.0011572980674367293,
            0.0148364575445645640
        ]
        f_swap_left = self.execute_trajectory(self.control_left, self.create_trajectory(left_go_up.data, left_swap_position))
 
        while not (f_swap_left.done()):
            pass

        self.get_logger().info(f"Right going to swap position")
        right_go_ahead = [
            0.428094913300203400,
            -0.39193761493704543,
            0.581332818025366700,
            -0.49825944907721800,
            -0.48449140384795880,
            -0.50072076613519280,
            0.516027436638486300
        ]
        f_right_ahead = self.execute_trajectory(self.control_right, self.create_trajectory(right_ready_to_pick, right_go_ahead))

        while not f_right_ahead.done():
            pass

        self.get_logger().info(f"Right Grasping")
        self.grasp(self.grasp_right, .06167, 0.05, 30.0)
        
        time.sleep(5)

        self.get_logger().info(f"Left Releasing")
        self.grasp(self.grasp_left, .5, 0.05, 0.0)

        time.sleep(5)

        self.get_logger().info(f"Right going back")
        right_go_back = right_ready_to_pick
        f_right_go_back = self.execute_trajectory(self.control_right, self.create_trajectory(right_go_ahead, right_go_back))

        while not f_right_go_back.done():
            pass

        self.get_logger().info(f"Returning to the start")
        f_last_left  = self.execute_trajectory(self.control_left, self.create_trajectory(left_swap_position, start))
        f_last_right = self.execute_trajectory(self.control_right, self.create_trajectory(right_go_back, start))

        while not (f_last_left.done() and f_last_right.done()):
            pass

    def create_trajectory(self, start_pose, end_pose):
        # Number of points in trajectory
        num_steps = 100
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
                time.sleep(0.05)
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
    node = GearPick()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
