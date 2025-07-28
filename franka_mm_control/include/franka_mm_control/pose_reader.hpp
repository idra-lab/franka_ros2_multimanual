
/*
 * Copyright 2025 IDRA, University of Trento
 * Author: Alessandro Moscatelli (ale.moska002@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef POSE_READER_HPP
#define POSE_READER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/controller_interface.hpp>

/**
 * This controller reads the values inside Franka Robots pose interfaces, 
 * concatenates as double array and publishes them inside 
 * the /current_pose and /current_q_pose topic.
 * 
 * This two topics describe end-effector position and rotation in the
 * following formats:
 *  - /current_pose expresses the value as an homogeneous transformations matrices in 
 *    column-major format. The vector as length equal to 16 * robot_number.
 *  - /current_q_pose expresses the value as concatenation of position vector and
 *    quaternion. The vector as length equal to 7 * robot_number.
 * 
 * This controller has an input, robot_names, that is a list of 
 * robot names that needs to be searched.
 * 
 * @author Alessandro Moscatelli
 */
class PoseReader : public controller_interface::ControllerInterface {
public:
    PoseReader() = default;

    /**
     * Initializes robot names vector from controller parameter and initialises /current_pose publisher 
     * 
     * @returns Success if all the setup went smoothly.
     */
    controller_interface::CallbackReturn on_init() override;

    /**
     * Loads the command interface configuration. No command interfaces are used in this controller.
     * 
     * @returns Object containing the requested interfaces
     */
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    /**
     * Loads the state interface configuration. 
     * State interfaces must are in the format <robot_name>_i/cartesian_pose_command, 
     * where i is [0, 15] representing the indexes of the transformation matrix.
     * 
     * @returns Object containing the requested interfaces
     */
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /**
     * Does nothigh.
     * 
     * @param previous_state ROS lifecycle previous state
     * 
     * @return Status of the function after the call
     */
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    /**
    * Reads the values from the state interfaces and publishes them inside the topic
    *
    * @param time   Time when the function was called
    * @param period Duration difference since the last call
    *
    * @return Status of the function after the call
    */
    controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

    /**
    * Get RCLCPP logger object
    *  
    * @return Logger
    */
    rclcpp::Logger& get_logger() { return logger; }

    RCLCPP_SHARED_PTR_DEFINITIONS(PoseReader)
private:
    /**
    * RCLCPP Logger
    */
    rclcpp::Logger logger = rclcpp::get_logger("pose_reader");

    /**
     * Robot names
     */
    std::vector<std::string> robot_names;

    /**
     * Publisher object where poses will be outputed in matrix format
     */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;

    /**
     * Publisher object where poses will be outputed position + quaternion
     */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_publisher;
};

#endif  // POSE_READER_HPP