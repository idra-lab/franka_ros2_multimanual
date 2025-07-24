
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
#ifndef LAMBDA_CONTROL_HPP
#define LAMBDA_CONTROL_HPP

#include "franka_mm_hardware_interface/franka_wrapper.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

/**
 * This namespace collects the function used to start the control loop of a FR3 robot.
 * 
 * All the function return a std::function<void()> that is used in @link FrankaRobotWrapper @endlink
 * by the control thread of the robot. 
 * The all the lambdas used returned by these funcitons simply call robot.control(...)
 * with some specifically implemented robot control. 
 * 
 * @author Alessandro Moscatelli
 */
namespace LambdaControl {

    /**
     * Return a function that, if called, starts a joint position control on the robot.
     * 
     * @param robot Robot where the controller will be activated
     * @param limit_override If set to False, the controller will use franka::limitRate.
     * 
     * @return Lambda function used to by the control thread.
     */
    std::function<void()> startJointPositionControl(FrankaRobotWrapper& robot, bool limit_override);

    /**
     * Return a function that, if called, starts a joint velocity control on the robot.
     * 
     * @param robot Robot where the controller will be activated
     * @param limit_override If set to False, the controller will use franka::limitRate.
     * 
     * @return Lambda function used to by the control thread.
     */
    std::function<void()> startJointVelocityControl(FrankaRobotWrapper& robot, bool limit_override);

    /**
     * Return a function that, if called, starts a joint torque control on the robot.
     * 
     * @param robot Robot where the controller will be activated
     * @param limit_override If set to False, the controller will use franka::limitRate.
     * 
     * @return Lambda function used to by the control thread.
     */
    std::function<void()> startJointEffortControl(FrankaRobotWrapper& robot, bool limit_override);

    /**
     * Return a function that, if called, starts a cartesian pose control on the robot.
     * 
     * @param robot Robot where the controller will be activated
     * @param limit_override If set to False, the controller will use franka::limitRate.
     * 
     * @return Lambda function used to by the control thread.
     */
    std::function<void()> startCartesianPositionControl(FrankaRobotWrapper& robot, bool limit_override);
    
    /**
     * Return a function that, if called, starts a cartesian position control on the robot.
     * 
     * @param robot Robot where the controller will be activated
     * @param limit_override If set to False, the controller will use franka::limitRate.
     * 
     * @return Lambda function used to by the control thread.
     */
    std::function<void()> startCartesianVelocityControl(FrankaRobotWrapper& robot, bool limit_override);
    
}

#endif  // LAMBDA_CONTROL_HPP