
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

#include "franka_bimanual_hardware_interface/franka_wrapper.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace LambdaControl {

    std::function<void()> startJointPositionControl(FrankaRobotWrapper& robot, bool limit_override);

    std::function<void()> startJointVelocityControl(FrankaRobotWrapper& robot, bool limit_override);

    std::function<void()> startJointEffortControl(FrankaRobotWrapper& robot, bool limit_override);

    std::function<void()> startCartesianPositionControl(FrankaRobotWrapper& robot, bool limit_override);
    
    std::function<void()> startCartesianVelocityControl(FrankaRobotWrapper& robot, bool limit_override);
    
}

#endif  // LAMBDA_CONTROL_HPP