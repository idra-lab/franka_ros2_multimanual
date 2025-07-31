
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
#ifndef MODE_SWITCH_PLAN_HPP
#define MODE_SWITCH_PLAN_HPP

#include "franka_mm_hardware_interface/franka_wrapper.hpp"
#include "franka_mm_hardware_interface/control_modes.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

/**
 * This object used as helper to perform efficiently the prepare_command_mode_switch and the 
 * perform_command_mode_switch.
 *
 * It is populated by a two pairs of vectors: the first pair describe 
 * which robot is activating or deactivating a particular interface. 
 * The second pair contains the indexes of the robot that are activating
 * or deactivating their elbow interface.
 * 
 * @author Alessandro Moscatelli
 */
class ModeSwitchPlan {
public:
/**
 * Pair representing the index of the robot that will modify 
 * it's interface type and the type of interface that is needed.
 */
using ModeSwitch = std::pair<long, ControlMode>;

/**
 * Vector describing which interfaces will be activated.
 */
std::vector<ModeSwitch> activations;

/**
 * Vector describing which interfaces will be deactivated.
 */
std::vector<ModeSwitch> deactivations;

/**
 * Vector containing the indexes of the robots that will activate their elbow interface.
 */
std::vector<long> elbow_activations;

/**
 * Vector containing the indexes of the robots that will deactivate their elbow interface.
 */
std::vector<long> elbow_deactivations;

/**
 * This constructor parses the starting and stopping interfaces, 
 * populating the class' vectors.
 *
 * @param start_interfaces Vector listing the interfaces that will be started
 * @param stop_interfaces  Vector listing the interfaces that will be stopped
 * @param robots           Vector containing the robots
 *
 * @throw runtime_error if some unknown robot or interface type are found, or there are inconsistent 
 * modifications to the same robot (e.g. a robot tries to activate both position and velocity).
 */
ModeSwitchPlan(
    const std::vector<std::string>& start_interfaces, 
    const std::vector<std::string>& stop_interfaces,
    const std::vector<FrankaRobotWrapper>& robots
);
~ModeSwitchPlan() = default;

/**
 * Tells if the robot will activate an interface.
 * 
 * @param robot_index Index of the robot referring to the arms array
 * 
 * @return True if the robot is activating an interface, False otherwise
 */
bool is_being_activated(long robot_index) const;

/**
 * Tells if the robot will activate an cartesian interface, either pose, velocity or impedance.
 * 
 * @param robot_index Index of the robot referring to the arms array
 * 
 * @return True if the robot is activating a cartesian interface, False otherwise
 */
bool is_activating_cartesian(long robot_index) const;

/**
 * Tells what type of control is the robot activating.
 * 
 * @param robot_index Index of the robot referring to the arms array
 * 
 * @return @link FrankaRobotWrapper::ControlMode Control mode @endlink of the robot, if found
 * 
 * @throws range_error if the robot with relative index is not found
 */
ControlMode what_is_being_activated(long robot_index) const;

/**
 * Tells if the robot will deactivate an interface.
 * 
 * @param robot_index Index of the robot referring to the arms array
 * 
 * @return True if the robot is deactivating an interface, False otherwise
 */
bool is_being_deactivated(long robot_index) const;

/**
 * Tells if the robot will activate an elbow interface.
 * 
 * @param robot_index Index of the robot referring to the arms array
 * 
 * @return True if the robot is activating an elbow interface, False otherwise
 */
bool is_elbow_being_activated(long robot_index) const;

/**
 * Tells if the robot will deactivate an elbow interface.
 * 
 * @param robot_index Index of the robot referring to the arms array
 * 
 * @return True if the robot is deactivating an elbow interface, False otherwise
 */
bool is_elbow_being_deactivated(long robot_index) const;

private:
/**
 * Function used to elaborate which interfaces will be changing operational mode.
 *
 * @param interfaces    Vector with interface list to be parsed
 * @param robots        Vector with the list of robots to check
 * @param changes       Output vector with robot index and relative operational mode, if changed
 * @param elbow_changes Output vector with robot index if changed elbow interface
 *
 * @throw runtime_error if some unknown robot or interface type are found, or there are inconsistent 
 * modifications to the same robot (e.g. a robot tries to activate both position and velocity).
 */
void who_and_what_switched(
    const std::vector<std::string>& interfaces,
    const std::vector<FrankaRobotWrapper>& robots,
    std::vector<ModeSwitch>& changes, 
    std::vector<long>&       elbow_changes
);

inline bool find_interface_type(const std::string& iface, const char* iface_type) {
    return (iface.find(std::string("/") + iface_type) != std::string::npos);
}

};

#endif  // MODE_SWITCH_PLAN_HPP