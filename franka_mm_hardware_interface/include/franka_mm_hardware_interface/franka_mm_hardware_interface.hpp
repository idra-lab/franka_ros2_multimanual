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
#ifndef FRANKA_MM_HW_INTERFACE_HPP
#define FRANKA_MM_HW_INTERFACE_HPP

#include <thread>
#include <pthread.h>
#include <sched.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "franka/robot.h"
#include "franka/active_control_base.h"

#include "franka_mm_hardware_interface/franka_wrapper.hpp"
#include "franka_mm_hardware_interface/mode_switch_plan.hpp"
#include "franka_mm_hardware_interface/interfaces.hpp"
    
/** 
* This class implements the hardware interface for handling multiple 
* Franka Research 3 (FR3) robots.
*
* @author Alessandro Moscatelli
*/
class HardwareInterface : public hardware_interface::SystemInterface {
public:    
    RCLCPP_SHARED_PTR_DEFINITIONS(HardwareInterface)

    HardwareInterface()           = default;
    ~HardwareInterface() override = default;

    HardwareInterface(const HardwareInterface&)             = delete;
    HardwareInterface(const HardwareInterface&&)            = delete;
    HardwareInterface& operator=(const HardwareInterface&)  = delete;
    HardwareInterface& operator=(const HardwareInterface&&) = delete;

    /**
    * Initializes the count, the names and the IPs of the robots.
    * This function searches for 2 parameters inside the URDF file,
    * both to be expressed in CSV format:
    * - robot_names: ordered list of robot's name
    * - robot_ips:   ordered list of robot's name
    * Creates the @link FrankaRobotWapper @endlink objects,
    * that handle the parameter and connections of the robots
    * 
    * @param hardware_info SystemInterface hardware info
    *
    * @return Status of the function after the call
    */
    hardware_interface::CallbackReturn on_init (
        const hardware_interface::HardwareInfo &hardware_info
    ) override;

    /**
    * Starts the threads of the robot services to on an
    * executor
    * 
    * @param prev_state Previous state in the lifecycle    
    *
    * @return Status of the function after the call
    */
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    /**
    * Stops the robot, the controllers and the service servers.
    *
    * @param prev_state Previous state in the lifecycle
    *
    * @return Status of the function after the call
    */
    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    /**
    * Does the first state reading on the robots
    *
    * @param prev_state Previous state in the lifecycle
    *
    * @return Status of the function after the call
    */
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    /**
    * For each robot, prints the percentage of received messages
    * (last 100), the current errors and the last motion errors. 
    *
    * @param prev_state Previous state in the lifecycle
    *
    * @return Status of the function after the call
    */
    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    /**
    * Exports the state interfaces for each robot.
    * 
    * Exports the state interfaces for each robot giving access to: 
    * - Joint position, velocities and torques
    * - Cartesian pose, expressed as transformation matrix
    * - Cartesian pose, expressed as position + quaternion
    *
    * @return List of state interfaces to be exported
    */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
    * Exports the command interfaces for each robot.
    * 
    * Gives write access to the variables of exported_cmds, making possible to
    * communicate command with the robot. The interfaces exported for each robot are:
    * - Joint position, velocities and torques
    * - Cartesian pose, expressed as transformation matrix
    * - Cartesian pose, expressed as position + quaternion
    *
    * @return List of command interfaces to be exported
    */
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /**
    * Reads and updates the current state of the robot if there are no controller in execution.
    *
    * @param time   Time when the function was called
    * @param period Duration difference since the last call
    *
    * @return Status of the function after the call
    */
    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    /**
    * Reads the values of the exported interfaces and copies them in the control values of the robot.
    * This operation is done to avoid partial readings from the control threads, 
    * because the exported commands could be updated during the control thread read phase.
    *
    * @param time   Time when the function was called
    * @param period Duration difference since the last call
    *
    * @return Status of the function after the call
    */
    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    /**
    * Parses the starting and stopping interfaces for checking if there are conflicts present.
    * A switch is blocked if there is an active interface of different type active on the robot,
    * that is not programmed to be shut down (e.g. velocity is being activated, 
    * but position is already running and won't be deactivated)
    *
    * @param start_interfaces Interfaces that are requested to be started
    * @param stop_interfaces  Interfaces that are requested to be stopped
    *
    * @return Status of the function after the call
    */
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    /**
    * Performs the activation and deactivation of the interfaces.
    * Shuts down active controllers, initializes control variables, 
    * activates requested control loops and switches command mode of the robots
    *
    * @param start_interfaces Interfaces that are requested to be started
    * @param stop_interfaces  Interfaces that are requested to be stopped
    *
    * @return Status of the function after the call
    */
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    /**
    * Get RCLCPP logger object
    *  
    * @return Logger
    */
    rclcpp::Logger& get_logger() { return logger; }

private:
    /**
    * RCLCPP Logger
    */
    rclcpp::Logger logger = rclcpp::get_logger("franka_mm_hardware_interface");

    /**
    * Vector used to handle N connection objects with N franka arms
    */
    std::vector<FrankaRobotWrapper> arms; 

    /**
    * Object used to store command switches between functions
    */  
    std::unique_ptr<ModeSwitchPlan> mode_switch_plan;  

    /**
     * Thread of the executor used for running the service servers.
     */
    std::unique_ptr<std::thread> executor_thread;

    /**
     * Atomic boolean for signaling the stopping of the interface.
     */
    std::atomic<bool> stopped = false;

    /**
    * Parameter used to regulate if franka::limitRate should be called in the control loops.
    * If set to true, limitRate function will NOT be used.
    */
    bool limit_override = false;
};

#endif  // FRANKA_MM_HW_INTERFACE_HPP