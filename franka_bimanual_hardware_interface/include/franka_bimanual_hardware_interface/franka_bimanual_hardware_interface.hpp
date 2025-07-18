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
#ifndef FRANKA_BIMANUAL_HW_INTERFACE_HPP
#define FRANKA_BIMANUAL_HW_INTERFACE_HPP

#include <Eigen/Dense>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "franka/robot.h"
#include "franka/active_control_base.h"

#include <thread>
#include <pthread.h>
#include <sched.h>

namespace franka {

/** 
* This class implements the hardware interface for handling multiple 
* Franka Emika robots.
*
* @author Alessandro Moscatelli
*/
class HardwareInterface : public hardware_interface::SystemInterface {
public:
    /**
    * Array of doubles used to represent joint states
    */
    using Vec7       = std::array<double, 7>;

    /**
    * Unique pointer to a robot
    */
    using FrankaPtr  = std::unique_ptr<Robot>;

    /** 
    * Unique pointer to a control thread of a robot
    */
    using ControlPtr = std::unique_ptr<std::thread>;

    /**
    * Unique pointer to a mutex
    */
    using MutexPtr   = std::unique_ptr<std::mutex>;

    /**
    * Pair representing the index of the robot that will modify 
    * it's interface type and which interface will be modified
    */
    using ModeSwitch = std::pair<long, ControlMode>;
    
    /**
    * Enumeration class used to describe controller's possible states.
    */
    enum class ControlMode {
        INACTIVE,
        POSITION,
        VELOCITY,
        EFFORT
    };

    /**
    * Struct used to populate control values that will be forwarded to the robot controller.
    */
    struct ControlValues { 
        /**
        * Joint position values
        */
        Vec7 q;
        /**
        *Joint velocity values
        */
        Vec7 qd;
        /**
        * Joint torque values
        */
        Vec7 tau;
    };

    /**
    * This object used as helper to perform efficiently the prepare_command_mode_switch and the 
    * perform_command_mode_switch.
    *
    * It is populated by two vectors, that describe which robot is activating or deactivating
    * a particular interface. 
    */
    struct ModeSwitchPlan {
        /**
        * Vector describing which interfaces will be activated
        */
        std::vector<ModeSwitch> activations;

        /**
        * Vector describing which interfaces will be deactivated
        */
        std::vector<ModeSwitch> deactivations;
    };

    /**
    * Struct used to handle a Franka robot connection.
    */
    struct RobotUnit {
        ControlValues if_states, if_cmds;
        ControlValues exported_cmds;

        /**
        * Current state of the robot
        */
        RobotState current_state;

        /**
        * Boolean value that signals if the robot has to update his q values.
        *
        * This is done because when the position control is being activated, 
        * if_cmds and exported_cmds must be initialized with the same value of 
        * if_states to avoid errors. 
        */
        bool first_position_update = true;

        /**
        * Name of the robot
        */
        std::string name = "";

        /**
        * IP of the robot
        */
        std::string ip   = "";

        /**
        * Mutex used to handle concurrency in the read phase of the robot's state
        */
        MutexPtr control_mutex = std::make_unique<std::mutex>();

        /**
        * Mutex used to handle concurrency in the write phase of the robot command.
        * 
        * This is mainly used to avoid partial readings from exported_cmds.
        */
        MutexPtr write_mutex   = std::make_unique<std::mutex>();

        /**
        * Unique pointer to the Robot object of libfranka, that is used to communicate
        * commands and read states.
        */
        FrankaPtr arm      = nullptr;

        /**
        * Unique pointer to the Thread used to handle the robot controller.
        */
        ControlPtr control = nullptr;

        /**
        * Current operational mode of the robot
        */
        ControlMode control_mode = ControlMode::INACTIVE;
    };

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
    *
    * @param hardware_info SystemInterface hardware info
    *
    * @return Status of the function after the call
    */
    hardware_interface::CallbackReturn on_init (
        const hardware_interface::HardwareInfo &hardware_info
    ) override;

    /**
    * Creates the connection between the machine and the robots
    * and configures the controller as inactive.
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
    * Stops the robot and their controllers
    *
    * @param prev_state Previous state in the lifecycle
    *
    * @return Status of the function after the call
    */
    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    /**
    * Does the first reading of the robots
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
    * Exports the reading of states from if_states, giving access
    * to joint position, velocities and torques for each robot.
    *
    * @return List of state interfaces to be exported
    */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
    * Exports the command interfaces for each robot.
    * 
    * Gives write access to the variables of exported_cmds, making possible to
    * communicate command with the robot. The interfaces exported for each robot are
    * joint positions, velocities and torques as well cartesian velocities.
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

    /**
    * Converts a ControlMode into a string
    *
    * @param mode Mode to be converted
    *  
    * @return String representing the control mode
    */
    std::string control_to_string(const ControlMode& mode);

private:
    /**
    * RCLCPP Logger
    */
    rclcpp::Logger logger = rclcpp::get_logger("franka_bimanual_hardware_interface");

    /**
    * Vector used to handle N connection objects with N franka arms
    */
    std::vector<RobotUnit> arms; 

    /**
    * Object used to store command switches between functions
    */
    ModeSwitchPlan mode_switch_plan;  

    /**
    * Parameter used to regulate if franka::limitRate should be called in the control loops.
    * If set to true, limitRate function will NOT be used.
    */
    bool limit_override = false;

    /**
    * Function used to elaborate which interfaces will be changing operational mode.
    *
    * @param interfaces Vector with interface list to be parsed
    * @param changes    Output vector with robot index and relative operational mode, if changed
    *
    * @return Error, if some unknown robot or interface type are found, or there are inconsistent 
    * modifications to the same robot (e.g. a robot tries to activate both position and velocity),
    * else the function returns ok.
    */
    hardware_interface::return_type who_and_what_switched(
        const std::vector<std::string>& interfaces,
        std::vector<ModeSwitch>& changes
    );

    /**
    * This function is used to activate a controller thread in a RobotUnit object.
    *
    * @param robot Robot that is activating a controller
    * @param mode  Operational mode of the controller
    */
    void setup_controller(RobotUnit& robot, ControlMode mode);

    /**
    * Stops the controller of a robot.
    *
    * This function sets the operational mode of the robot to inactive, 
    * waits the control thread to join and stops the robot.
    *
    * @param robot Robot that is deactivating a controller
    */
    void reset_controller(RobotUnit& robot);   
};

}

#endif  // FRANKA_BIMANUAL_HW_INTERFACE_HPP