/*
 * Copyright 2025 IDRA, University of Trento
 * Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)
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

class HardwareInterface : public hardware_interface::SystemInterface {
public:
    using Vec7       = std::array<double, 7>;
    using Vec14      = std::array<double, 14>;
    using FrankaPtr  = std::unique_ptr<Robot>;
    using StatePtr   = std::unique_ptr<RobotState>;
    // using ControlPtr = std::unique_ptr<ActiveControlBase>;
    using ControlPtr = std::unique_ptr<std::thread>;

    enum class ControlMode {
        INACTIVE,
        POSITION,
        VELOCITY,
        EFFORT
    };

    struct ModeSwitchPlan {
        std::vector<std::pair<long, ControlMode>> activations;
        std::vector<std::pair<long, ControlMode>> deactivations;
    };

    struct RobotUnit {
        struct { 
            Vec7 q;
            Vec7 qd;
            Vec7 tau;
        } if_states, if_cmds;

        RobotState current_state;
        bool first_position_update = true;

        std::string name = "";
        std::string ip   = "";

        std::unique_ptr<std::mutex> control_mutex = std::make_unique<std::mutex>();

        FrankaPtr arm      = nullptr;
        ControlPtr control = nullptr;
        ControlMode control_mode = ControlMode::INACTIVE;
    };

    RCLCPP_SHARED_PTR_DEFINITIONS(HardwareInterface)

    HardwareInterface()           = default;
    ~HardwareInterface() override = default;

    HardwareInterface(const HardwareInterface&)             = delete;
    HardwareInterface(const HardwareInterface&&)            = delete;
    HardwareInterface& operator=(const HardwareInterface&)  = delete;
    HardwareInterface& operator=(const HardwareInterface&&) = delete;

    hardware_interface::CallbackReturn on_init (
        const hardware_interface::HardwareInfo &hardware_info
    ) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& prev_state
    ) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period
    ) override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    rclcpp::Logger& get_logger() { return logger; }
    std::string control_to_string(const ControlMode& mode);

private:
    rclcpp::Logger logger = rclcpp::get_logger("franka_bimanual_hardware_interface");

    std::vector<RobotUnit> arms; 
    ModeSwitchPlan mode_switch_plan;  
    bool limit_override = false;

    hardware_interface::return_type who_and_what_switched(
        const std::vector<std::string>& interfaces,
        std::vector<std::pair<long, ControlMode>>& changes
    );
    void setup_controller(RobotUnit& robot, ControlMode mode);
    void reset_controller(RobotUnit& robot);   
};

}

#endif  // FRANKA_BIMANUAL_HW_INTERFACE_HPP