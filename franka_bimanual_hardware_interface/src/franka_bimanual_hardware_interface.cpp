#include "franka_bimanual_hardware_interface/franka_bimanual_hardware_interface.hpp"

#include <array>
#include <algorithm>
#include <ctime>
#include <fmt/format.h>
#include <memory>
#include <stdexcept>
#include <regex>    

#include <rclcpp/duration.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

#include <franka/control_tools.h>
#include <franka/rate_limiting.h>

//  ____            _                 _   _
// |  _ \  ___  ___| | __ _ _ __ __ _| |_(_) ___  _ __  ___
// | | | |/ _ \/ __| |/ _` | '__/ _` | __| |/ _ \| '_ \/ __|
// | |_| |  __/ (__| | (_| | | | (_| | |_| | (_) | | | \__ \
// |____/ \___|\___|_|\__,_|_|  \__,_|\__|_|\___/|_| |_|___/
//

using franka::HardwareInterface;

//  ____   ____ _     ____ ____  ____    _     _  __       ____           _
// |  _ \ / ___| |   / ___|  _ \|  _ \  | |   (_)/ _| ___ / ___|   _  ___| | ___
// | |_) | |   | |  | |   | |_) | |_) | | |   | | |_ / _ \ |  | | | |/ __| |/ _ \
// |  _ <| |___| |__| |___|  __/|  __/  | |___| |  _|  __/ |__| |_| | (__| |  __/
// |_| \_\\____|_____\____|_|   |_|     |_____|_|_|  \___|\____\__, |\___|_|\___|
//                                                             |___/

hardware_interface::CallbackReturn 
HardwareInterface::on_init (
    const hardware_interface::HardwareInfo &info
) {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Initializing parameters...");

    std::regex name_regex("([\\w-]+)");
    const std::string& name_list = info.hardware_parameters.at("robot_names");
    auto name_begin = std::sregex_iterator(name_list.begin(), name_list.end(), name_regex);
    auto name_end   = std::sregex_iterator();
    long name_size  = std::distance(name_begin, name_end);

    RCLCPP_INFO(get_logger(), "Found %lu robots", name_size);

    std::regex ip_regex("((\\d{1,3}\\.){3}\\d{1,3}\\b)");
    const std::string& ip_list = info.hardware_parameters.at("robot_ips");
    auto ip_begin = std::sregex_iterator(ip_list.begin(), ip_list.end(), ip_regex);
    auto ip_end   = std::sregex_iterator();
    long ip_size  = std::distance(ip_begin, ip_end);
   
    RCLCPP_INFO(get_logger(), "Found %lu IPs", ip_size);

    if (name_size != ip_size) {
        RCLCPP_ERROR(get_logger(), "Names and IPs of robots do not match");
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto name_it = name_begin;
    auto ip_it = ip_begin;
    for ( long i = 0; i < ip_size; ++i ) {
        RobotUnit frk;

        frk.name = name_it->str();
        frk.ip = ip_it->str();

        arms.push_back(std::move(frk));

        ++name_it;
        ++ip_it;
    }

    RCLCPP_INFO(get_logger(), "Initialized %lu robots", arms.size());

    limit_override = info.hardware_parameters.at("limit_override") == "true";
    RCLCPP_INFO(get_logger(), "franka::limitRate will %sbe used", !limit_override ? "" : "not ");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_configure(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_configure()");
    if (hardware_interface::SystemInterface::on_configure(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_configure() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
    if (!franka::hasRealtimeKernel()) {
        rt_config = franka::RealtimeConfig::kIgnore;
        RCLCPP_WARN(get_logger(), "RT kernel is not in use");
    }

    for ( long i = 0; i < arms.size(); ++i ) {
        RCLCPP_INFO(get_logger(), "Connection with arm %s @ %s", 
            arms[i].name.c_str(), arms[i].ip.c_str());

        arms[i].arm = std::make_unique<Robot>(arms[i].ip, rt_config);
        //TODO? setDefaultBehavior(frk)
        
        // Controller state
        arms[i].control_mode = ControlMode::INACTIVE;

        RCLCPP_INFO(get_logger(), "Done!");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_cleanup(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_cleanup()");
    if (hardware_interface::SystemInterface::on_cleanup(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_cleanup() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_cleanup() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_shutdown(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_shutdown()");
    if (hardware_interface::SystemInterface::on_shutdown(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_shutdown() failed");
    }

    RCLCPP_INFO(get_logger(), "Shutting down arms...");

    for (auto& robot : arms) {
        reset_controller(robot);
    }
    
    RCLCPP_INFO(get_logger(), "Done!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwareInterface::on_activate(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_activate()");
    if (hardware_interface::SystemInterface::on_activate(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_shutdown() failed");
    }

    read(rclcpp::Time(0),rclcpp::Duration(0, 0));  

    RCLCPP_DEBUG(get_logger(), "on_activate() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should deactivate the hardware.
 */
hardware_interface::CallbackReturn
HardwareInterface::on_deactivate(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_deactivate()");
    if (hardware_interface::SystemInterface::on_deactivate(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_deactivate() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO
    RCLCPP_DEBUG(get_logger(), "on_deactivate() completed successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * This function should handle errors.
 */
hardware_interface::CallbackReturn
HardwareInterface::on_error(const rclcpp_lifecycle::State& prev_state) {
    RCLCPP_DEBUG(get_logger(), "calling on_error()");
    if (hardware_interface::SystemInterface::on_error(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_error() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (RobotUnit& arm : arms) {
        reset_controller(arm);
    }

    for (const RobotUnit& arm : arms) {
        RCLCPP_INFO(get_logger(), "Error dump of %s", 
            arm.name.c_str()
        );

        RCLCPP_INFO(get_logger(), "Command success rate: %f", 
            arm.current_state.control_command_success_rate
        );

        RCLCPP_INFO(get_logger(), "Current errors: %s", 
            std::string(arm.current_state.current_errors).c_str()
        );

        RCLCPP_INFO(get_logger(), "Last motion errors: %s", 
            std::string(arm.current_state.last_motion_errors).c_str()
        );
    }

    RCLCPP_INFO(get_logger(), "System is now inactive and requires reconfiguration.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

//  _   ___        __  ___       _             __
// | | | \ \      / / |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
// | |_| |\ \ /\ / /   | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
// |  _  | \ V  V /    | || | | | ||  __/ |  |  _| (_| | (_|  __/
// |_| |_|  \_/\_/    |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
//

std::vector<hardware_interface::StateInterface>
HardwareInterface::export_state_interfaces() {
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    std::vector<hardware_interface::StateInterface> state_interfaces;
    std::string jnt_name = {};

    state_interfaces.reserve(7 * 3 * arms.size());  // NOLINT: 7 joints * 3 states * n robots

    for (long p = 0; p < arms.size(); ++p) {
        for (long i = 0; i < 7; ++i) {
            jnt_name = arms[p].name + "_fr3_joint" + std::to_string(i+1);
            RCLCPP_INFO(get_logger(), "%s", jnt_name.c_str());

            state_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &arms[p].if_states.q[i]);
            state_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &arms[p].if_states.qd[i]);
            state_interfaces.emplace_back(jnt_name, HW_IF_EFFORT,   &arms[p].if_states.tau[i]);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
HardwareInterface::export_command_interfaces() {
    using hardware_interface::HW_IF_EFFORT;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;

    std::vector<hardware_interface::CommandInterface> cmd_interfaces;
    std::string jnt_name = {};

    // TODO: GPIO intefaces?
    cmd_interfaces.reserve(7 * 3 * arms.size());  // NOLINT: 7 joints * 3 cmd interfaces * n robots

    for (long p = 0; p < arms.size(); ++p) {
        for (long i = 0; i < 7; ++i) {
            jnt_name = arms[p].name + "_fr3_joint" + std::to_string(i+1);

            cmd_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &arms[p].exported_cmds.q[i]);
            cmd_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &arms[p].exported_cmds.qd[i]);
            cmd_interfaces.emplace_back(jnt_name, HW_IF_EFFORT,   &arms[p].exported_cmds.tau[i]);
        }
    }

    return cmd_interfaces;
}

hardware_interface::return_type
HardwareInterface::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    // This is used only when there is no controller loaded

    for (RobotUnit& robot : arms) {
        if (!robot.control) {
            std::lock_guard<std::mutex> lock(*robot.control_mutex);
            robot.current_state = robot.arm->readOnce();

            robot.if_states.q   = robot.current_state.q;
            robot.if_states.qd  = robot.current_state.dq;
            robot.if_states.tau = robot.current_state.tau_J;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::write(const rclcpp::Time& /* time */, const rclcpp::Duration& period ) {
    // Copies the command from the exported interfaces to the command used by the robot to avoid concurrency problems

    for (RobotUnit& robot : arms) {
        if (robot.control) {
            std::lock_guard<std::mutex> lock(*robot.write_mutex);

            robot.if_cmds = robot.exported_cmds;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type HardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    mode_switch_plan.activations.clear();
    mode_switch_plan.deactivations.clear();

    if (who_and_what_switched(stop_interfaces, mode_switch_plan.deactivations) == hardware_interface::return_type::ERROR) {
        return hardware_interface::return_type::ERROR;
    }

    if (who_and_what_switched(start_interfaces, mode_switch_plan.activations) == hardware_interface::return_type::ERROR) {
        return hardware_interface::return_type::ERROR;
    }

    // Lambda to check if an arm is being deactivated
    const auto is_being_deactivated = [&](long arm_index) -> bool {
        return std::any_of(
            mode_switch_plan.deactivations.begin(),
            mode_switch_plan.deactivations.end(),
            [&](const auto& item) { return item.first == arm_index; }
        );
    };

    // Check for conflicts
    for (const auto& change : mode_switch_plan.activations) {
        const RobotUnit& arm = arms[change.first];

        if (arm.control_mode != ControlMode::INACTIVE && !is_being_deactivated(change.first)) {
            RCLCPP_ERROR(get_logger(), "%s already has an active interface %s, that is not planned to be deactivated",
                arm.name.c_str(), control_to_string(arm.control_mode).c_str()
            );

            return hardware_interface::return_type::ERROR;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
) {
    for (const auto& change : mode_switch_plan.deactivations) {
        RobotUnit& arm = arms[change.first];

        RCLCPP_INFO(get_logger(), "%s will shut down interface %s", 
            arm.name.c_str(), control_to_string(change.second).c_str()
        );

        reset_controller(arm);

        RCLCPP_INFO(get_logger(), "%s correctly shutted down the interface", 
            arm.name.c_str()
        );
    }

    for (const auto& change : mode_switch_plan.activations) {
        RobotUnit& arm = arms[change.first];

        RCLCPP_INFO(get_logger(), "%s will activate %s interface", 
            arm.name.c_str(), control_to_string(change.second).c_str()
        );

        if (change.second == ControlMode::POSITION) {
            arm.first_position_update = true;
        } else if (change.second == ControlMode::VELOCITY) {
            std::fill(arm.exported_cmds.qd.begin(), arm.exported_cmds.qd.end(), 0);
            std::fill(arm.if_cmds.qd.begin(), arm.if_cmds.qd.end(), 0);
        } else if (change.second == ControlMode::EFFORT) {
            std::fill(arm.exported_cmds.tau.begin(), arm.exported_cmds.tau.end(), 0);
            std::fill(arm.if_cmds.tau.begin(), arm.if_cmds.tau.end(), 0);
        } 

        reset_controller(arm);
        setup_controller(arm, change.second);

        arm.control_mode = change.second;

        RCLCPP_INFO(get_logger(), "%s correctly activated the interface", 
            arm.name.c_str()
        );

    }

    return hardware_interface::return_type::OK;
}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___
// | |_) | '__| \ \ / / _` | __/ _ \
// |  __/| |  | |\ V / (_| | ||  __/
// |_|   |_|  |_| \_/ \__,_|\__\___|
//
hardware_interface::return_type HardwareInterface::who_and_what_switched(const std::vector<std::string>& interfaces, std::vector<std::pair<long, ControlMode>>& changes) {

    for (const std::string& iface : interfaces) {
        long who = -1;
        ControlMode what = ControlMode::INACTIVE;

        for (long i = 0; i < arms.size(); ++i) {
            if (iface.find(arms[i].name) != std::string::npos) {
                who = i;
            }
        }

        if (who < 0) {
            RCLCPP_ERROR(get_logger(), "An unknown robot tried to modifty an interface");
            return hardware_interface::return_type::ERROR;
        }

        if (iface.find("position") != std::string::npos) {
            what = ControlMode::POSITION;
        } else if (iface.find("velocity") != std::string::npos) {
            what = ControlMode::VELOCITY;
        } else if (iface.find("effort") != std::string::npos) {
            what = ControlMode::EFFORT;
        } else {
            RCLCPP_ERROR(get_logger(), "%s tried to modify an unsupported interface", 
                arms[who].name.c_str()
            );
            return hardware_interface::return_type::ERROR; 
        }

        bool already_in = false;
        for (const auto& change : changes) {
            if (change.first == who) {
                already_in = true;
                if (change.second != what) {
                    RCLCPP_ERROR(get_logger(), "%s tried to modify %s interface, but it has modified %s", 
                        arms[who].name.c_str(), control_to_string(what).c_str(), control_to_string(change.second).c_str()
                    );
                    return hardware_interface::return_type::ERROR;   
                }
            }
        }
        
        if (!already_in) {
            changes.push_back(std::pair(who, what));
        } 
    }

    return hardware_interface::return_type::OK;
}

void HardwareInterface::setup_controller(RobotUnit& robot, ControlMode mode) {
    const auto jointPositionControl = [this, &robot]() {
        try{
            robot.arm->control(
            [this, &robot](const franka::RobotState& state, const franka::Duration& /*period*/) {
                {
                    std::lock_guard<std::mutex> lock(*robot.control_mutex);
                    robot.current_state = state;

                    robot.if_states.q   = robot.current_state.q;
                    robot.if_states.qd  = robot.current_state.dq;
                    robot.if_states.tau = robot.current_state.tau_J;

                    if (robot.first_position_update) {
                        // std::lock_guard<std::mutex> lock(*robot.write_mutex);
                        RCLCPP_INFO(get_logger(), "First position initialized in arm %s", robot.name.c_str());
                        std::copy(robot.if_states.q.begin(), robot.if_states.q.end(), robot.exported_cmds.q.begin());
                        std::copy(robot.if_states.q.begin(), robot.if_states.q.end(), robot.if_cmds.q.begin());
                        robot.first_position_update = false;
                    }
                }

                {
                    std::lock_guard<std::mutex> lock(*robot.write_mutex);
                    JointPositions out = JointPositions(robot.if_cmds.q);
                    
                    if (!limit_override) {
                        out.q = franka::limitRate(
                            franka::computeUpperLimitsJointVelocity(robot.current_state.q_d),
                            franka::computeLowerLimitsJointVelocity(robot.current_state.q_d),
                            franka::kMaxJointAcceleration, franka::kMaxJointJerk, out.q,
                            robot.current_state.q_d, robot.current_state.dq_d, robot.current_state.ddq_d);
                    }

                    return out;
                }
            });
        }
        catch(franka::ControlException& e){
            RCLCPP_ERROR(get_logger(), "Exception %s: %s", robot.name.c_str(), e.what());
        }
    };

    const auto jointVelocityControl = [this, &robot]() {
        try{
            robot.arm->control(
            [this, &robot](const franka::RobotState& state, const franka::Duration& /*period*/) {
                {
                    std::lock_guard<std::mutex> lock(*robot.control_mutex);
                    robot.current_state = state;

                    robot.if_states.q   = robot.current_state.q;
                    robot.if_states.qd  = robot.current_state.dq;
                    robot.if_states.tau = robot.current_state.tau_J;
                }
            
                {
                    std::lock_guard<std::mutex> lock(*robot.write_mutex);
                    JointVelocities out = JointVelocities(robot.if_cmds.qd);
                    
                    if (!limit_override) {
                        out.dq = franka::limitRate(
                            franka::computeUpperLimitsJointVelocity(robot.current_state.q_d),
                            franka::computeLowerLimitsJointVelocity(robot.current_state.q_d), 
                            franka::kMaxJointAcceleration, franka::kMaxJointJerk, 
                            out.dq, robot.current_state.dq_d, robot.current_state.ddq_d);
                    }
                    return out;
                }
            });
        }
        catch(franka::ControlException& e){
            RCLCPP_ERROR(get_logger(), "Exception %s: %s", robot.name.c_str(), e.what());
        }
    };

    const auto jointEffortControl = [this, &robot]() {
        try{
            robot.arm->control(
            [this, &robot](const franka::RobotState& state, const franka::Duration& /*period*/) {
                {
                    std::lock_guard<std::mutex> lock(*robot.control_mutex);
                    robot.current_state = state;

                    robot.if_states.q   = robot.current_state.q;
                    robot.if_states.qd  = robot.current_state.dq;
                    robot.if_states.tau = robot.current_state.tau_J;
                }

                {
                    std::lock_guard<std::mutex> lock(*robot.write_mutex);
                    Torques out = Torques(robot.if_cmds.tau);
                    if (!limit_override) {
                        out.tau_J =
                            franka::limitRate(franka::kMaxTorqueRate, out.tau_J, robot.current_state.tau_J_d);
                    }
                    return out;
                }
            });
        }
        catch(franka::ControlException& e){
            RCLCPP_ERROR(get_logger(), "Exception %s: %s", robot.name.c_str(), e.what());
        }
    };

    try {
        if(mode == ControlMode::POSITION) {
            robot.control = std::make_unique<std::thread>(jointPositionControl);
        } else if (mode == ControlMode::VELOCITY) {
            robot.control = std::make_unique<std::thread>(jointVelocityControl);
        } else if (mode == ControlMode::EFFORT) {
            robot.control = std::make_unique<std::thread>(jointEffortControl);
        } else {
            robot.control.reset(nullptr);
        }
    } catch (const franka::ControlException& e) {
        RCLCPP_WARN(get_logger(), "Initial attempt on %s to start controller failed: %s", robot.name.c_str(), e.what());
        robot.arm->automaticErrorRecovery();
        if(mode == ControlMode::POSITION) {
            robot.control = std::make_unique<std::thread>(jointPositionControl);
        } else if (mode == ControlMode::VELOCITY) {
            robot.control = std::make_unique<std::thread>(jointVelocityControl);
        } else if (mode == ControlMode::EFFORT) {
            robot.control = std::make_unique<std::thread>(jointEffortControl);
        } else {
            robot.control.reset(nullptr);
        }
        RCLCPP_INFO(get_logger(), "Attempt of recovery on %s successful", robot.name.c_str());
    }

}

void HardwareInterface::reset_controller(RobotUnit& robot) {
    robot.control_mode = ControlMode::INACTIVE;
    robot.arm->stop();

    // There is a loaded control
    if (robot.control) {
        robot.control->join();
        robot.control.reset(nullptr);
    }
}

std::string HardwareInterface::control_to_string(const ControlMode& mode) {
    switch(mode) { 
        case ControlMode::INACTIVE:
        return "inactive";
        case ControlMode::POSITION:
        return "position";
        case ControlMode::VELOCITY:
        return "velocity";
        case ControlMode::EFFORT:
        return "effort";
        default:
        return "???";
    }
}

//  _____                       _
// | ____|_  ___ __   ___  _ __| |_
// |  _| \ \/ / '_ \ / _ \| '__| __|
// | |___ >  <| |_) | (_) | |  | |_
// |_____/_/\_\ .__/ \___/|_|   \__|
//            |_|
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    franka::HardwareInterface, hardware_interface::SystemInterface
);