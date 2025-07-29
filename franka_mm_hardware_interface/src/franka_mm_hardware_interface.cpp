#include "franka_mm_hardware_interface/franka_mm_hardware_interface.hpp"
#include "franka_mm_hardware_interface/franka_param_service_server.hpp"

#include <array>
#include <exception>
#include <stdexcept>
#include <algorithm>
#include <ctime>
#include <fmt/format.h>
#include <memory>
#include <regex>    

#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"

#include "franka/control_tools.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"


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

    // Check RT config
    franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
    if (!franka::hasRealtimeKernel()) {
        rt_config = franka::RealtimeConfig::kIgnore;
        RCLCPP_WARN(get_logger(), "RT kernel is not in use");
    }

    auto name_it = name_begin;
    auto ip_it = ip_begin;
    for ( long i = 0; i < ip_size; ++i ) {
        FrankaRobotWrapper frk(name_it->str(), ip_it->str());

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

    executor_thread = std::make_unique<std::thread>([this](){
        rclcpp::executors::SingleThreadedExecutor tassadar;

        for (const FrankaRobotWrapper& robot : arms) {
            tassadar.add_node(robot.param_server);
        }
        
        while (rclcpp::ok && !stopped.load()) {
            tassadar.spin_some();
        }
    });

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
        robot.reset_controller();
    }

    stopped.store(true);
    executor_thread->join();
    
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

    for (FrankaRobotWrapper& arm : arms) {
        arm.reset_controller();
    }

    for (const FrankaRobotWrapper& arm : arms) {
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

    /*
    * NOLINT: 
    * 7 joints * 3 cmd interfaces * n robots + 
    * 16 cartesian positions * n robots +
    * 7 quaternion pose * n robots +
    * 2 elbow * n robots
    */
    state_interfaces.reserve(
        7 * 3 * arms.size() +
        16 * arms.size() +
        7 * arms.size() + 
        2 * arms.size()
    );  

    for (long p = 0; p < arms.size(); ++p) {
        for (long i = 0; i < 7; ++i) {
            jnt_name = arms[p].name + "_fr3_joint" + std::to_string(i+1);
            RCLCPP_INFO(get_logger(), "%s", jnt_name.c_str());

            state_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &arms[p].if_states.q[i]);
            state_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &arms[p].if_states.qd[i]);
            state_interfaces.emplace_back(jnt_name, HW_IF_EFFORT,   &arms[p].if_states.tau[i]);
        }

        for (long i = 0; i < 16; ++i) {
            jnt_name = arms[p].name + "_" + std::to_string(i);

            state_interfaces.emplace_back(jnt_name, HW_IF_CART_POSITION, &arms[p].if_states.x[i]);
        }

        for (long i = 0; i < 7; ++i) {
            jnt_name = arms[p].name + "_" + cartesian_pose_q_interface_names[i];

            state_interfaces.emplace_back(jnt_name, HW_IF_CART_POSITION_Q, &arms[p].if_states.qx[i]);
        }

        for (long i = 0; i < 2; ++i) {
            jnt_name = arms[p].name + "_" + elbow_interfaces_names[i];

            state_interfaces.emplace_back(jnt_name, HW_IF_ELBOW, &arms[p].if_states.elbow[i]);
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

    /*
    * NOLINT: 
    * 7 joints * 3 cmd interfaces * n robots + 
    * 16 cartesian positions * n robots +
    * 7 quaternion pose * n robots +
    * 2 elbow * n robots
    */
    cmd_interfaces.reserve(
        7 * 3 * arms.size() +
        16 * arms.size() +
        7 * arms.size() + 
        2 * arms.size()
    );  

    for (long p = 0; p < arms.size(); ++p) {
        for (long i = 0; i < 7; ++i) {
            jnt_name = arms[p].name + "_fr3_joint" + std::to_string(i+1);

            cmd_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &arms[p].exported_cmds.q[i]);
            cmd_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &arms[p].exported_cmds.qd[i]);
            cmd_interfaces.emplace_back(jnt_name, HW_IF_EFFORT,   &arms[p].exported_cmds.tau[i]);
        }
        
        for (long i = 0; i < 6; ++i) {
            jnt_name = arms[p].name + "_" + cartesian_velocity_interfaces_names[i];
            
            cmd_interfaces.emplace_back(jnt_name, HW_IF_CART_VELOCITY, &arms[p].exported_cmds.xd[i]);
        }

        for (long i = 0; i < 16; ++i) {
            jnt_name = arms[p].name + "_" + std::to_string(i);

            cmd_interfaces.emplace_back(jnt_name, HW_IF_CART_POSITION, &arms[p].exported_cmds.x[i]);
        }

        for (long i = 0; i < 7; ++i) {
            jnt_name = arms[p].name + "_" + cartesian_pose_q_interface_names[i];

            cmd_interfaces.emplace_back(jnt_name, HW_IF_CART_POSITION_Q, &arms[p].exported_cmds.qx[i]);
        }

        for (long i = 0; i < 2; ++i) {
            jnt_name = arms[p].name + "_" + elbow_interfaces_names[i];

            cmd_interfaces.emplace_back(jnt_name, HW_IF_ELBOW, &arms[p].exported_cmds.elbow[i]);
        }
    }

    return cmd_interfaces;
}

hardware_interface::return_type
HardwareInterface::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    // This is used only when there is no controller loaded

    for (FrankaRobotWrapper& robot : arms) {
        if (!robot.control) {
            std::lock_guard<std::mutex> lock(*robot.control_mutex);
            robot.current_state = robot.arm->readOnce();

            robot.copy_state_to_ifs(robot.current_state);
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::write(const rclcpp::Time& /* time */, const rclcpp::Duration& period ) {
    // Copies the command from the exported interfaces to the command used by the robot to avoid concurrency problems

    for (FrankaRobotWrapper& robot : arms) {
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
    using ControlMode = FrankaRobotWrapper::ControlMode;

    try {
        mode_switch_plan = std::make_unique<ModeSwitchPlan>(start_interfaces, stop_interfaces, arms);
    } catch (std::runtime_error& e) {
        RCLCPP_ERROR(get_logger(), "Error creating the switch plan: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    for (const auto& change : mode_switch_plan->activations) {
        const FrankaRobotWrapper& arm = arms[change.first];

        // Check: Are there any activations on robot that are not planned to be deactivated?
        if (arm.control_mode != ControlMode::INACTIVE && !mode_switch_plan->is_being_deactivated(change.first)) {
            RCLCPP_ERROR(get_logger(), "%s already has an active interface %s, that is not planned to be deactivated",
                arm.name.c_str(), FrankaRobotWrapper::control_to_string(arm.control_mode).c_str()
            );

            return hardware_interface::return_type::ERROR;
        }

        // Check: Are there any elbow interfaces that will be active on erratic interface types?
        if (
            ( (change.second != ControlMode::CARTESIAN_POSITION && change.second != ControlMode::CARTESIAN_VELOCITY) && mode_switch_plan->is_elbow_being_activated(change.first) ) ||
            ( (change.second != ControlMode::CARTESIAN_POSITION && change.second != ControlMode::CARTESIAN_VELOCITY) && arm.elbow_control && !mode_switch_plan->is_elbow_being_deactivated(change.first) )
        ) {
            RCLCPP_ERROR(get_logger(), "%s is trying to activate an elbow interface on an erratic controller: %s (to be activated)",
                arm.name.c_str(), FrankaRobotWrapper::control_to_string(change.second).c_str()
            );

            return hardware_interface::return_type::ERROR;
        }
    }

    for (const long& change : mode_switch_plan->elbow_activations) { 
        const FrankaRobotWrapper& arm = arms[change]; 

        // Check: Are there any elbow interfaces that will be activated on erratic interface types on unchanged robots?
        if (
            (arm.control_mode != ControlMode::CARTESIAN_POSITION && arm.control_mode != ControlMode::CARTESIAN_VELOCITY) && 
            !mode_switch_plan->is_being_activated(change) && mode_switch_plan->is_elbow_being_activated(change) 
        ) {
            RCLCPP_ERROR(get_logger(), "%s is trying to activate an elbow interface on an erratic controller: %s (currently active)",
                arm.name.c_str(), FrankaRobotWrapper::control_to_string(arm.control_mode).c_str()
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
    using ControlMode = FrankaRobotWrapper::ControlMode;

    // Deactivations
    for (const auto& change : mode_switch_plan->elbow_deactivations) {
        FrankaRobotWrapper& arm = arms[change];

        RCLCPP_INFO(get_logger(), "%s will shut down elbow interface", 
            arm.name.c_str()
        );

        arm.elbow_control = false;

        RCLCPP_INFO(get_logger(), "%s correctly shutted down the elbow interface", 
            arm.name.c_str()
        );
    }

    for (const auto& change : mode_switch_plan->deactivations) {
        FrankaRobotWrapper& arm = arms[change.first];

        RCLCPP_INFO(get_logger(), "%s will shut down interface %s", 
            arm.name.c_str(), FrankaRobotWrapper::control_to_string(change.second).c_str()
        );

        arm.reset_controller();

        RCLCPP_INFO(get_logger(), "%s correctly shutted down the interface", 
            arm.name.c_str()
        );
    }

    // Activations
    for (const auto& change : mode_switch_plan->elbow_activations) {
        FrankaRobotWrapper& arm = arms[change];

        RCLCPP_INFO(get_logger(), "%s will activate elbow interface", 
            arm.name.c_str()
        );

        arm.first_elbow_update = true;
        arm.elbow_control = true;

        RCLCPP_INFO(get_logger(), "%s correctly activated the elbow interface", 
            arm.name.c_str()
        );
    }

    for (const auto& change : mode_switch_plan->activations) {
        FrankaRobotWrapper& arm = arms[change.first];

        RCLCPP_INFO(get_logger(), "%s will activate %s interface", 
            arm.name.c_str(), FrankaRobotWrapper::control_to_string(change.second).c_str()
        );

        if (change.second == ControlMode::POSITION) {
            arm.first_joint_position_update = true;
        } else if (change.second == ControlMode::VELOCITY) {
            std::fill(arm.exported_cmds.qd.begin(), arm.exported_cmds.qd.end(), 0);
            std::fill(arm.if_cmds.qd.begin(), arm.if_cmds.qd.end(), 0);
        } else if (change.second == ControlMode::EFFORT) {
            std::fill(arm.exported_cmds.tau.begin(), arm.exported_cmds.tau.end(), 0);
            std::fill(arm.if_cmds.tau.begin(), arm.if_cmds.tau.end(), 0);
        } else if (change.second == ControlMode::CARTESIAN_POSITION) {
            arm.first_cartesian_pose_update = true;  
        } else if (change.second == ControlMode::CARTESIAN_VELOCITY) {
            std::fill(arm.exported_cmds.xd.begin(), arm.exported_cmds.xd.end(), 0);
            std::fill(arm.if_cmds.xd.begin(), arm.if_cmds.xd.end(), 0);
        } 

        //TODO: CHECK, this reset_controller could be removed
        arm.reset_controller();
        arm.setup_controller(change.second, limit_override);

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

//  _____                       _
// | ____|_  ___ __   ___  _ __| |_
// |  _| \ \/ / '_ \ / _ \| '__| __|
// | |___ >  <| |_) | (_) | |  | |_
// |_____/_/\_\ .__/ \___/|_|   \__|
//            |_|
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    HardwareInterface, hardware_interface::SystemInterface
);