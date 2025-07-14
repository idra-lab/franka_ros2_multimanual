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

/*
INFO
    - Tutti e due si controllano singolarmente rimuovendo le interfacce degli altri
    - Spawnano contemporaneamente senza controlli

    - Effort interface funziona ma va in blocco dopo 360, 
    probabilmente perche' serve velocita' infinita dato che e' finita la rotazione
*/

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

        /*arms[i].e_read = std::thread([this]() {
            //set_realtime_priority(90);  // SCHED_FIFO

            while (running_) {
                try {
                    auto state = robot_.readOnce();

                    {
                        // std::lock_guard<std::mutex> lock(control_mutex);
                        arms[i].if_states.q   = arms[i].current_state.q;
                        arms[i].if_states.qd  = arms[i].current_state.dq;
                        arms[i].if_states.tau = arms[i].current_state.tau_J;
                    }

                    // Sleep if needed to avoid CPU saturation
                    std::this_thread::sleep_for(std::chrono::microseconds(300));
                } catch (const franka::Exception& e) {
                    RCLCPP_ERROR(get_logger(), "FR3 read error: %s", e.what());
                }
            }
        });*/

        ++name_it;
        ++ip_it;
    }

    RCLCPP_INFO(get_logger(), "Initialized %lu robots", arms.size());

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

    reset_controllers();
    
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
    RCLCPP_INFO(get_logger(), "Handling error in on_error()");
    if (hardware_interface::SystemInterface::on_error(prev_state)
        != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "parent on_error() failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Stop everything and attempt recovery
    RCLCPP_INFO(get_logger(), "%f %f", 
        arms[0].current_state.control_command_success_rate, 
        arms[1].current_state.control_command_success_rate
    );

    RCLCPP_INFO(get_logger(), "E1: %s - %s", 
        std::string(arms[0].current_state.current_errors).c_str(), 
        std::string(arms[0].current_state.last_motion_errors).c_str()
    );

    RCLCPP_INFO(get_logger(), "E2: %s - %s", 
        std::string(arms[1].current_state.current_errors).c_str(), 
        std::string(arms[1].current_state.last_motion_errors).c_str()
    );

    // After recovery, you are in an "Unconfigured" like state. The system
    // will need to be configured and activated again.
    RCLCPP_INFO(get_logger(), "on_error() processed. System is now inactive and requires reconfiguration.");
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
            state_interfaces.emplace_back(jnt_name, HW_IF_EFFORT, &arms[p].if_states.tau[i]);
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

            cmd_interfaces.emplace_back(jnt_name, HW_IF_POSITION, &arms[p].if_cmds.q[i]);
            cmd_interfaces.emplace_back(jnt_name, HW_IF_VELOCITY, &arms[p].if_cmds.qd[i]);
            cmd_interfaces.emplace_back(jnt_name, HW_IF_EFFORT, &arms[p].if_cmds.tau[i]);
        }
    }

    return cmd_interfaces;
}

hardware_interface::return_type
HardwareInterface::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */) {
    auto start = std::chrono::high_resolution_clock::now();
    auto call_diff = std::chrono::duration_cast<std::chrono::microseconds> ( start - last_call );

    if (call_diff.count() > 1500) {
        RCLCPP_INFO(get_logger(), "Last call diff: %lu micros", call_diff.count());
    }
    last_call = start;

    std::lock_guard<std::mutex> lock(control_mutex);

    for (long i = 0; i < arms.size(); ++i) {
        auto inner_start = std::chrono::high_resolution_clock::now();
        if ( !update_state(arms[i]) ) {
            return hardware_interface::return_type::ERROR;
        }
        auto inner_diff = std::chrono::duration_cast<std::chrono::microseconds> ( std::chrono::high_resolution_clock::now() - inner_start );
        // RCLCPP_INFO(get_logger(), "Read of %lu: %lu micros", i, inner_diff.count());
    }

    auto diff = std::chrono::duration_cast<std::chrono::microseconds> ( std::chrono::high_resolution_clock::now() - start );
    //RCLCPP_INFO(get_logger(), "Read: %lu micros", diff.count());

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::write(const rclcpp::Time& /* time */, const rclcpp::Duration& period ) {
    auto start = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> lock(control_mutex);

    Vec7 joint_command{};

    bool all_active = std::all_of(arms.begin(), arms.end(), [](const RobotUnit &item) {
        return item.control && item.control_mode != ControlMode::INACTIVE;
    });

    bool all_loaded = std::all_of(arms.begin(), arms.end(), [](const RobotUnit &item) {
        return item.control != nullptr;
    });

    // if(all_active)
    for (long i = 0; i < arms.size(); ++i) {
        const RobotUnit& arm = arms[i];
        if (arm.control_mode != ControlMode::INACTIVE) {
            if (arm.control_mode == ControlMode::POSITION && !arms[i].first_position_update) {
                std::copy(arm.if_cmds.q.begin(), arm.if_cmds.q.end(), joint_command.begin());
                JointPositions position_command{joint_command};
                /*
                position_command.q = franka::limitRate(
                    franka::computeUpperLimitsJointVelocity(arm.current_state.q_d),
                    franka::computeLowerLimitsJointVelocity(arm.current_state.q_d),
                    franka::kMaxJointAcceleration, franka::kMaxJointJerk, position_command.q,
                    arm.current_state.q_d, arm.current_state.dq_d, arm.current_state.ddq_d);
                */
                arm.control->writeOnce(position_command);
            } else if (arm.control_mode == ControlMode::VELOCITY) {
                std::copy(arm.if_cmds.qd.begin(), arm.if_cmds.qd.end(), joint_command.begin());
                JointVelocities velocity_command = JointVelocities(joint_command);
                
                velocity_command.dq = franka::limitRate(
                    franka::computeUpperLimitsJointVelocity(arm.current_state.q_d),
                    franka::computeLowerLimitsJointVelocity(arm.current_state.q_d), 
                    franka::kMaxJointAcceleration, franka::kMaxJointJerk, 
                    velocity_command.dq, arm.current_state.dq_d, arm.current_state.ddq_d);
                
                // arm.control->writeOnce(velocity_command);
            } else if (arm.control_mode == ControlMode::EFFORT) {
                std::copy(arm.if_cmds.tau.begin(), arm.if_cmds.tau.end(), joint_command.begin());
                Torques torque_command = Torques(joint_command);
                /*
                torque_command.tau_J =
                    franka::limitRate(franka::kMaxTorqueRate, torque_command.tau_J, arm.current_state.tau_J_d);
                */
                arm.control->writeOnce(torque_command);
            }
        }
    }

    auto diff = std::chrono::duration_cast<std::chrono::microseconds> ( std::chrono::high_resolution_clock::now() - start );
    // auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::high_resolution_clock::now() - e_start );
    //RCLCPP_INFO(get_logger(), "Write: %lu micros", diff.count());
    
    // RCLCPP_INFO(get_logger(), "Elapsed time since last write: %lu micros",  period.nanoseconds() / 1000 );

    /*
    RCLCPP_INFO(get_logger(), "%f %f %f %f %f %f %f ", 
        position_command.q[0], position_command.q[1], position_command.q[2], position_command.q[3],
        position_command.q[4], position_command.q[5], position_command.q[6]);
    */

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& /* stop_interfaces */
) {
    auto start = std::chrono::high_resolution_clock::now();

    ControlMode temp_control_mode = ControlMode::INACTIVE;

    std::lock_guard<std::mutex> lock(control_mutex);

    RCLCPP_INFO(get_logger(), "Switching command mode...");
    
    //POSSIBLE ERROR: TWO TIMES SETUP. CHECK NAME OF THE ROBOT

    std::string iface = start_interfaces[0];

    int who_triggered;
    if ( iface.find("franka1") != std::string::npos) who_triggered = 0;
    else if ( iface.find("franka2") != std::string::npos) who_triggered = 1;
    else who_triggered = -1;

    RCLCPP_INFO(get_logger(), "Triggered interface switch %s by %d", iface.c_str(), who_triggered);

    if (iface.find("position") != std::string::npos) {
        RCLCPP_INFO(get_logger(), "Starting interface position...");

        temp_control_mode = ControlMode::POSITION;
        arms[who_triggered].first_position_update = true;
    } else if (iface.find("velocity") != std::string::npos) {
        RCLCPP_INFO(get_logger(), "Starting interface velocity...");

        temp_control_mode = ControlMode::VELOCITY;
        std::fill(arms[who_triggered].if_cmds.qd.begin(), arms[who_triggered].if_cmds.qd.end(), 0);
    } else if (iface.find("effort") != std::string::npos) {
        RCLCPP_INFO(get_logger(), "Starting interface effort...");
        
        temp_control_mode = ControlMode::EFFORT;   
        std::fill(arms[who_triggered].if_cmds.tau.begin(), arms[who_triggered].if_cmds.tau.end(), 0);
    } else {
        RCLCPP_ERROR(get_logger(), "Unsupported command interface: %s", iface.c_str());
        return hardware_interface::return_type::ERROR;
    }

    arms[who_triggered].control_mode = ControlMode::INACTIVE;
    arms[who_triggered].arm->stop();
    arms[who_triggered].control.reset(nullptr);

    setup_controller(arms[who_triggered], temp_control_mode);

    arms[who_triggered].control_mode = temp_control_mode;

    RCLCPP_INFO(get_logger(), "Setup complete");

    auto diff = std::chrono::duration_cast<std::chrono::microseconds> ( std::chrono::high_resolution_clock::now() - start );
    RCLCPP_INFO(get_logger(), "PCS %lu", diff.count());

    return hardware_interface::return_type::OK;
}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___
// | |_) | '__| \ \ / / _` | __/ _ \
// |  __/| |  | |\ V / (_| | ||  __/
// |_|   |_|  |_| \_/ \__,_|\__\___|
//
void HardwareInterface::setup_controller(RobotUnit& robot, ControlMode mode) {

    try {
       if(mode == ControlMode::POSITION) {
            robot.control = robot.arm->startJointPositionControl(research_interface::robot::Move::ControllerMode::kJointImpedance);
        } else if (mode == ControlMode::VELOCITY) {
            const auto kJointVelocityControl = [this, &robot]() {
                try{
                    robot.arm->control(
                    [this, &robot](const franka::RobotState& state, const franka::Duration& /*period*/) {
                        {
                            std::lock_guard<std::mutex> lock(control_mutex);
                            robot.current_state = state;

                            robot.if_states.q   = robot.current_state.q;
                            robot.if_states.qd  = robot.current_state.dq;
                            robot.if_states.tau = robot.current_state.tau_J;
                        }
                        // std::lock_guard<std::mutex> lock(write_mutex_);

                        // std::copy(arm.if_cmds.qd.begin(), arm.if_cmds.qd.end(), joint_command.begin());
                        JointVelocities out = JointVelocities(robot.if_cmds.qd);
                        
                        out.dq = franka::limitRate(
                            franka::computeUpperLimitsJointVelocity(robot.current_state.q_d),
                            franka::computeLowerLimitsJointVelocity(robot.current_state.q_d), 
                            franka::kMaxJointAcceleration, franka::kMaxJointJerk, 
                            out.dq, robot.current_state.dq_d, robot.current_state.ddq_d);

                        return out;
                    });
                }
                catch(franka::ControlException& e){
                    RCLCPP_ERROR(get_logger(), "Exception %s: %s", robot.name.c_str(), e.what());
                    // setError(true);
                }
            };
            robot.e_ctrl = std::make_unique<std::thread>(kJointVelocityControl);
        } else if (mode == ControlMode::EFFORT) {
            robot.control = robot.arm->startTorqueControl();
        } else {
            robot.control.reset(nullptr);
        }
    } catch (const franka::ControlException& e) {
        RCLCPP_WARN(get_logger(), "Initial attempt on %s to start controller failed: %s", robot.name.c_str(), e.what());
        robot.arm->automaticErrorRecovery();
        if(mode == ControlMode::POSITION) {
            robot.control = robot.arm->startJointPositionControl(research_interface::robot::Move::ControllerMode::kJointImpedance);
        } else if (mode == ControlMode::VELOCITY) {
            robot.control = robot.arm->startJointVelocityControl(research_interface::robot::Move::ControllerMode::kJointImpedance);
        } else if (mode == ControlMode::EFFORT) {
            robot.control = robot.arm->startTorqueControl();
        } else {
            robot.control.reset(nullptr);
        }
        RCLCPP_INFO(get_logger(), "Attempt of recovery on %s successful", robot.name.c_str());
    }

}

bool HardwareInterface::update_state(RobotUnit& robot) {
    // TOO SLOW

    try {
        /*
        if (!robot.control) {
            robot.current_state = robot.arm->readOnce();
        } else {
            std::tie(robot.current_state, std::ignore) = robot.control->readOnce(); 
        }
        
        robot.if_states.q   = robot.current_state.q;
        robot.if_states.qd  = robot.current_state.dq;
        robot.if_states.tau = robot.current_state.tau_J;
        */

        if (robot.first_position_update && robot.control_mode == ControlMode::POSITION) {
            RCLCPP_INFO(get_logger(), "First position initialized in arm %s", robot.name.c_str());
            std::copy(robot.if_states.q.begin(), robot.if_states.q.end(), robot.if_cmds.q.begin());
            robot.first_position_update = false;
        }
    } catch (const franka::ControlException& e) {
        RCLCPP_ERROR(get_logger(), "Exception in arm %s: %s", robot.name.c_str(), e.what());
        return false;
    }

    return true;
}

void HardwareInterface::reset_controllers() {
    for (long i = 0; i < arms.size(); ++i) {
        arms[i].control_mode = ControlMode::INACTIVE;
        arms[i].control.reset(nullptr);
        arms[i].arm->stop();
        arms[i].e_ctrl->join();
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