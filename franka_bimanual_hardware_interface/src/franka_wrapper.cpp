#include "franka_bimanual_hardware_interface/franka_wrapper.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "franka/active_control_base.h"

#include "franka/control_tools.h"
#include "franka/rate_limiting.h"

void FrankaRobotWrapper::copy_state_to_ifs(const franka::RobotState& state) {
    if_states.q     = state.q;
    if_states.qd    = state.dq;
    if_states.tau   = state.tau_J;
    if_states.elbow = state.elbow;
}

void FrankaRobotWrapper::setup_controller(ControlMode mode) {

    bool limit_override = false;

    const auto jointPositionControl = [this, limit_override]() {
        try{
            arm->control(
            [this, limit_override](const franka::RobotState& state, const franka::Duration& /*period*/) {
                {
                    std::lock_guard<std::mutex> lock(*control_mutex);
                    current_state = state;

                    copy_state_to_ifs(state);

                    if (first_position_update) {
                        // std::lock_guard<std::mutex> lock(*write_mutex);
                        RCLCPP_INFO(get_logger(), "First position initialized in arm %s", name.c_str());
                        std::copy(if_states.q.begin(), if_states.q.end(), exported_cmds.q.begin());
                        std::copy(if_states.q.begin(), if_states.q.end(), if_cmds.q.begin());
                        first_position_update = false;
                    }
                }

                {
                    std::lock_guard<std::mutex> lock(*write_mutex);
                    franka::JointPositions out = franka::JointPositions(if_cmds.q);
                    
                    if (!limit_override) {
                        out.q = franka::limitRate(
                            franka::computeUpperLimitsJointVelocity(current_state.q_d),
                            franka::computeLowerLimitsJointVelocity(current_state.q_d),
                            franka::kMaxJointAcceleration, franka::kMaxJointJerk, out.q,
                            current_state.q_d, current_state.dq_d, current_state.ddq_d);
                    }

                    out.motion_finished = (control_mode == ControlMode::INACTIVE);

                    return out;
                }
            });
        }
        catch(franka::ControlException& e){
            RCLCPP_ERROR(get_logger(), "Exception %s: %s", name.c_str(), e.what());
        }
    };

    const auto jointVelocityControl = [this, limit_override]() {
        try{
            arm->control(
            [this, limit_override](const franka::RobotState& state, const franka::Duration& /*period*/) {
                {
                    std::lock_guard<std::mutex> lock(*control_mutex);
                    current_state = state;

                    copy_state_to_ifs(state);
                }
            
                {
                    std::lock_guard<std::mutex> lock(*write_mutex);
                    franka::JointVelocities out = franka::JointVelocities(if_cmds.qd);
                    
                    if (!limit_override) {
                        out.dq = franka::limitRate(
                            franka::computeUpperLimitsJointVelocity(current_state.q_d),
                            franka::computeLowerLimitsJointVelocity(current_state.q_d), 
                            franka::kMaxJointAcceleration, franka::kMaxJointJerk, 
                            out.dq, current_state.dq_d, current_state.ddq_d);
                    }

                    out.motion_finished = (control_mode == ControlMode::INACTIVE);

                    return out;
                }
            });
        }
        catch(franka::ControlException& e){
            RCLCPP_ERROR(get_logger(), "Exception %s: %s", name.c_str(), e.what());
        }
    };

    const auto jointEffortControl = [this, limit_override]() {
        try{
            arm->control(
            [this, limit_override](const franka::RobotState& state, const franka::Duration& /*period*/) {
                {
                    std::lock_guard<std::mutex> lock(*control_mutex);
                    current_state = state;

                    copy_state_to_ifs(state);
                }

                {
                    std::lock_guard<std::mutex> lock(*write_mutex);
                    franka::Torques out = franka::Torques(if_cmds.tau);
                    if (!limit_override) {
                        out.tau_J =
                            franka::limitRate(franka::kMaxTorqueRate, out.tau_J, current_state.tau_J_d);
                    }

                    out.motion_finished = (control_mode == ControlMode::INACTIVE);

                    return out;
                }
            });
        }
        catch(franka::ControlException& e){
            RCLCPP_ERROR(get_logger(), "Exception %s: %s", name.c_str(), e.what());
        }
    };

    const auto cartesianVelocityControl = [this, limit_override]() {
        try{
            arm->control(
            [this, limit_override](const franka::RobotState& state, const franka::Duration& /*period*/) {
                {
                    std::lock_guard<std::mutex> lock(*control_mutex);
                    current_state = state;

                    copy_state_to_ifs(state);

                    if (first_elbow_update) {
                        // std::lock_guard<std::mutex> lock(*write_mutex);
                        RCLCPP_INFO(get_logger(), "First elbow initialized in arm %s", name.c_str());
                        std::copy(if_states.elbow.begin(), if_states.elbow.end(), exported_cmds.elbow.begin());
                        std::copy(if_states.elbow.begin(), if_states.elbow.end(), if_cmds.elbow.begin());
                        first_elbow_update = false;
                    }
                }

                {
                    std::lock_guard<std::mutex> lock(*write_mutex);

                    franka::CartesianVelocities out = elbow_control ? 
                        franka::CartesianVelocities(if_cmds.xd, if_cmds.elbow) : 
                        franka::CartesianVelocities(if_cmds.xd);

                    // RCLCPP_INFO(get_logger(), "%f %f", if_cmds.xd[0], exported_cmds.xd[0]);

                    if (!limit_override) {
                        out.O_dP_EE = franka::limitRate(
                            franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
                            franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
                            franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk, out.O_dP_EE,
                            current_state.O_dP_EE_c, current_state.O_ddP_EE_c);
                    }

                    out.motion_finished = (control_mode == ControlMode::INACTIVE);

                    return out;
                }
            });
        }
        catch(franka::ControlException& e){
            RCLCPP_ERROR(get_logger(), "Exception %s: %s", name.c_str(), e.what());
        }
    };

    try {
        if(mode == ControlMode::POSITION) {
            control = std::make_unique<std::thread>(jointPositionControl);
        } else if (mode == ControlMode::VELOCITY) {
            control = std::make_unique<std::thread>(jointVelocityControl);
        } else if (mode == ControlMode::EFFORT) {
            control = std::make_unique<std::thread>(jointEffortControl);
        } else if (mode == ControlMode::CARTESIAN_VELOCITY) {
            control = std::make_unique<std::thread>(cartesianVelocityControl);
        } else {
            control.reset(nullptr);
        }
    } catch (const franka::ControlException& e) {
        RCLCPP_WARN(get_logger(), "Initial attempt on %s to start controller failed: %s", name.c_str(), e.what());
        arm->automaticErrorRecovery();
        if(mode == ControlMode::POSITION) {
            control = std::make_unique<std::thread>(jointPositionControl);
        } else if (mode == ControlMode::VELOCITY) {
            control = std::make_unique<std::thread>(jointVelocityControl);
        } else if (mode == ControlMode::EFFORT) {
            control = std::make_unique<std::thread>(jointEffortControl);
        } else if (mode == ControlMode::CARTESIAN_VELOCITY) {
            control = std::make_unique<std::thread>(cartesianVelocityControl);
        } else {
            control.reset(nullptr);
        }
        RCLCPP_INFO(get_logger(), "Attempt of recovery on %s successful", name.c_str());
    }

}

void FrankaRobotWrapper::reset_controller() {
    control_mode = ControlMode::INACTIVE;

    // There is a loaded control
    if (control) {
        control->join();
        control.reset(nullptr);
    }

    arm->stop();
}