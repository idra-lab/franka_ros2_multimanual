#include "franka_bimanual_hardware_interface/lambda_control.hpp"
#include "franka_bimanual_hardware_interface/franka_wrapper.hpp"

#include "rclcpp/rclcpp.hpp"

#include "franka/robot.h"
#include "franka/control_tools.h"
#include "franka/rate_limiting.h"
#include "franka/active_control_base.h"

std::function<void()> LambdaControl::startJointPositionControl(FrankaRobotWrapper& robot, bool limit_override) {
    return [&robot, limit_override](){
        robot.arm->control([&robot, limit_override](const franka::RobotState& state, const franka::Duration& /*period*/) {
            {
                std::lock_guard<std::mutex> lock(*robot.control_mutex);
                robot.current_state = state;

                robot.copy_state_to_ifs(state);

                if (robot.first_joint_position_update) {
                    // std::lock_guard<std::mutex> lock(*robot.write_mutex);
                    RCLCPP_INFO(robot.get_logger(), "First joint position initialized in arm %s", robot.name.c_str());
                    std::copy(robot.if_states.q.begin(), robot.if_states.q.end(), robot.exported_cmds.q.begin());
                    std::copy(robot.if_states.q.begin(), robot.if_states.q.end(), robot.if_cmds.q.begin());
                    robot.first_joint_position_update = false;
                }
            }

            {
                std::lock_guard<std::mutex> lock(*robot.write_mutex);
                franka::JointPositions out = franka::JointPositions(robot.if_cmds.q);
                
                if (!limit_override) {
                    out.q = franka::limitRate(
                        franka::computeUpperLimitsJointVelocity(robot.current_state.q_d),
                        franka::computeLowerLimitsJointVelocity(robot.current_state.q_d),
                        franka::kMaxJointAcceleration, franka::kMaxJointJerk, out.q,
                        robot.current_state.q_d, robot.current_state.dq_d, robot.current_state.ddq_d);
                }

                out.motion_finished = (robot.control_mode == FrankaRobotWrapper::ControlMode::INACTIVE);

                return out;
            }
        });
    };
}

std::function<void()> LambdaControl::startJointVelocityControl(FrankaRobotWrapper& robot, bool limit_override) {
    return [&robot, limit_override](){
        robot.arm->control([&robot, limit_override](const franka::RobotState& state, const franka::Duration& /*period*/) {
            {
                std::lock_guard<std::mutex> lock(*robot.control_mutex);
                robot.current_state = state;

                robot.copy_state_to_ifs(state);
            }
        
            {
                std::lock_guard<std::mutex> lock(*robot.write_mutex);
                franka::JointVelocities out = franka::JointVelocities(robot.if_cmds.qd);
                
                if (!limit_override) {
                    out.dq = franka::limitRate(
                        franka::computeUpperLimitsJointVelocity(robot.current_state.q_d),
                        franka::computeLowerLimitsJointVelocity(robot.current_state.q_d), 
                        franka::kMaxJointAcceleration, franka::kMaxJointJerk, 
                        out.dq, robot.current_state.dq_d, robot.current_state.ddq_d);
                }

                out.motion_finished = (robot.control_mode == FrankaRobotWrapper::ControlMode::INACTIVE);

                return out;
            }
        });
    };
}

std::function<void()> LambdaControl::startJointEffortControl(FrankaRobotWrapper& robot, bool limit_override) {
    return [&robot, limit_override](){
        robot.arm->control([&robot, limit_override](const franka::RobotState& state, const franka::Duration& /*period*/) {
            {
                std::lock_guard<std::mutex> lock(*robot.control_mutex);
                robot.current_state = state;

                robot.copy_state_to_ifs(state);
            }

            {
                std::lock_guard<std::mutex> lock(*robot.write_mutex);
                franka::Torques out = franka::Torques(robot.if_cmds.tau);
                if (!limit_override) {
                    out.tau_J =
                        franka::limitRate(franka::kMaxTorqueRate, out.tau_J, robot.current_state.tau_J_d);
                }

                out.motion_finished = (robot.control_mode == FrankaRobotWrapper::ControlMode::INACTIVE);

                return out;
            }
        });
    };
}

std::function<void()> LambdaControl::startCartesianPositionControl(FrankaRobotWrapper& robot, bool limit_override) {
    return [&robot, limit_override](){
        robot.arm->control([&robot, limit_override](const franka::RobotState& state, const franka::Duration& /*period*/) {
            {
                std::lock_guard<std::mutex> lock(*robot.control_mutex);
                robot.current_state = state;

                robot.copy_state_to_ifs(state);

                if (robot.first_cartesian_position_update) {
                    // std::lock_guard<std::mutex> lock(*write_mutex);
                    RCLCPP_INFO(robot.get_logger(), "First cartesian position initialized in arm %s", robot.name.c_str());
                    std::copy(robot.if_states.x.begin(), robot.if_states.x.end(), robot.exported_cmds.x.begin());
                    std::copy(robot.if_states.x.begin(), robot.if_states.x.end(), robot.if_cmds.x.begin());
                    robot.first_cartesian_position_update = false;
                }

                if (robot.first_elbow_update) {
                    // std::lock_guard<std::mutex> lock(*write_mutex);
                    RCLCPP_INFO(robot.get_logger(), "First elbow initialized in arm %s", robot.name.c_str());
                    std::copy(robot.if_states.elbow.begin(), robot.if_states.elbow.end(), robot.exported_cmds.elbow.begin());
                    std::copy(robot.if_states.elbow.begin(), robot.if_states.elbow.end(), robot.if_cmds.elbow.begin());
                    robot.first_elbow_update = false;
                }
            }

            {
                std::lock_guard<std::mutex> lock(*robot.write_mutex);

                franka::CartesianPose out = robot.elbow_control ? 
                    franka::CartesianPose(robot.if_cmds.x, robot.if_cmds.elbow) : 
                    franka::CartesianPose(robot.if_cmds.x);

                // RCLCPP_INFO(get_logger(), "%f %f", if_cmds.xd[0], exported_cmds.xd[0]);

                if (!limit_override) {
                    out.O_T_EE = franka::limitRate(franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
                        franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
                        franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk,
                        out.O_T_EE, robot.current_state.O_T_EE_c,
                        robot.current_state.O_dP_EE_c, robot.current_state.O_ddP_EE_c);
                }

                out.motion_finished = (robot.control_mode == FrankaRobotWrapper::ControlMode::INACTIVE);

                return out;
            }
        });
    };
}

std::function<void()> LambdaControl::startCartesianVelocityControl(FrankaRobotWrapper& robot, bool limit_override) {
    return [&robot, limit_override](){
        robot.arm->control([&robot, limit_override](const franka::RobotState& state, const franka::Duration& /*period*/) {
            {
                std::lock_guard<std::mutex> lock(*robot.control_mutex);
                robot.current_state = state;

                robot.copy_state_to_ifs(state);

                if (robot.first_elbow_update) {
                    // std::lock_guard<std::mutex> lock(*write_mutex);
                    RCLCPP_INFO(robot.get_logger(), "First elbow initialized in arm %s", robot.name.c_str());
                    std::copy(robot.if_states.elbow.begin(), robot.if_states.elbow.end(), robot.exported_cmds.elbow.begin());
                    std::copy(robot.if_states.elbow.begin(), robot.if_states.elbow.end(), robot.if_cmds.elbow.begin());
                    robot.first_elbow_update = false;
                }
            }

            {
                std::lock_guard<std::mutex> lock(*robot.write_mutex);

                franka::CartesianVelocities out = robot.elbow_control ? 
                    franka::CartesianVelocities(robot.if_cmds.xd, robot.if_cmds.elbow) : 
                    franka::CartesianVelocities(robot.if_cmds.xd);

                // RCLCPP_INFO(get_logger(), "%f %f", if_cmds.xd[0], exported_cmds.xd[0]);

                if (!limit_override) {
                    out.O_dP_EE = franka::limitRate(
                        franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
                        franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
                        franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk, out.O_dP_EE,
                        robot.current_state.O_dP_EE_c, robot.current_state.O_ddP_EE_c);
                }

                out.motion_finished = (robot.control_mode == FrankaRobotWrapper::ControlMode::INACTIVE);

                return out;
            }
        });
    };
}