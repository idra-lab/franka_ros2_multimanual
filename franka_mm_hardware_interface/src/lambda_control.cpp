#include "franka_mm_hardware_interface/lambda_control.hpp"
#include "franka_mm_hardware_interface/franka_wrapper.hpp"
#include "franka_mm_hardware_interface/control_modes.hpp"

#include "rclcpp/rclcpp.hpp"

#include "franka/robot.h"
#include "franka/model.h"
#include "franka/control_tools.h"
#include "franka/rate_limiting.h"
#include "franka/active_control_base.h"

#include <Eigen/Dense>

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
                
                /*
                RCLCPP_INFO(robot.get_logger(), "%f %f %f %f %f %f %f ", 
                                        robot.if_cmds.q[0], robot.if_cmds.q[1], robot.if_cmds.q[2], robot.if_cmds.q[3],
                                        robot.if_cmds.q[4], robot.if_cmds.q[5], robot.if_cmds.q[6]);
                */

                if (!limit_override) {
                    out.q = franka::limitRate(
                        franka::computeUpperLimitsJointVelocity(robot.current_state.q_d),
                        franka::computeLowerLimitsJointVelocity(robot.current_state.q_d),
                        franka::kMaxJointAcceleration, franka::kMaxJointJerk, out.q,
                        robot.current_state.q_d, robot.current_state.dq_d, robot.current_state.ddq_d);
                }

                out.motion_finished = (robot.control_mode == ControlMode::INACTIVE);

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

                out.motion_finished = (robot.control_mode == ControlMode::INACTIVE);

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
                
                std::array<double, 7> coriolis_array = robot.model->coriolis(state);
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_d(robot.if_cmds.tau.data());
                
                Eigen::VectorXd tau_out(7);
                tau_out << tau_d + coriolis;

                std::array<double, 7> tau_d_array{};
                Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_out;

                franka::Torques out = franka::Torques(tau_d_array);
                if (!limit_override) {
                    out.tau_J =
                        franka::limitRate(franka::kMaxTorqueRate, out.tau_J, robot.current_state.tau_J_d);
                }

                out.motion_finished = (robot.control_mode == ControlMode::INACTIVE);

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

                if (robot.first_cartesian_pose_update) {
                    // std::lock_guard<std::mutex> lock(*write_mutex);
                    RCLCPP_INFO(robot.get_logger(), "First cartesian position initialized in arm %s", robot.name.c_str());
                    std::copy(robot.if_states.x.begin(), robot.if_states.x.end(), robot.exported_cmds.x.begin());
                    std::copy(robot.if_states.x.begin(), robot.if_states.x.end(), robot.if_cmds.x.begin());
                    robot.first_cartesian_pose_update = false;
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

                out.motion_finished = (robot.control_mode == ControlMode::INACTIVE);

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

                out.motion_finished = (robot.control_mode == ControlMode::INACTIVE);

                return out;
            }
        });
    };
}

std::function<void()> LambdaControl::startCartesianImpedanceControl(FrankaRobotWrapper& robot, bool limit_override) {
    return [&robot, limit_override](){

        // Compliance parameters
        double dt = 1.0 / 1000.0;
        const double translational_stiffness{500.0};
        const double rotational_stiffness{100.0};
        Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        damping.setZero();
        damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                            Eigen::MatrixXd::Identity(3, 3);
        damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                                Eigen::MatrixXd::Identity(3, 3);

        Eigen::Matrix<double, 6, 1> error_old = Eigen::Matrix<double, 6, 1>::Zero();

        robot.arm->control([&robot, limit_override, stiffness, damping, dt, &error_old](const franka::RobotState& state, const franka::Duration& /*period*/) -> franka::Torques {
            {
                std::lock_guard<std::mutex> lock(*robot.control_mutex);
                robot.current_state = state;

                robot.copy_state_to_ifs(state);
            }

            {
                std::lock_guard<std::mutex> lock(*robot.write_mutex);

                // equilibrium point is the setted state
                Eigen::Affine3d transform_d(Eigen::Matrix4d::Map(state.O_T_EE.data()));
                Eigen::Vector3d position_d(robot.if_cmds.qx[0], robot.if_cmds.qx[1], robot.if_cmds.qx[2]);
                Eigen::Quaterniond orientation_d(robot.if_cmds.qx[3], robot.if_cmds.qx[4], robot.if_cmds.qx[5], robot.if_cmds.qx[6]); // w, x, y, z

                // get state variables
                std::array<double, 7> coriolis_array  = robot.model->coriolis(state);
                std::array<double, 42> jacobian_array = robot.model->zeroJacobian(franka::Frame::kEndEffector, state);
            
                // convert to Eigen
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
                Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(state.q.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
                Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
                Eigen::Vector3d position(transform.translation());  
                Eigen::Quaterniond orientation(transform.rotation());
            
                // compute error to desired equilibrium pose
                // position error
                Eigen::Matrix<double, 6, 1> error;
                error.head(3) << position - position_d;
            
                // orientation error
                // "difference" quaternion
                if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                    orientation.coeffs() << -orientation.coeffs();
                }
                // "difference" quaternion
                Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
                error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
                // Transform to base frame
                error.tail(3) << -transform.rotation() * error.tail(3);
            
                // compute control
                Eigen::VectorXd force_task(6), tau_task(7), tau_d(7);
            
                // Spring damper system with damping ratio=1
                force_task << (-stiffness * error - damping * ((error - error_old) / dt));
                error_old = error;
                tau_task << jacobian.transpose() * force_task;
                tau_d << tau_task + coriolis;
            
                std::array<double, 7> tau_d_array{};
                Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

                franka::Torques out = franka::Torques(tau_d_array);
                if (!limit_override) {
                    out.tau_J = franka::limitRate(franka::kMaxTorqueRate, out.tau_J, robot.current_state.tau_J_d);
                }
                
                out.motion_finished = (robot.control_mode == ControlMode::INACTIVE);

                return out;
            }
        });
    };
}