#include "franka_bimanual_hardware_interface/franka_wrapper.hpp"
#include "franka_bimanual_hardware_interface/lambda_control.hpp"

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
    std::function<void()> startController;

    if(mode == ControlMode::POSITION) {
        startController = LambdaControl::startJointPositionControl(*this, limit_override);
    } else if (mode == ControlMode::VELOCITY) {
        startController = LambdaControl::startJointVelocityControl(*this, limit_override);
    } else if (mode == ControlMode::EFFORT) {
        startController = LambdaControl::startJointEffortControl(*this, limit_override);
    } else if (mode == ControlMode::CARTESIAN_VELOCITY) {
        startController = LambdaControl::startCartesianVelocityControl(*this, limit_override);
    } else {
        control.reset(nullptr);
        return;
    }

    control = std::make_unique<std::thread>([this, startController, limit_override]() {
        try {
            startController();
        } catch(franka::ControlException& e){
            try {
                RCLCPP_WARN(get_logger(), "Initial attempt on %s to start controller failed: %s", name.c_str(), e.what());
                arm->automaticErrorRecovery();
                startController();
                RCLCPP_INFO(get_logger(), "Attempt of recovery on %s successful", name.c_str());
            } catch(franka::ControlException& fatal) {
                RCLCPP_ERROR(get_logger(), "Exception %s: %s", name.c_str(), fatal.what());
            }
        }
    });
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