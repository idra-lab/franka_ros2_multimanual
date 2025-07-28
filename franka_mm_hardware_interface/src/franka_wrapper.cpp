#include "franka_mm_hardware_interface/franka_wrapper.hpp"
#include "franka_mm_hardware_interface/lambda_control.hpp"

#include "franka/active_control_base.h"

#include "franka/control_tools.h"
#include "franka/rate_limiting.h"

void FrankaRobotWrapper::copy_state_to_ifs(const franka::RobotState& state) {
    if_states.q     = state.q;
    if_states.qd    = state.dq;
    if_states.tau   = state.tau_J;
    if_states.x     = state.O_T_EE;
    if_states.elbow = state.elbow;
}

void FrankaRobotWrapper::setup_controller(ControlMode mode, bool limit_override) {
    std::function<void()> startController;

    if(mode == ControlMode::POSITION) {
        startController = LambdaControl::startJointPositionControl(*this, limit_override);
    } else if (mode == ControlMode::VELOCITY) {
        startController = LambdaControl::startJointVelocityControl(*this, limit_override);
    } else if (mode == ControlMode::EFFORT) {
        startController = LambdaControl::startJointEffortControl(*this, limit_override);
    } else if (mode == ControlMode::CARTESIAN_POSITION) {
        startController = LambdaControl::startCartesianPositionControl(*this, limit_override);
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

bool FrankaRobotWrapper::is_cartesian() const {
    return control_mode == ControlMode::CARTESIAN_POSITION ||
           control_mode == ControlMode::CARTESIAN_VELOCITY;
}

std::string FrankaRobotWrapper::control_to_string(const ControlMode& mode) {
    switch(mode) { 
        case ControlMode::INACTIVE:
        return "inactive";
        case ControlMode::POSITION:
        return "position";
        case ControlMode::VELOCITY:
        return "velocity";
        case ControlMode::EFFORT:
        return "effort";
        case ControlMode::CARTESIAN_POSITION:
        return "cartesian position";
        case ControlMode::CARTESIAN_VELOCITY:
        return "cartesian velocity";
        default:
        return "???";
    }
}

/* --- START OF COPY --- */

void FrankaRobotWrapper::setJointStiffness(const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(*write_mutex);
  std::array<double, 7> joint_stiffness{};
  std::copy(req->joint_stiffness.cbegin(), req->joint_stiffness.cend(), joint_stiffness.begin());
  arm->setJointImpedance(joint_stiffness);
}

void FrankaRobotWrapper::setCartesianStiffness(
    const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(*write_mutex);
  std::array<double, 6> cartesian_stiffness{};
  std::copy(req->cartesian_stiffness.cbegin(), req->cartesian_stiffness.cend(),
            cartesian_stiffness.begin());
  arm->setCartesianImpedance(cartesian_stiffness);
}

void FrankaRobotWrapper::setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(*write_mutex);
  double mass(req->mass);
  std::array<double, 3> center_of_mass{};  // NOLINT [readability-identifier-naming]
  std::copy(req->center_of_mass.cbegin(), req->center_of_mass.cend(), center_of_mass.begin());
  std::array<double, 9> load_inertia{};
  std::copy(req->load_inertia.cbegin(), req->load_inertia.cend(), load_inertia.begin());

  arm->setLoad(mass, center_of_mass, load_inertia);
}

void FrankaRobotWrapper::setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(*write_mutex);

  std::array<double, 16> transformation{};  // NOLINT [readability-identifier-naming]
  std::copy(req->transformation.cbegin(), req->transformation.cend(), transformation.begin());
  arm->setEE(transformation);
}

void FrankaRobotWrapper::setStiffnessFrame(const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(*write_mutex);

  std::array<double, 16> transformation{};
  std::copy(req->transformation.cbegin(), req->transformation.cend(), transformation.begin());
  arm->setK(transformation);
}

void FrankaRobotWrapper::setForceTorqueCollisionBehavior(
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(*write_mutex);

  std::array<double, 7> lower_torque_thresholds_nominal{};
  std::copy(req->lower_torque_thresholds_nominal.cbegin(),
            req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal{};
  std::copy(req->upper_torque_thresholds_nominal.cbegin(),
            req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_nominal{};
  std::copy(req->lower_force_thresholds_nominal.cbegin(),
            req->lower_force_thresholds_nominal.cend(), lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal{};
  std::copy(req->upper_force_thresholds_nominal.cbegin(),
            req->upper_force_thresholds_nominal.cend(), upper_force_thresholds_nominal.begin());

  arm->setCollisionBehavior(lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                               lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

void FrankaRobotWrapper::setFullCollisionBehavior(
    const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(*write_mutex);

  std::array<double, 7> lower_torque_thresholds_acceleration{};
  std::copy(req->lower_torque_thresholds_acceleration.cbegin(),
            req->lower_torque_thresholds_acceleration.cend(),
            lower_torque_thresholds_acceleration.begin());
  std::array<double, 7> upper_torque_thresholds_acceleration{};
  std::copy(req->upper_torque_thresholds_acceleration.cbegin(),
            req->upper_torque_thresholds_acceleration.cend(),
            upper_torque_thresholds_acceleration.begin());
  std::array<double, 7> lower_torque_thresholds_nominal{};
  std::copy(req->lower_torque_thresholds_nominal.cbegin(),
            req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal{};
  std::copy(req->upper_torque_thresholds_nominal.cbegin(),
            req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_acceleration{};
  std::copy(req->lower_force_thresholds_acceleration.cbegin(),
            req->lower_force_thresholds_acceleration.cend(),
            lower_force_thresholds_acceleration.begin());
  std::array<double, 6> upper_force_thresholds_acceleration{};
  std::copy(req->upper_force_thresholds_acceleration.cbegin(),
            req->upper_force_thresholds_acceleration.cend(),
            upper_force_thresholds_acceleration.begin());
  std::array<double, 6> lower_force_thresholds_nominal{};
  std::copy(req->lower_force_thresholds_nominal.cbegin(),
            req->lower_force_thresholds_nominal.cend(), lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal{};
  std::copy(req->upper_force_thresholds_nominal.cbegin(),
            req->upper_force_thresholds_nominal.cend(), upper_force_thresholds_nominal.begin());
  arm->setCollisionBehavior(
      lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
      lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
      lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
      lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

/* --- END OF COPY --- */