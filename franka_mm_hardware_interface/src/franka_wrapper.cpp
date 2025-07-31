#include "franka_mm_hardware_interface/franka_wrapper.hpp"
#include "franka_mm_hardware_interface/lambda_control.hpp"

#include "franka/active_control_base.h"

#include "franka/control_tools.h"
#include "franka/rate_limiting.h"

#include <Eigen/Dense>

FrankaRobotWrapper::FrankaRobotWrapper(
    const std::string& r_name,
    const std::string& r_ip,

    const franka::RealtimeConfig& rt_config
) : name(r_name), ip(r_ip) { 
    RCLCPP_INFO(get_logger(), "Connection with arm %s @ %s", name.c_str(), ip.c_str());

    arm = std::make_unique<franka::Robot>(ip, rt_config);
    model = std::make_unique<franka::Model>(arm->loadModel());
    
    // Controller state
    control_mode = ControlMode::INACTIVE;

    // Initialize stiffness and impedance values
    setDefaultBehaviour();

    RCLCPP_INFO(get_logger(), "Done!");
}

void FrankaRobotWrapper::copy_state_to_ifs(const franka::RobotState& state) {
    if_states.q     = state.q;
    if_states.qd    = state.dq;
    if_states.tau   = state.tau_J;
    if_states.x     = state.O_T_EE;
    if_states.elbow = state.elbow;

    Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.rotation());
    if_states.qx[0] = position.x();
    if_states.qx[1] = position.y();
    if_states.qx[2] = position.z();
    if_states.qx[3] = orientation.w();
    if_states.qx[4] = orientation.x();
    if_states.qx[5] = orientation.y();
    if_states.qx[6] = orientation.z();

    if (first_joint_position_update) {
        RCLCPP_INFO(get_logger(), "First joint position initialized in arm %s", name.c_str());
        std::copy(if_states.q.begin(), if_states.q.end(), exported_cmds.q.begin());
        std::copy(if_states.q.begin(), if_states.q.end(), if_cmds.q.begin());
        first_joint_position_update = false;
    }

    if (first_cartesian_pose_update) {
        RCLCPP_INFO(get_logger(), "First cartesian position initialized in arm %s", name.c_str());
        std::copy(if_states.x.begin(), if_states.x.end(), exported_cmds.x.begin());
        std::copy(if_states.x.begin(), if_states.x.end(), if_cmds.x.begin());

        std::copy(if_states.qx.begin(), if_states.qx.end(), exported_cmds.qx.begin());
        std::copy(if_states.qx.begin(), if_states.qx.end(), if_cmds.qx.begin());

        first_cartesian_pose_update = false;
    }

    if (first_elbow_update) {
        RCLCPP_INFO(get_logger(), "First elbow initialized in arm %s", name.c_str());
        std::copy(if_states.elbow.begin(), if_states.elbow.end(), exported_cmds.elbow.begin());
        std::copy(if_states.elbow.begin(), if_states.elbow.end(), if_cmds.elbow.begin());
        first_elbow_update = false;
    }
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
    } else if (mode == ControlMode::CARTESIAN_IMPEDANCE) {
        startController = LambdaControl::startCartesianImpedanceControl(*this, limit_override);
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
           control_mode == ControlMode::CARTESIAN_VELOCITY ||
           control_mode == ControlMode::CARTESIAN_IMPEDANCE;
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
        case ControlMode::CARTESIAN_IMPEDANCE:
        return "cartesian impedance";
        default:
        return "???";
    }
}

void FrankaRobotWrapper::setDefaultBehaviour() {
    arm->setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    arm->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    arm->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
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