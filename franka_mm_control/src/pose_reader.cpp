#include "franka_mm_control/pose_reader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <controller_interface/controller_interface.hpp>

#include <franka_mm_hardware_interface/interfaces.hpp>

controller_interface::CallbackReturn PoseReader::on_init() {
    auto_declare<std::vector<std::string>>("robot_names", std::vector<std::string>()); 

    robot_names = get_node()->get_parameter("robot_names").as_string_array();

    publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("current_pose", 10);
    q_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("current_q_pose", 10);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PoseReader::command_interface_configuration() const {
    return {
        controller_interface::interface_configuration_type::NONE, {}
    };
}

controller_interface::InterfaceConfiguration PoseReader::state_interface_configuration() const  {
    using interfaces::names::cartesian_pose_q_interface_names;

    std::vector<std::string> state_interface_names;
    
    state_interface_names.reserve((16 + 7) * robot_names.size());
    for (const auto& robot_name : robot_names) {
        for (long unsigned i = 0; i < 16; ++i) {
            state_interface_names.push_back(
                robot_name + "_" + std::to_string(i) + "/" + interfaces::types::HW_IF_CART_POSITION
            );
        }
    }

    for (const auto& robot_name : robot_names) {
        for (long unsigned i = 0; i < cartesian_pose_q_interface_names.size(); ++i) {
            state_interface_names.push_back(
                robot_name + "_" + cartesian_pose_q_interface_names[i] + "/" + interfaces::types::HW_IF_CART_POSITION_Q
            );
        }
    }

    return {
        controller_interface::interface_configuration_type::INDIVIDUAL,
        state_interface_names
    };
}

controller_interface::CallbackReturn PoseReader::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PoseReader::update(const rclcpp::Time&, const rclcpp::Duration&) {
    using interfaces::names::cartesian_pose_q_interface_names;

    std_msgs::msg::Float64MultiArray msg;
    
    msg.data = {};
    long unsigned i = 0;
    while (i < 16 * robot_names.size()){
        const auto& iface = state_interfaces_[i];
        msg.data.push_back(iface.get_value());
        ++i;
    }
    publisher->publish(msg);

    msg.data = {};
    long unsigned j = 0;
    while (j < cartesian_pose_q_interface_names.size() * robot_names.size()){
        const auto& iface = state_interfaces_[i + j];
        msg.data.push_back(iface.get_value());
        ++j;
    }
    q_publisher->publish(msg);

    return controller_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(PoseReader, controller_interface::ControllerInterface)