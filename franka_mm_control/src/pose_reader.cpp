#include "franka_mm_control/pose_reader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/controller_interface.hpp>

controller_interface::CallbackReturn PoseReader::on_init() {
    auto_declare<std::vector<std::string>>("robot_names", std::vector<std::string>()); 

    robot_names = get_node()->get_parameter("robot_names").as_string_array();

    publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("current_pose", 10);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PoseReader::command_interface_configuration() const {
    return {
        controller_interface::interface_configuration_type::NONE, {}
    };
}

controller_interface::InterfaceConfiguration PoseReader::state_interface_configuration() const  {
    std::vector<std::string> state_interface_names;
    
    state_interface_names.reserve(16 * robot_names.size());
    for (const auto& robot_name : robot_names) {
        for (long i = 0; i < 16; ++i) {
            state_interface_names.push_back(robot_name + "_" + std::to_string(i) + "/" + "cartesian_pose_command");
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
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {};

    for (const auto& iface : state_interfaces_){
        msg.data.push_back(iface.get_value());
    }

    publisher->publish(msg);

    return controller_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(PoseReader, controller_interface::ControllerInterface)