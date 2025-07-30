#include "franka_mm_hardware_interface/mode_switch_plan.hpp"
#include "franka_mm_hardware_interface/franka_wrapper.hpp"

#include <exception>
#include <stdexcept>

ModeSwitchPlan::ModeSwitchPlan(
    const std::vector<std::string>& start_interfaces, 
    const std::vector<std::string>& stop_interfaces,
    const std::vector<FrankaRobotWrapper>& robots
) {
    who_and_what_switched(start_interfaces, robots, activations,   elbow_activations);
    who_and_what_switched(stop_interfaces,  robots, deactivations, elbow_deactivations);
}

bool ModeSwitchPlan::is_being_activated(long robot_index) const {
    return std::any_of(
        activations.begin(), activations.end(),
        [&](const auto& item) { return item.first == robot_index; }
    );
}

bool ModeSwitchPlan::is_activating_cartesian(long robot_index) const {
    return std::any_of(
        activations.begin(), activations.end(),
        [&](const auto& item) { 
            return 
                item.first == robot_index && (
                    item.second == FrankaRobotWrapper::ControlMode::CARTESIAN_POSITION  || 
                    item.second == FrankaRobotWrapper::ControlMode::CARTESIAN_VELOCITY  ||
                    item.second == FrankaRobotWrapper::ControlMode::CARTESIAN_IMPEDANCE ||
                ); 
        }
    );
}

FrankaRobotWrapper::ControlMode ModeSwitchPlan::what_is_being_activated(long robot_index) const {
    for (const ModeSwitch& change : activations) {
        if(change.first == robot_index){
            return change.second;
        }
    }

    throw std::range_error("No robot with index " + std::to_string(robot_index) + " is being activated");
}

bool ModeSwitchPlan::is_being_deactivated(long robot_index) const {
    return std::any_of(
        deactivations.begin(), deactivations.end(),
        [&](const auto& item) { return item.first == robot_index; }
    );
}

bool ModeSwitchPlan::is_elbow_being_activated(long robot_index) const {
    return std::any_of(
        elbow_activations.begin(), elbow_activations.end(),
        [&](const auto& item) { return item == robot_index; }
    );
}

bool ModeSwitchPlan::is_elbow_being_deactivated(long robot_index) const {
    return std::any_of(
        elbow_deactivations.begin(), elbow_deactivations.end(),
        [&](const auto& item) { return item == robot_index; }
    );
}

void ModeSwitchPlan::who_and_what_switched(
    const std::vector<std::string>&        interfaces,
    const std::vector<FrankaRobotWrapper>& robots,
    std::vector<ModeSwitch>&               changes,
    std::vector<long>&                     elbow_changes
) {
    using ControlMode = FrankaRobotWrapper::ControlMode;

    for (const std::string& iface : interfaces) {
        long who = -1;
        bool is_elbow = false;
        ControlMode what = ControlMode::INACTIVE;

        for (long i = 0; i < robots.size(); ++i) {
            if (iface.find(robots[i].name) != std::string::npos) {
                who = i;
                break;
            }
        }

        if (who < 0) {
            throw std::runtime_error("An unknown robot tried to modifty an interface");
        }

        if (iface.find("/position") != std::string::npos) {
            what = ControlMode::POSITION;
        } else if (iface.find("/velocity") != std::string::npos) {
            what = ControlMode::VELOCITY;
        } else if (iface.find("/effort") != std::string::npos) {
            what = ControlMode::EFFORT;
        } else if (iface.find("/cartesian_pose_command") != std::string::npos) {
            what = ControlMode::CARTESIAN_POSITION;
        } else if (iface.find("/cartesian_velocity") != std::string::npos) {
            what = ControlMode::CARTESIAN_VELOCITY;
        } else if (iface.find("/cartesian_pose") != std::string::npos) {
            what = ControlMode::CARTESIAN_IMPEDANCE;
        } else if (iface.find("/elbow_command") != std::string::npos) {
            is_elbow = true;
        } else {
            throw std::runtime_error(robots[who].name + " tried to modify an unsupported interface");
        }

        if (!is_elbow) {
            bool already_in = false;
            for (const auto& change : changes) {
                if (change.first == who) {
                    already_in = true;
                    if (change.second != what) {
                        throw std::runtime_error(
                            robots[who].name + " tried to modify " + 
                            FrankaRobotWrapper::control_to_string(what) + " interface, but it has modified " + 
                            FrankaRobotWrapper::control_to_string(change.second)
                        ); 
                    }
                }
            }
            
            if (!already_in) {
                changes.push_back(std::pair(who, what));
            } 
        } else {
            bool already_in = false;
            for (const auto& change : elbow_changes) {
                if (change == who) {
                    already_in = true;
                }
            }
            
            if (!already_in) {
                elbow_changes.push_back(who);
            } 
        }
    }
}