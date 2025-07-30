
/*
 * Copyright 2025 IDRA, University of Trento
 * Author: Alessandro Moscatelli (ale.moska002@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef FRANKA_WRAPPER_HPP
#define FRANKA_WRAPPER_HPP

#include <thread>
#include <ostream>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"

#include "franka/robot.h"
#include "franka/model.h"

#include <franka_msgs/srv/set_cartesian_stiffness.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <franka_msgs/srv/set_joint_stiffness.hpp>
#include <franka_msgs/srv/set_load.hpp>
#include <franka_msgs/srv/set_stiffness_frame.hpp>
#include <franka_msgs/srv/set_tcp_frame.hpp>

#include "franka_mm_hardware_interface/franka_param_service_server.hpp"

/**
 * Class used to handle the connection and the control managment of a franka::Robot object.
 * 
 * @author Alessandro Moscatelli
 */
class FrankaRobotWrapper {
    public:

    /**
     * Array of doubles
     */
    template <std::size_t size>
    using DoubleArr  = std::array<double, size>;

    /**
     * Array of six doubles used to represent cartesian velocities
     */
    using Arr6       = std::array<double, 6>;
    
    /**
     * Array of seven doubles used to represent joint variables
     */
    using Arr7       = std::array<double, 7>;

    /**
    * Array of sixteen doubles used to represent cartesian pose.
    * Pose in expressed by a column-major transformation matrix.
    */
    using Arr16      = std::array<double, 16>;

    /**
     * From Franka hardware interface: elbow configuration.
     *
     * The values of the array are:
     *  - elbow[0]: Position of the 3rd joint in \f$[rad]\f$.
     *  - elbow[1]: Flip direction of the elbow (4th joint):
     *    - +1 if \f$q_4 > \alpha\f$
     *    - 0 if \f$q_4 == \alpha \f$
     *    - -1 if \f$q_4 < \alpha \f$
     *    .
     *    with \f$\alpha = -0.467002423653011\f$ \f$rad\f$
     */
    using ElbowArr   = std::array<double, 2>; 

    /**
     * Unique pointer to a robot
     */
    using FrankaPtr  = std::unique_ptr<franka::Robot>;

    /** 
     * Unique pointer to the thread used to spawn the control of a robot.
     */
    using ControlPtr = std::unique_ptr<std::thread>;

    /**
     * Unique pointer to the robot model
     */
    using ModelPtr   = std::unique_ptr<franka::Model>;

    /**
     * Unique pointer to a mutex
     */
    using MutexPtr   = std::unique_ptr<std::mutex>;
    
    /**
     * Enumeration class used to describe controller's possible states.
     */
    enum class ControlMode {
        INACTIVE,
        POSITION,
        VELOCITY,
        EFFORT,
        CARTESIAN_POSITION,
        CARTESIAN_VELOCITY,
        CARTESIAN_IMPEDANCE
    };

    /**
     * Struct used to populate state values that will be uploaded in the robot's state broadcaster.
     */
    struct StateValues { 
        /**
         * Joint position values
         */
        Arr7 q;

        /**
         * Joint velocity values
         */
        Arr7 qd;

        /**
         * Joint torque values
         */
        Arr7 tau;

        /**
         * Cartesian position values 
         */
        Arr16 x;

        Arr7 qx;

        /**
         * State of the elbow configuration
         */
        ElbowArr elbow;
    };

    /**
    * Struct used to populate control values uysed by the robot controller.
    */
    struct ControlValues { 
        /**
         * Joint position values
         */
        Arr7 q;

        /**
         *Joint velocity values
         */
        Arr7 qd;

        /**
         * Joint torque values
         */
        Arr7 tau;

        /**
         * Cartesian position values, expressed in column-major format.
         */
        Arr16 x;
        
        Arr7 qx;

        /**
         * Cartesian velocity values
         */
        Arr6 xd;

        /**
         * Elbow values.
         * This values are used only if the robot is controlled in cartesian position or velocity.
         */
        ElbowArr elbow;
    };

    /**
     * Values of the state to be exported as output in ros
     */
    StateValues if_states;

    /**
     * Control values exported and writible by the external controllers.
     */
    ControlValues exported_cmds;

    /**
     * Control values used by the robot controller. 
     * These values are copied form exported_cmds in a thread safe way to avoid the usage of
     * partial read values from the exported interfaces.
     */
    ControlValues if_cmds;
    
    /**
     * Current state of the robot
     */
    franka::RobotState current_state;

    /**
     * Model of the robot.
     */
    ModelPtr model;

    /**
     * Boolean value that signals if the robot has to update his joint position (q) values.
     *
     * This is done because when the position control is being activated, 
     * if_cmds and exported_cmds must be initialized with the same value of 
     * if_states to avoid errors. 
     */
    bool first_joint_position_update = false;

    /**
     * Boolean value that signals if the robot has to update his cartesian pose (x) values.
     *
     * This is done because when the position control is being activated, 
     * if_cmds and exported_cmds must be initialized with the same value of 
     * if_states to avoid errors. 
     */
    bool first_cartesian_pose_update = false;

    /**
     * Boolean value that signals if the robot has to update his elbow configuration values.
     *
     * This is done because when the position control is being activated, 
     * if_cmds and exported_cmds must be initialized with the same value of 
     * if_states to avoid errors. 
     */
    bool first_elbow_update = false;

    /**
     * Name of the robot
     */
    std::string name = "";

    /**
     * IP of the robot
     */
    std::string ip   = "";

    /**
     * Mutex used to handle concurrency in the read phase of the robot's state.
     */
    MutexPtr control_mutex = std::make_unique<std::mutex>();

    /**
     * Mutex used to handle concurrency in the write phase of the robot command.
     * 
     * This is mainly used to avoid partial readings from exported_cmds.
     */
    MutexPtr write_mutex = std::make_unique<std::mutex>();

    /**
     * Unique pointer to the Robot object of libfranka, that is used to communicate
     * commands and read states.
     */
    FrankaPtr arm = nullptr;

    /**
     * Unique pointer to the thread used to handle the robot controller.
     */
    ControlPtr control = nullptr;
    
    std::shared_ptr<FrankaParamServiceServer> param_server;

    /**
     * Current operational mode of the robot
     */
    ControlMode control_mode = ControlMode::INACTIVE;


    /**
     * Flag that signals if the robot is also being controlled with elbow inferfaces.
     * This flag can be set to true if and only if control_mode is set to CARTESIAN_VELOCITY or CARTESIAN_POSITION
     */
    bool elbow_control = false;

    RCLCPP_SHARED_PTR_DEFINITIONS(FrankaRobotWrapper);

    FrankaRobotWrapper(
        const std::string& name,
        const std::string& ip,

        const franka::RealtimeConfig& rt_config = franka::RealtimeConfig::kEnforce
    );
    ~FrankaRobotWrapper() = default;

    FrankaRobotWrapper(const FrankaRobotWrapper&)                = delete;
    FrankaRobotWrapper& operator=(const FrankaRobotWrapper&)     = delete;
    FrankaRobotWrapper(FrankaRobotWrapper&&) noexcept            = default;        
    FrankaRobotWrapper& operator=(FrankaRobotWrapper&&) noexcept = default;

    /**
     * Populates the values of if_state (q, qd, tau, x) using a robot state.
     * 
     * @param state Source state used to read the values
     */
    void copy_state_to_ifs(const franka::RobotState& state);

    /**
     * This function is used to activate a controller thread in a RobotUnit object.
     *
     * @param mode           Operational mode of the controller
     * @param limit_override If True, skips fraka::limitRate before sending a command.
     */
    void setup_controller(ControlMode mode, bool limit_override);

    /**
     * Stops the controller of a robot.
     *
     * This function sets the operational mode of the robot to inactive, 
     * waits the control thread to join and stops the robot.
     *
     * @param robot Robot that is deactivating a controller
     */
    void reset_controller();  

    /**
     * Returns a boolen that tells if the controller is commanded in 
     * cartesian pose or velocity.
     * 
     * @returns True if the controller is commandend in cartesian mode, False otherwise. 
     */
    bool is_cartesian() const;

    /* --- COPIED FROM FRANKA --- */

    /**
     * Sets the impedance for each joint in the internal controller.
     *
     * User-provided torques are not affected by this setting.
     *
     * @param[in] franka_msgs::srv::SetJointStiffness::Request::SharedPtr requests with JointStiffness
     * values
     *
     * @throw CommandException if the Control reports an error.
     * @throw NetworkException if the connection is lost, e.g. after a timeout.
     */
    virtual void setJointStiffness(
        const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& req);

    /**
     * Sets the Cartesian stiffness (for x, y, z, roll, pitch, yaw) in the internal
     * controller.
     *
     * The values set using Robot::SetCartesianStiffness are used in the direction of the
     * stiffness frame, which can be set with Robot::setK.
     *
     * Inputs received by the torque controller are not affected by this setting.
     *
     * @param[in] franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr request
     * @throw CommandException if the Control reports an error.
     * @throw NetworkException if the connection is lost, e.g. after a timeout.
     */
    virtual void setCartesianStiffness(
        const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& req);

    /**
     * Sets dynamic parameters of a payload.
     *
     * @note
     * This is not for setting end effector parameters, which have to be set in the administrator's
     * interface.
     *
     * @param[in] franka_msgs::srv::SetLoad::Request::SharedPtr request
     *
     * @throw CommandException if the Control reports an error.
     * @throw
     */
    virtual void setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& req);

    /**
     * Sets the transformation \f$^{NE}T_{EE}\f$ from nominal end effector to end effector frame.
     *
     * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
     *
     * @param[in] franka_msgs::srv::SetTCPFrame::Request::SharedPtr req
     *
     * @throw CommandException if the Control reports an error.
     * @throw NetworkException if the connection is lost, e.g. after a timeout.
     */
    virtual void setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& req);

    /**
     * Sets the transformation \f$^{EE}T_K\f$ from end effector frame to stiffness frame.
     *
     * The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
     *
     * @param[in] franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr req.
     *
     * @throw CommandException if the Control reports an error.
     * @throw NetworkException if the connection is lost, e.g. after a timeout.
     *
     */
    virtual void setStiffnessFrame(
        const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& req);

    /**
     * Changes the collision behavior.
     *
     * Set common torque and force boundaries for acceleration/deceleration and constant velocity
     * movement phases.
     *
     * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
     * Forces or torques above the upper threshold are registered as collision and cause the robot to
     * stop moving.
     *
     * @param[in] franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr req
     *
     * @throw CommandException if the Control reports an error.
     * @throw NetworkException if the connection is lost, e.g. after a timeout.
     */
    virtual void setForceTorqueCollisionBehavior(
        const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& req);

    /**
     * Changes the collision behavior.
     *
     * Set separate torque and force boundaries for acceleration/deceleration and constant velocity
     * movement phases.
     *
     * Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
     * Forces or torques above the upper threshold are registered as collision and cause the robot to
     * stop moving.
     *
     * @param[in] franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr request msg
     *
     * @throw CommandException if the Control reports an error.
     * @throw NetworkException if the connection is lost, e.g. after a timeout.
     */
    virtual void setFullCollisionBehavior(
        const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& req);
    
    /* --- COPY END --- */

    /**
    * Get RCLCPP logger object
    *  
    * @return Logger
    */
    rclcpp::Logger& get_logger() { return logger; }

    /**
    * Converts a ControlMode into a string
    *
    * @param mode Mode to be converted
    *  
    * @return String representing the control mode
    */
    static std::string control_to_string(const ControlMode& mode);
    
    private:
    /**
    * RCLCPP Logger
    */
    rclcpp::Logger logger = rclcpp::get_logger("franka_robot_wrapper");
};

#endif  // FRANKA_WRAPPER_HPP