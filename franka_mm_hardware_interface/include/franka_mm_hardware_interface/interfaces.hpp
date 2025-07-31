




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
#ifndef FRANKA_MM_INTERFACES_HPP
#define FRANKA_MM_INTERFACES_HPP

#include "hardware_interface/types/hardware_interface_type_values.hpp"

/**
 * This namespace defines the constant values used by the hardware interface
 * when exporting the state and command interfaces.
 * 
 * @author Alessandro Moscatelli
 */
namespace interfaces {

    /**
     * Namespace containing the types of interface used
     */
    namespace types {
        // Already defined by hardware_interface

        /**
         * Interface type for effort
         */
        constexpr const char* HW_IF_EFFORT   = hardware_interface::HW_IF_EFFORT;

        /**
         * Interface type for position
         */
        constexpr const char* HW_IF_POSITION = hardware_interface::HW_IF_POSITION;

        /**
         * Interface type for velocity
         */
        constexpr const char* HW_IF_VELOCITY = hardware_interface::HW_IF_VELOCITY;

        //Custom Defined

        /**
         * Interface type for cartesian pose (expressed as column-major transformation matrix)
         */
        constexpr const char* HW_IF_CART_POSITION_Q = "cartesian_pose";

        /**
         * Interface type for cartesian pose (expressed as position + quaternion)
         */
        constexpr const char* HW_IF_CART_POSITION   = "cartesian_pose_command";

        /**
         * Interface type for cartesian velocity
         */
        constexpr const char* HW_IF_CART_VELOCITY   = "cartesian_velocity";

        /**
         * Interface type for elbow commands
         */
        constexpr const char* HW_IF_ELBOW           = "elbow_command";
    };

    /**
     * Namespace containing the names of the interfaces used to read and command the robot
     */
    namespace names {
        /**
         * Interface names used to export cartesian pose (expressed as position + quaternion)
         */
        constexpr const auto cartesian_pose_q_interface_names    = std::array{"x", "y", "z", "qw", "qx", "qy", "qz"};

        /**
         * Interface names used to export cartesian velocities
         */
        constexpr const auto cartesian_velocity_interfaces_names = std::array{"vx", "vy", "vz", "wx", "wy", "wz"};

        /**
         * Interface names used to export elbow commands
         */
        constexpr const auto elbow_interfaces_names              = std::array{"joint_3_position", "joint_4_sign"};
    }

};

#endif  // FRANKA_MM_INTERFACES_HPP