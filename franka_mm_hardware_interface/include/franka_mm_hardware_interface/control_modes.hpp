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
#ifndef FRANKA_MM_CONTROL_MODES_HPP
#define FRANKA_MM_CONTROL_MODES_HPP

#include <string>
#include <stdexcept>

/**
 * Enumeration class used to describe controller's possible states.
 * 
 * @author Alessandro Moscatelli
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
 * Namespace containing useful functions to parse ControlMode objects. 
 * 
 * @author Alessandro Moscatelli
 */
namespace control_mode_utils {
    /**
     * Converts a ControlMode into a string.
     *
     * @param mode Mode to be converted
     * @return String representing the control mode
     */
    inline std::string to_string(const ControlMode& mode) {
        switch(mode) { 
            case ControlMode::INACTIVE:             return "inactive";
            case ControlMode::POSITION:             return "position";
            case ControlMode::VELOCITY:             return "velocity";
            case ControlMode::EFFORT:               return "effort";
            case ControlMode::CARTESIAN_POSITION:   return "cartesian position";
            case ControlMode::CARTESIAN_VELOCITY:   return "cartesian velocity";
            case ControlMode::CARTESIAN_IMPEDANCE:  return "cartesian impedance";
            default:                                return "???";
        }
    }

    /**
     * Converts a string into a ControlMode.
     *
     * @param mode_name The string representing the control mode
     * 
     * @return The corresponding ControlMode enum value
     * 
     * @throws std::invalid_argument if the string does not match any known mode
     */
    inline ControlMode from_string(const std::string& mode_name) {
        if (mode_name == "inactive")             return ControlMode::INACTIVE;
        if (mode_name == "position")             return ControlMode::POSITION;
        if (mode_name == "velocity")             return ControlMode::VELOCITY;
        if (mode_name == "effort")               return ControlMode::EFFORT;
        if (mode_name == "cartesian position")   return ControlMode::CARTESIAN_POSITION;
        if (mode_name == "cartesian velocity")   return ControlMode::CARTESIAN_VELOCITY;
        if (mode_name == "cartesian impedance")  return ControlMode::CARTESIAN_IMPEDANCE;

        throw std::invalid_argument("Unknown control mode: " + mode_name);
    }

    /**
     * Returns a boolean that tells if the mode is 
     * cartesian pose, velocity or impedance.
     *
     * @param mode Mode to be checked
     * 
     * @returns True if the mode is cartesian, False otherwise. 
     */
    inline bool is_cartesian(const ControlMode& mode) {
        return mode == ControlMode::CARTESIAN_POSITION ||
               mode == ControlMode::CARTESIAN_VELOCITY ||
               mode == ControlMode::CARTESIAN_IMPEDANCE;
    }
}

#endif  // FRANKA_MM_CONTROL_MODES_HPP