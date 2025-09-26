#pragma once

#include <string>

const std::string front_steer_topic = 
        "/front_wheel_steer_position_controller/commands";
const std::string left_wheel_vel_topic = 
        "/left_wheel_velocity_controller/commands";
const std::string right_wheel_vel_topic = 
        "/right_wheel_velocity_controller/commands";
const std::string joint_states_topic = 
        "/joint_states";
const std::string joint_states_pos_topic = 
        "/joint_states_pos";
const std::string control_topic = 
        "/Latty/ControlTarget";