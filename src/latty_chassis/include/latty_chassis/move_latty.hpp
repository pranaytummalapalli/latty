#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/qos.hpp"
#include "topic_names.hpp"

class MoveLatty : public rclcpp::Node
{
public:
    MoveLatty();

private:
    void move_latty();
    void populate_control(const std_msgs::msg::Float64MultiArray::SharedPtr control_msg);
    
    double steer_angle_rad = 0.0;
    double left_wheel_vel_rps = 0.0;    
    double right_wheel_vel_rps = 0.0;

    rclcpp::QoS qos_;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Float64MultiArray steer_angle_rad_;
    std_msgs::msg::Float64MultiArray left_wheel_vel_rps_;
    std_msgs::msg::Float64MultiArray right_wheel_vel_rps_;
    sensor_msgs::msg::JointState joint_states_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr knuckle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr control_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

};