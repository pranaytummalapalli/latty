#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner();

private:
    void parse_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose);
    void parse_odom(const nav_msgs::msg::Odometry::SharedPtr odom);

    void update();

    rclcpp::QoS qos_;
    rclcpp::TimerBase::SharedPtr timer_;

    const float lookahead = 0.5;
    const float wheelbase = 0.25;
    double velocity_cmd = 0.2;

    double tp_x_;
    double tp_y_;
    double to_yaw_;

    double state_x_;
    double state_y_;
    double state_yaw_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std_msgs::msg::Float64MultiArray control_msg_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_pub_;
};