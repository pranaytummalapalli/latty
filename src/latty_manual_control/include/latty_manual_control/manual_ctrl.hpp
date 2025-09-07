#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/qos.hpp"

#define TARGET_POSE_X 5.0
#define TARGET_POSE_Y 7.0
#define TARGET_POSE_Z 0.0
#define TARGET_ORIENTATION_X 0.0
#define TARGET_ORIENTATION_Y 0.0
#define TARGET_ORIENTATION_Z 0.0
#define TARGET_ORIENTATION_W 1.0

class ManualControl : public rclcpp::Node
{
public:
    ManualControl();

private:
    void set_pose();

    rclcpp::QoS qos_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped pose_;
};