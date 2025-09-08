#include "latty_manual_control/manual_ctrl.hpp"

#define TARGET_POSE_X 5.0
#define TARGET_POSE_Y 7.0
#define TARGET_POSE_Z 0.0
#define TARGET_ORIENTATION_X 0.0
#define TARGET_ORIENTATION_Y 0.0
#define TARGET_ORIENTATION_Z 0.0
#define TARGET_ORIENTATION_W 1.0

ManualControl::ManualControl()
    : Node("latty_manual_control"),
        qos_(rclcpp::QoS(rclcpp::KeepLast(10))
                .reliability(rclcpp::ReliabilityPolicy::Reliable)
                .durability(rclcpp::DurabilityPolicy::Volatile))
{
    
    
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("Latty/TargetPose", qos_);

    timer_ = this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(&ManualControl::set_pose, this)
            );
}

void ManualControl::set_pose()
{
    pose_.header.stamp = this->get_clock()->now();
    pose_.header.frame_id = "map";

    pose_.pose.position.x = TARGET_POSE_X;
    pose_.pose.position.y = TARGET_POSE_Y;
    pose_.pose.position.z = TARGET_POSE_Z;

    pose_.pose.orientation.x = TARGET_ORIENTATION_X;
    pose_.pose.orientation.y = TARGET_ORIENTATION_Y;
    pose_.pose.orientation.z = TARGET_ORIENTATION_Z;
    pose_.pose.orientation.w = TARGET_ORIENTATION_W;

    publisher_->publish(pose_);

    RCLCPP_INFO(this->get_logger(), "Published target pose: (%.2f, %.2f, %.2f)",
                pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
}