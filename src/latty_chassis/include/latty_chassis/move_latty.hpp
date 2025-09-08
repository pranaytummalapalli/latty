#pragma once

#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/qos.hpp"

class MoveLatty : public rclcpp::Node
{
public:
    MoveLatty();

private:
    void move_latty();
    void PoseStampedToEntityState(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);

    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
    double q_x_ = 0.0;
    double q_y_ = 0.0;
    double q_z_ = 0.0;
    double q_w_ = 0.0;

    rclcpp::QoS qos_;
    gazebo_msgs::msg::EntityState state_;
    std::shared_ptr<gazebo_msgs::srv::SetEntityState::Request> request_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};