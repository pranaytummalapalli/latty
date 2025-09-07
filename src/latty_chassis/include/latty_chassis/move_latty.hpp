#pragma once

#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

class MoveLatty : public rclcpp::Node
{
public:
    MoveLatty();

private:
    void move_latty();

    double x_, y_;

    gazebo_msgs::msg::EntityState state_;
    std::shared_ptr<gazebo_msgs::srv::SetEntityState::Request> request_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};