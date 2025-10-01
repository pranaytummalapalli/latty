#include "latty_chassis/move_latty.hpp"

#define MOVE_WHEELS_EQUAL 1

using namespace std::chrono_literals;

MoveLatty::MoveLatty() 
    : Node("move_latty"),
        qos_(rclcpp::QoS(rclcpp::KeepLast(10))
                .reliability(rclcpp::ReliabilityPolicy::Reliable)
                .durability(rclcpp::DurabilityPolicy::Volatile))
{
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    steer_angle_rad_.data.resize(1);
    left_wheel_vel_rps_.data.resize(1);
    right_wheel_vel_rps_.data.resize(1);

    knuckle_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>
                                (front_steer_topic, qos_);

    if(knuckle_pub_ == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create %s publisher!", front_steer_topic.c_str());
        return;
    }

    left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>
                                (left_wheel_vel_topic, qos_);

    if(left_wheel_pub_ == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create %s publisher!", left_wheel_vel_topic.c_str());
    }

    right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>
                                (right_wheel_vel_topic, qos_);

    if(right_wheel_pub_ == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create %s publisher!", right_wheel_vel_topic.c_str());
    }

    control_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>
                            (control_topic, qos_,
                            std::bind(&MoveLatty::populate_control, this, std::placeholders::_1));
    
    if(control_sub_ == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create control subscriber!");
    }

    // Timer to move the robot every 100ms
    timer_ = this->create_wall_timer(50ms, std::bind(&MoveLatty::move_latty, this));
}

void MoveLatty::populate_control(const std_msgs::msg::Float64MultiArray::SharedPtr control_msg)
{   
    if(control_msg->data.size() >= 2)
    {   
        
        double latty_velocity = control_msg->data[0];
        if(latty_velocity > 0.3) latty_velocity = 0.3;
        if(latty_velocity < -0.3) latty_velocity = -0.3;

        steer_angle_rad = control_msg->data[1];
        right_wheel_vel_rps = latty_velocity / wheelradius;
        left_wheel_vel_rps = latty_velocity / wheelradius;
    }
}

void MoveLatty::move_latty()
{
    steer_angle_rad_.data[0] = steer_angle_rad;
    left_wheel_vel_rps_.data[0] = left_wheel_vel_rps;
    right_wheel_vel_rps_.data[0] = right_wheel_vel_rps;

    knuckle_pub_->publish(steer_angle_rad_);
    left_wheel_pub_->publish(left_wheel_vel_rps_);
    right_wheel_pub_->publish(right_wheel_vel_rps_);
}
