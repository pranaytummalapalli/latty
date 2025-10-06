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

    delta_.data.resize(1);
    theta_dot_l_.data.resize(1);
    theta_dot_r_.data.resize(1);

    v_com = 0.0;

    knuckle_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>
                                (front_steer_topic, qos_);

    left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>
                                (left_wheel_vel_topic, qos_);

    right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>
                                (right_wheel_vel_topic, qos_);

    control_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>
                            (control_topic, qos_,
                            std::bind(&MoveLatty::populate_control, this, std::placeholders::_1));

    // Timer to move the robot every 100ms
    timer_ = this->create_wall_timer(50ms, std::bind(&MoveLatty::move_latty, this));
}

void MoveLatty::populate_control(const std_msgs::msg::Float64MultiArray::SharedPtr control_msg)
{   
    if(control_msg->data.size() >= 2)
    {   
        v_com = control_msg->data[0];
        delta = control_msg->data[1];
        if(v_com > 0.3) v_com = 0.3;
        if(v_com < -0.3) v_com = -0.3;

        double vl, vr;

        if (std::abs(std::tan(delta)) < 1e-6) {
            vl = v_com;
            vr = v_com;
        } else {
            double icr   = wheelbase / std::tan(delta);           
            double yaw_rate = (v_com * std::tan(delta)) / wheelbase;   
            vl = yaw_rate * (icr + track_width / 2.0);
            vr = yaw_rate * (icr - track_width / 2.0);
        }

        theta_dot_l = vl / wheelradius;
        theta_dot_r = vr / wheelradius;
    }
}

void MoveLatty::move_latty()
{
    delta_.data[0] = delta;
    theta_dot_l_.data[0] = theta_dot_l;
    theta_dot_r_.data[0] = theta_dot_r;

    knuckle_pub_->publish(delta_);
    left_wheel_pub_->publish(theta_dot_l_);
    right_wheel_pub_->publish(theta_dot_r_);
}
