#pragma once 

#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class SineWaveTrajectory : public rclcpp::Node
{
public:
    SineWaveTrajectory() : Node("steering_sine_publisher"),
                            qos_(rclcpp::QoS(rclcpp::KeepLast(10))
                            .reliability(rclcpp::ReliabilityPolicy::Reliable)
                            .durability(rclcpp::DurabilityPolicy::Volatile)),
                              start_time_(this->get_clock()->now())
    {
        sine_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/Latty/ControlTarget", 10);
        
        sine_control_.data.resize(2);

        timer_ = this->create_wall_timer(
            20ms, std::bind(&SineWaveTrajectory::generate_sine, this));

    }

private:

    void generate_sine()
    {
        auto now = this->get_clock()->now();
        double t = (now - start_time_).seconds();

        double amplitude = 0.785;
        double f = 0.3;
        double offset = 0.05;

        double delta = amplitude * std::sin(2.0 * M_PI * f * t) - offset;

        sine_control_.data[0] = velocity;
        sine_control_.data[1] = delta;

        RCLCPP_INFO(this->get_logger(), "Steering = %.3f rad", delta);
        sine_pub_->publish(sine_control_);
    }

    double velocity = 0.2;

    rclcpp::QoS qos_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sine_pub_;
    std_msgs::msg::Float64MultiArray sine_control_;
};