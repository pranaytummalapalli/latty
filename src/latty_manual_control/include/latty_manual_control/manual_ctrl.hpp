#pragma once

#include <string>
#include <ncurses.h>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <cmath>
#include <sstream>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;

class ManualControl : public rclcpp::Node
{
public:
    explicit ManualControl(std::atomic<bool> &running);
    ~ManualControl();

private:
    void init();

    std::string fmt(double v);

    void run_ui();
    void stop_ui();
    
    double steer_angle_rad = 0.0;
    double wheelspeed = 0.0;    
    
    double steer_increment = 0.1;
    double wheelspeed_increment = 0.5;

    std::atomic<bool> &running_;

    std::thread ui_thread_;
    std::mutex ui_mutex_;

    rclcpp::QoS qos_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Float64MultiArray controls_;

    ManualControl(const ManualControl &) = delete;
    ManualControl & operator=(const ManualControl &) = delete;
};