#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <utility>

struct IMUState
{
    struct pose
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    } pose;
    struct twist
    {
        Eigen::Vector3d linear;
        Eigen::Vector3d angular;
    } twist; 
};

class IMUPublisher : public rclcpp::Node
{
public:
    IMUPublisher();

private:
    void integrate_sensor_(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    IMUState integrate_euler_(const IMUState& state,
                              const Eigen::Vector3d& w_b,
                              const Eigen::Vector3d& a_b,
                              double dt);

    std::pair<Eigen::Vector3d, Eigen::Vector3d> lowpass_filer_( 
                            const Eigen::Vector3d& w_b,
                            const Eigen::Vector3d& a_b,
                            double dt);

    // IMUState lowpass_filter(const IMUState& state);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    nav_msgs::msg::Odometry odom_euler_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_euler_pub_;   

    bool first_msg_;
    rclcpp::Time last_time_;

    IMUState state_;
    Eigen::Vector3d last_filtered_w_;
    Eigen::Vector3d last_filtered_a_;
};


