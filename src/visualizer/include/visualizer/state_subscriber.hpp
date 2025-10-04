#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

#include <mutex>
#include <Eigen/Dense>

struct RPY
{
    Eigen::Vector3d data;
    double& r() {return data.x();}
    double& p() {return data.y();}
    double& y() {return data.z();}

    const double& r() const {return data.x();}
    const double& p() const {return data.y();}
    const double& y() const {return data.z();}

    void setZero() {data.setZero();}
};

struct StateRPY
{
    double timestamp;

    struct pose
    {
        Eigen::Vector3d position;
        RPY q_rpy; //orientation in rpy
    } pose;
    struct twist
    {
        Eigen::Vector3d linear;
        Eigen::Vector3d angular;
    } twist;
};


class StateSubscriber : public rclcpp::Node
{
public: 
    StateSubscriber();

    std::shared_ptr<const StateRPY> get_odom();
    std::shared_ptr<const StateRPY> get_imu();
    std::shared_ptr<const StateRPY> get_imu_euler();
    std::shared_ptr<const StateRPY> get_model_states();

private:

    void read_odom_(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void read_raw_imu_(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void read_imu_euler_(const nav_msgs::msg::Odometry::SharedPtr imu_msg);
    void read_model_states_(const gazebo_msgs::msg::ModelStates::SharedPtr model_states);


    inline double wrap_pi_(double a) {
        a = std::fmod(a + M_PI, 2*M_PI);
        if (a < 0) a += 2*M_PI;
        return a - M_PI;
    }

    inline double wrap_mpi_pi_(double a) { return wrap_pi_(a); } 
    

    rclcpp::QoS qos_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_euler_sub_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;

    std::shared_ptr<StateRPY> odom_pose_;
    std::shared_ptr<StateRPY> imu_pose_;
    std::shared_ptr<StateRPY> imu_euler_pose_;
    std::shared_ptr<StateRPY> model_pose_;

    tf2::Quaternion q_;

    rclcpp::Time last_time_;
    bool first_msg_;

    std::mutex mutex_;
};

