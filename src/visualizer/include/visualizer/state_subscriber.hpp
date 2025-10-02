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

struct PoseRPY
{
    double timestamp;

    struct Position
    {
        double x;
        double y;
        double z;
    } position;

    struct Orientation
    {
        double roll;
        double pitch;
        double yaw;
    } orientation;
};

class StateSubscriber : public rclcpp::Node
{
public: 
    StateSubscriber() : Node("Visualizer"),
                        qos_(rclcpp::QoS(rclcpp::KeepLast(10))
                        .reliability(rclcpp::ReliabilityPolicy::Reliable)
                        .durability(rclcpp::DurabilityPolicy::Volatile)) 
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                            "/latty_odom", qos_,
                            std::bind(&StateSubscriber::read_odom, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                            "/imu_plugin/out", qos_,
                            std::bind(&StateSubscriber::read_imu, this, std::placeholders::_1));

        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
                            "/gazebo/model_states", qos_,
                            std::bind(&StateSubscriber::read_model_states, this, std::placeholders::_1));

        odom_pose_ = std::make_shared<PoseRPY>();
        imu_pose_  = std::make_shared<PoseRPY>();
        model_pose_ = std::make_shared<PoseRPY>();

    }

    // Odom getters

    PoseRPY get_odom() 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return *odom_pose_;
    }

    //imu getters

    PoseRPY get_imu() 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return *imu_pose_;
    }

    PoseRPY get_model_states()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return *model_pose_;
    }

private:

    void read_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        tf2::Quaternion q(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w
        );

        tf2::Matrix3x3 m(q);

        std::lock_guard<std::mutex> lock(mutex_);
        m.getRPY(
            odom_pose_->orientation.roll,
            odom_pose_->orientation.pitch,
            odom_pose_->orientation.yaw
        );

        odom_pose_->timestamp = odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec * 1e-9; 

    }

    void read_imu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        static tf2::Quaternion q_est(0, 0, 0, 1);  // estimated orientation (start at identity)
        static rclcpp::Time last_time = imu_msg->header.stamp; 

        // Time delta
        rclcpp::Time current_time = imu_msg->header.stamp;
        double dt = (current_time - last_time).seconds();
        last_time = current_time;

        // Extract angular velocity (rad/s)
        double wx = imu_msg->angular_velocity.x;
        double wy = imu_msg->angular_velocity.y;
        double wz = imu_msg->angular_velocity.z;

        // Small rotation quaternion (Δq ≈ [ω*dt/2, 1])
        tf2::Quaternion dq;
        dq.setX(0.5 * wx * dt);
        dq.setY(0.5 * wy * dt);
        dq.setZ(0.5 * wz * dt);
        dq.setW(1.0);

        // Normalize (good practice for quaternions)
        dq.normalize();

        // Update estimated orientation
        q_est = q_est * dq;
        q_est.normalize();

        // Convert to roll, pitch, yaw
        tf2::Matrix3x3 m(q_est);

        std::lock_guard<std::mutex> lock(mutex_);
        m.getRPY(
            imu_pose_->orientation.roll,
            imu_pose_->orientation.pitch,
            imu_pose_->orientation.yaw
        );

        imu_pose_->timestamp =
            imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
    }

    void read_model_states(const gazebo_msgs::msg::ModelStates::SharedPtr model_states)
    {   
        for (size_t i = 0; i < model_states->name.size(); i++) {
            if (model_states->name[i] == "latty_chassis") {
                
                const auto &pose = model_states->pose[i];

                tf2::Quaternion q(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                );

                tf2::Matrix3x3 m(q);

                std::lock_guard<std::mutex> lock(mutex_);
                m.getRPY(
                    model_pose_->orientation.roll,
                    model_pose_->orientation.pitch,
                    model_pose_->orientation.yaw
                );

                // RCLCPP_INFO(this->get_logger(), "Found chassis pose: %.9f", model_pose_->orientation.yaw);
            }
        }
    }
    

    rclcpp::QoS qos_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;

    std::shared_ptr<PoseRPY> odom_pose_;
    std::shared_ptr<PoseRPY> imu_pose_;
    std::shared_ptr<PoseRPY> model_pose_;

    std::mutex mutex_;
};

