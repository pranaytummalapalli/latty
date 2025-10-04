#include "latty_chassis/imu_publisher.hpp"

using namespace std::chrono_literals;

IMUPublisher::IMUPublisher() : Node("Sensors_IMU_Euler"), first_msg_(false), last_time_(-1)
{
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                        "/imu_plugin/out", 10,
                        std::bind(&IMUPublisher::integrate_sensor_, this, std::placeholders::_1));

    odom_euler_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/imu/euler", 10);
    
    first_msg_ = true;

    state_.pose.orientation = Eigen::Quaterniond::Identity();
    state_.pose.position.setZero();
    state_.twist.linear.setZero();
    state_.twist.angular.setZero();
}

void IMUPublisher::integrate_sensor_(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    Eigen::Vector3d w(
            imu_msg->angular_velocity.x,
            imu_msg->angular_velocity.y,
            imu_msg->angular_velocity.z);

    Eigen::Vector3d a(
            imu_msg->linear_acceleration.x,
            imu_msg->linear_acceleration.y,
            imu_msg->linear_acceleration.z);

    rclcpp::Time now = imu_msg->header.stamp;
    if(first_msg_)
    {
        last_time_ = now;
        first_msg_ = false;
        return;
    }

    double dt = (now - last_time_).seconds();

    state_ = integrate_euler_(state_, w, a, dt);

    odom_euler_.header = imu_msg->header;

    odom_euler_.pose.pose.position.x = state_.pose.position.x();
    odom_euler_.pose.pose.position.y = state_.pose.position.y();
    odom_euler_.pose.pose.position.z = state_.pose.position.z();

    odom_euler_.pose.pose.orientation.x = state_.pose.orientation.x();
    odom_euler_.pose.pose.orientation.y = state_.pose.orientation.y();
    odom_euler_.pose.pose.orientation.z = state_.pose.orientation.z();
    odom_euler_.pose.pose.orientation.w = state_.pose.orientation.w();

    odom_euler_.twist.twist.linear.x = state_.twist.linear.x();
    odom_euler_.twist.twist.linear.y = state_.twist.linear.y();
    odom_euler_.twist.twist.linear.z = state_.twist.linear.z();

    odom_euler_.twist.twist.angular.x = state_.twist.angular.x();
    odom_euler_.twist.twist.angular.y = state_.twist.angular.y();
    odom_euler_.twist.twist.angular.z = state_.twist.angular.z();

    odom_euler_pub_->publish(odom_euler_);

}

IMUState IMUPublisher::integrate_euler_(const IMUState& state,
                              const Eigen::Vector3d& w_b,
                              const Eigen::Vector3d& a_b,
                              double dt)
{
    IMUState next_state = state;

    Eigen::Quaterniond q = state.pose.orientation;
    Eigen::Quaterniond q_w (0, w_b.x(), w_b.y(), w_b.z());

    Eigen::Quaterniond dq = q * q_w;

    dq.coeffs() *= 0.5;
    q.coeffs() += dq.coeffs() * dt;
    q.normalize();

    next_state.pose.orientation = q;

    Eigen::Matrix3d R_w_b = next_state.pose.orientation.toRotationMatrix();

    Eigen::Vector3d g(0, 0, 9.81);
    
    next_state.twist.linear = state.twist.linear + (R_w_b * a_b - g) * dt;
    next_state.pose.position = state.pose.position + next_state.twist.linear * dt;
    next_state.twist.angular = state.twist.angular;

    return next_state;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}