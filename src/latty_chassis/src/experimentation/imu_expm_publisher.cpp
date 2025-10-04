#include "latty_chassis/experimentation/imu_publisher_em.hpp"
#include <Eigen/Geometry>
#include <cmath>

using namespace std::chrono_literals;

IMUPublisher::IMUPublisher() : Node("Sensors_IMU_ExpM"), first_msg_(false), last_time_(-1)
{
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Declare cutoff frequency parameter (default 5 Hz)
    this->declare_parameter<double>("cutoff_frequency", 5.0);
    this->declare_parameter<int>("calib_samples", 5000);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_plugin/out", 10,
        std::bind(&IMUPublisher::integrate_sensor_, this, std::placeholders::_1));

    odom_euler_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/imu/expm", 10);

    first_msg_ = true;

    state_.pose.orientation = Eigen::Quaterniond::Identity();
    state_.pose.position.setZero();
    state_.twist.linear.setZero();
    state_.twist.angular.setZero();

    last_filtered_w_.setZero();
    last_filtered_a_.setZero();
}

void IMUPublisher::integrate_sensor_(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    Eigen::Vector3d w_raw(
        imu_msg->angular_velocity.x,
        imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z);

    Eigen::Vector3d a(
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z);

    rclcpp::Time now = imu_msg->header.stamp;
    if (first_msg_) {
        last_time_ = now;
        first_msg_ = false;
        return;
    }

    double dt = (now - last_time_).seconds();
    last_time_ = now;

    int calib_samples = this->get_parameter("calib_samples").as_int();

    // Drift Bias Calibration
    if (!bias_calib_done_) {
        gyro_acc_ += w_raw;
        if (++calib_count_ >= calib_samples) {
            gyro_bias_ = gyro_acc_ / double(calib_count_);
            bias_calib_done_ = true;
        }
    }

    // Correct drift bias
    Eigen::Vector3d w = w_raw - gyro_bias_;

    // -------- Optional Online Bias Update (stationary detection) --------
    double gyro_thr = 0.02;  // rad/s
    double accel_thr = 0.15; // m/s²
    bool stationary = (w_raw.norm() < gyro_thr) && (fabs(a.norm() - 9.81) < accel_thr);
    if (stationary && bias_calib_done_) {
        double beta = 0.001;  // slow learning rate
        gyro_bias_ = (1.0 - beta) * gyro_bias_ + beta * w_raw;
    }

    // Apply low-pass filter
    auto filtered = lowpass_filter_(w, a, dt);
    w = filtered.first;
    a = filtered.second;

    // Integrate orientation + position
    state_ = integrate_exp_map_(state_, w, a, dt);

    // Populate odometry msg
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


IMUState IMUPublisher::integrate_exp_map_(const IMUState& state,
                                          const Eigen::Vector3d& w_b,
                                          const Eigen::Vector3d& a_b,
                                          double dt)
{
    IMUState next_state = state;

    Eigen::Quaterniond q = state.pose.orientation;
    Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();

    // --- Quaternion exponential update ---
    const double norm_w = w_b.norm();
    if (norm_w > 1e-9)
    {
        const double theta = norm_w * dt;
        const Eigen::Vector3d axis = w_b / norm_w;
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));

        q = q * dq;
    }
    else
    {
        // Small-angle approximation
        dq = Eigen::Quaterniond(1.0,
                                0.5 * w_b.x() * dt,
                                0.5 * w_b.y() * dt,
                                0.5 * w_b.z() * dt);
        q = q * dq;
    }
    q.normalize();

    next_state.pose.orientation = q;

    const Eigen::Matrix3d R_w_b = q.toRotationMatrix();
    const Eigen::Vector3d g(0, 0, 9.81);

    next_state.twist.linear  = state.twist.linear + (R_w_b * a_b - g) * dt;
    next_state.pose.position = state.pose.position + next_state.twist.linear * dt;
    next_state.twist.angular = w_b;

    return next_state;
}


std::pair<Eigen::Vector3d, Eigen::Vector3d> IMUPublisher::lowpass_filter_(
    const Eigen::Vector3d& w_b,
    const Eigen::Vector3d& a_b,
    double dt)
{
    if (dt <= 1e-6) {
        return {last_filtered_w_, last_filtered_a_};
    }

    // Get cutoff frequency from ROS param
    double fc = this->get_parameter("cutoff_frequency").as_double();

    double RC = 1.0 / (2.0 * M_PI * fc);
    double alpha = dt / (RC + dt);

    Eigen::Vector3d filtered_w = alpha * w_b + (1 - alpha) * last_filtered_w_;
    Eigen::Vector3d filtered_a = alpha * a_b + (1 - alpha) * last_filtered_a_;

    last_filtered_w_ = filtered_w;
    last_filtered_a_ = filtered_a;

    return {filtered_w, filtered_a};
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}
