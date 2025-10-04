#include "visualizer/state_subscriber.hpp"

StateSubscriber::StateSubscriber() : Node("Visualizer"),
                        qos_(rclcpp::QoS(rclcpp::KeepLast(10))
                        .reliability(rclcpp::ReliabilityPolicy::Reliable)
                        .durability(rclcpp::DurabilityPolicy::Volatile)) 
{
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                        "/odom/wheel", qos_,
                        std::bind(&StateSubscriber::read_odom_, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                        "/imu_plugin/out", qos_,
                        std::bind(&StateSubscriber::read_raw_imu_, this, std::placeholders::_1));

    imu_euler_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                        "/odom/imu/euler", qos_,
                        std::bind(&StateSubscriber::read_imu_euler_, this, std::placeholders::_1));

    model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
                        "/gazebo/model_states", qos_,
                        std::bind(&StateSubscriber::read_model_states_, this, std::placeholders::_1));

    odom_pose_ = std::make_shared<StateRPY>();
    imu_pose_  = std::make_shared<StateRPY>();
    imu_euler_pose_ = std::make_shared<StateRPY>();
    model_pose_ = std::make_shared<StateRPY>();
    
    first_msg_ = true;

    odom_pose_->pose.q_rpy.setZero();
    odom_pose_->pose.position.setZero();
    odom_pose_->twist.linear.setZero();
    odom_pose_->twist.angular.setZero();

    imu_pose_->pose.q_rpy.setZero();
    imu_pose_->pose.position.setZero();
    imu_pose_->twist.linear.setZero();
    imu_pose_->twist.angular.setZero();

    imu_euler_pose_->pose.q_rpy.setZero();
    imu_euler_pose_->pose.position.setZero();
    imu_euler_pose_->twist.linear.setZero();
    imu_euler_pose_->twist.angular.setZero();

    model_pose_->pose.q_rpy.setZero();
    model_pose_->pose.position.setZero();
    model_pose_->twist.linear.setZero();
    model_pose_->twist.angular.setZero();
}

void StateSubscriber::read_odom_(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w
    );

    tf2::Matrix3x3 m(q);

    auto new_state = std::make_shared<StateRPY>();

    m.getRPY(
        new_state->pose.q_rpy.r(),
        new_state->pose.q_rpy.p(),
        new_state->pose.q_rpy.y()
    );

    new_state->pose.position.x() = odom_msg->pose.pose.position.x;
    new_state->pose.position.y() = odom_msg->pose.pose.position.y;
    new_state->pose.position.z() = odom_msg->pose.pose.position.z;

    new_state->twist.linear.x() = odom_msg->twist.twist.linear.x;
    new_state->twist.linear.y() = odom_msg->twist.twist.linear.y;
    new_state->twist.linear.z() = odom_msg->twist.twist.linear.z;

    new_state->twist.angular.x() = odom_msg->twist.twist.angular.x;
    new_state->twist.angular.y() = odom_msg->twist.twist.angular.y;
    new_state->twist.angular.z() = odom_msg->twist.twist.angular.z;

    new_state->timestamp = odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec * 1e-9; 
    
    {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_pose_ = new_state;
    }
}

void StateSubscriber::read_raw_imu_(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    rclcpp::Time now = imu_msg->header.stamp;
    if(first_msg_)
    {
        q_.setValue(0,0,0,1);
        last_time_ = now;
        first_msg_ = false;
        return;
    }

    double dt = (now - last_time_).seconds();
    last_time_ = now;

    double wx = imu_msg->angular_velocity.x;
    double wy = imu_msg->angular_velocity.y;
    double wz = imu_msg->angular_velocity.z;

    RCLCPP_INFO(this->get_logger(), "imu z: %.4f", wz);

    double norm_w = std::sqrt(wx*wx + wy*wy + wz*wz);

    tf2::Quaternion dq;
    if(norm_w > 1e-9)
    {
        double theta = norm_w * dt;
        double half_theta = theta / 2;
        double sin_ht = std::sin(half_theta);

        double ux = wx / norm_w;
        double uy = wy / norm_w;
        double uz = wz / norm_w;

        dq.setValue(ux * sin_ht, uy * sin_ht, uz * sin_ht, std::cos(half_theta));
    } 
    else
    {
        dq.setValue(0.5 * wx * dt, 0.5 * wy * dt, 0.5 * wz * dt, 1.0);
    }

    q_ = q_ * dq;
    q_.normalize();

    tf2::Matrix3x3 m(q_);

    auto new_state = std::make_shared<StateRPY>();

    m.getRPY(
        new_state->pose.q_rpy.r(),
        new_state->pose.q_rpy.p(),
        new_state->pose.q_rpy.y()
    );

    new_state->twist.angular.x() = imu_msg->angular_velocity.x;
    new_state->twist.angular.y() = imu_msg->angular_velocity.y;
    new_state->twist.angular.z() = imu_msg->angular_velocity.z;
    
    new_state->timestamp = imu_msg->header.stamp.sec
                    + imu_msg->header.stamp.nanosec * 1e-9;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_pose_ = new_state;
    }
    
}

void StateSubscriber::read_imu_euler_(const nav_msgs::msg::Odometry::SharedPtr imu_msg)
{
    tf2::Quaternion q(
        imu_msg->pose.pose.orientation.x,
        imu_msg->pose.pose.orientation.y,
        imu_msg->pose.pose.orientation.z,
        imu_msg->pose.pose.orientation.w
    );

    tf2::Matrix3x3 m(q);

    auto new_state = std::make_shared<StateRPY>();

    m.getRPY(
        new_state->pose.q_rpy.r(),
        new_state->pose.q_rpy.p(),
        new_state->pose.q_rpy.y()
    );

    new_state->pose.position.x() = imu_msg->pose.pose.position.x;
    new_state->pose.position.y() = imu_msg->pose.pose.position.y;
    new_state->pose.position.z() = imu_msg->pose.pose.position.z;

    new_state->twist.linear.x() = imu_msg->twist.twist.linear.x;
    new_state->twist.linear.y() = imu_msg->twist.twist.linear.y;
    new_state->twist.linear.z() = imu_msg->twist.twist.linear.z;

    new_state->twist.angular.x() = imu_msg->twist.twist.angular.x;
    new_state->twist.angular.y() = imu_msg->twist.twist.angular.y;
    new_state->twist.angular.z() = imu_msg->twist.twist.angular.z;

    new_state->timestamp = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9; 

    {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_euler_pose_ = new_state;
    }
}

void StateSubscriber::read_model_states_(const gazebo_msgs::msg::ModelStates::SharedPtr model_states)
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

            auto new_state = std::make_shared<StateRPY>();
            m.getRPY(
                new_state->pose.q_rpy.r(),
                new_state->pose.q_rpy.p(),
                new_state->pose.q_rpy.y()
            );
            {
                std::lock_guard<std::mutex> lock(mutex_);
                model_pose_ = new_state;
            }
        }
    }
}

std::shared_ptr<const StateRPY> StateSubscriber::get_odom() 
{
    std::lock_guard<std::mutex> lock(mutex_);
    return odom_pose_;
}

std::shared_ptr<const StateRPY> StateSubscriber::get_imu() 
{
    std::lock_guard<std::mutex> lock(mutex_);
    return imu_pose_;
}

std::shared_ptr<const StateRPY> StateSubscriber::get_imu_euler() 
{
    std::lock_guard<std::mutex> lock(mutex_);
    return imu_euler_pose_;
}

std::shared_ptr<const StateRPY> StateSubscriber::get_model_states()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return model_pose_;
}

