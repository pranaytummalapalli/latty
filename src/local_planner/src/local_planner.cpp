#include "local_planner/local_planner.hpp"

LocalPlanner::LocalPlanner()
    : Node("local_planner"),
        qos_(rclcpp::QoS(rclcpp::KeepLast(10))
                .reliability(rclcpp::ReliabilityPolicy::Reliable)
                .durability(rclcpp::DurabilityPolicy::Volatile))
{
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
                        ("/goal_pose", qos_,
                        std::bind(&LocalPlanner::parse_goal_pose, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>
                        ("/latty_odom", qos_,
                        std::bind(&LocalPlanner::parse_odom, this, std::placeholders::_1));

    control_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>
                        ("/Latty/ControlTarget", qos_);

    control_msg_.data.resize(2);
    timer_ = this->create_wall_timer(50ms, std::bind(&LocalPlanner::update, this));
    
}

void LocalPlanner::parse_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose)
{
    tp_x_ = goal_pose->pose.position.x;
    tp_y_ = goal_pose->pose.position.y;
    
    tf2::Quaternion q(
        goal_pose->pose.orientation.x,
        goal_pose->pose.orientation.y,
        goal_pose->pose.orientation.z,
        goal_pose->pose.orientation.w
    );
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    to_yaw_ = yaw;
}

void LocalPlanner::parse_odom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    state_x_ = odom->pose.pose.position.x;
    state_y_ = odom->pose.pose.position.y;

    tf2::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    state_yaw_ = yaw;
}

void LocalPlanner::update()
{
    double dx = tp_x_ - state_x_;
    double dy = tp_y_ - state_y_;

    double local_x = cos(state_yaw_)*dx + sin(state_yaw_)*dy;
    double local_y = -sin(state_yaw_)*dx + cos(state_yaw_)*dy;

    double ld = sqrt(local_x * local_x + local_y * local_y);
    if(ld < 0.1){
        velocity_cmd = 0;
    }
    else
    {
        velocity_cmd = 0.2;
    }

    double steering = atan2(2.0 * wheelbase * local_y, ld * ld);

    control_msg_.data[0] = velocity_cmd;
    control_msg_.data[1] = steering;

    control_pub_->publish(control_msg_);

}