#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

const double r = 0.1;  //wheel radius
const double L = 0.25; //wheelbase

class Odom : public rclcpp::Node
{
public:
    Odom() : Node("latty_odometry"), x_(0.0), y_(0.0), theta_(0.0)
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                        "/joint_states", 10,
                        std::bind(&Odom::calculate_odom, this, std::placeholders::_1));
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/latty_odom", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        last_time_ = this->get_clock()->now();
        
    }

    

private:
    void calculate_odom(const sensor_msgs::msg::JointState::SharedPtr joint_states)
    {
        double delta = 0.0;
        double wl = 0.0, wr = 0.0;

        for(size_t i = 0; i < joint_states->name.size(); i++)
        {
            if(joint_states->name[i] == "front_steer_joint")
            {
                delta = joint_states->position[i];
            }
            if(joint_states->name[i] == "left_wheel_joint")
            {
                wl = joint_states->velocity[i];
            }
            if(joint_states->name[i] == "right_wheel_joint")
            {
                wr = joint_states->velocity[i];
            }
        }

        double v = (r * (wr + wl) * 0.5);

        rclcpp::Time current_time(joint_states->header.stamp);
        
        double dt = (last_time_.nanoseconds() > 0)
                  ? (current_time - last_time_).seconds()
                  : 0.0;

        last_time_ = current_time;

        double theta_dot = (v / L) * tan(delta);
        x_ += v * cos(theta_) * dt;
        y_ += v * sin(theta_) * dt;
        theta_ += theta_dot * dt;
        
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = theta_dot;

        odom.pose.covariance[0] = 0.01;   // x
        odom.pose.covariance[7] = 0.01;   // y
        odom.pose.covariance[35] = 0.05;  // yaw

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = odom.pose.pose.orientation;

        tf_broadcaster_->sendTransform(odom_tf);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Time last_time_;
    double x_,y_, theta_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odom>());
    rclcpp::shutdown();
    return 0;
}