#include "latty_chassis/move_latty.hpp"

using namespace std::chrono_literals;

MoveLatty::MoveLatty() 
    : Node("move_latty"),
        qos_(rclcpp::QoS(rclcpp::KeepLast(10))
                .reliability(rclcpp::ReliabilityPolicy::Reliable)
                .durability(rclcpp::DurabilityPolicy::Volatile))
{
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>
                    ("Latty/TargetPose",qos_,
                    std::bind(&MoveLatty::PoseStampedToEntityState, this, std::placeholders::_1));


    // Create service client
    client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");

    // Wait until the service is available
    while (!client_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /gazebo/set_entity_state...");
    }

    // Timer to move the robot every 100ms
    timer_ = this->create_wall_timer(10ms, std::bind(&MoveLatty::move_latty, this));
}

void MoveLatty::PoseStampedToEntityState(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{
    x_ = pose_msg->pose.position.x;
    y_ = pose_msg->pose.position.y;
    z_ = pose_msg->pose.position.z;

    q_x_ = pose_msg->pose.orientation.x;
    q_y_ = pose_msg->pose.orientation.y;
    q_z_ = pose_msg->pose.orientation.z;
    q_w_ = pose_msg->pose.orientation.w;

    RCLCPP_INFO(
        this->get_logger(),
        "Received PoseStamped -> (x = %.2f, y = %.2f, z = %.2f, qx = %.2f, qy = %.2f, qz = %.2f, qw = %.2f)",
        x_, y_, z_, q_x_, q_y_, q_z_, q_w_);

}

void MoveLatty::move_latty()
{
    // Create request
    request_ = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

    state_.name = "warehouse_robot";   // must match the model name in Gazebo
    state_.pose.position.x = x_;
    state_.pose.position.y = y_;
    state_.pose.position.z = z_;
    state_.pose.orientation.x = q_x_;
    state_.pose.orientation.y = q_y_;
    state_.pose.orientation.z = q_z_;
    state_.pose.orientation.w = q_w_;

    request_->state = state_;

    // Call service asynchronously
    auto future = client_->async_send_request(request_,
        [this](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture future_response) {
            try {
                auto result = future_response.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Moved robot to (%.2f, %.2f)", x_, y_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to move robot");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        });
}
