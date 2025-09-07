#include "latty_chassis/move_latty.hpp"

using namespace std::chrono_literals;

MoveLatty::MoveLatty() 
    : Node("move_latty"), x_(0.0), y_(0.0)
{
    // Create service client
    client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");

    // Wait until the service is available
    while (!client_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for /gazebo/set_entity_state...");
    }

    // Timer to move the robot every 100ms
    timer_ = this->create_wall_timer(100ms, std::bind(&MoveLatty::move_latty, this));
}

void MoveLatty::move_latty()
{
    // Create request
    request_ = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

    state_.name = "warehouse_robot";   // must match the model name in Gazebo
    state_.pose.position.x = x_;
    state_.pose.position.y = y_;
    state_.pose.position.z = 0.0;
    state_.pose.orientation.x = 0.0;
    state_.pose.orientation.y = 0.0;
    state_.pose.orientation.z = 0.0;
    state_.pose.orientation.w = 1.0;

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

    // Increment x and y for demo
    x_ += 0.0;
    y_ += 0.05;
}
