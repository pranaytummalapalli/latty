#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class MoveRobot : public rclcpp::Node
{
public:
    MoveRobot() : Node("move_warehouse_robot")
    {
        client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");

        // Wait for service
        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /gazebo/set_entity_state...");
        }

        // Timer to move the robot every 2 seconds
        timer_ = this->create_wall_timer(0.1s, std::bind(&MoveRobot::move_robot, this));
    }

private:
    void move_robot()
    {
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        gazebo_msgs::msg::EntityState state;

        state.name = "warehouse_robot";   // must match <model name="warehouse_robot">
        state.pose.position.x = x_;
        state.pose.position.y = y_;
        state.pose.position.z = 0.0;
        state.pose.orientation.w = 1.0;  // no rotation

        request->state = state;

        auto future = client_->async_send_request(request,
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
        x_ += 0;
        y_ += 0.05;
    }

    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_ = 0.0, y_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveRobot>());
    rclcpp::shutdown();
    return 0;
}
