#include "rclcpp/rclcpp.hpp"
#include "latty_manual_control/manual_ctrl.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControl>());
    rclcpp::shutdown();
    return 0;
}