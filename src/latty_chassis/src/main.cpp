#include "latty_chassis/move_latty.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveLatty>());
    rclcpp::shutdown();
    return 0;
}
