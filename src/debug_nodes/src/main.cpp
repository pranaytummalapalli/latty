#include "rclcpp/rclcpp.hpp"
#include "debug_nodes/sine_wave.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SineWaveTrajectory>());
    rclcpp::shutdown();
    return 0;
}