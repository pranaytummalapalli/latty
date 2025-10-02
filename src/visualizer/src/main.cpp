#include "rclcpp/rclcpp.hpp"
#include "visualizer/state_subscriber.hpp"
#include "visualizer/visualizer.hpp"

using namespace rerun::demo;

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);

    auto state_sub = std::make_shared<StateSubscriber>();
    auto visualizer = std::make_shared<Visualizer>(state_sub);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(state_sub);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}