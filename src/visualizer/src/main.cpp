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
    
    try {
        exec.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(state_sub->get_logger(), "Exception: %s", e.what());
    }

    exec.cancel();
    rclcpp::shutdown();
    return 0;
}