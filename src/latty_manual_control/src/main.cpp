#include <atomic>
#include <csignal>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "latty_manual_control/manual_ctrl.hpp"

static std::atomic<bool> g_running{true};

void sigint_handler(int)
{
    g_running = false;
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, sigint_handler);

    auto node = std::make_shared<ManualControl>(g_running);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}