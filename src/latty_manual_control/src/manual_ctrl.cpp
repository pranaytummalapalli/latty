#include "latty_manual_control/manual_ctrl.hpp"

ManualControl::ManualControl(std::atomic<bool> &running)
    : Node("latty_manual_control"),
      qos_(rclcpp::QoS(rclcpp::KeepLast(10))
                .reliability(rclcpp::ReliabilityPolicy::Reliable)
                .durability(rclcpp::DurabilityPolicy::Volatile)),
      running_(running)
{
    
    
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("Latty/ControlTarget", qos_);

    controls_.data.resize(2);

    timer_ = this->create_wall_timer(
        20ms,  // 50 Hz publishing
        [this]() {
            std::lock_guard<std::mutex> lock(ui_mutex_);
            controls_.data[0] = wheelspeed;
            controls_.data[1] = steer_angle_rad;
            publisher_->publish(controls_);
        }
    );
    init();
    ui_thread_ = std::thread(&ManualControl::run_ui, this);

    RCLCPP_INFO(this->get_logger(), "ManualControl node started; UI thread launched.");
}

ManualControl::~ManualControl()
{
    stop_ui();

    RCLCPP_INFO(this->get_logger(), "ManualControl shutting down.");
}

void ManualControl::init()
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
}

void ManualControl::stop_ui()
{
    running_ = false;

    if (ui_thread_.joinable()) {
        ui_thread_.join();
    }
}

std::string ManualControl::fmt(double v)
{
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);
    ss << v;
    return ss.str();
}

void ManualControl::run_ui()
{
    while(running_)
    {
        int c = getch();
        if(c != ERR)
        {
            switch(c)
            {
                case KEY_UP:
                    wheelspeed += wheelspeed_increment;
                    break;
                case KEY_DOWN:
                    wheelspeed -= wheelspeed_increment;
                    break;
                case KEY_LEFT:
                    steer_angle_rad += steer_increment;
                    break;
                case KEY_RIGHT:
                    steer_angle_rad -= steer_increment;
                    break;
                case ' ':

                case 'q': case 'Q':
                    running_ = false;
                    break;
                default: break;
            }

            if (std::fabs(wheelspeed) < 1e-6) wheelspeed = 0.0;
            if (std::fabs(steer_angle_rad) < 1e-6) steer_angle_rad = 0.0;
        }

        clear();
        mvprintw(0, 2, "Simple Teleop (ncurses)");
        mvprintw(2, 2, "Use arrow keys: UP/DOWN=Speed, LEFT/RIGHT=Steer");
        mvprintw(3, 2, "SPACE=stop, q=quit");
        mvprintw(5, 2, "Wheelspeed:  %s", fmt(wheelspeed).c_str());
        mvprintw(6, 2, "Steer_angle_rad: %s", fmt(steer_angle_rad).c_str());
        refresh();

        // RCLCPP_INFO(this->get_logger(), "Published target pose: (%.2f, %.2f, %.2f)",
        //         pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);

        std::this_thread::sleep_for(50ms);
    }

    endwin();
    
}