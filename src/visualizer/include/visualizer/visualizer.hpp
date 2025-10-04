#pragma once

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
#include "visualizer/state_subscriber.hpp"
#include <thread>
#include <chrono>
#include <atomic>

// For fork/exec/kill
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/wait.h>

using namespace rerun::demo;

class Visualizer
{
public:
    Visualizer(std::shared_ptr<StateSubscriber> state)
    : state_(state), running_(true), rec_("rerun_visualizer"), start_time_(-1.0)
    {
        rec_.spawn().exit_on_failure();
        
        vis_thread_ = std::thread(&Visualizer::rr_log, this); 
    }

    ~Visualizer()
    {
        running_ = false;
        if(vis_thread_.joinable()) vis_thread_.join();
    }

private:
    void rr_log()
    {
        while(rclcpp::ok() && running_)
        {
            auto odom = state_->get_odom();
            auto imu = state_->get_imu();
            auto imu_euler = state_->get_imu_euler();
            auto imu_expm = state_->get_imu_expm();
            auto model_states = state_->get_model_states();

            if(start_time_ < 0) start_time_ = odom->timestamp;
            double rel_time = odom->timestamp - start_time_;

            double deviation = model_states->pose.q_rpy.y() - imu_expm->pose.q_rpy.y();

            rec_.set_time_seconds("rel_time", rel_time);
            rec_.log("yaw/odom", rerun::Scalars(odom->pose.q_rpy.y()));
            rec_.log("yaw/imu_expm_raw", rerun::Scalars(imu->pose.q_rpy.y()));
            rec_.log("yaw/imu_lpf_zupt", rerun::Scalars(imu_expm->pose.q_rpy.y()));
            rec_.log("yaw/model", rerun::Scalars(model_states->pose.q_rpy.y()));
            rec_.log("yaw/deviation", rerun::Scalars(deviation));
        

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }


    }

    std::shared_ptr<StateSubscriber> state_;
    std::atomic<bool> running_;
    std::thread vis_thread_;
    rerun::RecordingStream rec_;
    double start_time_;
};