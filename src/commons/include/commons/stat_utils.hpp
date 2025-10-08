#pragma once 

#include <cmath>
#include <algorithm>
#include <deque>

namespace stats_util {

namespace rmse {

struct RMSEOfflineBatch 
{
    double expected = 0.0;
    double instant_error = 0.0;
    double squared_error = 0.0;
    double sum_squared_error = 0.0;
    int num_samples = 0;
    bool is_error_inverted = false;

    void update(double measured, double expected)
    {   
        instant_error = is_error_inverted ? (expected - measured)
                            : (measured - expected); 
        squared_error = instant_error * instant_error;

        sum_squared_error += squared_error;
        num_samples++;
    }

    double value() const
    {
        if(num_samples == 0) return 0.0;
        return std::sqrt(sum_squared_error / num_samples);
    }

    void reset()
    {
        instant_error = 0.0;
        squared_error = 0.0;
        expected = 0.0;
        sum_squared_error = 0.0;
        num_samples = 0;
    }
};

struct RMSEOnlineNSamples 
{
    size_t window_size_;
    double squared_error;
    bool is_error_inverted_ = false;

    std::deque<double> sample_buffer;
    double sum_squared_error = 0.0;

    explicit RMSEOnlineNSamples(size_t window_size, bool is_error_inverted)
        : window_size_(window_size), is_error_inverted_(is_error_inverted) {}

    void update(double measured, double expected)
    {   
        double error = is_error_inverted_ ? (expected - measured)
                            : (measured - expected);
        squared_error = error * error;

        sample_buffer.push_back(squared_error);
        sum_squared_error += squared_error;

        while(sample_buffer.size() > window_size_)
        {
            sum_squared_error -= sample_buffer.front();
            sample_buffer.pop_front();
        }
    }

    double value() const 
    {
        if(sample_buffer.empty()) return 0.0;
        return std::sqrt(sum_squared_error / static_cast<double>(sample_buffer.size()));
    }

    void reset()
    {
        squared_error = 0.0;
        sample_buffer.clear();
    }
};

struct RMSEOnlineTime 
{
    double time_window_;
    bool is_error_inverted_ = false;

    struct data 
    {
        double timestamp;
        double squared_error;
    };
    
    std::deque<data> sample_buffer;

    double sum_squared_error = 0.0;

    explicit RMSEOnlineTime(double time_window, bool is_error_inverted) 
        : time_window_(time_window), is_error_inverted_(is_error_inverted) {}

    void update(double measured, double expected, double timestamp)
    {
        double error = is_error_inverted_ ? (expected - measured)
                            : (measured - expected);
        double squared_error_ = error * error;

        sample_buffer.push_back({timestamp, squared_error_});
        sum_squared_error += squared_error_;

        double oldest_sample = timestamp - time_window_;
        while(!sample_buffer.empty() && sample_buffer.front().timestamp < oldest_sample)
        {
            sum_squared_error -= sample_buffer.front().squared_error;
            sample_buffer.pop_front();
        }
    }

    double value() const 
    {
        if(sample_buffer.empty()) return 0.0;
        return std::sqrt(sum_squared_error / static_cast<double>(sample_buffer.size()));
    }

    void reset()
    {
        sum_squared_error = 0.0;
        sample_buffer.clear();
    }
};

}; //stats_utils::rmse

}; //stats_util