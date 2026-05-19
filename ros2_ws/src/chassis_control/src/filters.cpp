#include "chassis_control/filters.h"

double LowPassFilter::filter(double input)
{
    if (!initialized_) {
      output_ = input;
      initialized_ = true;
      return output_;
    }
    // 一阶低通滤波: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    output_ = alpha_ * input + (1.0 - alpha_) * output_;
    return output_;
}

void LowPassFilter::reset()
{
    initialized_ = false;
    output_ = 0.0;
}

double LowPassFilter::getOutput() const
{
    return output_; 
}

double RateLimiter::limit(double target, double dt)
{
    if (!initialized_) {
      last_output_ = target;
      initialized_ = true;
      return target;
    }
    
    double diff = target - last_output_;
    double max_change = max_rate_ * dt;
    
    if (diff > max_change) {
      last_output_ += max_change;
    } else if (diff < -max_change) {
      last_output_ -= max_change;
    } else {
      last_output_ = target;
    }
    
    return last_output_;
}

void RateLimiter::reset(void)
{
    initialized_ = false;
    last_output_ = 0.0;
}


