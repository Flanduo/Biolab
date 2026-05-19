#ifndef __FILTERS_H__
#define __FILTERS_H__

#include <iostream>

#define FILTER_ALPHA 0.75f        // 低通滤波系数 (0-1, 越小越平滑但响应越慢)
#define MAX_ANGLE_RATE 5.0f       // 最大角度变化率 (rad/s)
#define MAX_WHEEL_SPEED 100.0f    // 最大轮子速度 (rpm)，根据实际电机能力调整

class LowPassFilter {
public:
  LowPassFilter(double alpha = FILTER_ALPHA) : alpha_(alpha), output_(0.0), initialized_(false) {}
  
  double filter(double input);
  
  void reset();
  
  double getOutput() const;
  
private:
  double alpha_;
  double output_;
  bool initialized_;
};

// 角度变化率限制器
class RateLimiter {
public:
  RateLimiter(double max_rate = MAX_ANGLE_RATE) : max_rate_(max_rate), last_output_(0.0), initialized_(false) {}
  
  double limit(double target, double dt);
  void reset(void);
  
private:
  double max_rate_;
  double last_output_;
  bool initialized_;
};

#endif