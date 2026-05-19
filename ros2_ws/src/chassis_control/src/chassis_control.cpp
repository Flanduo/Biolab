#include "chassis_control/chassis_control.h"

chassis_control::chassis_control(void):rclcpp::Node("chassis_control_node"),
                                        motor1(RobStrideMotor(STEER_MOTOR_CAN, 0xFF, 0x65, 0)),
                                        motor2(RobStrideMotor(STEER_MOTOR_CAN, 0xFF, 0x66, 0)), 
                                        motor3(RobStrideMotor(STEER_MOTOR_CAN, 0xFF, 0x67, 0)),
                                        motor4(RobStrideMotor(STEER_MOTOR_CAN, 0xFF, 0x68, 0)),
                                        fl_angle_filter(FILTER_ALPHA),
                                        fr_angle_filter(FILTER_ALPHA),
                                        rl_angle_filter(FILTER_ALPHA),
                                        rr_angle_filter(FILTER_ALPHA),
                                        fl_rate_limiter(MAX_ANGLE_RATE),
                                        fr_rate_limiter(MAX_ANGLE_RATE),
                                        rl_rate_limiter(MAX_ANGLE_RATE),
                                        rr_rate_limiter(MAX_ANGLE_RATE){
    this->declare_parameter<double>("robot.chassis_radius");
    this->declare_parameter<double>("robot.wheel_perimeter");

    this->declare_parameter<double>("robot.fl_motor_start_angle");
    this->declare_parameter<double>("robot.fr_motor_start_angle");
    this->declare_parameter<double>("robot.rl_motor_start_angle");
    this->declare_parameter<double>("robot.rr_motor_start_angle");

    this->get_parameter("robot.chassis_radius", chassis_param.chassis_radius);
    this->get_parameter("robot.wheel_perimeter",chassis_param.wheel_perimeter);

    this->get_parameter("robot.fl_motor_start_angle", chassis_param.fl_motor_start_angle);
    this->get_parameter("robot.fr_motor_start_angle", chassis_param.fr_motor_start_angle);
    this->get_parameter("robot.rl_motor_start_angle", chassis_param.rl_motor_start_angle);
    this->get_parameter("robot.rr_motor_start_angle", chassis_param.rr_motor_start_angle);

    svtrobot_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>("/svtrobot_cmd", 10, 
                       std::bind(&chassis_control::svtrobot_cmd_callback, this, std::placeholders::_1));

    motor1.Get_RobStrite_Motor_parameter(0x7005);
    usleep(100);
    motor2.Get_RobStrite_Motor_parameter(0x7005);
    usleep(100);
    motor3.Get_RobStrite_Motor_parameter(0x7005);
    usleep(100);
    motor4.Get_RobStrite_Motor_parameter(0x7005);
    usleep(100);

    motor1.enable_motor();
    usleep(100);
    motor2.enable_motor();
    usleep(100);
    motor3.enable_motor();
    usleep(100);
    motor4.enable_motor();
    usleep(100);

    // RCLCPP_INFO(this->get_logger(), "Creating front and rear ZLAC8015D on interface '%s'...", WHEEL_MOTOR_CAN);
    try 
    {
      front_ = std::make_unique<ZLAC8015D>(WHEEL_MOTOR_CAN, 1, 0.3);
      rear_  = std::make_unique<ZLAC8015D>(WHEEL_MOTOR_CAN, 2, 0.3);

      for (auto *drv : std::vector<ZLAC8015D*>{front_.get(), rear_.get()}) 
      {
        int hb = drv->wait_heartbeat(1.0);
        (void)hb;
        try 
        { 
          drv->clear_fault(); 
          } 
        catch (...) 
        {

        }
        drv->set_velocity_mode();
        drv->enable_operation();
      }
    } 
    catch (const std::exception &e) 
    {
      RCLCPP_ERROR(this->get_logger(), "Error during initial device setup: %s", e.what());
      throw;
    }
    signal(SIGINT, [](int sig) 
    {
      (void)sig;
      rclcpp::shutdown();
    });
    RCLCPP_INFO(this->get_logger(), "chassis is initing");
    chassis_control_para.front_left_angle = chassis_param.fl_motor_start_angle;
    chassis_control_para.front_right_angle = chassis_param.fr_motor_start_angle;
    chassis_control_para.rear_left_angle = chassis_param.rl_motor_start_angle;
    chassis_control_para.rear_right_angle = chassis_param.rr_motor_start_angle;
    RCLCPP_INFO(this->get_logger(), "chassis init finished");
    worker_thread_ = std::thread(&chassis_control::excute_loop, this);
}

void chassis_control::excute_loop(void)
{
    const double LOOP_DT = 0.001;      // 循环周期 1ms
    
    // 记录上次循环时间
    auto last_time = std::chrono::steady_clock::now();

    while (running_) {
      try {
        // 计算实际循环时间
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_time).count();
        last_time = current_time;
        // 限制dt在合理范围内，避免异常值
        if (dt > 0.1) dt = LOOP_DT;  // 如果dt过大，使用默认值
        if (dt < 0.0001) dt = LOOP_DT;  // 如果dt过小，使用默认值

        chassis_control_loop();
        arc_judge();

        double fl_angle_target = chassis_control_para.front_left_angle;
        double fr_angle_target = chassis_control_para.front_right_angle;
        double rl_angle_target = chassis_control_para.rear_left_angle;
        double rr_angle_target = chassis_control_para.rear_right_angle;
    
            // 应用变化率限制
        double fl_angle_limited = fl_rate_limiter.limit(fl_angle_target, dt);
        double fr_angle_limited = fr_rate_limiter.limit(fr_angle_target, dt);
        double rl_angle_limited = rl_rate_limiter.limit(rl_angle_target, dt);
        double rr_angle_limited = rr_rate_limiter.limit(rr_angle_target, dt);
        
        // 应用低通滤波
        chassis_control_para.front_left_angle = fl_angle_filter.filter(fl_angle_limited);
        chassis_control_para.front_right_angle = fr_angle_filter.filter(fr_angle_limited);
        chassis_control_para.rear_left_angle = rl_angle_filter.filter(rl_angle_limited);
        chassis_control_para.rear_right_angle = rr_angle_filter.filter(rr_angle_limited);
        
        if (!std::isfinite(chassis_control_para.front_left_angle )) 
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Invalid fl_angle: %f, using previous value", chassis_control_para.front_left_angle );
            chassis_control_para.front_left_angle  = fl_angle_filter.getOutput();  // 使用上次有效值
        }
        if (!std::isfinite(chassis_control_para.front_right_angle)) 
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,   
                                "Invalid fr_angle: %f, using previous value", chassis_control_para.front_right_angle);
            chassis_control_para.front_right_angle = fr_angle_filter.getOutput();
        }  
        if (!std::isfinite(chassis_control_para.rear_left_angle )) 
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Invalid rl_angle: %f, using previous value", chassis_control_para.rear_left_angle );
            chassis_control_para.rear_left_angle  = rl_angle_filter.getOutput();
        }
        if (!std::isfinite(chassis_control_para.rear_right_angle)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "Invalid rr_angle: %f, using previous value", chassis_control_para.rear_right_angle);
            chassis_control_para.rear_right_angle = rr_angle_filter.getOutput();
        }
        try {
          float fl_angle_f = static_cast<float>(chassis_control_para.front_left_angle);
          if (std::isfinite(fl_angle_f)) {
            // motor1.send_motion_command(0.0f, fl_angle_f, 0.0f, motor_kp, motor_kd);
            motor1.RobStrite_Motor_PosCSP_control(20.0f, fl_angle_f);
          } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Motor1: Skipping invalid angle %f", fl_angle_f);
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "Motor1 command failed: %s", e.what());
        }
        
        try {
          float fr_angle_f = static_cast<float>(chassis_control_para.front_right_angle);
          if (std::isfinite(fr_angle_f)) {
            // motor2.send_motion_command(0.0f, fr_angle_f, 0.0f, motor_kp, motor_kd);
            motor2.RobStrite_Motor_PosCSP_control(20.0f, fr_angle_f);
          } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Motor2: Skipping invalid angle %f", fr_angle_f);
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "Motor2 command failed: %s", e.what());
        }
        
        try {
          float rl_angle_f = static_cast<float>(chassis_control_para.rear_left_angle);
          if (std::isfinite(rl_angle_f)) {
           // motor3.send_motion_command(0.0f, rl_angle_f, 0.0f, motor_kp, motor_kd);
            motor3.RobStrite_Motor_PosCSP_control(20.0f, rl_angle_f);
          } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Motor3: Skipping invalid angle %f", rl_angle_f);
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "Motor3 command failed: %s", e.what());
        }
        
        try {
          float rr_angle_f = static_cast<float>(chassis_control_para.rear_right_angle);
          if (std::isfinite(rr_angle_f)) {
            // motor4.send_motion_command(0.0f, rr_angle_f, 0.0f, motor_kp, motor_kd);
            motor4.RobStrite_Motor_PosCSP_control(20.0f, rr_angle_f);
          } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Motor4: Skipping invalid angle %f", rr_angle_f);
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "Motor4 command failed: %s", e.what());
        }
      }catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                              "Error in control loop: %s", e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 出错时稍长等待
      }
      if((fabs(chassis_control_para.rear_right_angle-motor4.position_)<=0.1)
        && fabs(chassis_control_para.rear_left_angle-motor3.position_)<=0.1
        && fabs(chassis_control_para.front_right_angle-motor2.position_)<=0.1
        && fabs(chassis_control_para.front_left_angle-motor1.position_)<=0.1)
      {
          front_->set_target_speed_lr_rpm(chassis_control_para.front_right_speed*WHEEL_FR_DIRETION,
                                        chassis_control_para.front_left_speed*WHEEL_FL_DIRETION);
          rear_->set_target_speed_lr_rpm(chassis_control_para.rear_left_speed*WHEEL_RL_DIRETION,
                                        chassis_control_para.rear_right_speed*WHEEL_RR_DIRETION);
  
      }
      else
      {
        front_->set_target_speed_lr_rpm(0,0);
        rear_->set_target_speed_lr_rpm(0,0);
      }
    }

}

chassis_control::~chassis_control(void)
{
    motor1.Disenable_Motor(0);
    motor2.Disenable_Motor(0);
    motor3.Disenable_Motor(0);
    motor4.Disenable_Motor(0);


    running_ = false; // 停止线程
    if (worker_thread_.joinable())
      worker_thread_.join(); // 等待线程结束
}

void chassis_control::chassis_control_loop(void)
{
    this->get_parameter("robot.chassis_radius", chassis_param.chassis_radius);
    this->get_parameter("robot.wheel_perimeter",chassis_param.wheel_perimeter);

    float wheel_rpm_ratio;
	
    wheel_rpm_ratio = 60.0f/chassis_param.wheel_perimeter;	
	
    chassis_control_para.front_left_speed = sqrt(	pow(chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       +	pow(chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    chassis_control_para.rear_left_speed = sqrt(	pow(chassis_control_para.vy_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       +	pow(chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    chassis_control_para.front_right_speed = sqrt(	pow(chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       +	pow(chassis_control_para.vx_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    chassis_control_para.rear_right_speed = sqrt(	pow(chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2)
                       +	pow(chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f,2) 
                       ) * wheel_rpm_ratio ;
    
    // // 按比例限制速度：找出最大速度，如果超过限制则按比例缩放所有速度
    double max_speed = fabs(chassis_control_para.front_left_speed);
    if (fabs(chassis_control_para.front_right_speed) > max_speed)
        max_speed = fabs(chassis_control_para.front_right_speed);
    if (fabs(chassis_control_para.rear_left_speed) > max_speed)
        max_speed = fabs(chassis_control_para.rear_left_speed);
    if (fabs(chassis_control_para.rear_right_speed) > max_speed)
        max_speed = fabs(chassis_control_para.rear_right_speed);
    
    // // 如果最大速度超过限制，按比例缩放所有速度（保持运动方向）
    if (max_speed > MAX_WHEEL_SPEED) {
        double scale = MAX_WHEEL_SPEED / max_speed;
        chassis_control_para.front_left_speed *= scale;
        chassis_control_para.front_right_speed *= scale;
        chassis_control_para.rear_left_speed *= scale;
        chassis_control_para.rear_right_speed *= scale;
    }

    //舵向控制
    chassis_control_para.front_left_angle = atan2((chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f)) + chassis_param.fl_motor_start_angle;        
    chassis_control_para.front_right_angle = atan2((chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f)) + chassis_param.fr_motor_start_angle;
    chassis_control_para.rear_left_angle = atan2((chassis_control_para.vy_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f)) + chassis_param.rl_motor_start_angle;
    chassis_control_para.rear_right_angle = atan2((chassis_control_para.vy_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f)) + chassis_param.rr_motor_start_angle;
}

void chassis_control::arc_judge(void)
{


        double fl_angle = atan2((chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f));
        double fr_angle = atan2((chassis_control_para.vy_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f));
        double rl_angle = atan2((chassis_control_para.vy_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f));
        double rr_angle = atan2((chassis_control_para.vy_set - chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f),
                                                (chassis_control_para.vx_set + chassis_control_para.wz_set * chassis_param.chassis_radius * 0.707107f));

        if(fabs(chassis_control_para.front_left_angle- motor1.position_)>PI/2.0f+0.02f)
        {
          if(fl_angle>0)
          {
            chassis_control_para.front_left_angle = chassis_param.fl_motor_start_angle-(PI-fabs(fl_angle));
            chassis_control_para.front_left_speed = -chassis_control_para.front_left_speed;
          }
          else
          {
            chassis_control_para.front_left_angle = chassis_param.fl_motor_start_angle+PI-fabs(fl_angle);
            chassis_control_para.front_left_speed = -chassis_control_para.front_left_speed;
          }
        }
        if(fabs(chassis_control_para.front_right_angle-motor2.position_)>PI/2.0f+0.02f)
        {
          if(fr_angle>0)
          {
            chassis_control_para.front_right_angle = chassis_param.fr_motor_start_angle-(PI-fabs(fr_angle));
            chassis_control_para.front_right_speed = -chassis_control_para.front_right_speed;
          }
          else
          {
            chassis_control_para.front_right_angle = chassis_param.fr_motor_start_angle+PI-fabs(fr_angle);
            chassis_control_para.front_right_speed = -chassis_control_para.front_right_speed;
          }
        }
        if(fabs(chassis_control_para.rear_left_angle-motor3.position_)>PI/2.0f+0.02f)
        {
          if(rl_angle>0)
          {
            chassis_control_para.rear_left_angle = chassis_param.rl_motor_start_angle-(PI-fabs(rl_angle));
            chassis_control_para.rear_left_speed = -chassis_control_para.rear_left_speed;
          }
          else
          {
            chassis_control_para.rear_left_angle = chassis_param.rl_motor_start_angle+PI-fabs(rl_angle);
            chassis_control_para.rear_left_speed = -chassis_control_para.rear_left_speed;
          }
        }
        
        if(fabs(chassis_control_para.rear_right_angle-motor4.position_)>PI/2.0f+0.02f)
        {
          if(rr_angle>0)
          {
            chassis_control_para.rear_right_angle = chassis_param.rr_motor_start_angle-(PI-fabs(rr_angle));
            chassis_control_para.rear_right_speed = -chassis_control_para.rear_right_speed;
          }
          else
          {
            chassis_control_para.rear_right_angle = chassis_param.rr_motor_start_angle+PI-fabs(rr_angle);
            chassis_control_para.rear_right_speed = -chassis_control_para.rear_right_speed;
          }
        }
}

void chassis_control::svtrobot_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    chassis_control_para.vx_set = msg->linear.x;
    chassis_control_para.vy_set = msg->linear.y;
    chassis_control_para.wz_set = msg->angular.z;
}
