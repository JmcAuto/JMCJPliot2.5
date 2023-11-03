### OLD PROGRAM
void LatController::LoadSteerCalibrationTable(const LatControllerConf &lat_controller_conf){
  const auto &steer_torque_table = lat_controller_conf.steer_calibration_table() ;
  AINFO << "Steer_torque calibration table is loaded,the size of calibration is" 
      << steer_torque_table.steer_calibration_size() ;
  Interpolation2D::DataType xyz ;
  for (const auto &calibration : steer_torque_table.steer_calibration()){
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.angle(),
                                  calibration.torque())) ;
  }
  steer_torque_interpolation_.reset(new Interpolation2D) ;
  CHECK(steer_torque_interpolation_->Init(xyz))<<"Fail to init steer_torque_interpolation"  ;
}
## A
    /*
    A matrix (Gear Reverse)
    [0.0, 0.0, 1.0 * v 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */

   /*
    A matrix (Gear Drive)
    [0.0, 1.0, 0.0, 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */


 auto vehicle_state = VehicleStateProvider::instance() ;
  vehicle_state->set_linear_velocity(chassis->speed_mps());
//planning发布的轨迹
   trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(planning_published_trajectory)); 

  SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
  debug->Clear();
  if (FLAGS_use_dynamic_model) {
  //将planning轨迹点由后轴中心转化为车辆质心
//  if ((FLAGS_trajectory_transform_to_com_reverse && vehicle_state->gear()==canbus::Chassis::GEAR_REVERSE)||
//       FLAGS_trajectory_transform_to_com_drive && vehicle_state->gear()==canbus::Chassis::GEAR_DRIVE)
  if(FLAGS_trajectory_transform_to_com_reverse && planning_published_trajectory->decision().main_decision().has_parking()) 
 {
        ADEBUG << "R,planning trajectory coordinate change to center of mass" ;
        trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
  }
  //判断档位,根据档位设置模型参数
   if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
    ADEBUG << "Gear R coeff" ;
    cf_ = -control_conf_->lat_controller_conf().cf();
    cr_ = -control_conf_->lat_controller_conf().cr();
    matrix_a_(0, 1) = 0.0;
    matrix_a_coeff_(0, 2) = 1.0;
  } else {
    ADEBUG << "Gear D coeff" ;
    cf_ = control_conf_->lat_controller_conf().cf();
    cr_ = control_conf_->lat_controller_conf().cr();
    matrix_a_(0, 1) = 1.0;
    matrix_a_coeff_(0, 2) = 0.0;
  }
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;
  //更新R档下的航向，航向角在原基础上增加派
  UpdateDrivingOrientation();
  // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
  // Error Rate, preview lateral error1 , preview lateral error2, ...]
  UpdateState(debug);    //state matrix X
  UpdateMatrix();    //获取系数矩阵A，  matrix_a_，采用离散化公式获得matrix_ad_
  UpdateMatrixCompound(); //道路预览模型复合化离散矩阵 matrix_adc_

  //R档下更新q
  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
  int reverse_q_param_size = control_conf_->lat_controller_conf().reverse_matrix_q_size();
  if (vehicle_state->gear() ==
      canbus::Chassis::GEAR_REVERSE) {
    for (int i = 0; i < reverse_q_param_size; ++i) {
      matrix_q_(i, i) =
          control_conf_->lat_controller_conf().reverse_matrix_q(i);
    }
  } else {
    for (int i = 0; i < q_param_size; ++i) {
      matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
    }
  }

  // Add gain sheduler for higher speed steering增加增益sheduler以实现更高的转向速度，
  //在原来的q矩阵的基础上乘以系数，速度越大，系数越小
  if (FLAGS_enable_gain_scheduler) {
    matrix_q_updated_(0, 0) =
        matrix_q_(0, 0) *
        lat_err_interpolation_->Interpolate(
            vehicle_state->linear_velocity());//车速越高ratio越小，q(0,0)越小
    matrix_q_updated_(2, 2) =
        matrix_q_(2, 2) *
        heading_err_interpolation_->Interpolate(
            vehicle_state->linear_velocity());
    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_,
                                  matrix_r_, lqr_eps_, lqr_max_iteration_,
                                  &matrix_k_);
  } else {
    common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_,
                                  matrix_r_, lqr_eps_, lqr_max_iteration_,
                                  &matrix_k_);
  }
  AINFO << "Solve LQR succeed!" ;
  // feedback = - K * state
  // Convert vehicle steer angle from rad to degree and then to steering degree
  // then to 100% ratio
  double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0) * 180 /
                                      M_PI * steer_transmission_ratio_ /
                                      steer_single_direction_max_degree_ * 100 ;
  AINFO << "steer_angle_feedback = " << steer_angle_feedback ;
  //计算前馈
   double steer_angle_feedforward = ComputeFeedForward(debug->curvature());
  AINFO << "steer_angle_feedforward = " << steer_angle_feedforward ;
  // Clamp the steer angle to -100.0 to 100.0
  double steer_angle = common::math::Clamp(
      steer_angle_feedback + 0*steer_angle_feedforward, -108.0, 108.0);

   AINFO << "steer_angle = " << steer_angle ;
  if (FLAGS_set_steer_limit) {
    AINFO << "Limited steer angle according to vehicle speed";
    const double steer_limit =
        std::atan(max_lat_acc_ * wheelbase_ /
                  (vehicle_state->linear_velocity() *
                   vehicle_state->linear_velocity())) *
        steer_transmission_ratio_ * 180 / M_PI /
        steer_single_direction_max_degree_ * 100;//根据车速限制车轮最大转向角
    // Clamp the steer angle
    double steer_angle_limited =
        common::math::Clamp(steer_angle, -steer_limit, steer_limit);//将计算出的车轮转角限制在最大转向角内
    steer_angle_limited = digital_filter_.Filter(steer_angle_limited);//数字滤波，滤去高频信号
    steer_angle = steer_angle_limited;
    debug->set_steer_angle_limited(steer_angle);
  } else {
    steer_angle = digital_filter_.Filter(steer_angle);
  }
//车速<锁轮速度且D档且完全自动驾驶
  if (vehicle_state->linear_velocity() <
          FLAGS_lock_steer_speed && vehicle_state->linear_velocity() > 0.01) {
    steer_angle = pre_steer_angle_;
    AINFO << "vehicle speed <lock_steer_speed, Use pre_steer_angle!" ;
  }
  pre_steer_angle_ = steer_angle;
  double steering_angle_ = steer_angle * steer_single_direction_max_degree_ / 100 ;
  //xian zhi liangcijisuande zhuanjiao chazhi
  ADEBUG << "LQR+FeedForward steering_angle " << steering_angle_ ;

  double steering_angle_error = 0.0 ;


  steering_angle_error = steering_angle_ - chassis->steering_percentage();
  steering_angle_error = common::math::Clamp(steering_angle_error,-30.0,30.0);
  steering_angle_ = chassis->steering_percentage() + steering_angle_error ;
  ADEBUG << "chassis angle + steering_angle_error" << chassis->steering_percentage() << "," << steering_angle_error ;
  steering_angle_ = common::math::Clamp(steering_angle_,-482.0,482.0);
  previous_steering_angle = chassis->steering_percentage() ;
//  if(VehicleStateProvider::instance()->gear() == canbus::Chassis::GEAR_REVERSE){
  steering_angle_ = lateral_error_filter_.Update(steering_angle_);
//}

  cmd->set_steering_angle(steering_angle_);
  use_dymethed = false ;
  ADEBUG << "CAR speed" <<vehicle_state->linear_velocity() ;
  ADEBUG << "steering angle command " << steering_angle_ ;
  cmd->set_steering_rate(FLAGS_steer_angle_rate);
  const double steer_angle_lateral_contribution =
      -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI * steer_transmission_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_lateral_rate_contribution =
      -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI * steer_transmission_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_heading_contribution =
      -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI * steer_transmission_ratio_ /
      steer_single_direction_max_degree_ * 100;

  const double steer_angle_heading_rate_contribution =
      -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI * steer_transmission_ratio_ /
      steer_single_direction_max_degree_ * 100;
  ADEBUG << "Q(0-3) contribution" << steer_angle_lateral_contribution << "," 
            << steer_angle_lateral_rate_contribution << ","
            << steer_angle_heading_contribution << ","
            << steer_angle_heading_rate_contribution ;

  debug->set_heading(vehicle_state->heading());
 // debug->set_steer_angle(steering_angle_);
  debug->set_steer_angle_feedforward(steer_angle_feedforward);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  debug->set_steering_position(chassis->steering_percentage()/steer_transmission_ratio_/180*M_PI);
  debug->set_ref_speed(vehicle_state->linear_velocity());
  debug->set_steer_targe_position_error(steering_angle_ - chassis->steering_percentage()) ;//增加车轮目标位置和实际位置误差输出信息
  ADEBUG << "steering_position:" << debug->steering_position();
  ADEBUG << "steering_angle :" << steering_angle_ ;
  ProcessLogs(debug, chassis); 
  return Status::OK();
  } else {
  // if(vehicle_state->gear()==canbus::Chassis::GEAR_REVERSE){
     ADEBUG << "use kinetic model";
    //定义参数
    double kv = 0.1 ; //前视距离系数
    double Ld0 = FLAGS_drive_pure_distance ; //预瞄距离下限值
   if(vehicle_state->gear()==canbus::Chassis::GEAR_REVERSE){
         Ld0 = FLAGS_reverse_pure_distance ;
     }

    UpdateDrivingOrientation();
     //step1:寻找预瞄距离范围内最近的路径点
          //1.先找到离当前位置最近的路径点的下标
     auto pure_point = trajectory_analyzer_.QueryPurePathPoint(
      vehicle_state->x(), vehicle_state->y() , vehicle_state->linear_velocity(),kv , Ld0);
     double aphi = std::atan2(pure_point.y() - vehicle_state->y() , pure_point.x() - vehicle_state->x()) - driving_orientation ;
     if(vehicle_state->gear()==canbus::Chassis::GEAR_REVERSE){
         aphi = -aphi ;
     }  
  double Ld = std::sqrt((pure_point.x() - vehicle_state->x())*(pure_point.x() - vehicle_state->x())+(pure_point.y() - vehicle_state->y())*(pure_point.y() - vehicle_state->y())) ;
   ADEBUG << "Ld origin " << Ld ;
 //   Ld = std::max(Ld , Ld0);
 //   double aphi =std::asin(std::fabs(-(pure_point.x()-vehicle_state->x())*std::sin(driving_orientation)
  //                             +(pure_point.y()-vehicle_state->y()*std::cos(driving_orientation)))/Ld); 
    ADEBUG << "Ld = " << Ld ;
 if ( Ld < Ld0) {
     use_dymethed = true ;
    AINFO << "Ld<0.5,aphi=0!" ;
   return Status::OK();
  }
     double steer_angle_r = std::atan(2 * wheelbase_ * std::sin(aphi) /Ld );
//  pre_steer_angle_ = steer_angle_r;

     ADEBUG << "steer_angle_r " << steer_angle_r << ",aphi " << aphi << ", sin(aphi) " << std::sin(aphi) ;
     double steering_angle = steer_angle_r*180 / M_PI * steer_transmission_ratio_ ;
    ADEBUG << "kinetic steering angle " << steering_angle ;
     steering_angle = common::math::Clamp(steering_angle,-480.0,480.0);
     double steering_angle_diff = steering_angle - previous_steering_angle ;
     ADEBUG << "Last chassis steering angle " << previous_steering_angle ;
      ADEBUG << "steering_angle_diff " << steering_angle_diff ;
     if(steering_angle_diff <= 10 && steering_angle_diff >=-10){
        steering_angle_diff = 0 ;
	}
     steering_angle = previous_steering_angle + steering_angle_diff ;
     ADEBUG << "After limited steering_angle " << steering_angle ;
     double steering_angle_error = 0.0 ;
     steering_angle_error = steering_angle - chassis->steering_percentage();
     steering_angle_error = common::math::Clamp(steering_angle_error,-70.0,70.0);
     steering_angle = chassis->steering_percentage() + steering_angle_error ;
     ADEBUG << "chassis angle + steering_angle_error" << chassis->steering_percentage() << "," << steering_angle_error ;
     steering_angle = common::math::Clamp(steering_angle,-480.0,480.0);
  //   previous_steering_angle = chassis->steering_percentage() ;
     steering_angle = digital_filter_.Filter(steering_angle);
     ADEBUG << "Kinetic steering angle :" << steering_angle ;
     cmd->set_steering_angle(steering_angle);




##//UpdateState

PathPoint target_point = trajectory_analyzer.QueryMatchedPathPoint(x,y);
     const auto &preview_xy = VehicleStateProvider::instance()->EstimateFuturePosition(ts_*preview_window_);
    //  TrajectoryPoint target_point = trajectory_analyzer.QueryNearestPointByPosition(preview_xy.x(), preview_xy.y());
      auto d_theta = linear_v * std::tan(debug->steering_position()) / wheelbase_ ;
      auto preview_theta = theta + d_theta * ts_ * preview_window_ ;
      double dx = x - target_point.x();
      double dy = y - target_point.y();
   //   double dx = preview_xy.x() - target_point.path_point().x();
   //   double dy = preview_xy.y() - target_point.path_point().y(); 
      double cos_target_heading = std::cos(target_point.theta());
      double sin_target_heading = std::sin(target_point.theta());
          ADEBUG << "no preview kappa " << target_point.kappa() ;

      // 倒车时前几个点曲率很小，采用预瞄，预估未来ts_*preview_window_时间后的 车辆位置
 if(VehicleStateProvider::instance()->gear() ==
      canbus::Chassis::GEAR_REVERSE){
      target_point = trajectory_analyzer.QueryMatchedPathPoint(preview_xy.x(), preview_xy.y());
      dx = preview_xy.x() - target_point.x();
      dy = preview_xy.y() - target_point.y(); 
      cos_target_heading = std::cos(target_point.theta()+M_PI);
      sin_target_heading = std::sin(target_point.theta()+M_PI);
    }
//   TrajectoryPoint target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);
     AINFO << "x point: " << x << " y point: " << y;
     AINFO << "match point information : " << target_point.ShortDebugString();
     const double es = dx * cos_target_heading + dy * sin_target_heading ;
     lateral_error =cos_target_heading * dy - sin_target_heading * dx;
     AINFO << "cos_target_heading =" << cos_target_heading << ",sin_target_heading = " << sin_target_heading ;
     AINFO << "dx = " << dx << ",dy = " << dy << ",lateral_error =" << lateral_error ;
     debug->set_lateral_error(lateral_error);
     double target_theta = target_point.theta();
//     double target_theta_diff = target_theta - last_target_theta ;
//     if (std::fabs(target_theta_diff) > 0.008 && std::fabs(target_theta_diff) <0.06){
//        target_theta_diff = 0 ;
//     }
//     target_theta = last_target_theta +target_theta_diff ;
     debug->set_ref_heading(target_theta);
//     last_target_theta = target_theta ;
     heading_error = common::math::NormalizeAngle(theta - debug->ref_heading());
//     heading_error = heading_error_filter_.Update(heading_error);
     debug->set_heading_error(heading_error);
     debug->set_curvature(target_point.kappa());//参考曲率
     if(VehicleStateProvider::instance()->gear() ==canbus::Chassis::GEAR_REVERSE){
         
         debug->set_lateral_error(lateral_error);
         debug->set_ref_heading(common::math::NormalizeAngle(target_point.theta()+M_PI));

         heading_error = common::math::NormalizeAngle(preview_theta - (debug->ref_heading()+target_point.kappa()*es));
         debug->set_heading_error(heading_error);
   //     debug->set_curvature(curvature);
    }
      ADEBUG << "vehile_theta " << theta ;
	    ADEBUG << "target_theta " << debug->ref_heading() ;
  if(VehicleStateProvider::instance()->gear() ==canbus::Chassis::GEAR_REVERSE ){
   //    double kappa_diff = 0.0 ;
  //     kappa_diff = debug->curvature() - previous_kappa;
 //      ADEBUG << "kappa " << debug->curvature() << "previous_kappa " << previous_kappa << " kappa_diff " << kappa_diff ;
//       kappa_diff = common::math::Clamp(kappa_diff,-0.05,0.05);
      if (debug->curvature() > -0.04 && debug->curvature() < 0 ) {
         
         ADEBUG << "Limited finnal point kappa";
         debug->set_curvature(0);
    }
  }
 // previous_kappa = debug->curvature() ;
  ADEBUG << "curvature " << debug->curvature() ;
// Estimate the heading error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double heading_error_feedback;
  if (VehicleStateProvider::instance()->gear() ==
      canbus::Chassis::GEAR_REVERSE) {
    heading_error_feedback = debug->heading_error();
  } else {
    auto lookahead_point = trajectory_analyzer.QueryNearestPointByRelativeTime(
        lookahead_station_ /
            (std::max(std::fabs(linear_v), 0.1) * std::cos(debug->heading_error())));
    heading_error_feedback = common::math::NormalizeAngle(
        debug->heading_error() + target_point.theta() -
        lookahead_point.path_point().theta());
  }
  debug->set_heading_error_feedback(heading_error_feedback);

  // Estimate the lateral error with look-ahead/look-back windows as feedback
  // signal for special driving scenarios
  double lateral_error_feedback;
  if (VehicleStateProvider::instance()->gear() ==
      canbus::Chassis::GEAR_REVERSE) {
    lateral_error_feedback =
        debug->lateral_error() - lookback_station_ * std::sin(debug->heading_error());
  } else {
    lateral_error_feedback =
        debug->lateral_error() + lookahead_station_ * std::sin(heading_error);
  }
  debug->set_lateral_error_feedback(lateral_error_feedback);

  auto lateral_error_dot = linear_v * std::sin(debug->heading_error());
  auto lateral_error_dot_dot = linear_a * std::sin(debug->heading_error());
  if (FLAGS_reverse_heading_control) {
    if (VehicleStateProvider::instance()->gear() ==
        canbus::Chassis::GEAR_REVERSE) {
      lateral_error_dot = lateral_error_dot;
      lateral_error_dot_dot = lateral_error_dot_dot;
    }
  }
  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  if (VehicleStateProvider::instance()->gear() ==
      canbus::Chassis::GEAR_REVERSE) {
    debug->set_heading_rate(-angular_v);
    ADEBUG << "angular_v" << angular_v ;
  } else {
    debug->set_heading_rate(angular_v);
  }
  debug->set_ref_heading_rate(target_point.kappa() *
                              linear_v);
  ADEBUG << "angular_v" << debug->heading_rate() << ",ref_heading_rate " << debug->ref_heading_rate() ;
  debug->set_heading_error_rate(debug->heading_rate() -
                                debug->ref_heading_rate());
//if (VehicleStateProvider::instance()->gear() ==
//      canbus::Chassis::GEAR_REVERSE) {
//  debug->set_heading_error_rate(-(debug->heading_rate() -
//                                debug->ref_heading_rate()));
 
// } 

  debug->set_heading_acceleration(
      (debug->heading_rate() - previous_heading_rate_) / ts_);
  debug->set_ref_heading_acceleration(
      (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                        debug->ref_heading_acceleration());
  previous_heading_rate_ = debug->heading_rate();
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  debug->set_heading_jerk(
      (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  debug->set_ref_heading_jerk(
      (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
      ts_);
  debug->set_heading_error_jerk(debug->heading_jerk() -
                                debug->ref_heading_jerk());
  previous_heading_acceleration_ = debug->heading_acceleration();
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();

 // debug->set_curvature(target_point.path_point().kappa());